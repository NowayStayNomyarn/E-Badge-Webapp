// E-badge firmware: BLE -> e-ink image, PN532 UID -> BLE notify
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <GxEPD2_3C.h>
#include <Adafruit_PN532.h>
#include <NimBLEDevice.h>
// ---- Pins (from schematic; VERIFY e-ink pins against your netlist) ----
#define PIN_I2C_SDA 5     // PN532 I2C
#define PIN_I2C_SCL 6
#define PIN_EPD_BUSY 9    // 2.9" e-paper J7
#define PIN_EPD_CS   10
#define PIN_EPD_DC   11
#define PIN_EPD_RST  12
#define PIN_EPD_SDI  13   // MOSI
#define PIN_EPD_SCK  14
// Reserved: USB D-/D+ = IO19/IO20, BOOT = IO0, EN = reset button

// BLE protocol (must match webapp/index.html)
#define SVC_UUID  "7a1c0001-5b7e-4c1a-9d2e-0a1b2c3d4e5f"
#define CTRL_UUID "7a1c0002-5b7e-4c1a-9d2e-0a1b2c3d4e5f" // write: 0x01 begin, 0x02 commit
#define DATA_UUID "7a1c0003-5b7e-4c1a-9d2e-0a1b2c3d4e5f" // write(no resp): raw image chunks
#define STAT_UUID "7a1c0004-5b7e-4c1a-9d2e-0a1b2c3d4e5f" // notify: [code, lo, hi]
#define NFC_UUID  "7a1c0005-5b7e-4c1a-9d2e-0a1b2c3d4e5f" // notify: UID as hex text
#define LINK_UUID "7a1c0006-5b7e-4c1a-9d2e-0a1b2c3d4e5f" // write: URL to write to the next NTAG card (empty = cancel)

enum : uint8_t { ST_READY = 0, ST_RECV = 1, ST_DONE = 2, ST_ERROR = 3, ST_WAIT = 4, ST_WROTE = 5, ST_WFAIL = 6 };

constexpr int W = 296, H = 128;               // landscape
constexpr size_t PLANE = (W / 8) * H;         // 4736 bytes per colour plane, 1 = ink
constexpr size_t IMG_BYTES = PLANE * 2;       // black plane, then red plane

// Tri-colour 2.9" (SSD1680). If blank or garbled, try GxEPD2_290_Z13c instead.
GxEPD2_3C<GxEPD2_290_C90c, GxEPD2_290_C90c::HEIGHT> epd(
    GxEPD2_290_C90c(PIN_EPD_CS, PIN_EPD_DC, PIN_EPD_RST, PIN_EPD_BUSY));
Adafruit_PN532 nfc(0xFF, 0xFF, &Wire);        // I2C polling, no IRQ/RST pins

static uint8_t img[IMG_BYTES];
static size_t got = 0;
static bool receiving = false;
static volatile bool redraw = false;
static NimBLECharacteristic *statC, *nfcC;
static String pendingUrl;
static volatile bool armed = false;           // waiting for a card to write the link to
static uint32_t armedAt = 0;

void status(uint8_t code) {
  uint8_t v[3] = {code, (uint8_t)(got & 0xFF), (uint8_t)(got >> 8)};
  statC->setValue(v, 3);
  statC->notify();
}

class CtrlCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c) override {
    std::string v = c->getValue();
    if (v.empty()) return;
    if (v[0] == 1) { got = 0; receiving = true; status(ST_RECV); }
    else if (v[0] == 2) {
      if (receiving && got == IMG_BYTES) { receiving = false; redraw = true; }
      else { receiving = false; status(ST_ERROR); }
    }
  }
};

class DataCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c) override {
    if (!receiving) return;
    std::string v = c->getValue();
    if (got + v.size() > IMG_BYTES) { receiving = false; status(ST_ERROR); return; }
    memcpy(img + got, v.data(), v.size());
    got += v.size();
  }
};

class LinkCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c) override {
    std::string v = c->getValue();
    if (v.empty()) { armed = false; status(ST_READY); return; }
    pendingUrl = String(v.c_str());
    armedAt = millis();
    armed = true;
    status(ST_WAIT);
  }
};

// Write the URL as an NDEF URI record (NTAG213/215/216 only). Returns true on success.
bool writeLink(const String &u) {
  uint8_t id = NDEF_URIPREFIX_NONE;
  String rest = u;
  if (u.startsWith("https://www.")) { id = NDEF_URIPREFIX_HTTPS_WWWDOT; rest = u.substring(12); }
  else if (u.startsWith("http://www.")) { id = NDEF_URIPREFIX_HTTP_WWWDOT; rest = u.substring(11); }
  else if (u.startsWith("https://")) { id = NDEF_URIPREFIX_HTTPS; rest = u.substring(8); }
  else if (u.startsWith("http://")) { id = NDEF_URIPREFIX_HTTP; rest = u.substring(7); }
  char buf[128];
  rest.toCharArray(buf, sizeof(buf));
  return nfc.ntag2xx_WriteNDEFURI(id, buf, 144);   // 144 = NTAG213 user memory (smallest)
}

void drawImage() {
  epd.init(0, true, 2, false);
  epd.setRotation(1);                         // use 3 if the picture is upside down
  epd.setFullWindow();
  epd.firstPage();
  do {
    epd.fillScreen(GxEPD_WHITE);
    epd.drawBitmap(0, 0, img, W, H, GxEPD_BLACK);
    epd.drawBitmap(0, 0, img + PLANE, W, H, GxEPD_RED);
  } while (epd.nextPage());
  epd.hibernate();
}

void setup() {
  Serial.begin(115200);

  SPI.begin(PIN_EPD_SCK, -1, PIN_EPD_SDI, PIN_EPD_CS);
  epd.epd2.selectSPI(SPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  nfc.begin();
  if (!nfc.getFirmwareVersion()) Serial.println("PN532 not found (check I2C mode switch)");
  else nfc.SAMConfig();

  const char *name = "E-paper badge";            // Bluetooth name shown in the browser
  NimBLEDevice::init(name);
  NimBLEDevice::setMTU(247);
  NimBLEServer *srv = NimBLEDevice::createServer();
  NimBLEService *svc = srv->createService(SVC_UUID);
  svc->createCharacteristic(CTRL_UUID, NIMBLE_PROPERTY::WRITE)->setCallbacks(new CtrlCb());
  svc->createCharacteristic(DATA_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR)
      ->setCallbacks(new DataCb());
  statC = svc->createCharacteristic(STAT_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  nfcC = svc->createCharacteristic(NFC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  svc->createCharacteristic(LINK_UUID, NIMBLE_PROPERTY::WRITE)->setCallbacks(new LinkCb());
  svc->start();
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);
  adv->start();
  Serial.printf("Advertising as %s\n", name);
}

void loop() {
  if (redraw) {                               // draw outside the BLE callback (takes ~3 s)
    redraw = false;
    drawImage();
    status(ST_DONE);
  }

  if (armed && millis() - armedAt > 30000) { armed = false; status(ST_WFAIL); }   // no card in 30 s

  static uint32_t lastPoll = 0, lastSeen = 0;
  static String lastUid = "";
  if (millis() - lastPoll > 300) {
    lastPoll = millis();
    uint8_t uid[7], len = 0;
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &len, 80)) {
      String s;
      for (uint8_t i = 0; i < len; i++) { if (uid[i] < 16) s += '0'; s += String(uid[i], HEX); }
      s.toUpperCase();
      lastSeen = millis();
      if (armed) {                            // NTAG cards have a 7-byte UID
        bool ok = (len == 7) && writeLink(pendingUrl);
        armed = false;
        status(ok ? ST_WROTE : ST_WFAIL);
        Serial.println(ok ? "Link written" : "Link write failed");
      }
      if (s != lastUid) {                     // notify once per tap
        lastUid = s;
        nfcC->setValue((uint8_t *)s.c_str(), s.length());
        nfcC->notify();
        Serial.println("NFC " + s);
      }
    } else if (millis() - lastSeen > 1500) lastUid = "";
  }
}