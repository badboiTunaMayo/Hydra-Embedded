/*
 * ESP32-Hydra20 : Heart rate + SpO2 (MAX30102) + Body temp (GY-906 / MLX90614)
 * ส่งข้อมูลผ่าน Bluetooth Low Energy (BLE) เป็น JSON 1 บรรทัดต่อ 1 ตัวอย่าง ทุก 1 วินาที
 * ใช้ Nordic UART Service (NUS) เพื่อให้หน้าเว็บ (Web Bluetooth) ต่อได้ทั้งคอมพิวเตอร์และ Android
 *
 * การต่อสาย
 *   MAX30102 : GND-GND, VIN-3V3, SDA-IO21, SCL-IO22
 *   GY-906   : GND-GND, VIN-3V3, SDA-IO21, SCL-IO22
 *   LED(WS2812): VDD-5V, VSS-GND, DIN-IO5
 *
 * ไลบรารีที่ต้องติดตั้ง (Library Manager)
 *   - SparkFun MAX3010x Pulse and Proximity Sensor Library
 *   - Adafruit MLX90614 Library
 *   - Adafruit NeoPixel
 *   (BLE มากับ ESP32 core แบบ Arduino-ESP32 2.x / 3.x ไม่ต้องติดตั้งเพิ่ม)
 *   เลือกบอร์ด: ESP32 Dev Module (ESP32 / S3 / C3 ที่มี BLE ใช้ได้หมด)
 *   ถ้า Flash เต็ม ให้เลือก Partition Scheme = "Huge APP" หรือ "No OTA (2MB APP)"
 *
 * BLE (Nordic UART Service)
 *   Service  6E400001-B5A5-F393-E0A9-E50E24DCCA9E
 *   RX (เว็บ -> ESP32, write)   6E400002-B5A5-F393-E0A9-E50E24DCCA9E
 *   TX (ESP32 -> เว็บ, notify)  6E400003-B5A5-F393-E0A9-E50E24DCCA9E
 *   ข้อความยาวเกิน 20 ไบต์จะถูกหั่นเป็นชิ้นละ 20 ไบต์ ปลายทางต่อกลับแล้วแบ่งบรรทัดด้วย '\n'
 *
 * คำสั่งที่รับ (ทาง BLE หรือ USB Serial, 1 บรรทัด ลงท้าย Enter)
 *   LED ON | LED OFF | BRIGHT <0-255> | STATUS | PING | HELP
 *
 * หลักการสำคัญ: loop() ไม่มี delay() / ไม่มีการรอ BLE
 *   ทุกอย่างใช้ millis() ส่งข้อมูลทีละชิ้นต่อรอบ loop และส่งเฉพาะเมื่อมีผู้เชื่อมต่ออยู่
 *
 * SpO2: คำนวณแบบ ratio-of-ratios จากสัญญาณ RED/IR (ค่าประมาณ ไม่ใช่เครื่องมือแพทย์)
 *   SpO2 ~= 110 - 25 * R   โดย R = (ACred/DCred) / (ACir/DCir)
 *   ถ้าเทียบกับเครื่องวัดมาตรฐานแล้วคลาด ให้ปรับ SPO2_OFFSET
 *
 * ประเมินความเสี่ยงฮีทสโตรก (heat 0-3) จากคะแนนรวม: อุณหภูมิผิว + ชีพจร + อุณหภูมิรอบข้าง
 *   0 ปกติ | 1 เฝ้าระวัง | 2 เสี่ยงสูง | 3 อันตราย (สงสัยฮีทสโตรก)
 *   ต้องมีอุณหภูมิสูงกว่า TEMP_HIGH_C ก่อน จึงจะประเมินว่ามีความเสี่ยง
 *   หมายเหตุ: GY-906 วัดอุณหภูมิผิว ซึ่งต่ำกว่าอุณหภูมิแกนกลางร่างกาย เป็นค่าประมาณเท่านั้น
 *
 * ไฟ LED ติดเฉพาะเมื่อ (ถ้าไม่เข้าเงื่อนไขใดเลย ไฟดับ)
 *   1. ชีพจร < 60 หรือ > 100 bpm
 *   2. SpO2 ผิดปกติ (< 95 %)
 *   3. อุณหภูมิสูงกว่าปกติ ขณะมีภาวะเสี่ยงฮีทสโตรก (heat >= 1)
 */

#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
#include <BLE2902.h>              // Arduino-ESP32 2.x ต้องเพิ่ม descriptor เอง (3.x เพิ่มให้อัตโนมัติ)
#endif
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MLX90614.h>
#include <Adafruit_NeoPixel.h>

#if !defined(CONFIG_BT_ENABLED)
#error "Bluetooth ไม่ถูกเปิดใช้งาน ให้เลือกบอร์ด ESP32 Dev Module"
#endif

// ---------- กำหนดค่า ----------
#define DEVICE_NAME      "ESP32-Hydra20"
#define SDA_PIN          21
#define SCL_PIN          22
#define LED_PIN          5
#define LED_COUNT        8          // จำนวนดวง LED บนโมดูล (แก้ให้ตรงกับของจริง)
#define SEND_INTERVAL_MS 1000
#define FINGER_IR_ON     70000      // IR สูงกว่านี้ = มีนิ้ววาง (ต้องแตะจริง ไม่ใช่แค่ลอยใกล้ ๆ)
#define FINGER_IR_OFF    50000      // IR ต่ำกว่านี้ = นิ้วออก (ต่ำกว่าค่า ON เพื่อกันสถานะกระพริบ)
#define TEMP_OFFSET_C    0.0f       // ชดเชยอุณหภูมิ (ถ้าต้องการสอบเทียบ)
#define SENSOR_RETRY_MS  5000

// BLE
#define NUS_SERVICE_UUID "6E400001-B5A5-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A5-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A5-F393-E0A9-E50E24DCCA9E"
#define BLE_CHUNK        20         // ขนาดต่อชิ้น (MTU ขั้นต่ำ 23 - 3)
#define BLE_TX_GAP_MS    10         // เว้นระยะระหว่างชิ้น กัน notify ล้นคิว
#define BLE_TX_MAX       700        // คิวส่งสูงสุด (ไบต์) เกินนี้ทิ้งข้อมูลใหม่

// ตัวตรวจจับชีพจร
#define HR_SAMPLE_MS     10         // 1 ตัวอย่าง = 10 ms (400 sps / เฉลี่ย 4 = 100 sps) ถ้าแก้ setup() ต้องแก้ตรงนี้ด้วย
#define HR_DC_ALPHA      0.02f      // ตัวกรอง DC (ตัดฐานสัญญาณ)
#define HR_LP_ALPHA      0.25f      // ตัวกรองความถี่ต่ำ ลดสัญญาณรบกวน
#define HR_ENV_DECAY     0.995f     // ซองสัญญาณลดลงต่อตัวอย่าง (~60% ต่อวินาที)
#define HR_MIN_AMP       20.0f      // แอมพลิจูดต่ำสุดที่นับเป็นจังหวะ (เพิ่มถ้ามีจังหวะปลอมเยอะ)
#define HR_REFRACT_MS    300        // ห้ามนับจังหวะถี่กว่านี้ (= 200 bpm)
#define HR_TIMEOUT_MS    4000       // ไม่มีจังหวะนานกว่านี้ = ล้างค่า HR เป็น null

// SpO2
#define SPO2_WINDOW      300        // ตัวอย่างต่อหนึ่งรอบคำนวณ (~3 วินาทีที่ 100 sps)
#define SPO2_DC_ALPHA    0.02f      // ตัวกรองหา DC (ยิ่งน้อยยิ่งช้า)
#define SPO2_OFFSET      0.0f       // ชดเชยค่า SpO2 (ปรับเทียบกับเครื่องมาตรฐาน)
#define SPO2_MAX_BAD     2          // รอบที่สัญญาณแย่ติดกันก่อนจะล้างค่า

// เกณฑ์เตือน / ฮีทสโตรก
#define HR_LOW           60         // ชีพจรต่ำกว่านี้ = ผิดปกติ
#define HR_HIGH          100        // ชีพจรสูงกว่านี้ = ผิดปกติ
#define HR_VERY_HIGH     120        // ชีพจรสูงมาก (เพิ่มคะแนนฮีทสโตรก)
#define SPO2_LOW         95         // SpO2 ต่ำกว่านี้ = ผิดปกติ
#define TEMP_HIGH_C      37.5f      // อุณหภูมิสูงกว่าปกติ (ผิวหนัง)
#define TEMP_FEVER_C     38.0f      // อุณหภูมิสูงมาก
#define TEMP_DANGER_C    39.0f      // อุณหภูมิอันตราย
#define AMB_HOT_C        35.0f      // อากาศร้อน (เพิ่มคะแนนฮีทสโตรก)

MAX30105 particleSensor;
Adafruit_MLX90614 mlx;
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---------- สถานะ ----------
bool maxOk = false, mlxOk = false;
unsigned long lastMaxTry = 0, lastMlxTry = 0;

// heart rate
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
unsigned long lastBeat = 0;
unsigned long lastBeatWall = 0;   // millis() ตอนได้จังหวะที่ใช้ได้ล่าสุด (ไว้ตรวจ timeout)
float beatsPerMinute = 0;
int beatAvg = 0;
bool fingerOn = false;
long lastIr = 0;                // ค่า IR ล่าสุด (ไว้ดีบัก)
unsigned long beatFlashUntil = 0;

// SpO2
float dcRed = 0, dcIr = 0;
double sumSqRed = 0, sumSqIr = 0;
int spo2N = 0;
int spo2Skip = 1;               // ทิ้งรอบแรกหลังวางนิ้ว (สัญญาณยังไม่นิ่ง)
int spo2Bad = 0;
float spo2Smooth = 0;
int spo2Val = 0;                // 0 = ยังไม่มีค่าที่เชื่อถือได้

// temperature
float objTemp = NAN, ambTemp = NAN;

// LED
bool ledEnabled = true;
uint8_t ledBrightness = 60;

// การแจ้งเตือน / ฮีทสโตรก
int heatScore = 0;              // คะแนนความเสี่ยงฮีทสโตรก
int heatLevel = 0;              // 0 ปกติ 1 เฝ้าระวัง 2 เสี่ยงสูง 3 อันตราย
bool alertHr = false;           // ชีพจรผิดปกติ
bool alertSpo2 = false;         // SpO2 ผิดปกติ
bool alertTemp = false;         // อุณหภูมิสูง + เสี่ยงฮีทสโตรก

// timers
unsigned long lastSend = 0;

// command buffers (แยก USB / BLE)
String cmdBufUsb, cmdBufBle;

// ---------- BLE ----------
BLEServer* bleServer = nullptr;
BLECharacteristic* txChar = nullptr;
volatile bool bleConnected = false;
volatile bool bleNeedAdv = false;

String txQueue;                         // แตะเฉพาะใน loop() เท่านั้น
unsigned long lastTx = 0;

// คิววงกลมรับคำสั่งจาก BLE task -> loop() (ผู้เขียน 1 คน ผู้อ่าน 1 คน ไม่ต้องล็อก)
char rxRing[128];
volatile uint8_t rxHead = 0, rxTail = 0;

class ServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { bleConnected = true; }
  void onDisconnect(BLEServer*) override { bleConnected = false; bleNeedAdv = true; }
};

class RxCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue().c_str();   // 2.x คืน std::string, 3.x คืน String ใช้ c_str() ได้ทั้งคู่
    for (size_t i = 0; i < v.length(); i++) {
      uint8_t n = (rxHead + 1) & 127;
      if (n == rxTail) break;           // คิวเต็ม ทิ้งส่วนที่เหลือ
      rxRing[rxHead] = v[i];
      rxHead = n;
    }
  }
};

void initBle() {
  BLEDevice::init(DEVICE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCb());

  BLEService* svc = bleServer->createService(NUS_SERVICE_UUID);

  txChar = svc->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
  txChar->addDescriptor(new BLE2902());
#endif

  BLECharacteristic* rxChar = svc->createCharacteristic(
      NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxChar->setCallbacks(new RxCb());

  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setScanResponse(true);           // ชื่ออุปกรณ์อยู่ใน scan response
  BLEDevice::startAdvertising();
}

// ใส่ข้อความลงคิวส่ง (ไม่บล็อก) ส่งจริงทีละชิ้นใน bleFlush()
void bleSend(const char* s) {
  if (!bleConnected) return;
  if (txQueue.length() + strlen(s) + 1 > BLE_TX_MAX) return;
  txQueue += s;
  txQueue += '\n';
}

void bleFlush() {
  if (!bleConnected) { if (txQueue.length()) txQueue = ""; return; }
  if (!txQueue.length() || txChar == nullptr) return;
  unsigned long now = millis();
  if (now - lastTx < BLE_TX_GAP_MS) return;
  lastTx = now;
  int n = txQueue.length() < BLE_CHUNK ? (int)txQueue.length() : BLE_CHUNK;
  txChar->setValue((uint8_t*)txQueue.c_str(), n);
  txChar->notify();
  txQueue.remove(0, n);
}

// ---------- Sensor init (ไม่บล็อก ถ้าหาไม่เจอจะลองใหม่เป็นระยะ) ----------
void initMax() {
  if (particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {   // 100 kHz ให้ GY-906 ใช้บัสเดียวกันได้
    // ledBrightness, sampleAverage, ledMode(2=Red+IR), sampleRate, pulseWidth, adcRange
    particleSensor.setup(0x3C, 4, 2, 400, 411, 4096);
    particleSensor.setPulseAmplitudeRed(0x3C);   // SpO2 ต้องใช้ไฟแดงสว่างใกล้เคียง IR
    particleSensor.setPulseAmplitudeGreen(0);
    maxOk = true;
  } else {
    maxOk = false;
  }
}

void initMlx() {
  mlxOk = mlx.begin();   // address 0x5A
}

// ---------- LED ----------
void showColor(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t c = strip.Color(r, g, b);
  for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, c);
  strip.show();
}

// ---------- ประเมินความผิดปกติ + ความเสี่ยงฮีทสโตรก ----------
void computeAlerts() {
  bool hrValid   = fingerOn && beatAvg > 0;
  bool spo2Valid = fingerOn && spo2Val > 0;
  bool tValid    = mlxOk && !isnan(objTemp);
  bool aValid    = mlxOk && !isnan(ambTemp);

  alertHr   = hrValid && (beatAvg < HR_LOW || beatAvg > HR_HIGH);
  alertSpo2 = spo2Valid && spo2Val < SPO2_LOW;

  // คะแนนฮีทสโตรก: ต้องมีอุณหภูมิสูงกว่าปกติเป็นเงื่อนไขหลัก
  int score = 0;
  if (tValid && objTemp > TEMP_HIGH_C) {
    score += 1;
    if (objTemp >= TEMP_FEVER_C)  score += 1;
    if (objTemp >= TEMP_DANGER_C) score += 1;
    if (hrValid && beatAvg > HR_HIGH)      score += 1;
    if (hrValid && beatAvg >= HR_VERY_HIGH) score += 1;
    if (aValid && ambTemp >= AMB_HOT_C)    score += 1;
  }
  heatScore = score;
  if (score >= 5)      heatLevel = 3;   // อันตราย
  else if (score >= 3) heatLevel = 2;   // เสี่ยงสูง
  else if (score >= 2) heatLevel = 1;   // เฝ้าระวัง
  else                 heatLevel = 0;

  // อุณหภูมิสูงที่ทำให้ LED ติด ต้องอยู่ในภาวะเสี่ยงฮีทสโตรกด้วย
  alertTemp = (heatLevel >= 1);
}

// LED ติดเฉพาะเมื่อมีความผิดปกติ นอกนั้นดับ
void updateLed() {
  static unsigned long lastUpd = 0;
  unsigned long now = millis();
  if (now - lastUpd < 30) return;
  lastUpd = now;

  computeAlerts();

  strip.setBrightness(ledBrightness);
  if (!ledEnabled) { showColor(0, 0, 0); return; }

  if (alertTemp) {
    if (heatLevel >= 3) {
      // อันตราย: แดงกะพริบเร็ว
      if ((now / 200) % 2) showColor(255, 0, 0); else showColor(0, 0, 0);
    } else if (heatLevel == 2) {
      showColor(255, 80, 0);             // ส้มแดง เสี่ยงสูง
    } else {
      showColor(255, 170, 0);            // ส้มเหลือง เฝ้าระวัง
    }
  } else if (alertSpo2) {
    showColor(40, 90, 255);              // น้ำเงิน: SpO2 ผิดปกติ
  } else if (alertHr) {
    if (now < beatFlashUntil) showColor(255, 105, 150);   // ชมพูตามจังหวะหัวใจ
    else showColor(90, 40, 60);
  } else {
    showColor(0, 0, 0);                  // ปกติ: ไฟดับ
  }
}

// ---------- ส่งข้อมูล ----------
void sendStatus() {
  char buf[400];
  char hrS[12], spS[12], tS[12], aS[12];
  computeAlerts();                        // ให้ค่าที่ส่งตรงกับสถานะล่าสุด
  if (fingerOn && beatAvg > 0) snprintf(hrS, sizeof(hrS), "%d", beatAvg);
  else snprintf(hrS, sizeof(hrS), "null");
  if (fingerOn && spo2Val > 0) snprintf(spS, sizeof(spS), "%d", spo2Val);
  else snprintf(spS, sizeof(spS), "null");
  if (mlxOk && !isnan(objTemp)) snprintf(tS, sizeof(tS), "%.2f", objTemp);
  else snprintf(tS, sizeof(tS), "null");
  if (mlxOk && !isnan(ambTemp)) snprintf(aS, sizeof(aS), "%.2f", ambTemp);
  else snprintf(aS, sizeof(aS), "null");

  snprintf(buf, sizeof(buf),
           "{\"ts\":%lu,\"hr\":%s,\"spo2\":%s,\"temp\":%s,\"ambient\":%s,\"finger\":%s,"
           "\"ir\":%ld,\"led\":%s,\"bright\":%u,\"max\":%s,\"mlx\":%s,"
           "\"heat\":%d,\"heatScore\":%d}",
           millis(), hrS, spS, tS, aS,
           fingerOn ? "true" : "false",
           lastIr,
           ledEnabled ? "true" : "false", ledBrightness,
           maxOk ? "true" : "false", mlxOk ? "true" : "false",
           heatLevel, heatScore);

  Serial.println(buf);                    // USB debug เสมอ
  bleSend(buf);                           // BLE เฉพาะเมื่อมีคนต่ออยู่
}

void reply(const char* msg) {
  char buf[120];
  snprintf(buf, sizeof(buf), "{\"msg\":\"%s\"}", msg);
  Serial.println(buf);
  bleSend(buf);
}

// ---------- รับคำสั่ง (ไม่บล็อก) ----------
void handleCommand(String c) {
  c.trim();
  c.toUpperCase();
  if (c.length() == 0) return;

  if (c == "LED ON")        { ledEnabled = true;  reply("led on"); }
  else if (c == "LED OFF")  { ledEnabled = false; reply("led off"); }
  else if (c.startsWith("BRIGHT")) {
    int v = c.substring(6).toInt();
    ledBrightness = (uint8_t)constrain(v, 0, 255);
    reply("brightness set");
  }
  else if (c == "STATUS")   { sendStatus(); }
  else if (c == "PING")     { reply("pong"); }
  else if (c == "HELP")     { reply("LED ON|LED OFF|BRIGHT n|STATUS|PING"); }
  else                      { reply("unknown command"); }
}

void feedCmd(String& buf, char ch) {
  if (ch == '\n' || ch == '\r') {
    if (buf.length()) { handleCommand(buf); buf = ""; }
  } else if (buf.length() < 48) {
    buf += ch;
  } else {
    buf = "";                             // บัฟเฟอร์เกิน ทิ้ง
  }
}

void readCommands(Stream& s) {
  int guard = 64;                         // จำกัดจำนวนไบต์ต่อรอบ กัน loop ค้าง
  while (s.available() && guard-- > 0) feedCmd(cmdBufUsb, (char)s.read());
}

void readBleCommands() {
  int guard = 64;
  while (rxTail != rxHead && guard-- > 0) {
    char ch = rxRing[rxTail];
    rxTail = (rxTail + 1) & 127;
    feedCmd(cmdBufBle, ch);
  }
}

// ---------- SpO2 ----------
void resetSpo2() {
  dcRed = dcIr = 0;
  sumSqRed = sumSqIr = 0;
  spo2N = 0;
  spo2Skip = 1;
  spo2Bad = 0;
  spo2Smooth = 0;
  spo2Val = 0;
}

void processSpo2(long red, long ir) {
  if (dcRed == 0) { dcRed = (float)red; dcIr = (float)ir; }
  dcRed += SPO2_DC_ALPHA * ((float)red - dcRed);
  dcIr  += SPO2_DC_ALPHA * ((float)ir  - dcIr);
  float acR = (float)red - dcRed;
  float acI = (float)ir  - dcIr;
  sumSqRed += (double)acR * acR;
  sumSqIr  += (double)acI * acI;

  if (++spo2N < SPO2_WINDOW) return;

  float rmsR = (float)sqrt(sumSqRed / spo2N);
  float rmsI = (float)sqrt(sumSqIr  / spo2N);
  sumSqRed = sumSqIr = 0;
  spo2N = 0;

  if (spo2Skip > 0) { spo2Skip--; return; }

  bool valid = false;
  float s = 0;
  if (dcRed > 1 && dcIr > 1 && rmsI > 0) {
    float perfusion = rmsI / dcIr;                         // สัญญาณชีพจรต้องไม่อ่อน/แรงผิดปกติ
    float R = (rmsR / dcRed) / perfusion;
    if (perfusion > 0.0005f && perfusion < 0.2f && R > 0.2f && R < 1.8f) {
      s = 110.0f - 25.0f * R + SPO2_OFFSET;
      s = constrain(s, 70.0f, 100.0f);
      valid = true;
    }
  }

  if (valid) {
    spo2Bad = 0;
    spo2Smooth = (spo2Smooth <= 0) ? s : (0.7f * spo2Smooth + 0.3f * s);
    spo2Val = (int)(spo2Smooth + 0.5f);
  } else if (++spo2Bad >= SPO2_MAX_BAD) {
    spo2Smooth = 0;
    spo2Val = 0;
  }
}

// ---------- ตรวจจับจังหวะหัวใจ (แทน checkForBeat ที่ค้างได้) ----------
// ทำงานต่อ 1 ตัวอย่าง IR ที่ ~100 sps (400 / เฉลี่ย 4)
// เวลาคิดจากจำนวนตัวอย่าง ไม่ใช่ millis() เพราะ FIFO ถูกอ่านเป็นก้อน ทำให้ millis() เหลื่อมกัน
float hrDc = 0, hrLp = 0, hrPrev1 = 0, hrPrev2 = 0, hrEnv = 0;
unsigned long hrClock = 0;              // เวลาจากตัวอย่าง (ms)
unsigned long hrLastPeak = 0;           // เวลาของพีคล่าสุด (ms ตามนาฬิกาตัวอย่าง)
byte hrRejects = 0;                     // จำนวนช่วงที่ถูกปัดทิ้งติดกัน

void resetHr() {
  beatAvg = 0; rateSpot = 0; lastBeat = 0; lastBeatWall = 0; hrRejects = 0;
  for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0;
  hrDc = hrLp = hrPrev1 = hrPrev2 = hrEnv = 0;
  hrLastPeak = 0;
}

void addBeat(float bpm) {
  rates[rateSpot++] = (byte)bpm;
  rateSpot %= RATE_SIZE;
  int sum = 0, n = 0;
  for (byte i = 0; i < RATE_SIZE; i++) if (rates[i]) { sum += rates[i]; n++; }
  if (n) beatAvg = sum / n;
}

void processHr(long ir) {
  hrClock += HR_SAMPLE_MS;
  if (hrDc == 0) hrDc = (float)ir;
  hrDc += HR_DC_ALPHA * ((float)ir - hrDc);
  float ac = hrDc - (float)ir;                 // กลับขั้ว: เลือดมาก = ค่าบวก
  hrLp += HR_LP_ALPHA * (ac - hrLp);           // กรองความถี่สูงออก (~4 Hz)

  // ซองสัญญาณ (ค่ายอดที่ค่อย ๆ ลดลง) ใช้ปรับเกณฑ์ตามแอมพลิจูดจริง ไม่ค้างที่ค่าเก่า
  hrEnv = (hrLp > hrEnv) ? hrLp : hrEnv * HR_ENV_DECAY;
  float thr = hrEnv * 0.5f;
  if (thr < HR_MIN_AMP) thr = HR_MIN_AMP;

  // พีค = จุดที่ค่าก่อนหน้าสูงกว่าทั้งสองข้าง และเกินเกณฑ์
  bool peak = (hrPrev1 > hrPrev2) && (hrPrev1 >= hrLp) && (hrPrev1 > thr);
  hrPrev2 = hrPrev1;
  hrPrev1 = hrLp;
  if (!peak) return;

  unsigned long t = hrClock;
  if (hrLastPeak != 0) {
    unsigned long delta = t - hrLastPeak;
    if (delta < HR_REFRACT_MS) return;         // ใกล้เกินไป = พีคซ้อน (dicrotic) ข้ามไป
    float bpm = 60000.0f / (float)delta;
    bool inRange = (bpm >= 30.0f && bpm <= 220.0f);
    bool consistent = (beatAvg == 0) || (fabsf(bpm - beatAvg) <= beatAvg * 0.35f);
    if (inRange && (consistent || hrRejects >= 2)) {
      if (!consistent) {                       // ปัดทิ้งติดกัน 3 ครั้ง = ชีพจรเปลี่ยนจริง เริ่มเฉลี่ยใหม่
        for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0;
        rateSpot = 0;
      }
      hrRejects = 0;
      addBeat(bpm);
      lastBeatWall = millis();
    } else {
      hrRejects++;                             // ช่วงผิดปกติ (ขยับ/พลาดจังหวะ) ไม่นำมาเฉลี่ย
    }
  } else {
    lastBeatWall = millis();                   // พีคแรกหลังวางนิ้ว เริ่มจับเวลา timeout
  }
  hrLastPeak = t;
  lastBeat = t;
  beatFlashUntil = millis() + 120;
}

// ---------- อ่าน MAX30102 แบบไม่บล็อก ----------
void readHeart() {
  if (!maxOk) return;

  particleSensor.check();                 // ดึงข้อมูลใหม่จาก FIFO (ไม่รอ)
  while (particleSensor.available()) {
    long ir  = particleSensor.getFIFOIR();
    long red = particleSensor.getFIFORed();
    particleSensor.nextSample();
    lastIr = ir;

    fingerOn = fingerOn ? (ir > FINGER_IR_OFF) : (ir > FINGER_IR_ON);
    if (!fingerOn) {
      resetHr();
      resetSpo2();
      continue;
    }

    processSpo2(red, ir);
    processHr(ir);
  }

  // ชีพจรไม่มีจังหวะใหม่นานเกินกำหนด -> ล้างค่า (กันค่าค้างจากจังหวะสุดท้าย)
  if (beatAvg > 0 && millis() - lastBeatWall > HR_TIMEOUT_MS) resetHr();
}

// ---------- อ่านอุณหภูมิ (ทุก 1 วินาที) ----------
void readTemp() {
  if (!mlxOk) return;
  float o = mlx.readObjectTempC();
  float a = mlx.readAmbientTempC();
  // ทิ้งค่าที่เป็นไปไม่ได้ (เช่น 1037.55 = อ่าน I2C ล้มเหลว)
  bool oOk = !isnan(o) && o > -40.0f && o < 125.0f;
  bool aOk = !isnan(a) && a > -40.0f && a < 125.0f;
  objTemp = oOk ? o + TEMP_OFFSET_C : NAN;
  ambTemp = aOk ? a : NAN;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeOut(50);                    // กัน I2C ค้างนาน (หน่วย ms)

  strip.begin();
  strip.setBrightness(ledBrightness);
  strip.show();

  initMax();
  initMlx();
  Wire.setClock(100000);                  // MLX90614 (SMBus) ทำงานได้ไม่เกิน ~100 kHz

  initBle();                              // เปิด BLE + เริ่มโฆษณา
  cmdBufUsb.reserve(48);
  cmdBufBle.reserve(48);
  txQueue.reserve(BLE_TX_MAX + 16);
  Serial.println("ESP32-Hydra20 ready (BLE)");
}

void loop() {
  unsigned long now = millis();

  // ลองต่อเซนเซอร์ใหม่เป็นระยะถ้ายังไม่พบ
  if (!maxOk && now - lastMaxTry > SENSOR_RETRY_MS) { lastMaxTry = now; initMax(); }
  if (!mlxOk && now - lastMlxTry > SENSOR_RETRY_MS) { lastMlxTry = now; initMlx(); }

  readHeart();                            // เรียกถี่ ๆ เพื่อจับจังหวะ
  updateLed();

  // รับคำสั่ง จาก BLE และจาก USB
  readBleCommands();
  readCommands(Serial);

  // หลุดการเชื่อมต่อ -> เริ่มโฆษณาใหม่ให้ต่อได้อีก
  if (bleNeedAdv) { bleNeedAdv = false; BLEDevice::startAdvertising(); }

  // ส่งสถานะทุก 1 วินาที (ลงคิว) แล้วทยอยส่งทีละชิ้น
  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;
    readTemp();
    sendStatus();
  }
  bleFlush();
  // ไม่มี delay() : loop วิ่งต่อเสมอ ไม่ว่า BLE จะต่ออยู่หรือไม่
}
