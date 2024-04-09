/*
#include <Arduino.h>

#define PIN_12 12 // 12番ピン
#define PIN_13 13 // 13番ピン
#define PIN_14 14 // 14番ピン
#define PIN_15 15 // 15番ピン
#define PIN_42 42 // 42番ピン
#define PIN_41 41 // 41番ピン
#define PIN_40 40 // 40番ピン
#define PIN_39 39 // 39番ピン
#define PIN_38 38 // 38番ピン
#define PIN_37 37 // 37番ピン
#define PIN_36 36 // 36番ピン
#define PIN_35 35 // 35番ピン
#define PIN_0   0 // 0番ピン

// ピンの配列を定義
const int pins[] = {PIN_12, PIN_13, PIN_14, PIN_15, PIN_42, PIN_41, PIN_40, PIN_39, PIN_38, PIN_37, PIN_36, PIN_35, PIN_0};
const int numPins = sizeof(pins) / sizeof(pins[0]);ı˜

void setup() {
  // 全てのピンを出力モードに設定
  for (int i = 0; i < numPins; i++) {
    pinMode(pins[i], OUTPUT);
  }
}

void loop() {
  // 順番にLOWに設定
  for (int i = 0; i < numPins; i++) {
    digitalWrite(pins[i], LOW);
    delay(500); // 0.5秒待つ
  }

  // 順番にHIGHに設定
  for (int i = 0; i < numPins; i++) {
    digitalWrite(pins[i], HIGH);
    delay(500); // 0.5秒待つ
  }
}
*/

/*
#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME_SDA 1 // ESP-WROOM-32のSDAピン
#define BME_SCL 2 // ESP-WROOM-32のSCLピン

Adafruit_BME280 bme; // BME280オブジェクトを作成

void setup() {
  Wire.begin(BME_SDA, BME_SCL); // I2Cを初期化
  Serial.begin(115200);

  bool status = bme.begin(0x76); // BME280のI2Cアドレスを指定して初期化

  if (!status) {
    Serial.println("BME280が見つかりません。接続を確認してください。");
    while (1);
  }
}

void loop() {
  Serial.print("温度: ");
  Serial.print(bme.readTemperature());
  Serial.print(" °C");

  Serial.print("湿度: ");
  Serial.print(bme.readHumidity());
  Serial.print(" %");

  Serial.print("気圧: ");
  Serial.print(bme.readPressure() / 100.0F); // パスカルからヘクトパスカルに変換
  Serial.println(" hPa");

  delay(500); // 2秒待機
}
*/

/*
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_ADDRESS (0x28) // BNO055のI2Cアドレス

#define BNO_SDA 1 // ESP-WROOM-32のSDAピン
#define BNO_SCL 2 // ESP-WROOM-32のSCLピン

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS);

void setup() {
  Wire.begin(BNO_SDA, BNO_SCL); // I2Cを初期化
  Serial.begin(115200);
  delay(1000);

  if (!bno.begin())
  {
    Serial.println("BNO055が見つかりませんでした。接続を確認してください。");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);

  Serial.print("姿勢: ");
  Serial.print(event.orientation.x);
  Serial.print(" ");
  Serial.print(event.orientation.y);
  Serial.print(" ");
  Serial.print(event.orientation.z);
  Serial.println();

  delay(100);
}
*/

/*
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define SDA_PIN 1 // ESP-WROOM-32のSDAピン
#define SCL_PIN 2 // ESP-WROOM-32のSCLピン

Adafruit_BME280 bme; // BME280オブジェクトを作成
Adafruit_BNO055 bno = Adafruit_BNO055(55); // BNO055オブジェクトを作成

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN); // I2Cを初期化
  Serial.begin(115200);

  bool status = bme.begin(); // BME280のデフォルトI2Cアドレスで初期化
  if (!status) {
    Serial.println("BME280が見つかりません。接続を確認してください。");
    while (1);
  }

  if (!bno.begin())
  {
    Serial.println("BNO055が見つかりませんでした。接続を確認してください。");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop() {
  // BME280から温度、湿度、気圧を取得してシリアルモニタに表示
  Serial.print("BME280 - 温度: ");
  Serial.print(bme.readTemperature());
  Serial.print(" °C, 湿度: ");
  Serial.print(bme.readHumidity());
  Serial.print(" %, 気圧: ");
  Serial.print(bme.readPressure() / 100.0F); // パスカルからヘクトパスカルに変換
  Serial.println(" hPa");

  // BNO055から姿勢を取得してシリアルモニタに表示
  sensors_event_t event;
  bno.getEvent(&event);
  Serial.print("BNO055 - 姿勢: ");
  Serial.print(event.orientation.x);
  Serial.print(" ");
  Serial.print(event.orientation.y);
  Serial.print(" ");
  Serial.print(event.orientation.z);
  Serial.println();

  delay(1000); // 1秒待機
}
*/

/*
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BME_SDA 1 // ESP-WROOM-32のSDAピン
#define BME_SCL 2 // ESP-WROOM-32のSCLピン
#define BME280_ADDRESS 0x76 // BME280のI2Cアドレス
#define BNO_SDA 1 // ESP-WROOM-32のSDAピン
#define BNO_SCL 2 // ESP-WROOM-32のSCLピン
#define BNO055_ADDRESS (0x28) // BNO055のI2Cアドレス

Adafruit_BME280 bme; // BME280オブジェクトを作成
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS);

void setup() {
  Wire.begin(BME_SDA, BME_SCL); // BME280用のI2Cを初期化
  Wire.begin(BNO_SDA, BNO_SCL); // BNO055用のI2Cを初期化
  Serial.begin(115200);

  bool status = bme.begin(BME280_ADDRESS); // BME280のI2Cアドレスを指定して初期化

  if (!status) {
    Serial.println("BME280が見つかりません。接続を確認してください。");
    while (1);
  }

  if (!bno.begin())
  {
    Serial.println("BNO055が見つかりませんでした。接続を確認してください。");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

void loop() {
  // BME280からのデータ取得
  Serial.print("BME280 - 温度: ");
  Serial.print(bme.readTemperature());
  Serial.print(" °C, 湿度: ");
  Serial.print(bme.readHumidity());
  Serial.print(" %, 気圧: ");
  Serial.print(bme.readPressure() / 100.0F); // パスカルからヘクトパスカルに変換
  Serial.print(" hPa");

  // BNO055からのデータ取得
  sensors_event_t event;
  bno.getEvent(&event);

  Serial.print("BNO055 - 姿勢: ");
  Serial.print(event.orientation.x);
  Serial.print(" ");
  Serial.print(event.orientation.y);
  Serial.print(" ");
  Serial.print(event.orientation.z);
  Serial.println();

  delay(100); // 1秒待機
}
*/








#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <CAN.h>
#include <SPI.h>
#include <MCP2515.h>

#define BME_SDA 1 // ESP-WROOM-32のSDAピン
#define BME_SCL 2 // ESP-WROOM-32のSCLピン
#define BME280_ADDRESS 0x76 // BME280のI2Cアドレス
#define BNO_SDA 1 // ESP-WROOM-32のSDAピン
#define BNO_SCL 2 // ESP-WROOM-32のSCLピン
#define BNO055_ADDRESS (0x28) // BNO055のI2Cアドレス

Adafruit_BME280 bme; // BME280オブジェクトを作成
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS);

// #define MODE 0 //0=rx, 1=tx

// CANMSG msg(6);
// MCP2515 can(7);


// void sendCANMessage(float temperature, float humidity, float pressure, float orientationX, float orientationY, float orientationZ) {
//   can.initCAN(); // CAN通信を初期化（250kbps）
//   // CAN_message_t msg;
//   msg.adrsValue = 0x123; // 送信するメッセージのID
//   msg.dataLength = 8; // データのバイト数
//   msg.data[0] = temperature; // 温度
//   msg.data[1] = humidity; // 湿度
//   msg.data[2] = pressure; // 気圧
//   msg.data[3] = orientationX; // 姿勢 X
//   msg.data[4] = orientationY; // 姿勢 Y
//   msg.data[5] = orientationZ; // 姿勢 Z
//   can.transmitCANMessage(msg, 1000); // メッセージを送信
// }


// void setup() {
//   Wire.begin(BME_SDA, BME_SCL); // I2Cを初期化
//   Serial.begin(115200);

//   // BME280の初期化
//   bool status = bme.begin(BME280_ADDRESS); // BME280のI2Cアドレスを指定して初期化
//   if (!status) {
//     Serial.println("BME280が見つかりません。接続を確認してください。");
//     while (1);
//   }

//   // BNO055の初期化
//   if (!bno.begin()) {
//     Serial.println("BNO055が見つかりませんでした。接続を確認してください。");
//     while (1);
//   }

//   bno.setExtCrystalUse(true); // 外部クリスタルを使用するように設定
//   Serial.println("Setup completed");
// }

// void loop() {
//   // BME280からのデータ取得
//   float temperature = bme.readTemperature();
//   float humidity = bme.readHumidity();
//   float pressure = bme.readPressure() / 100.0F; // パスカルからヘクトパスカルに変換

//   // BNO055からのデータ取得
//   sensors_event_t event;
//   bno.getEvent(&event);
//   float orientationX = event.orientation.x;
//   float orientationY = event.orientation.y;
//   float orientationZ = event.orientation.z;

//   // 取得したデータをCAN-BUSに送信
//   sendCANMessage(temperature, humidity, pressure, orientationX, orientationY, orientationZ);

//   delay(1000); // 1秒待機
// }


#include <Arduino.h>
#include <CAN.h>

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("CAN Sender");

  CAN.setPins(7, 6); // CANピンの設定（7: CAN RX, 6: CAN TX）
  if (!CAN.begin(500E3)) { // CAN通信の開始（500kbps）
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

int nCount = 0;

void loop() {
  Serial.print("start:");
  Serial.println(nCount);

  // パケットを送信
  CAN.beginPacket(0x12); // メッセージIDが0x12のパケットを作成
  char buf[256];
  sprintf(buf, "w:%d", nCount); // データの作成
  CAN.write((uint8_t*)buf, strlen(buf)); // データの書き込み
  CAN.endPacket();

  nCount++;

  // パケットを受信して処理
  char receivedBuf[256];
  memset(receivedBuf, '\0', sizeof(receivedBuf)); // 受信バッファの初期化
  int size = receive(receivedBuf); // 受信したデータを処理

  if (size > 0) {
    Serial.println(receivedBuf); // 受信したデータをシリアルモニターに表示
  }

  delay(10);
}

// パケットを受信して処理する関数
int receive(char *buf) {
  // パケットを受信
  int packetSize = CAN.parsePacket();

  if (packetSize || CAN.packetId() != -1) {
    // パケットを受信した場合

    // 受信したパケットの情報を表示
    if (CAN.packetExtended()) {
      Serial.println("extended ");
    }

    if (CAN.packetRtr()) {
      Serial.println("RTR ");
    }

    // 受信したパケットのIDを表示
    Serial.print("packet with id 0x");
    Serial.print(CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
      Serial.print(" and requested length ");
      Serial.println(CAN.packetDlc());
    } else {
      Serial.print(" and length ");
      Serial.println(packetSize);

      // 受信したデータを読み取ってバッファに格納
      if (CAN.available()){
        int size = 0;
        while (CAN.available()) {
          buf[size] = CAN.read();
          size++;
        }
        return size;
      }
    }
  }
  return 0;
}


#include <CAN.h>

// Most cars support 11-bit address, others (like Honda),
// require 29-bit (extended) addressing, set the next line
// to true to use extended addressing
const bool useStandardAddressing = true;

void setup() {
  Serial.begin(9600);
  Serial.println("CAN OBD-II VIN reader");
  CAN.setPins(7,6);

  // Start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  // Add filter to only receive the CAN bus ID's we care about
  if (useStandardAddressing) {
    CAN.filter(0x7e8);
  } else {
    CAN.filterExtended(0x18daf110);
  }
}


void loop() {
  // Send the request for the first chunk
  if (useStandardAddressing) {
    CAN.beginPacket(0x7df, 8);
  } else {
    CAN.beginExtendedPacket(0x18db33f1, 8);
  }
  CAN.write(0x02); // Number of additional bytes
  CAN.write(0x09); // Request vehicle information
  CAN.write(0x02); // Vehicle Identification Number (VIN)
  CAN.endPacket();

  // Wait for response
  while (CAN.parsePacket() == 0 ||
         CAN.read() != 0x10 || CAN.read() != 0x14 || // Correct length
         CAN.read() != 0x49 ||                       // Correct mode
         CAN.read() != 0x02 ||                       // Correct PID
         CAN.read() != 0x01);

  // Print out
  while (CAN.available()) {
    Serial.write((char)CAN.read());
  }

  // Read in remaining chunks
  for (int i = 0; i < 2; i++) {
    // Send the request for the next chunk
    if (useStandardAddressing) {
      CAN.beginPacket(0x7e0, 8);
    } else {
      CAN.beginExtendedPacket(0x18db33f1, 8);
    }
    CAN.write(0x30);
    CAN.endPacket();

    // Wait for response
    while (CAN.parsePacket() == 0 ||
           CAN.read() != (0x21 + i)); // Correct sequence number

    // Print out
    while (CAN.available()) {
      Serial.write((char)CAN.read());
    }
  }

  Serial.println("That's all folks!");
  
  while (1); // All done
}
