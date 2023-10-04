#include <DHT.h>

#define DHTPIN 2          // กำหนดขาที่เชื่อมต่อเซนเซอร์ DHT11
#define DHTTYPE DHT22     // รุ่นของเซนเซอร์

DHT dht(DHTPIN, DHTTYPE); // สร้างอ็อบเจ็กต์ DHT

const int fanPin = 3;     // กำหนดขาที่เชื่อมต่อกับพัดลม

void setup() {
  Serial.begin(9600);
  dht.begin();
  pinMode(fanPin, OUTPUT);
}

void loop() {
  float temperature = dht.readTemperature(); // อ่านค่าอุณหภูมิจากเซนเซอร์

  Serial.print("Temperature: ");
  Serial.println(temperature);

  // ปรับความเร็วของพัดลมตามอุณหภูมิ
  if (temperature > 25) {
    analogWrite(fanPin, 255); // พัดลมที่ความเร็วเต็ม
  } else {
    analogWrite(fanPin, 0);   // ไม่พัดลม
  }

  delay(2000); // หน่วงเวลาสักพัก
}