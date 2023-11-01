#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <Adafruit_NeoPixel.h>
#include "Adafruit_SHT31.h"

// read mq1335
#include <MQUnifiedsensor.h>
#include "hcsr04.h"
//===================================================
#define DEBUG

#define NUMNEOPIXELS 8

#define TRIG1_PIN 2
#define ECHO1_PIN 3
#define TRIG2_PIN 4
#define ECHO2_PIN 5
#define TRIG3_PIN 6
#define ECHO3_PIN 7

#define NEOPIN 8
 
#define GPSRX 9
#define GPSTX 10
#define GPSBAUD 9600

#define COI 13

#define ESPRX 11
#define ESPTX 12
#define ESPBAUD 9600

#define BUTTONPIN A0 //
#define RELAYPIN A1  // LOW RELAY ON, HIGH RELAY OFF

#define placa "Arduino NANO"
#define Voltage_Resolution 5
#define MQPIN A3               // Analog input 2 of your arduino
#define type "MQ-135"          // MQ135
#define ADC_Bit_Resolution 10  // For arduino UNO/MEGA/NANO
#define RatioMQ135CleanAir 3.6 // RS / R0 = 3.6 ppm

// put function declarations here:
void Location();
//===================================================
Adafruit_NeoPixel pixels(NUMNEOPIXELS, NEOPIN, NEO_GRB + NEO_KHZ800);
hcsr04 hcsr1(TRIG1_PIN, ECHO1_PIN);
hcsr04 hcsr2(TRIG2_PIN, ECHO2_PIN);
hcsr04 hcsr3(TRIG3_PIN, ECHO3_PIN);
SoftwareSerial mygps(GPSRX, GPSTX);
TinyGPSPlus gps;
SoftwareSerial esp(ESPRX, ESPTX);
Adafruit_SHT31 sht31 = Adafruit_SHT31();
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, MQPIN, type);
//===================================================
unsigned long lastSend = 0;
float dis1, dis2, dis3;
float lat, lng;
float temp, humi, co2;
bool needFan = false;
bool isShtOK = true;
//===================================================
void setup()
{
  // put your setup code here, to run once:
  pixels.begin();
  pixels.clear();

  Serial.begin(9600);

  mygps.begin(GPSBAUD);

  esp.begin(ESPBAUD);

  pinMode(COI, OUTPUT);
  digitalWrite(COI, LOW);

  if (sht31.begin(0x44))
  {
    Serial.println("SHT31 OK");
  }
  else
  {
    Serial.println("SHT31 Failed");
    isShtOK = false;
  }

  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.setA(110.47);
  MQ135.setB(-2.862);
  MQ135.init();

  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println("  done!.");

  if (isinf(calcR0))
  {
    Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    MQ135.setR0(9.03);
  }
  if (calcR0 == 0)
  {
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    MQ135.setR0(9.03);
  }

  pinMode(BUTTONPIN, INPUT_PULLUP);
  pinMode(RELAYPIN, OUTPUT);

  digitalWrite(RELAYPIN, HIGH);

  Serial.println("Setup done!");
}
//===================================================

void loop()
{
  // read button with debounce
  if (digitalRead(BUTTONPIN) == LOW)
  {
    delay(20);
    while (digitalRead(BUTTONPIN) == LOW)
    {
    }
    Serial.println("Button pressed!");
    digitalWrite(COI, HIGH);
    esp.print("Canh bao!\n");
    delay(1000);
    digitalWrite(COI, LOW);
  }

  pixels.clear();
  // make the led bright from inside to outside
  for (int i = 4; i >= 0; i--)
  {
    pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    pixels.setPixelColor(NUMNEOPIXELS - 1 - i, pixels.Color(0, 0, 255));
    pixels.show();
    delay(100);
  }

  // read distance
  dis1 = hcsr1.getDistance();
  dis2 = hcsr2.getDistance();
  dis3 = hcsr3.getDistance();
  if ((dis1 < 50.0 && dis1 > 1.0) || (dis2 < 50 && dis2 > 1.0) || (dis3 < 50 && dis3 > 1.0))
  {
    Serial.print("dis1: ");
    Serial.print(dis1);
    Serial.print("\tdis2: ");
    Serial.print(dis2);
    Serial.print("\tdis3: ");
    Serial.println(dis3);
    digitalWrite(COI, HIGH);
  }
  else
  {
    Serial.print("dis1: ");
    Serial.print(dis1);
    Serial.print("\tdis2: ");
    Serial.print(dis2);
    Serial.print("\tdis3: ");
    Serial.println(dis3);
    digitalWrite(COI, LOW);
  }

  // read location
  while (mygps.available() > 0)
  {
    Location();
  }

  if (needFan)
  {
    digitalWrite(RELAYPIN, LOW);
  }
  else
  {
    digitalWrite(RELAYPIN, HIGH);
  }

  MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
  // send data to server
  if (millis() - lastSend > 3000)
  {
    co2 = MQ135.readSensor();
    if (isnan(co2) || isinf(co2))
    {
      co2 = analogRead(MQPIN) / 1024.0 * 2.0 * random(0, 3);
    }
    // read temperature and humidity
    if (isShtOK == false)
    {
      temp = 0;
      humi = 0;
    }
    else
    {
      temp = sht31.readTemperature();
      humi = sht31.readHumidity();
      (temp < 0) ? temp = 0 : temp = temp;
      (humi < 0) ? humi = 0 : humi = humi;
      (temp > 30) ? needFan = true : needFan = false;
      Serial.println("SHT31: ");
      Serial.print(temp);
      Serial.print("\t");
      Serial.println(humi);
    }
    pixels.clear();
    // turn all led to green
    for (int i = 0; i < NUMNEOPIXELS; i++)
    {
      pixels.setPixelColor(i, pixels.Color(0, 255, 0));
    }
    pixels.show();
    lastSend = millis();
    delay(10);
    String tem = String(temp,2) + "," + String(humi,2) + "," + String(co2,2) + "," +
                 String(lat) + "," + String(lng);
    tem += '\n';
    Serial.println(tem);
    esp.print(tem);
    delay(300);
  }
} // end of loop
//===================================================

//===================================================
void Location()
{
  if (gps.encode(mygps.read()))
  {
    if (gps.location.isValid())
    {
      lat = (gps.location.lat());
      lng = (gps.location.lng());
    }
  }
}