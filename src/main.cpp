#include <Arduino.h>
#include <SPI.h>              // include libraries
// #include <LoRa.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>

void displayInfo();
void transmitInfo();

unsigned long lastTransmitTime;

RH_RF95 rf95(8, 3); // Adafruit Feather M0 with RFM95 

// The TinyGPS++ object
TinyGPSPlus gps;

typedef struct Packet {
  unsigned int id;
  double lat;
  double lng;
  double gps_altitude;
};

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);

  if (!rf95.init())
    Serial.println("init failed");  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  rf95.setFrequency(915.0f);
  rf95.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096);
  // rf95.setPayloadCRC(false);
  // rf95.setSpreadingFactor(12);
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
 rf95.setTxPower(23, false);
  // If you are using Modtronix inAir4 or inAir9,or any other module which uses the
  // transmitter RFO pins and not the PA_BOOST pins
  // then you can configure the power transmitter power for -1 to 14 dBm and with useRFO true. 
  // Failure to do that will result in extremely low transmit powers.
//  driver.setTxPower(14, true);
  // LoRa.setPins(8, 4, 1);
  // if (!LoRa.begin(915E6)) {         // initialize ratio at 915 MHz
  //   while(true){
  //     delay(1000);
  //     Serial.println("LoRa init failed. Check your connections.");
  //   }
  //   // while (true);                   // if failed, do nothing
  // }
  // delay(5000);
  // // Serial.println("AAH");
  // LoRa.dumpRegisters(Serial);
  // LoRa.setSpreadingFactor(12);
  // LoRa.setSignalBandwidth(500E3);
  // LoRa.dumpRegisters(Serial);

  lastTransmitTime = millis();
  // // Serial.println(F("DeviceExample.ino"));
  // // Serial.println(F("A simple demonstration of TinyGPS++ with an attached GPS module"));
  // // Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  // // Serial.println(F("by Mikal Hart"));
  // // Serial.println();
}

void loop()
{
  static bool hasData = false;
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial1.available() > 0){
    char c = Serial1.read();
    // Serial.write(c);
    hasData = gps.encode(c);
  }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    // while(true);
  }
  if(lastTransmitTime+3000 < millis()){
    transmitInfo();
    displayInfo();
    lastTransmitTime = millis();
  }
}

void transmitInfo(){
  static int id = 0;
  if(rf95.mode() != RH_RF95_MODE_TX){
    Packet packet;
    packet.id = id++;
    packet.lat = gps.location.lat();
    packet.lng = gps.location.lng();
    packet.gps_altitude = gps.altitude.feet();
    Serial.println(sizeof(packet));
    rf95.send((uint8_t*)&packet, sizeof(packet));
  }else{
    Serial.println("Packet not sent");
  }
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 9);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 9);
    Serial.print(F(","));
    Serial.print(gps.altitude.feet());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}