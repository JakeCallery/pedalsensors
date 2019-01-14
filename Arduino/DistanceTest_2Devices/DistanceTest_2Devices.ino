#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);

  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");

  Serial.println("Resetting L0X sensors");
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  delay(10);

  Serial.println("Turning On Sensor1");
  digitalWrite(3, HIGH);
  delay(10);

  Serial.println("Setting Sensor1 Address");
  if(!lox1.begin(0x30)) {
    Serial.println("--------Failed to boot Sensor1");
    while(1);
  }

  Serial.println("Turning On Sensor2");
  digitalWrite(4, HIGH);
  delay(10);

  Serial.println("Setting Sensor2 Address");
  if(!lox2.begin(0x31)) {
    Serial.println("------Failed to boot Sensor2");
    while(1);
  }
  

  /*
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  */
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}


void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement 1... ");
  lox1.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance1 (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }

  Serial.print("Reading a measurement 2... ");
  lox2.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance1 (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
  
  delay(100);
}
