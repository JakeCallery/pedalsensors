#include "Adafruit_NeoPixel.h"
#include "Adafruit_VL53L0X.h"


//PIXELS
#define PIXEL_DATA_PIN    13
#define PIXEL_COUNT       16
#define BRAKE_PIXELS      8
#define BRAKE_PIXEL_START 15
#define BRAKE_PIXEL_END   (BRAKE_PIXEL_START - BRAKE_PIXELS) + 1
#define THROTTLE_PIXELS   8
#define THROTTLE_PIXEL_START 0
#define THROTTLE_PIXEL_END THROTTLE_PIXEL_START + (THROTTLE_PIXELS - 1)
Adafruit_NeoPixel neo_pixels(PIXEL_COUNT, PIXEL_DATA_PIN, NEO_GRB + NEO_KHZ800);

//DISTANCE_SENSORS
#define THROTTLE_LOX_XSHUT_PIN 6
#define BRAKE_LOX_XSHUT_PIN 5
#define THROTTLE_LOX_ADDRESS 0x31
#define BRAKE_LOX_ADDRESS 0x30
#define SENSOR_RANGE_MIN 20
#define SENSOR_RANGE_MAX 1200
Adafruit_VL53L0X throttleLox = Adafruit_VL53L0X();
Adafruit_VL53L0X brakeLox = Adafruit_VL53L0X();

//POTS
#define THROT_MIN_POT_PIN 16
#define THROT_MAX_POT_PIN 15
#define BRAKE_MIN_POT_PIN 17
#define BRAKE_MAX_POT_PIN 18
#define BRIGHTNESS_POT_PIN 19
#define POT_RANGE_MIN 20
#define POT_RANGE_MAX 1024

void setup() {
  
  /////// setup serial console ///////
  // wait until serial port opens for native USB devices
  Serial.begin(115200);
  /*
  while (! Serial) {
    delay(1);
  }
  */

  /////// Setup POTS ///////
  pinMode(THROT_MIN_POT_PIN, INPUT);
  pinMode(THROT_MAX_POT_PIN, INPUT);
  pinMode(BRAKE_MIN_POT_PIN, INPUT);
  pinMode(BRAKE_MIN_POT_PIN, INPUT);
  pinMode(BRIGHTNESS_POT_PIN, INPUT);

  /////// Setup Pixels /////////
  pinMode(PIXEL_DATA_PIN, OUTPUT);
  
  //Clear Pixels
  neo_pixels.begin();
  clearPixels(0, PIXEL_COUNT);
  neo_pixels.show();


  /////// Setup Distance Sensors ///////
  pinMode(THROTTLE_LOX_XSHUT_PIN, OUTPUT);
  pinMode(BRAKE_LOX_XSHUT_PIN, OUTPUT);
  Serial.println("Resetting L0X sensors");
  digitalWrite(THROTTLE_LOX_XSHUT_PIN, LOW);
  digitalWrite(BRAKE_LOX_XSHUT_PIN, LOW);
  delay(10);

  Serial.println("Turning On Brake Sensor");
  digitalWrite(BRAKE_LOX_XSHUT_PIN, HIGH);
  delay(10);

  Serial.println("Setting Brake Sensor Address");
  if(!brakeLox.begin(BRAKE_LOX_ADDRESS)) {
    Serial.println("------Failed to boot Brake Sensor");
    //TODO: Some failure indicator to the user
    while(1);
  }

  Serial.println("Turning On Throttle Sensor");
  digitalWrite(THROTTLE_LOX_XSHUT_PIN, HIGH);
  delay(10);

  Serial.println("Setting Throttle Sensor Address");
  if(!throttleLox.begin(THROTTLE_LOX_ADDRESS)) {
    Serial.println("--------Failed to boot Throttle Sensor");
    //TODO: Some failure indicator to the user
    while(1);
  }
  
}

void clearPixels(int startPixel, int endPixel) {
  int temp;

  if(endPixel < startPixel) {
    temp = endPixel;
    endPixel = startPixel;
    startPixel = temp;
  }
  
  for(int i = startPixel; i < endPixel; i++){
    neo_pixels.setPixelColor(i, 0, 0, 0);
  }
}

void updatePixels(int throtVal, int brakeVal, int throtMin, int throtMax, int brakeMin, int brakeMax) {
  int throtPercent; 
  int brakePercent; 

  if(throtVal != -1) {
    //throtPercent = 100 - map((throtMax - throtVal), throtMin, throtMax, 0, 100);  
    //throtPercent = 100 - map(throtVal, throtMin, throtMax, 0, 100);  
    throtPercent = 100 - round(100.0 * ((float)throtVal / (float)(throtMax - throtMin)));  //103 / 127 / 99 / -313
  } else {
    throtPercent = 0;
  }


  if(brakeVal != -1) {
    brakePercent = 100 - map(constrain(brakeVal, brakeMin, brakeMax), brakeMin, brakeMax, 0, 100);
  } else {
    brakePercent = 0;
  }
/*  
  Serial.print(throtMin);
  Serial.print(" / ");
  Serial.print(throtMax);
  Serial.print(" / ");
  Serial.print(throtVal);
  Serial.print(" / ");
  Serial.println(throtPercent);
*/
}

void loop() {

  //POTS
  int throttleRangeMin = SENSOR_RANGE_MIN;
  int throttleRangeMax = SENSOR_RANGE_MAX;
  int throttlePotMin = POT_RANGE_MIN;
  int throttlePotMax = POT_RANGE_MAX;

  //Distance to unpress pedal
  int throttleMaxPotRaw = constrain(analogRead(THROT_MAX_POT_PIN), POT_RANGE_MIN, POT_RANGE_MAX);
  int throttleMaxOffset = map(throttleMaxPotRaw, throttlePotMin, throttlePotMax, throttleRangeMin, throttleRangeMax);
  //Serial.print(throttleMaxOffset);
  int throttleMax = throttleRangeMax - throttleMaxOffset;

  //Distance to pressed pedal
  int throttleMinPotRaw = constrain(analogRead(THROT_MIN_POT_PIN), POT_RANGE_MIN, POT_RANGE_MAX);
  int throttleMinOffset = map(throttleMinPotRaw, throttlePotMin, throttlePotMax, throttleRangeMin, throttleRangeMax);
  Serial.print(throttleMinOffset);
  int throttleMin = throttleRangeMax - throttleMinOffset;

  
  
  int brakeRangeMin = SENSOR_RANGE_MIN;
  int brakeRangeMax = SENSOR_RANGE_MAX;
  int brakePotMin = POT_RANGE_MIN;
  int brakePotMax = POT_RANGE_MAX;

  int brakeMinPotRaw = constrain(analogRead(BRAKE_MIN_POT_PIN), POT_RANGE_MIN, POT_RANGE_MAX);
  int brakeMinOffset = map(brakeMinPotRaw, brakePotMin, brakePotMax, brakeRangeMin, brakeRangeMax);
  int brakeMin = brakeRangeMax - brakeMinOffset;

  int brakeMaxPotRaw = constrain(analogRead(BRAKE_MAX_POT_PIN), POT_RANGE_MIN, POT_RANGE_MAX);
  int brakeMaxOffset = map(brakeMaxPotRaw, brakePotMin, brakePotMax, brakeRangeMin, brakeRangeMax);
  int brakeMax = brakeRangeMax - brakeMaxOffset;
  
/*
  Serial.print(throttleMin);
  Serial.print(" / ");
  Serial.print(throttleMax);
  Serial.print(" / ");
  Serial.print(brakeMin);
  Serial.print(" / ");
  Serial.print(brakeMax);
  Serial.println("");
*/

  
  //RANGE SENSORS
  VL53L0X_RangingMeasurementData_t measure;

  int throttleDist = -1;
  int brakeDist = -1;
  
  throttleLox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if(measure.RangeStatus != 4) {
    throttleDist = constrain(measure.RangeMilliMeter, SENSOR_RANGE_MIN, SENSOR_RANGE_MAX);
    Serial.print(" / ");
    Serial.println(throttleDist);
    throttleDist = map(throttleDist, SENSOR_RANGE_MIN, SENSOR_RANGE_MAX, throttleMin, throttleMax);
  } else {
    //out of range
  }
  
  brakeLox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if(measure.RangeStatus != 4) {
    brakeDist = constrain(measure.RangeMilliMeter, SENSOR_RANGE_MIN, SENSOR_RANGE_MAX);
  } else {
    //out of range
  }

/*
  Serial.print("Distances: ");
  Serial.print(throttleDist);
  Serial.print(" / ");
  Serial.println(brakeDist);
*/








  //PIXELS
  updatePixels(throttleDist, brakeDist, throttleMin, throttleMax, brakeMin, brakeMax);
  
  //TEST FULL PIXELS
  for(int i = THROTTLE_PIXEL_START; i < THROTTLE_PIXELS; i++){
    neo_pixels.setPixelColor(i, 0, 5, 0);
  }

  for(int i = BRAKE_PIXEL_START; i >= BRAKE_PIXEL_END; i--){
    neo_pixels.setPixelColor(i, 5, 0, 0);
  }

  //clearPixels(0,PIXEL_COUNT);
  neo_pixels.show();
  delay(100);
}
