#include "Adafruit_NeoPixel.h"
#include "Adafruit_VL53L0X.h"

//PIXELS
#define PIXEL_DATA_PIN        13
#define NUM_BRAKE_PIXELS      8
#define BRAKE_PIXEL_START     15
#define BRAKE_PIXEL_END       (BRAKE_PIXEL_START - NUM_BRAKE_PIXELS) - 1
#define NUM_THROTTLE_PIXELS   8
#define THROTTLE_PIXEL_START  0
#define THROTTLE_PIXEL_END    THROTTLE_PIXEL_START + (NUM_THROTTLE_PIXELS - 1)
#define PIXEL_COUNT           (NUM_BRAKE_PIXELS + NUM_THROTTLE_PIXELS)

Adafruit_NeoPixel neo_pixels(PIXEL_COUNT, PIXEL_DATA_PIN, NEO_GRB + NEO_KHZ800);

//DISTANCE_SENSORS
#define THROTTLE_LOX_XSHUT_PIN  6
#define BRAKE_LOX_XSHUT_PIN     5
#define THROTTLE_LOX_ADDRESS    0x31
#define BRAKE_LOX_ADDRESS       0x30
#define SENSOR_RANGE_MIN        20
#define SENSOR_RANGE_MAX        1200

Adafruit_VL53L0X throttleLox = Adafruit_VL53L0X();
Adafruit_VL53L0X brakeLox = Adafruit_VL53L0X();

//MOMENTARY CALIBRATION SWITCHES
#define THROT_MIN_PIN 14
#define THROT_MAX_PIN 15
#define BRAKE_MIN_PIN 17
#define BRAKE_MAX_PIN 19

//POTS
#define BRIGHTNESS_POT_PIN 16

int throttleMaxDist = -1;
int throttleMinDist = -1;
int brakeMaxDist = -1;
int brakeMinDist = -1;

void setup() {
    
  /////// setup serial console ///////
  // wait until serial port opens for native USB devices
  Serial.begin(9600);
  /*
  while (! Serial) {
    delay(1);
  }
  */

  /////// Setup Switches/POTS ///////
  pinMode(THROT_MIN_PIN, INPUT);
  pinMode(THROT_MAX_PIN, INPUT);
  pinMode(BRAKE_MIN_PIN, INPUT);
  pinMode(BRAKE_MIN_PIN, INPUT);
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

void updatePixels(int throtVal, int brakeVal, int throtMin, int throtMax, int brakeMin, int brakeMax, int maxBrightness) {
  int throtPercent; 
  int brakePercent; 

  clearPixels(0, PIXEL_COUNT);

  if(throtVal != -1 && throtMin != -1 && throtMax != -1) {
    throtPercent = 100 - map(constrain(throtVal,throtMin,throtMax), throtMin, throtMax, 0, 100);  
  } else {
    throtPercent = 0;
  }


  if(brakeVal != -1 && brakeMin != -1 && throtMax != -1) {
    brakePercent = 100 - map(constrain(brakeVal, brakeMin, brakeMax), brakeMin, brakeMax, 0, 100);
  } else {
    brakePercent = 0;
  }

  //THROTTLE
  float wholeThrottleLight = 100 / NUM_THROTTLE_PIXELS;
  int numFullThrottleLights = floor((float)throtPercent / wholeThrottleLight);
  double decimal = ((double)throtPercent / wholeThrottleLight) - (double)floor((float)throtPercent / wholeThrottleLight);
  int partialThrottleLightVal = round(decimal * maxBrightness);
  int throttleLightDirection = (THROTTLE_PIXEL_START <= THROTTLE_PIXEL_END) ? 1:-1;
  for(int i = THROTTLE_PIXEL_START, lightCount = 0; i != THROTTLE_PIXEL_END, lightCount < numFullThrottleLights; i += throttleLightDirection, lightCount++) {
      //Light up full lights
      neo_pixels.setPixelColor(i, 0, maxBrightness, 0);
  }

  //Set partial throttle light
  if(partialThrottleLightVal > 0 && numFullThrottleLights < NUM_THROTTLE_PIXELS) {
    neo_pixels.setPixelColor(THROTTLE_PIXEL_START + numFullThrottleLights, 0, partialThrottleLightVal, 0);
  }
  
  double wholeBrakeLight = (double)100 / (double)NUM_BRAKE_PIXELS;
  int numFullBrakeLights = floor((double)brakePercent / (double)wholeBrakeLight);
  decimal = ((double)brakePercent / wholeBrakeLight) - (double)floor((double)brakePercent / (double)wholeBrakeLight);
  int partialBrakeLightVal = round(decimal * maxBrightness);
  int brakeLightDirection = (BRAKE_PIXEL_START <= BRAKE_PIXEL_END) ? 1:-1;
  for(int i = BRAKE_PIXEL_START, lightCount = 0; i != BRAKE_PIXEL_END, lightCount < numFullBrakeLights; i += brakeLightDirection, lightCount++) {
      //Light up full lights
      neo_pixels.setPixelColor(i, maxBrightness, 0, 0);
  }

  //Set partial brake light
  if(partialBrakeLightVal > 0 && numFullBrakeLights < NUM_BRAKE_PIXELS) {
    neo_pixels.setPixelColor(BRAKE_PIXEL_START + numFullBrakeLights, partialBrakeLightVal, 0, 0);
  }

}

void loop() {

  //RANGE SENSORS
  VL53L0X_RangingMeasurementData_t throttleMeasure;
  VL53L0X_RangingMeasurementData_t brakeMeasure;
 
  int throttleDist = -1;
  int brakeDist = -1;
  
  throttleLox.rangingTest(&throttleMeasure, false); // pass in 'true' to get debug data printout!
  if(throttleMeasure.RangeStatus != 4) {
    throttleDist = throttleMeasure.RangeMilliMeter;
    if(throttleMaxDist == -1 || digitalRead(THROT_MAX_PIN)== HIGH){
      //Set Max distance
      Serial.print("Setting Throttle Max Dist: ");
      throttleMaxDist = throttleDist;
      Serial.println(throttleMaxDist);
    }

    if(throttleMinDist == -1 || digitalRead(THROT_MIN_PIN) == HIGH) {
      //Set Min Distance
      Serial.print("Setting Throttle Min Dist: ");
      throttleMinDist = throttleDist;
      Serial.println(throttleMinDist);
    }
   
  } else {
    //out of range
  }
  
  brakeLox.rangingTest(&brakeMeasure, false); // pass in 'true' to get debug data printout!
  if(brakeMeasure.RangeStatus != 4) {
    brakeDist = brakeMeasure.RangeMilliMeter;
    //Serial.println(digitalRead(BRAKE_MAX_PIN));
    //Serial.println(brakeMaxDist);
    if(brakeMaxDist == -1 || digitalRead(BRAKE_MAX_PIN) == HIGH){
    //Set Max distance
    Serial.print("Setting Brake Max Dist: ");
    brakeMaxDist = brakeDist;
    Serial.println(brakeMaxDist);
  }

  if(brakeMinDist == -1 || digitalRead(BRAKE_MIN_PIN) == HIGH) {
    //Set Min Distance
    Serial.print("Setting Brake Min Dist: ");
    brakeMinDist = brakeDist;
    Serial.println(brakeMinDist);
  }

  } else {
    //out of range
  }

  //BRIGHTNESS
  int maxBrightness = analogRead(BRIGHTNESS_POT_PIN);
  // Serial.print(maxBrightness);
  // Serial.print(" / ");
  maxBrightness = map(maxBrightness, 0, 1024, 0, 255);
  // Serial.print(maxBrightness);
  // Serial.println("");
  //PIXELS
  updatePixels(throttleDist, brakeDist, throttleMinDist, throttleMaxDist, brakeMinDist, brakeMaxDist, maxBrightness);
  neo_pixels.show();
  delay(16);
}
