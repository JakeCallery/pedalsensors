#include "Adafruit_NeoPixel.h"
#include "Adafruit_VL53L0X.h"
#include <FlashStorage.h>

//PIXELS
#define PIXEL_DATA_PIN        13
#define NUM_BRAKE_PIXELS      8
#define BRAKE_PIXEL_START     15
#define BRAKE_PIXEL_END       (BRAKE_PIXEL_START - NUM_BRAKE_PIXELS) - 1
#define NUM_THROTTLE_PIXELS   8
#define THROTTLE_PIXEL_START  0
#define THROTTLE_PIXEL_END    THROTTLE_PIXEL_START + (NUM_THROTTLE_PIXELS - 1)
#define PIXEL_COUNT           (NUM_BRAKE_PIXELS + NUM_THROTTLE_PIXELS)

//DISTANCE_SENSORS
#define THROTTLE_LOX_XSHUT_PIN  6
#define BRAKE_LOX_XSHUT_PIN     5
#define THROTTLE_LOX_ADDRESS    0x31
#define BRAKE_LOX_ADDRESS       0x30
#define SENSOR_RANGE_MIN        20
#define SENSOR_RANGE_MAX        1200

//MOMENTARY CALIBRATION SWITCHES
#define THROT_MIN_PIN 14
#define THROT_MAX_PIN 15
#define BRAKE_MIN_PIN 19
#define BRAKE_MAX_PIN 17

//POTS
#define BRIGHTNESS_POT_PIN 16

//OTHER
#define SAVE_TO_FLASH true
#define FLASH_UPDATE_DELAY 1000
#define WAIT_FOR_SERIAL_DELAY_MS 5000
#define SKIP_LOX false
#define DEBUG_LOX false

//GLOBALS
Adafruit_VL53L0X throttleLox = Adafruit_VL53L0X();
Adafruit_VL53L0X brakeLox = Adafruit_VL53L0X();
Adafruit_NeoPixel neo_pixels(PIXEL_COUNT, PIXEL_DATA_PIN, NEO_GRB + NEO_KHZ800);

//Initial sensor bounds (invalid)
int throttleMaxDist = -1;
int throttleMinDist = -1;
int brakeMaxDist = -1;
int brakeMinDist = -1;

//Initial Sensor Disances (invalid)
int throttleDist = -1;
int brakeDist = -1;

//Flash tracking
uint lastFlashWrite = 0;
boolean needToWriteSettings = false;

typedef struct {
  boolean valid;
  int throtMin;
  int throtMax;
  int brakeMin;
  int brakeMax;
} Settings;

FlashStorage(settings_flash_store, Settings);

void setup() {
    
  /////// setup serial console ///////
  Serial.begin(115200);
  
  // wait until serial port opens for native USB devices
  int delayStartTime = millis();
  while (!Serial && (millis() - delayStartTime <= WAIT_FOR_SERIAL_DELAY_MS)) {
    delay(1);
    Serial.println("waiting on serial comm...");
  }

  Serial.println("Staring Up...");

  /////// Setup Switches/POTS ///////
  pinMode(THROT_MIN_PIN, INPUT);
  pinMode(THROT_MAX_PIN, INPUT);
  pinMode(BRAKE_MIN_PIN, INPUT);
  pinMode(BRAKE_MAX_PIN, INPUT);
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

  if(!SKIP_LOX)
  {
    Serial.println("Turning On Brake Sensor");
    digitalWrite(BRAKE_LOX_XSHUT_PIN, HIGH);
    delay(10);

    Serial.println("Setting Brake Sensor Address");
    if(!brakeLox.begin(BRAKE_LOX_ADDRESS)) {
      Serial.println("------Failed to boot Brake Sensor");
      //TODO: Some failure indicator to the user
     // while(1);
    }

    Serial.println("Turning On Throttle Sensor");
    digitalWrite(THROTTLE_LOX_XSHUT_PIN, HIGH);
    delay(10);

    Serial.println("Setting Throttle Sensor Address");
    if(!throttleLox.begin(THROTTLE_LOX_ADDRESS)) {
      Serial.println("--------Failed to boot Throttle Sensor");
      //TODO: Some failure indicator to the user
      //while(1);
    }
  }
  

  //////// GET FLASH MEM VALUES ///////////
  Settings settings;
  settings = settings_flash_store.read();

  if(settings.valid == false) {
      if(SAVE_TO_FLASH) {
        Serial.println("Flash data not valid, setting default data");
        settings.throtMin = -1;
        settings.throtMax = -1;
        settings.brakeMin = -1;
        settings.brakeMax = -1;
        settings.valid = true;
        settings_flash_store.write(settings);
      }
  } else {
      Serial.println("Found valid flash data:");
      throttleMinDist = settings.throtMin;
      throttleMaxDist = settings.throtMax;
      brakeMinDist = settings.brakeMin;
      brakeMaxDist = settings.brakeMax;
      Serial.print("Throttle Min: "); Serial.println(throttleMinDist);
      Serial.print("Throttle Max: "); Serial.println(throttleMaxDist);
      Serial.print("Brake Min: "); Serial.println(brakeMinDist);
      Serial.print("Brake Max: "); Serial.println(brakeMaxDist);
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
    neo_pixels.setPixelColor(THROTTLE_PIXEL_START + (numFullThrottleLights * throttleLightDirection), 0, partialThrottleLightVal, 0);
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
    neo_pixels.setPixelColor(BRAKE_PIXEL_START + (numFullBrakeLights * brakeLightDirection), partialBrakeLightVal, 0, 0);
  }

}

void loop() {

  //RANGE SENSOR DATA
  VL53L0X_RangingMeasurementData_t throttleMeasure;
  VL53L0X_RangingMeasurementData_t brakeMeasure;  
 
  //THROTTLE
  if(!SKIP_LOX)
  {
    throttleLox.rangingTest(&throttleMeasure, DEBUG_LOX); // pass in 'true' to get debug data printout!
    
    if(throttleMeasure.RangeStatus != 4) {
      throttleDist = throttleMeasure.RangeMilliMeter;
      if(throttleMaxDist == -1 || digitalRead(THROT_MAX_PIN)== HIGH){
        //Set Max distance
        Serial.print("Setting Throttle Max Dist: ");
        throttleMaxDist = throttleDist;
        Serial.println(throttleMaxDist);
        needToWriteSettings = true;
      }

      if(throttleMinDist == -1 || digitalRead(THROT_MIN_PIN) == HIGH) {
        //Set Min Distance
        Serial.print("Setting Throttle Min Dist: ");
        throttleMinDist = throttleDist;
        Serial.println(throttleMinDist);
        needToWriteSettings = true;
      }
   
    } else {
      //out of range
    }
  }
  

  //BRAKE
  if(!SKIP_LOX)
  {
    brakeLox.rangingTest(&brakeMeasure, DEBUG_LOX); // pass in 'true' to get debug data printout!
    if(brakeMeasure.RangeStatus != 4) {
      brakeDist = brakeMeasure.RangeMilliMeter;
      if(brakeMaxDist == -1 || digitalRead(BRAKE_MAX_PIN) == HIGH){
        //Set Max distance
        Serial.print("Setting Brake Max Dist: ");
        brakeMaxDist = brakeDist;
        Serial.println(brakeMaxDist);
        needToWriteSettings = true;
      }
    
      if(brakeMinDist == -1 || digitalRead(BRAKE_MIN_PIN) == HIGH) {
        //Set Min Distance
        Serial.print("Setting Brake Min Dist: ");
        brakeMinDist = brakeDist;
        Serial.println(brakeMinDist);
        needToWriteSettings = true;
      }
    
    } else {
        //out of range
    }    
  }


  if(needToWriteSettings == true && 
    SAVE_TO_FLASH && 
    millis() - FLASH_UPDATE_DELAY >= lastFlashWrite) 
  {
    Settings settings;
    Serial.println("Saving Settings To Flash Memory");
    settings.throtMin = throttleMinDist;
    settings.throtMax = throttleMaxDist;
    settings.brakeMin = brakeMinDist;
    settings.brakeMax = brakeMaxDist;
    settings.valid = true;
    
    settings_flash_store.write(settings);
    needToWriteSettings = false;
    lastFlashWrite = millis();
  }

  //BRIGHTNESS
  int maxBrightness = analogRead(BRIGHTNESS_POT_PIN);
  maxBrightness = 255 - map(maxBrightness, 0, 1024, 0, 255);  
  
  //PIXELS
  updatePixels(throttleDist, brakeDist, throttleMinDist, throttleMaxDist, brakeMinDist, brakeMaxDist, maxBrightness);
  neo_pixels.show();
  delay(16);
}
