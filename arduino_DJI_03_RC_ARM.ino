/************************************************************************************
  @file       arduino_DJI_03_RC_ARM.ino
  @brief      Send DJI Arm using XIAO Seeeduino over MSP to keep power level at full.
  @author     Richard Amiss
  
  Release Note:  
  Complete rework for XIAO Seeeduino and MSP libraries. RC input is no longer used in 
  this version.  The AU will simply arm a few seconds after being turned on, as long 
  as the goggles are on.
  
  Code:        Richard Amiss
  Version:     1.1.0
  Date:        06/24/23

  Credits:
  This software is based on and uses software published by 2022 David Payne:
  QLiteOSD, which is based on work by Paul Kurucz (pkuruz):opentelem_to_bst_bridge 
  as well as software from d3ngit : djihdfpv_mavlink_to_msp_V2 and 
  crashsalot : VOT_to_DJIFPV

THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES, WHETHER EXPRESS, 
IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE 
COMPANY SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR 
CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
************************************************************************************/

#include "MSP.h"
#include "MSP_OSD.h"

#define DEBUG 1

HardwareSerial &mspSerial = Serial1;
MSP msp;

// Arm Logic
uint32_t unarmedMillis = 3000;   // number of milliseconds to wait before arming, after AU is initialized. Recommend at least 3000 (3 seconds).


//Other
char fcVariant[5] = "BTFL";
char craftname[15] = "goat";
uint32_t previousMillis_MSP = 0;
uint32_t activityDetectedMillis_MSP = 0;
bool activityDetected = false;
const uint32_t next_interval_MSP = 100;
uint32_t flightModeFlags = 0x00000002;

msp_name_t name = { 0 };
msp_fc_version_t fc_version = { 0 };
msp_fc_variant_t fc_variant = { 0 };
msp_status_DJI_t status_DJI = { 0 };

void setup() {

  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial);
  #endif

  int8_t rxPin = D3;
  int8_t txPin = D4;
  mspSerial.begin(115200, SERIAL_8N1, rxPin, txPin);
  while (!mspSerial);

  msp.begin(mspSerial);

  delay(1000);

  status_DJI.cycleTime = 0x0080;
  status_DJI.i2cErrorCounter = 0;
  status_DJI.sensor = 0x23;
  status_DJI.configProfileIndex = 0;
  status_DJI.averageSystemLoadPercent = 7;
  status_DJI.accCalibrationAxisFlags = 0;
  status_DJI.DJI_ARMING_DISABLE_FLAGS_COUNT = 20;
  status_DJI.djiPackArmingDisabledFlags = (1 << 24);
  flightModeFlags = 0x00000002;

  #ifdef DEBUG
    Serial.println("Initialized");
  #endif
}

void loop() { 
   
  if (!activityDetected) {
    #ifdef DEBUG
      Serial.println("Waiting for AU...");
    #endif

    // Wait for Air Unit to send data
    while(!msp.activityDetected());
    activityDetected = true;
    activityDetectedMillis_MSP = millis();   

    #ifdef DEBUG
      Serial.println("AU Detected, waiting (unarmedMillis) time till arm");
    #endif
  }
  
  uint32_t currentMillis_MSP = millis();

  if ((uint32_t)(currentMillis_MSP - previousMillis_MSP) >= next_interval_MSP) {
    previousMillis_MSP = currentMillis_MSP;

    if (currentMillis_MSP < (activityDetectedMillis_MSP + unarmedMillis)) {
      set_flight_mode_flags(false);
    } else {
      set_flight_mode_flags(true);
    }

    #ifdef DEBUG
      debugPrint();
    #endif

    send_msp_to_airunit();
  }

}


void set_flight_mode_flags(bool arm) {
    if ((flightModeFlags == 0x00000002) && arm) {
      flightModeFlags = 0x00000003;    // arm
      #ifdef DEBUG
        Serial.println("ARMING");
      #endif
    } else if ((flightModeFlags == 0x00000003) && !arm) {        
      flightModeFlags = 0x00000002;    // disarm 
      #ifdef DEBUG
        Serial.println("DISARMING");
      #endif
    }
}

void send_msp_to_airunit() {
  
  //MSP_FC_VARIANT
  memcpy(fc_variant.flightControlIdentifier, fcVariant, sizeof(fcVariant));
  msp.send(MSP_FC_VARIANT, &fc_variant, sizeof(fc_variant));

  //MSP_FC_VERSION
  fc_version.versionMajor = 4;
  fc_version.versionMinor = 5;
  fc_version.versionPatchLevel = 1;
  msp.send(MSP_FC_VERSION, &fc_version, sizeof(fc_version));

  //MSP_NAME
  memcpy(name.craft_name, craftname, sizeof(craftname));
  msp.send(MSP_NAME, &name, sizeof(name));

  //MSP_STATUS
  status_DJI.flightModeFlags = flightModeFlags;
  status_DJI.armingFlags = 0x0303;
  msp.send(MSP_STATUS_EX, &status_DJI, sizeof(status_DJI));
  status_DJI.armingFlags = 0x0000;
  msp.send(MSP_STATUS, &status_DJI, sizeof(status_DJI));

}


//*** USED ONLY FOR DEBUG ***
void debugPrint() {
  Serial.println("**********************************");
  Serial.print("Flight Mode: ");
  Serial.println(flightModeFlags);
}



