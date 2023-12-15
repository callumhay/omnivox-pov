#include <algorithm>

#include <OctoWS2811.h>


// Comm/Serial constants
#define RX_PIN 0
#define TX_PIN 1
#define CTS_PIN 18
#define RTS_PIN 17
#define TRANSMIT_ENABLE_PIN 22
#define USB_SERIAL_BAUD 9600
#define HW_SERIAL_BAUD 3000000
#define DEBUG_SERIAL Serial
#define DATA_SERIAL Serial1

// Peripheral constants
#define HALL_SENSOR_ANALOG_PIN 23


// TODO
/*
// OCTOWS2811 Constants/Variables *******************************************************
const int octoConfig = WS2811_800kHz; // All other settings are done on the server/computer that feeds the data

const int voxelCubeSize = DEFAULT_VOXEL_CUBE_SIZE;
const int ledsPerModule = NUM_OCTO_PINS * voxelCubeSize * voxelCubeSize;
const int ledsPerStrip  = voxelCubeSize * voxelCubeSize;
const int memBuffLen = ledsPerStrip*6;

DMAMEM int displayMemory[memBuffLen] = {0};
int drawingMemory[memBuffLen] = {0};

OctoWS2811 leds(ledsPerStrip, displayMemory, drawingMemory, octoConfig);
// **************************************************************************************
*/

// Hall sensor monitoring variables
#define HALL_SENSOR_THRESHOLD 900
#define HALL_SENSOR_DEBOUNCE_TIME_MICROS 2000
#define INVALID_HALL_SENSOR_VALUE -1
elapsedMicros hsSinceAboveThreshold;
elapsedMicros hsSinceMaxValue;
elapsedMicros hsSincePeak;
elapsedMicros hsDebounceTimeElapsed;
unsigned long hsLastPeakTime = 0;

int hsCurrMaxValue = INVALID_HALL_SENSOR_VALUE;

// POV variables
#define MAX_ROTATION_TIME_MICROS 1000000 // 1 second
unsigned long totalRotationTime = 0; // Total time (microsecs) for one rotation


void setup() {
  DEBUG_SERIAL.begin(USB_SERIAL_BAUD);
  pinMode(HALL_SENSOR_ANALOG_PIN, INPUT);
}

void loop() {
  // Read the hall sensor value
  int hallSensorValue = analogRead(HALL_SENSOR_ANALOG_PIN);
  
  // Use a debounce time for both the above threshold and below threshold to avoid
  // significant fluctuations in our estimation of the time between peaks
  if (hsDebounceTimeElapsed >= HALL_SENSOR_DEBOUNCE_TIME_MICROS) {

    if (hallSensorValue > HALL_SENSOR_THRESHOLD) {
      if (hallSensorValue > hsCurrMaxValue) {
          if (hsCurrMaxValue == INVALID_HALL_SENSOR_VALUE) {
            hsSinceAboveThreshold = elapsedMicros();
          }
          hsCurrMaxValue = hallSensorValue;
          hsSinceMaxValue = elapsedMicros();
      }
    }
    else {
      // Check whether the hall sensor was above the threshold previously
      if (hsCurrMaxValue != INVALID_HALL_SENSOR_VALUE) {
        
        // Before we start estimating the total rotation time we need to make sure
        // that we've rotated at least once!
        if (hsLastPeakTime != 0) {
          if (totalRotationTime == 0) {
            // If this is the first time we're calculating the rotation time then
            // we can use the time since the last peak as the total rotation time
            totalRotationTime = static_cast<unsigned long>(hsSincePeak);
          }
          else {
            // Otherwise we use a weighted average of the time since the last peak
            // and the total rotation time to estimate the total rotation time
            totalRotationTime = static_cast<unsigned long>(
              0.9 * static_cast<double>(hsSincePeak) + 
              0.1 * static_cast<double>(totalRotationTime)
            );
          }

          // If the total rotation time is too long then we can't use it, reinitialize it:
          // Conservatively, we should be rotating at least once per second.
          if (totalRotationTime > MAX_ROTATION_TIME_MICROS) {
            totalRotationTime = 0;
          }
        }

        // Calculate the total time (microsecs) the hall sensor was above the threshold
        auto hsAboveThresholdTime = static_cast<unsigned long>(hsSinceAboveThreshold);
        // Calculate the time (microsecs) since the hall sensor was at its max value
        auto hsMaxValueTime = static_cast<unsigned long>(hsSinceMaxValue);

        // Use a weighted average of half the time the sensor was above the threshold and
        // the time since the sensor was at its max value to estimate the time since the
        // sensor was at its peak value
        hsLastPeakTime = static_cast<unsigned long>(
          0.25 * static_cast<double>(hsAboveThresholdTime / 2) + 
          0.75 * static_cast<double>(hsMaxValueTime)
        );

        //DEBUG_SERIAL.print("Hall sensor last peak: ");
        //DEBUG_SERIAL.println(hsSincePeak);

        // We can figure out the time that the hall sensor was at its peak value 
        // by taking the midpoint of the time range that it was at its max value
        hsSincePeak = elapsedMicros(hsLastPeakTime);
        DEBUG_SERIAL.print("Hall sensor above threshold for ");
        DEBUG_SERIAL.print(hsAboveThresholdTime);
        DEBUG_SERIAL.println(" microseconds");
        DEBUG_SERIAL.print("Hall sensor time since max value: ");
        DEBUG_SERIAL.print(hsMaxValueTime);
        DEBUG_SERIAL.println(" microseconds");

        hsDebounceTimeElapsed = elapsedMicros();
      }

      hsCurrMaxValue = INVALID_HALL_SENSOR_VALUE;
    }
  }

  // If there's a rotation time established then we can start rendering based on 
  // the time since the last peak and the expected rotation time
  bool isRotationDataValid = false;
  if (hsLastPeakTime != 0) {
    auto timeSinceLastPeak = static_cast<unsigned long>(hsSincePeak);
    if (totalRotationTime > 0 && 
        totalRotationTime <= MAX_ROTATION_TIME_MICROS && 
        timeSinceLastPeak <= MAX_ROTATION_TIME_MICROS) {

      DEBUG_SERIAL.print("Current total rotation time (us): ");
      DEBUG_SERIAL.println(totalRotationTime);
      // TODO

      isRotationDataValid = true;
    }
  }

  // Clear the LEDs if we don't have proper rotation time data
  if (!isRotationDataValid) {
    // TODO
  }

}