#include <OctoWS2811.h>
#include <algorithm>

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
elapsedMicros hsSincePeak;
elapsedMicros hsDebounceTimeElapsed;
int hsCurrMaxValue = INVALID_HALL_SENSOR_VALUE;

// TODO: Have a exponential moving average of the time between peaks,
// which is used to determine the speed of the motor
//unsigned long avgRotationTimeMicros = 0;


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
          hsCurrMaxValue = hallSensorValue;
          hsSinceAboveThreshold = elapsedMicros();
        }
    }
    else {
      // Check whether the hall sensor was above the threshold previously
      if (hsCurrMaxValue != INVALID_HALL_SENSOR_VALUE) {
        // Calculate the time the hall sensor was above the threshold
        auto hsAboveThresholdTime = static_cast<unsigned long>(hsSinceAboveThreshold);

        DEBUG_SERIAL.print("Hall sensor last peak: ");
        DEBUG_SERIAL.println(hsSincePeak);

        // We can figure out the time that the hall sensor was at its peak value 
        // by taking the midpoint of the time range that it was at its max value
        hsSincePeak = elapsedMicros(hsAboveThresholdTime / 2);
        DEBUG_SERIAL.print("Hall sensor above threshold for ");
        DEBUG_SERIAL.print(hsAboveThresholdTime);
        DEBUG_SERIAL.println(" microseconds");

        hsDebounceTimeElapsed = elapsedMicros();
      }

      hsCurrMaxValue = INVALID_HALL_SENSOR_VALUE;
    }
  }
}