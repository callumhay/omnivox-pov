#include <algorithm>

#include <OctoWS2811.h>
#include <PacketSerial.h>

//#define PI 3.1415926535897932384626433832795
//#define TWO_PI 6.283185307179586476925286766559


// PoV Physical Constants ***************************************************************
#define NUM_LEDS_PER_ARM  32 // Number of controllable LEDs per arm/blade of the PoV
#define NUM_ARMS_PER_LEVEL 2 // Each full level/propeller has this many arms/blades
#define NUM_POV_LEVELS     1 // TODO: This should be set to LEDS_PER_ARM
#define NUM_CONTROLLABLE_LEDS (NUM_LEDS_PER_ARM * NUM_ARMS_PER_LEVEL * NUM_POV_LEVELS)

// Comm/Serial Constants ****************************************************************
#define RX_PIN 0
#define TX_PIN 1
#define CTS_PIN 18
#define RTS_PIN 17
#define TRANSMIT_ENABLE_PIN 22
#define USB_SERIAL_BAUD 9600
#define HW_SERIAL_BAUD 3000000
#define DEBUG_SERIAL Serial
#define DATA_SERIAL Serial1

#define MAX_BUFFER_LOOKAHEAD 32
// The serial buffer needs to be large enough to hold a full COBs encoded frame + lookahead
#define PACKET_BUFFER_MAX_SIZE (NUM_CONTROLLABLE_LEDS * 3 + 4 + MAX_BUFFER_LOOKAHEAD)
typedef PacketSerial_<COBS, 0, PACKET_BUFFER_MAX_SIZE> OmnivoxPoVPacketSerial;
OmnivoxPoVPacketSerial packetSerial;

// Packet Header/Identifier Constants ***************************************************
#define MY_SLAVE_ID 1
#define EMPTY_SLAVE_ID 255
#define WELCOME_HEADER 'W'
#define VOXEL_DATA_ALL_TYPE 'A'
#define GOODBYE_HEADER 'G'
#define SERVER_DATA_HEADER 'S'

// Peripheral/HW constants ***************************************************************
#define HALL_SENSOR_ANALOG_PIN 23


// OctoWS2811 LED Control Constants/Variables ********************************************
// All other LED-related settings are done on the server/computer that feeds the data
#define OCTO_CONFIG    WS2811_800kHz // | WS2811_GRB
#define LEDS_PER_STRIP 256 // TODO: Try increasing this instead of having more slave boards
#define MEM_BUFF_LEN   (LEDS_PER_STRIP * 6) // NOTE: 6 integers per LED are used in OctoWS2811

DMAMEM int displayMemory[MEM_BUFF_LEN] = {0};
int drawingMemory[MEM_BUFF_LEN] = {0};

OctoWS2811 leds(LEDS_PER_STRIP, displayMemory, drawingMemory, OCTO_CONFIG);


// Hall Sensor Monitoring Constants/Variables *******************************************
#define HALL_SENSOR_THRESHOLD 900
#define HALL_SENSOR_DEBOUNCE_TIME_MICROS 2000
#define INVALID_HALL_SENSOR_VALUE -1
elapsedMicros hsSinceAboveThreshold;
elapsedMicros hsSinceMaxValue;
elapsedMicros hsSincePeak;
elapsedMicros hsDebounceTimeElapsed;
unsigned long hsLastPeakTime = 0;
int hsCurrMaxValue = INVALID_HALL_SENSOR_VALUE;

// POV Control Constants/Variables *******************************************************
// Maximum allowable time for a single rotation of a PoV
#define MAX_ROTATION_TIME_MICROS 1000000 // (1 second)
#define INVALID_ROTATION_TIME_MICROS 0
// Current, tracked total rotation time (microseconds) for the PoV
unsigned long totalRotationTime = INVALID_ROTATION_TIME_MICROS;


int lastKnownIncomingFrameId = -1;
int outgoingFrameId = 0;


bool isRotationTimeValid(unsigned long rotationTime) {
  return rotationTime > 0 && rotationTime <= MAX_ROTATION_TIME_MICROS;
}

// Send a message to the server to update it on the current rotation time and any other relevant info
void sendServerUpdateInfo(unsigned long rotationTime) {
  // NOTE: The max unsigned long value is 4294967295 (10 characters)
  #define MAX_TEMP_BUFFER_SIZE 32
  char tempBuffer[MAX_TEMP_BUFFER_SIZE];
  snprintf(tempBuffer, MAX_TEMP_BUFFER_SIZE, "%c %d %d %lu\n", SERVER_DATA_HEADER, MY_SLAVE_ID, outgoingFrameId++, rotationTime); 
  packetSerial.send((const uint8_t*)tempBuffer, sizeof(tempBuffer));
}


void readWelcomeHeader(const uint8_t* buffer, size_t size, size_t startIdx) {
  DEBUG_SERIAL.printf("[Slave %i] Welcome Header / Init data recieved on slave.", MY_SLAVE_ID); DEBUG_SERIAL.println();
  lastKnownIncomingFrameId = -1;
  outgoingFrameId = 0;
}

int getFrameId(const uint8_t* buffer, size_t size) {
  return size > 3 ? static_cast<uint16_t>((buffer[2] << 8) + buffer[3]) : 0;
}

void readFullVoxelData(const uint8_t* buffer, size_t size, size_t startIdx, int frameId) {
  // TODO
  /*
  // Debug/Info status update
  statusUpdateFrameCounter++;
  if (statusUpdateFrameCounter % STATUS_UPDATE_FRAMES == 0) {
     DEBUG_SERIAL.printf("[Slave %i] LED Refresh FPS: %.2f, Frame#: %i", MY_SLAVE_ID, (1000000.0f/((float)frameDiffMicroSecs)), lastKnownFrameId); 
     DEBUG_SERIAL.println();
     statusUpdateFrameCounter = 0;
  }
  */
}


void onSerialPacketReceived(const void* sender, const uint8_t* buffer, size_t size) {
  if (sender == &packetSerial && size > 2) {
    
    // The first byte of the buffer has the ID of the slave that it's relevant to
    int bufferIdx = 0; 
    uint8_t slaveId = buffer[bufferIdx++];
    if (slaveId != MY_SLAVE_ID && slaveId != EMPTY_SLAVE_ID) {
      // Ignore
      return;
    }

    // The second byte of the buffer describes the type of data, the rest will be the data itself
    switch (static_cast<char>(buffer[bufferIdx++])) {

      case WELCOME_HEADER:
        if (slaveId == EMPTY_SLAVE_ID) {
          sendServerUpdateInfo(totalRotationTime);
        }
        else {
          readWelcomeHeader(buffer, static_cast<size_t>(size-bufferIdx), bufferIdx);
        }
        break;

      case VOXEL_DATA_ALL_TYPE:
        bufferIdx += 2; // Frame ID
        readFullVoxelData(buffer, static_cast<size_t>(size-bufferIdx), bufferIdx, getFrameId(buffer, size));
        break;

      default:
        DEBUG_SERIAL.println("Unspecified packet received.");
        break;
    }
  }
}



void setup() {
  DEBUG_SERIAL.begin(USB_SERIAL_BAUD);

  DATA_SERIAL.transmitterEnable(TRANSMIT_ENABLE_PIN);
  DATA_SERIAL.begin(HW_SERIAL_BAUD);
  DATA_SERIAL.attachCts(CTS_PIN);
  DATA_SERIAL.attachRts(RTS_PIN);

  pinMode(HALL_SENSOR_ANALOG_PIN, INPUT);

  packetSerial.setStream(&DATA_SERIAL);
  packetSerial.setPacketHandler(&onSerialPacketReceived);

  // Clear all displayed LEDs to black/off
  leds.begin();
  leds.show();
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
          if (totalRotationTime == INVALID_ROTATION_TIME_MICROS) {
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
            totalRotationTime = INVALID_ROTATION_TIME_MICROS;
          }

          // The rotation time has updated, the server should be informed of this
          sendServerUpdateInfo(totalRotationTime);
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
    if (isRotationTimeValid(totalRotationTime) && timeSinceLastPeak <= MAX_ROTATION_TIME_MICROS) {

      DEBUG_SERIAL.print("Current total rotation time (us): ");
      DEBUG_SERIAL.println(totalRotationTime);
      // TODO




      isRotationDataValid = true;
    }
  }

  // Clear the LEDs if we don't have proper rotation time data
  if (!isRotationDataValid) {
    totalRotationTime = INVALID_ROTATION_TIME_MICROS;
    // TODO
  }

}