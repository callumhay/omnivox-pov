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

int lastKnownIncomingFrameId = -1;
int outgoingFrameId = 0;

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


// Hall Sensor Monitoring and POV Constants/Variables **********************************

// NOTE: We assume that the typical rotation speed will be somewhere
// between 4 and 30 rotations per second, anything outside that range will
// require tuning of these constants

// Minimum allowable time for a single rotation of a PoV
#define MIN_ROTATION_TIME_MICROS 2800 // (~1/35 second)
// Maximum allowable time for a single rotation of a PoV
#define MAX_ROTATION_TIME_MICROS 250000 // (~1/4 second)
// The number of microseconds to wait before we allow new peaks to be detected after a peak
#define HALL_SENSOR_DEBOUNCE_TIME_MICROS 2500 // ~1/40th of a second
// Hall sensor threshold value (anything above this is considered worthy of recording as a peak)
#define HALL_SENSOR_THRESHOLD 800
#define INVALID_HALL_SENSOR_VALUE -1

elapsedMicros hsTimeSinceActivated;
elapsedMicros hsSincePeak;
elapsedMicros hsDebounceTimeElapsed;
unsigned long hsLastPeakTime = 0;
int hsCurrMaxValue = INVALID_HALL_SENSOR_VALUE;

// Current, tracked total rotation time (microseconds) for the PoV
unsigned long totalRotationTime = 0;


bool isRotationTimeValid(unsigned long rotationTime) {
  return rotationTime >= MIN_ROTATION_TIME_MICROS && rotationTime <= MAX_ROTATION_TIME_MICROS;
}

// Send a message to the server to update it on the current rotation time and any other relevant info
void sendServerUpdateInfo(unsigned long rotationTime) {
  // NOTE: The max unsigned long value is 4294967295 (10 characters)
  #define MAX_TEMP_BUFFER_SIZE 32 // Overkill, but better safe than sorry
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

int temp_count = 0;

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
  // IMPORTANT: The hall sensor is currently expected to be a TI DRV5055A2ELPGQ1
  // The sensor is a linear hall effect sensor, the non-activated value is around 700-800
  // in the presence of the proper magnetic field the value will increase to around 900-1023
  int hallSensorValue = analogRead(HALL_SENSOR_ANALOG_PIN);
  /*
  if (temp_count % 10 == 0) {
    //DEBUG_SERIAL.print("Hall sensor value: ");
    DEBUG_SERIAL.println(hallSensorValue);
  }
  temp_count++;
  return;
  */

  // Use a debounce time to avoid significant fluctuations in our estimation of the time between peaks
  if (hallSensorValue >= HALL_SENSOR_THRESHOLD && hsDebounceTimeElapsed >= HALL_SENSOR_DEBOUNCE_TIME_MICROS) {
    if (hsCurrMaxValue == INVALID_HALL_SENSOR_VALUE) {
      hsTimeSinceActivated = elapsedMicros();
    }
    if (hallSensorValue > hsCurrMaxValue) {
      hsCurrMaxValue = hallSensorValue;
    }
  }
  else {
    // Check whether the hall sensor was above the threshold previously
    if (hsCurrMaxValue != INVALID_HALL_SENSOR_VALUE) {
      bool haveRotatedOnce = hsLastPeakTime != 0;

      // Calculate the time (microsecs) since the hall sensor was at its maximum value
      hsLastPeakTime = static_cast<unsigned long>(hsTimeSinceActivated);

      // We need to rotate at least once before we start estimating the total rotation time
      if (haveRotatedOnce) {
        // NOTE: Since we've gone past the peak twice we need to subtract the time
        // since the last peak to get the actual rotation time.
        unsigned long actualRotationTime = static_cast<unsigned long>(hsSincePeak) - hsLastPeakTime;

        // One last check: Make sure the rotation time isn't ridiculously short or long
        // If it is, we can't use it and we'll just wait for the next rotation
        if (isRotationTimeValid(actualRotationTime)) {

          if (temp_count % 10 == 0) {
            DEBUG_SERIAL.print("Actual rotation time (us): ");
            DEBUG_SERIAL.println(actualRotationTime);
            DEBUG_SERIAL.print("Time since last peak (us): ");
            DEBUG_SERIAL.println(hsSincePeak);
            DEBUG_SERIAL.print("Last peak time (us): ");
            DEBUG_SERIAL.println(hsLastPeakTime);
            DEBUG_SERIAL.print("Last activated hall sensor value: ");
            DEBUG_SERIAL.println(hsCurrMaxValue);
          }

          if (totalRotationTime == 0) {
            // First time we're calculating the rotation time - just assign it
            totalRotationTime = actualRotationTime;
          }
          else {
            // Otherwise, we use a weighted average to smooth out the rotation time
            totalRotationTime = static_cast<unsigned long>(
              0.75 * static_cast<double>(actualRotationTime) +
              0.25 * static_cast<double>(totalRotationTime)
            );
          }

          // If the total rotation time is invalid then we can't use it, reinitialize it
          if (!isRotationTimeValid(totalRotationTime)) {
            totalRotationTime = 0;
          }
          // The rotation time has updated, the server should be informed of this
          sendServerUpdateInfo(totalRotationTime);

          // Debug/Info status update
          if (isRotationTimeValid(totalRotationTime) && temp_count % 10 == 0) {
            DEBUG_SERIAL.print("Current total rotation time (us): ");
            DEBUG_SERIAL.println(totalRotationTime);
          }
          temp_count++;
        }
      }

      hsSincePeak = hsTimeSinceActivated;
      hsDebounceTimeElapsed = elapsedMicros();
    }

    hsCurrMaxValue = INVALID_HALL_SENSOR_VALUE;
  }

  // If there's a rotation time established then we can start rendering based on
  // the time since the last peak and the expected rotation time
  bool isRotationDataValid = false;
  if (hsLastPeakTime != 0) {
    auto timeSinceLastPeak = static_cast<unsigned long>(hsSincePeak);
    if (isRotationTimeValid(totalRotationTime) && timeSinceLastPeak <= MAX_ROTATION_TIME_MICROS) {

      //DEBUG_SERIAL.print("Current total rotation time (us): ");
      //DEBUG_SERIAL.println(totalRotationTime);
      // TODO

      isRotationDataValid = true;
    }
  }

  // Clear the LEDs if we don't have proper rotation time data
  if (!isRotationDataValid) {
    totalRotationTime = 0;
    // TODO

  }

}