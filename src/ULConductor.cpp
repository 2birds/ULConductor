#include <vector>

#include "LeapC.h"
#include "RtMidi.h"

#include <cmath>
#include <cstring>
#include <mutex>
#include <optional>
#include <thread>

#ifdef _WIN32
#include <Windows.h>
#define SLEEP Sleep
#else
#include <unistd.h>
#define SLEEP usleep
#endif

static LEAP_CONNECTION connectionHandle;
static LEAP_TRACKING_EVENT* lastFrame = nullptr;
static LEAP_DEVICE_INFO* lastDevice = nullptr;

std::optional<float> xPos;
std::mutex lock;
bool isRunning = false;

/** Translates eLeapRS result codes into a human-readable string. */
const char* ResultString(eLeapRS r) {
  switch(r){
    case eLeapRS_Success:                  return "eLeapRS_Success";
    case eLeapRS_UnknownError:             return "eLeapRS_UnknownError";
    case eLeapRS_InvalidArgument:          return "eLeapRS_InvalidArgument";
    case eLeapRS_InsufficientResources:    return "eLeapRS_InsufficientResources";
    case eLeapRS_InsufficientBuffer:       return "eLeapRS_InsufficientBuffer";
    case eLeapRS_Timeout:                  return "eLeapRS_Timeout";
    case eLeapRS_NotConnected:             return "eLeapRS_NotConnected";
    case eLeapRS_HandshakeIncomplete:      return "eLeapRS_HandshakeIncomplete";
    case eLeapRS_BufferSizeOverflow:       return "eLeapRS_BufferSizeOverflow";
    case eLeapRS_ProtocolError:            return "eLeapRS_ProtocolError";
    case eLeapRS_InvalidClientID:          return "eLeapRS_InvalidClientID";
    case eLeapRS_UnexpectedClosed:         return "eLeapRS_UnexpectedClosed";
    case eLeapRS_UnknownImageFrameRequest: return "eLeapRS_UnknownImageFrameRequest";
    case eLeapRS_UnknownTrackingFrameID:   return "eLeapRS_UnknownTrackingFrameID";
    case eLeapRS_RoutineIsNotSeer:         return "eLeapRS_RoutineIsNotSeer";
    case eLeapRS_TimestampTooEarly:        return "eLeapRS_TimestampTooEarly";
    case eLeapRS_ConcurrentPoll:           return "eLeapRS_ConcurrentPoll";
    case eLeapRS_NotAvailable:             return "eLeapRS_NotAvailable";
    case eLeapRS_NotStreaming:             return "eLeapRS_NotStreaming";
    case eLeapRS_CannotOpenDevice:         return "eLeapRS_CannotOpenDevice";
    default:                               return "unknown result type.";
  }
}

void setMidiMessage(std::vector<unsigned char>& mmsg, unsigned char c1, unsigned char d1, unsigned char d2)
{
    if (mmsg.size() == 0)
    {
      for (int i = 0; i < 3; i++)
      {
        mmsg.push_back(0);
      }
    }  

    mmsg[0] = c1; // Control message
    mmsg[1] = d1; // Data 1
    mmsg[2] = d2; // Data 2
}

static void setDevice(const LEAP_DEVICE_INFO *deviceProps){
  // LockMutex(&dataLock);
  if(lastDevice){
    free(lastDevice->serial);
  } else {
    lastDevice = (LEAP_DEVICE_INFO *) malloc(sizeof(*deviceProps));
  }
  *lastDevice = *deviceProps;
  lastDevice->serial = (char *) malloc(deviceProps->serial_length);
  memcpy(lastDevice->serial, deviceProps->serial, deviceProps->serial_length);
  // UnlockMutex(&dataLock);
}

static void handleTrackingModeEvent(const LEAP_TRACKING_MODE_EVENT *device_event){
  if (device_event->current_tracking_mode != eLeapTrackingMode_Desktop)
  {
    std::cout << "Tracking mode is not set to desktop. Changing.." << std::endl;
    eLeapRS resultCode = LeapSetTrackingMode(connectionHandle, eLeapTrackingMode_Desktop);
    if (resultCode != eLeapRS_Success)
    {
      std::cout << "Unable to set tracking mode to desktop! " << ResultString(resultCode) << std::endl;
    }
  }
  else
  {
    std::cout << "Tracking mode set to Desktop." << std::endl;
  }
}

static void handleTrackingEvent(const LEAP_TRACKING_EVENT *tracking_event){
	float newXPos = 0;
    for (uint32_t i = 0; i < tracking_event->nHands; i++)
    {
        if (tracking_event->pHands[i].type == eLeapHandType_Right)
        {
            newXPos = tracking_event->pHands[i].palm.position.x;
        }
    }

    lock.lock();
	if (newXPos != 0)
	{
		xPos = newXPos;
	}
	else
	{
		xPos.reset();
	}
    lock.unlock();
}

static void handleDeviceEvent(const LEAP_DEVICE_EVENT *device_event){
  LEAP_DEVICE deviceHandle;
  //Open device using LEAP_DEVICE_REF from event struct.
  eLeapRS result = LeapOpenDevice(device_event->device, &deviceHandle);
  if(result != eLeapRS_Success){
    printf("Could not open device %s.\n", ResultString(result));
    return;
  }
  //Create a struct to hold the device properties, we have to provide a buffer for the serial string
  LEAP_DEVICE_INFO deviceProperties = { sizeof(deviceProperties) };
  // Start with a length of 1 (pretending we don't know a priori what the length is).
  // Currently device serial numbers are all the same length, but that could change in the future
  deviceProperties.serial_length = 1;
  deviceProperties.serial = (char *) malloc(deviceProperties.serial_length);
  //This will fail since the serial buffer is only 1 character long
  // But deviceProperties is updated to contain the required buffer length
  result = LeapGetDeviceInfo(deviceHandle, &deviceProperties);
  if(result == eLeapRS_InsufficientBuffer){
    //try again with correct buffer size
    deviceProperties.serial = (char *) realloc(deviceProperties.serial, deviceProperties.serial_length);
    result = LeapGetDeviceInfo(deviceHandle, &deviceProperties);
    if(result != eLeapRS_Success){
      printf("Failed to get device info %s.\n", ResultString(result));
      free(deviceProperties.serial);
      return;
    }
  }
  setDevice(&deviceProperties);
  // if(ConnectionCallbacks.on_device_found){
  //   ConnectionCallbacks.on_device_found(&deviceProperties);
  // }

  free(deviceProperties.serial);
  LeapCloseDevice(deviceHandle);
}

void setNoteOn(std::vector<unsigned char>& mmsg, unsigned char note = 0x3c)
{
    // Middle C, max velocity
    setMidiMessage(mmsg, 0x90, note, 0x40);
}

void setNoteOff(std::vector<unsigned char>& mmsg, unsigned char note = 0x3c)
{
    setMidiMessage(mmsg, 0x80, note, 0x40); // Max velocity
}

unsigned char posToMIDINote(float pos) {
	float range = 600.f;
	float halfRange = range / 2;

	std::cout << "pos: " << pos << std::endl;
    if (pos < -halfRange)
    {
        return 0;
    }

    if (pos > halfRange)
    {
        return 127;
    }

    unsigned char res = static_cast<unsigned char>(std::floor(((pos + halfRange) / range) * 127.f));
	std::cout << "MIDI note: " << static_cast<unsigned int>(res) << std::endl;
	return res;
}

int main()
{
    if (LeapCreateConnection(NULL, &connectionHandle) != eLeapRS_Success)
    {
        std::cout << "Could not create Leap connection." << std::endl;
        return 1;
    }

    eLeapRS result = LeapOpenConnection(connectionHandle);
    if (result == eLeapRS_Success)
    {
         isRunning = true;
//         std::cout << "Opened Leap connection." << std::endl;
// #if defined(_MSC_VER)
//         InitializeCriticalSection(&dataLock);
//         // pollingThread = (HANDLE)_beginthread(serviceMessageLoop, 0, NULL);
// #else
//         pthread_mutex_init(&dataLock, NULL);
//         pthread_create(&pollingThread, NULL, serviceMessageLoop, NULL);
// #endif
    }

    auto leapc_thread = std::thread(
        [&]
        {
            LEAP_CONNECTION_MESSAGE msg;
            while (isRunning)
            {
                unsigned int timeout = 1000;
                result = LeapPollConnection(connectionHandle, timeout, &msg);

                if (result != eLeapRS_Success)
                {
                    printf("LeapC PollConnection call was %s.\n", ResultString(result));
                    continue;
                }
                else
                {
                    // std::cout << "Message received" << std::endl;
                    // msgCount++;
                }

                switch (msg.type)
                {
                    // case eLeapEventType_Connection:
                    //   handleConnectionEvent(msg.connection_event);
                    //   break;
                    case eLeapEventType_ConnectionLost:
                        // handleConnectionLostEvent(msg.connection_lost_event);
                        break;
                    case eLeapEventType_Device:
                        handleDeviceEvent(msg.device_event);
                        break;
                    case eLeapEventType_DeviceLost:
                        // handleDeviceLostEvent(msg.device_event);
                        break;
                    case eLeapEventType_DeviceFailure:
                        // handleDeviceFailureEvent(msg.device_failure_event);
                        break;
                    case eLeapEventType_Tracking:
                        handleTrackingEvent(msg.tracking_event);
                        break;
                    case eLeapEventType_ImageComplete:
                        // Ignore
                        break;
                    case eLeapEventType_ImageRequestError:
                        // Ignore
                        break;
                    case eLeapEventType_LogEvent:
                        // handleLogEvent(msg.log_event);
                        break;
                    case eLeapEventType_Policy:
                        // handlePolicyEvent(msg.policy_event);
                        break;
                    case eLeapEventType_ConfigChange:
                        // handleConfigChangeEvent(msg.config_change_event);
                        break;
                    case eLeapEventType_ConfigResponse:
                        // handleConfigResponseEvent(msg.config_response_event);
                        break;
                    case eLeapEventType_Image:
                        // handleImageEvent(msg.image_event);
                        break;
                    case eLeapEventType_PointMappingChange:
                        // handlePointMappingChangeEvent(msg.point_mapping_change_event);
                        break;
                    case eLeapEventType_TrackingMode:
                        handleTrackingModeEvent(msg.tracking_mode_event);
                        break;
                    case eLeapEventType_LogEvents:
                        // handleLogEvents(msg.log_events);
                        break;
                    case eLeapEventType_HeadPose:
                        // handleHeadPoseEvent(msg.head_pose_event);
                        break;
                    case eLeapEventType_IMU:
                        // handleImuEvent(msg.imu_event);
                        break;
                    default:
                        // discard unknown message types
                        std::cout << "Unhandled message type " << msg.type << "." << std::endl;
                } // switch on msg.type
            }
        }
    );

    auto midi_thread = std::thread(
            [&]
            {
            unsigned char prevNote = 0;

            RtMidiOut *midiout;
            std::vector<unsigned char> message;

            try
            {
            midiout = new RtMidiOut();
            }
            catch (RtMidiError &error)
            {
            // Handle the exception here
			std::cout << "Error setting up MIDI" << std::endl;
            error.printMessage();
            exit(EXIT_FAILURE);
            }
			std::cout << "MIDI out created" << std::endl;

            unsigned int nPorts = midiout->getPortCount();
            if (nPorts == 0)
            {
            std::cout << "No ports available!" << std::endl;
            goto cleanup;
            }

            midiout->openVirtualPort();
			std::cout << "Virtual port opened" << std::endl;

            while (isRunning)
            {
				unsigned char newNote;
                lock.lock();
				if (xPos.has_value())
				{
                    newNote = posToMIDINote(*xPos);
				}
                lock.unlock();

                if (newNote != prevNote)
                {
                    setNoteOn(message, posToMIDINote(newNote));
                    midiout->sendMessage(&message);
                    setNoteOff(message, posToMIDINote(prevNote));
                    midiout->sendMessage(&message);
                    prevNote = newNote;
                }
            }

            midiout->closePort();

		  cleanup:
		  delete midiout;
  
            }
);

    std::thread stdin_thread(
        [&]
        {
          std::string input;
          while (true)
          {
            // Wait for keypress
            std::cin >> input;
            char cmd = input.at(0);
            if (cmd == 'x')
            {
              break;
            }
          }

          // Stop polling loop
          isRunning = false;
        });
    // Wait for threads
    if (stdin_thread.joinable())
    {
      stdin_thread.join();
    }

    if (leapc_thread.joinable())
    {
      leapc_thread.join();
    }

    if (midi_thread.joinable())
    {
      midi_thread.join();
    }

    // Tidy up
    LeapCloseConnection(connectionHandle);

  return 0;
}
