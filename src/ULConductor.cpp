#include <vector>

#include "LeapC.h"
#include "RtMidi.h"

#ifdef _WIN32
#include <Windows.h>
#define SLEEP Sleep
#else
#include <unistd.h>
#define SLEEP sleep
#endif

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

void setNoteOn(std::vector<unsigned char>& mmsg)
{
    // Middle C, max velocity
    setMidiMessage(mmsg, 0xb0, 0x3c, 0x0);
}

void setNoteOff(std::vector<unsigned char>& mmsg)
{
    setMidiMessage(mmsg, 0xb1, 0x3c, 0xef); // Max velocity
}

int main()
{
  RtMidiOut *midiout;
  std::vector<unsigned char> message;

  try
  {
    midiout = new RtMidiOut();
  }
  catch (RtMidiError &error)
  {
    // Handle the exception here
    error.printMessage();
    exit(EXIT_FAILURE);
  }
  
  unsigned int nPorts = midiout->getPortCount();
  if (nPorts == 0)
  {
    std::cout << "No ports available!" << std::endl;
    goto cleanup;
  }
  
  midiout->openVirtualPort();
  
  for (int i = 0; i < 30; i++)
  {
    setNoteOn(message);
    std::cout << "Switching middle C on.." << std::endl;
    midiout->sendMessage(&message);
    SLEEP(1000);
    setNoteOff(message);
    std::cout << "Switching middle C off.." << std::endl;
    midiout->sendMessage(&message);
    SLEEP(1000);
  }
  
  midiout->closePort();

  cleanup:
  delete midiout;
  
  return 0;
}
