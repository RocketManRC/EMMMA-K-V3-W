/* 

This is the source code for the EMMMA-K-W Wireless Hub (ESP32-S3)
which is an ATOM S3

Copyright 2025 RocketManRC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

If you use this code for any purpose please make sure that you
are not infringing on any third party's intellectual property rights.

*/

/* 
 * This program is enumerated as USB MIDI device and a SERIAL MIDI device
 * The following librarys are required:
 * - MIDI Library by Forty Seven Effects (for SERIAL MIDI)
 *   https://github.com/FortySevenEffects/arduino_midi_library
 * - adafruit/Adafruit TinyUSB Library (for USB MIDI)
 */

/*
  This works on the M5Stack ATOM S3 Lite
  
  Have to remove this file from the project to get it to link (not sure if this is still true in 2025):
    .pio/libdeps/esp32-s2-saola-1/Adafruit TinyUSB Library/src/portable/espressif/esp32sx/dcd_esp32sx.c

  Have to use these build flags in platformio.ini to get it to compile:
    build_unflags = -DARDUINO_USB_MODE=1
    build_flags =
     -DUSE_TINYUSB
     -DARDUINO_USB_MODE=0 
     '-DCFG_TUSB_CONFIG_FILE="/Users/rick/.platformio/packages/framework-arduinoespressif32/tools/sdk/esp32s2/include/arduino_tinyusb/include/tusb_config.h"'

  This comes from this forum:
    https://community.platformio.org/t/tinyusb-definition-errors-on-esp32s3/29382

  Version 3.1 added:
    - added volume and channel to note on/off packet
    - added channel to pitch bend packet
    - added CC packet
    
  There is a bug in the MIDI library for MIDI pitch bend where it will only bend up. Need to 
  change line 343 in MIDI.hpp to:

  const int value = int(fabs(inPitchValue) * double(scale));

  This change is automatically applied by the patch file patchfile.py by PlatformIO.
*/

/*
  Changes in May/June 2025:
    - Can now receive "raw" MIDI packets over ESP-Now but still supports custom packets
      from the EMMMA-K (for chords). The raw packets were added for MidiMech but will
      also be useful for data from a USB Host board.
    - Added Serial MIDI output which before was in a seperate project AtomEspNowHubToSerialMidi.

  The behaviour of the LED Now:
    - red on itialization and goes blue if USB MIDI is not available else it goes green
    - if USB MIDI is not available it will from blue to green when a ESP-Now packet is received.
    - if the button is pressed on startup the LED will turn blue indicating it is going to
      send a broadcast ESP-Now which will to any sender that is waiting to "bind" (e.g. an
      EMMMA-K, MidiMech or whatever comes next)

  Note that multiple devices can bind to one hub however the first one to send something to it
  will be the device that reeives return messages such as CCs!
      
*/

#include "Wire.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include <esp_now.h>
#include <WiFi.h>

// The newer versions of this board seem to have a neopixel instead of an led on pin 9
#define RGBLED 1 // Set to 1 to enable RGB LED for waveshare

#ifdef ATOMS3
#include <FastLED.h>
#include "Button.h"
#include "LED_DisPlay.h"
LED_DisPlay(AtomLed);

#else
#if RGBLED
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel pixels(1, 9, NEO_GRB + NEO_KHZ800);
#endif
#endif

#define ZYNTHIAN 0 // Set to 1 to enable some logic to handle Zynthian quirks

uint8_t midiChannel = 0; // set nonzero to override MIDI channel

bool usbMidiOn = false;

uint8_t returnAddress[6] = {};
uint8_t *returnAddressPointer = 0;
esp_now_peer_info_t peerInfo = {}; // must be initialized to 0

void handleNoteOn(byte channel, byte pitch, byte velocity);
void handleNoteOff(byte channel, byte pitch, byte velocity);

struct MySettings : public midi::DefaultSerialSettings
{
  static const long BaudRate = 31250;
};
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial2, SERIAL_MIDI, MySettings);

// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

// Variable that holds the current position in the startup note sequence.
uint32_t position = 0;

// Store startup melody as an array of note values
#ifdef ATOMS3
byte note_sequence[] = 
{
  74,78,81,86,90
};
#else
byte note_sequence[] = 
{
  //74,78,81,86,90,93,98,102,57,61,66,69,73,78,81,85,88,92,97,100,97,92,88,85,81,78,
  //74,69,66,62,57,62,66,69,74,78,81,86,90,93,97,102,97,93,90,85,81,78,73,68,64,61,
  56,61,64,68,74,78,81,86,90,93,98,60
};
#endif

void data_receive(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  // Save the return address if not saved already

  if(!returnAddressPointer)
  {
    for(int i = 0; i < 6; i++)
    {
      returnAddress[i] = mac[i];

      Serial.print(mac[i], 16);
      Serial.print(" ");
    }

    Serial.println();

    memcpy(peerInfo.peer_addr, returnAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;     

    if(esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      Serial.println("Failed to add peer");
    }
    else
    {
      returnAddressPointer = returnAddress; // all good!
#ifdef ATOMS3
      AtomLed.drawpix(0xff0000); // green for go!
      AtomLed.show();
#else
      // can add for waveshare here if it has a neopixel
#endif
    }
  }

  // The first type of packet is 4, 8 or 12 bytes for notes and chords. The first is the MIDI note value, the second is a flag 
  // for note on or off, the third is the volume and the fourth is the MIDI channel.
  // If it is a 9 byte packet (for a double) it is a pitch bend with the 9th byte being MIDI channel.
  // A 3 byte packet is for CCs with the first being the CC number, the second the value
  // and the third the MIDI channel

  // Modified June 2025 to test for an incoming raw MIDI packet as indicated by
  // the velocity field (incoming[1]) greater than 0x7F and send it directly.

  // This is compatible with both the EMMMA-Ks and UsbMidiToUsbNow.

  if(incomingData[0] > 0x7F)
  {
    //Serial.printf("Raw MIDI packet %X, %X, %X, %X\n", incomingData[0], incomingData[1],
    // incomingData[2], incomingData[3]);

    uint8_t type = incomingData[0];
    uint8_t data1 = incomingData[1];
    uint8_t data2 = incomingData[2];
    uint8_t channel = incomingData[3];

    // Do a sanity check on the data to make sure it is valid MIDI
    if(len == 4 && channel > 0 && channel <= 16)
    {
      if(usbMidiOn)
        MIDI.send((midi::MidiType)type, (midi::DataByte)data1, (midi::DataByte)data2, (midi::Channel)channel);
      SERIAL_MIDI.send((midi::MidiType)type, (midi::DataByte)data1, (midi::DataByte)data2, (midi::Channel)channel);
    }
    else
      Serial.println("Data doesn't appear to be valid MIDI");
  }
  else if(len == 4 || len == 8 || len == 12) // Notes
  {
    
    if(incomingData[1]) // first note
    {
      if(midiChannel)
      {
        if(usbMidiOn)
          MIDI.sendNoteOn(incomingData[0], incomingData[2], midiChannel);
        SERIAL_MIDI.sendNoteOn(incomingData[0], incomingData[2], midiChannel);
      }
      else
      {
        if(usbMidiOn)
          MIDI.sendNoteOn(incomingData[0], incomingData[2], incomingData[3]);
        SERIAL_MIDI.sendNoteOn(incomingData[0], incomingData[2], incomingData[3]);
      }

      digitalWrite(9, LOW);
    }
    else
    {
      if(midiChannel)
      {
        if(usbMidiOn)
          MIDI.sendNoteOff(incomingData[0], incomingData[2], midiChannel);
        SERIAL_MIDI.sendNoteOff(incomingData[0], incomingData[2], midiChannel);
      }
      else
      {
        if(usbMidiOn)
          MIDI.sendNoteOff(incomingData[0], incomingData[2], incomingData[3]);
        SERIAL_MIDI.sendNoteOff(incomingData[0], incomingData[2], incomingData[3]);
      }

      digitalWrite(9, HIGH);
    }
    
    if(len == 8 || len == 12) // second note if any
    {
      if(incomingData[5])
      {
        if(midiChannel)
        {
          if(usbMidiOn)
            MIDI.sendNoteOn(incomingData[4], incomingData[6], midiChannel);
          SERIAL_MIDI.sendNoteOn(incomingData[4], incomingData[6], midiChannel);
        }
        else
        {
          if(usbMidiOn)
            MIDI.sendNoteOn(incomingData[4], incomingData[6], incomingData[7]);
          SERIAL_MIDI.sendNoteOn(incomingData[4], incomingData[6], incomingData[7]);
        }

        digitalWrite(9, LOW);
      }
      else
      {
        if(midiChannel)
        {
          if(usbMidiOn)
            MIDI.sendNoteOff(incomingData[4], incomingData[6], midiChannel);
          SERIAL_MIDI.sendNoteOff(incomingData[4], incomingData[6], midiChannel);
        }
        else
        {
          if(usbMidiOn)
            MIDI.sendNoteOff(incomingData[4], incomingData[6], incomingData[7]);
          SERIAL_MIDI.sendNoteOff(incomingData[4], incomingData[6], incomingData[7]);
        }

        digitalWrite(9, HIGH);
      }
    }
    
    if(len == 12) // third note if any
    {
      if(incomingData[9])
      {
        if(midiChannel)
        {
          if(usbMidiOn)
            MIDI.sendNoteOn(incomingData[8], incomingData[10], midiChannel);
          SERIAL_MIDI.sendNoteOn(incomingData[8], incomingData[10], midiChannel);
        }
        else
        {
          if(usbMidiOn)
            MIDI.sendNoteOn(incomingData[8], incomingData[10], incomingData[11]);
          SERIAL_MIDI.sendNoteOn(incomingData[8], incomingData[10], incomingData[11]);
        }

        digitalWrite(9, LOW);
      }
      else
      {
        if(midiChannel)  
        {
          if(usbMidiOn)
            MIDI.sendNoteOff(incomingData[8], incomingData[10], midiChannel);
          SERIAL_MIDI.sendNoteOff(incomingData[8], incomingData[10], midiChannel);
        }
        else
        {
          if(usbMidiOn)
            MIDI.sendNoteOff(incomingData[8], incomingData[10], incomingData[11]);
          SERIAL_MIDI.sendNoteOff(incomingData[8], incomingData[10], incomingData[11]);
        }

        digitalWrite(9, HIGH);
      }
    }
  }
  else if(len == 9) // Pitch bend
  {
    double bendData;
    uint8_t *dp = (uint8_t *)&bendData;
    for(int i = 0; i < 8; i++)
    {
      dp[i] = incomingData[i];
    }
    if(midiChannel)
    {
      if(usbMidiOn)
        MIDI.sendPitchBend(bendData, midiChannel);
      SERIAL_MIDI.sendPitchBend(bendData, midiChannel);
    }
    else
    {
      if(usbMidiOn)
        MIDI.sendPitchBend(bendData, incomingData[8]);
      SERIAL_MIDI.sendPitchBend(bendData, incomingData[8]);
    }
    //Serial.println(bendData);
  }
  else if(len == 3) // CC
  {
    if(midiChannel)
    {
      if(usbMidiOn)
        MIDI.sendControlChange(incomingData[0], incomingData[1], midiChannel);
      SERIAL_MIDI.sendControlChange(incomingData[0], incomingData[1], midiChannel);
    }
    else
    {
      if(usbMidiOn)
        MIDI.sendControlChange(incomingData[0], incomingData[1], incomingData[2]);
      SERIAL_MIDI.sendControlChange(incomingData[0], incomingData[1], incomingData[2]);
    }
  }
}

void setup()
{
#if 1
  AtomLed.begin();
  AtomLed.drawpix(0x00FF00); // initialize LED to red (GRB)
  AtomLed.setBrightness(50);
  AtomLed.show();

  Serial.begin(115200);
  //myTransfer.begin(Serial);

  Serial2.setPins(2, 1); // change pins for serial midi for grove port
  SERIAL_MIDI.begin();
  SERIAL_MIDI.turnThruOff();
  //SERIAL_MIDI.setHandleNoteOn(handleSerialNoteOn);  
  //SERIAL_MIDI.setHandleNoteOff(handleSerialNoteOff);
  //SERIAL_MIDI.setHandleControlChange(handleSeriakControlChange);

  delay(2000);

  WiFi.mode(WIFI_MODE_STA);
  //while (!WiFi.STA.started()) // would need this for espressifarduino v3...
  //{
  //  delay(100);
  //}
  Serial.println(WiFi.macAddress()); 

  Serial.println("starting...");

  if(esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-Now");
  }
  else
  {
    esp_now_register_recv_cb((esp_now_recv_cb_t)data_receive);
    //esp_now_register_recv_cb((esp_now_recv_cb_t)data_receive);
    Serial.println("ESP-Now OK");
  }

  pinMode(41, INPUT_PULLUP);

  delay(100);

  if(!digitalRead(41)) // Is button pushed on startup?
  {
    AtomLed.drawpix(0x0000FF); // initialize LED to blue (GRB)
    AtomLed.show();

    while(!digitalRead(41))
    {
      delay(100); // wait for button to be released...
      Serial.print(".");
    }

    // Broadcast a message to every device in range
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peerInfo = {};
    String message = "EMMMA-K";
    
    memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
    if(!esp_now_is_peer_exist(broadcastAddress))
    {
      esp_now_add_peer(&peerInfo);
    }
    // Send message
    esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)message.c_str(), message.length());

    delay(100);

    ESP.restart();
  }
#else
#if RGBLED
  pixels.setBrightness(10);
  pixels.begin(); // INITIALIZE NeoPixel (REQUIRED)

  pixels.setPixelColor(0, 0xFF0000); // init to red

  pixels.show();   
#else
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
#endif
#endif

  
  //usb_midi.setStringDescriptor("TinyUSB MIDI");

  // Initialize MIDI, and listen to all MIDI channels
  // This will also call usb_midi's begin()
  MIDI.begin(MIDI_CHANNEL_OMNI);

  // Attach the handleNoteOn function to the MIDI Library. It will
  // be called whenever the Bluefruit receives MIDI Note On messages.
  MIDI.setHandleNoteOn(handleNoteOn);

  // Do the same for MIDI Note Off messages.
  MIDI.setHandleNoteOff(handleNoteOff);

  // wait until device mounted
  uint16_t count = 0;

  while(!TinyUSBDevice.mounted()) 
  {
    count++;
    delay(1);

    if(count > 100) // if 100ms have gone by then there is no USB MIDI
      break;
  }

  if(count < 100) 
    usbMidiOn = true;

  if(usbMidiOn)
  {
    delay(500); // need this or you get a squeal to start that doesn't go away!

    Serial.println("USB MIDI started...");

    AtomLed.drawpix(0xFF0000); // change LED to green (GRB)
    AtomLed.show();
  }
  else
  {
    AtomLed.drawpix(0x0000FF); // change LED to blue (GRB)
    AtomLed.show();
  }
}

void loop()
{
  static uint32_t start_ms = millis();

  if(millis() - start_ms > 133 && position < sizeof(note_sequence))
  {
    int previous = 0;

    start_ms += 133;
    
    // Setup variables for the current and previous
    // positions in the note sequence.
    if(position > 0)
      previous = position - 1;
    
    // Send Note On for current position at full velocity (127) on channel 1.
    if(position < sizeof(note_sequence) - 1)
    {
      if(usbMidiOn)
        MIDI.sendNoteOn(note_sequence[position] - 12, 127, 1);

      uint32_t us = micros();
      SERIAL_MIDI.sendNoteOn(note_sequence[position] - 12, 127, 1);
      Serial.println(micros() - us); // print how many us to send Serial MIDI (seems to be < 100 us)

      Serial.println(position);
    }
  
    // Send Note Off for previous note.
    if(position > 0)
    {
      if(usbMidiOn)
        MIDI.sendNoteOff(note_sequence[previous] - 12, 0, 1);
      SERIAL_MIDI.sendNoteOff(note_sequence[previous] - 12, 0, 1);
    }
  
    // Increment position
    position++;

#if ZYNTHIAN
    // Tell Zynthian to stop all notes playing (in case note stuck)
    if(position == sizeof(note_sequence))
    {
      for(int i = 1; i < 17; i++)
      {
        MIDI.sendControlChange(120, 127, i); // Send CC120 to master channel
        delay(50);
        MIDI.sendControlChange(123, 127, i); // Send CC123 to master channel
        delay(50);
      }
    }
#endif
  }
#if 0
  // Check for data from Serial
  if(myTransfer.available())
  {
    static uint32_t buf[4];
    uint8_t recSize = 0;

    recSize = myTransfer.rxObj(buf, recSize);

    // process the serial data
    if(buf[1])
      MIDI.sendNoteOn(buf[0], buf[2], buf[3]);
    else
      MIDI.sendNoteOff(buf[0], 0, buf[3]);
  }
#else
  static uint8_t data[5];
  static uint8_t idx = 0;

  while(Serial.available())
  {
    uint8_t c = Serial.read();

    data[idx++] = c;

    if(idx == sizeof(data))
    {
      idx = 0;

      if(data[0] == 4) // Is this a note packet?
      {
        // Packet will be size (4), note, on, volume channel
        if(data[2] == 1) // note on?
        {
          if(usbMidiOn)
            MIDI.sendNoteOn(data[1], data[3], data[4]);
          SERIAL_MIDI.sendNoteOn(data[1], data[3], data[4]);
        }
        else
        {
          // Packet will be size (4), note, on, volume channel
          if(usbMidiOn)
            MIDI.sendNoteOff(data[1], data[3], data[4]);
          SERIAL_MIDI.sendNoteOff(data[1], data[3], data[4]);
        }
      }
      else
      {
        // CC packet = size (3), cc number, cc value, channel
        if(usbMidiOn)
          MIDI.sendControlChange(data[1], data[2], data[3]);
        SERIAL_MIDI.sendControlChange(data[1], data[2], data[3]);
      }
    }
  }
  
#endif

    
  /*
    For testing AMY, send a program change with the patch number when
    the button is pressed and then released.

    Note CCk0 sets the bank number so would be 1 for DX7. I haven't
    tried that yet... All the info is in midi.c starting at
    lone 73.
  */

  static int patchNumber = 0;
  static bool buttonPushed = false;
  static uint32_t msPushed = 0;
  static uint32_t msReleased = 0;

  if(!digitalRead(41)) // Is button pushed?
  {
    buttonPushed = true;
    msPushed = millis();
  }
  else if(buttonPushed)
  {
    if(millis() - msPushed > 100) // 100 ms debounce
    {
      // Here if button pushed last time and more than 100 ms ago
      buttonPushed = false;

      SERIAL_MIDI.sendProgramChange(++patchNumber, 1);

      Serial.println(patchNumber);
    }
  }

// read any new MIDI messages
  if(MIDI.read())
  {
    uint8_t msg[3];
    uint8_t type = MIDI.getType();
    uint8_t data1 = MIDI.getData1();
    uint8_t data2 = MIDI.getData2();
    msg[0] = type;
    msg[1] = data1;
    msg[2] = data2;

    Serial.printf("%d %d %d\n", type, data1, data2);

    if(returnAddressPointer)
    {
      esp_err_t outcome = esp_now_send(returnAddressPointer, (uint8_t *) &msg, sizeof(msg)); 
    } 
  }
#if ZYNTHIAN
  const uint8_t numSongs = 2;
  const uint8_t songChannels[numSongs] = 
    {7, 12}; // MIDI channels for songs (starting from 8)
  static uint8_t currentSong = 0; // 0 is no song, 1 or greater is a song

  if(!digitalRead(41)) // Is button pressed
  {
    // wait for button to be released...
    while(!digitalRead(41))
    {
      delay(50); 
      Serial.print(".");
    }

    // stop the current song if any 
    if(currentSong)
    {
      MIDI.sendNoteOff(60, 0, songChannels[currentSong - 1]);
      delay(50);
    }

    // Play the next song unless this is the end in which case we stop
    if(currentSong == numSongs)
    {
      currentSong = 0; // indicate that stopped
    }
    else
    {
      currentSong++;    

      MIDI.sendNoteOn(60, 127, songChannels[currentSong - 1]);
      delay(50);
    }
  }
#endif
}

void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  // Log when a note is pressed.
  Serial.print("Note on: channel = ");
  Serial.print(channel);

  Serial.print(" pitch = ");
  Serial.print(pitch);

  Serial.print(" velocity = ");
  Serial.println(velocity);
}

void handleNoteOff(byte channel, byte pitch, byte velocity)
{
  // Log when a note is released.
  Serial.print("Note off: channel = ");
  Serial.print(channel);

  Serial.print(" pitch = ");
  Serial.print(pitch);

  Serial.print(" velocity = ");
  Serial.println(velocity);
}
