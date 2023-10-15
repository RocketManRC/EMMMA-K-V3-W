/* 

This is the source code for the EMMMA-K-v3.2 Wireless Hub (ESP32-S3).

Copyright 2023 RocketManRC

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
 * This program is enumerated as USB MIDI device. 
 * The following library is required
 * - MIDI Library by Forty Seven Effects
 *   https://github.com/FortySevenEffects/arduino_midi_library
 */

/*
  This works on the ESP32-S2-DevKitC or the Waveshare ESP-32-S2-Pico. See
  platformio.ini to configure.
  
  Have to remove this file from the project to get it to link:
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

uint8_t midiChannel = 0; // set nonzero to override MIDI channel

uint8_t returnAddress[6] = {};
uint8_t *returnAddressPointer = 0;
esp_now_peer_info_t peerInfo = {}; // must be initialized to 0

void handleNoteOn(byte channel, byte pitch, byte velocity);
void handleNoteOff(byte channel, byte pitch, byte velocity);

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

  if(len == 4 || len == 8 || len == 12) // Notes
  {
    
    if(incomingData[1]) // first note
    {
      if(midiChannel)
        MIDI.sendNoteOn(incomingData[0], incomingData[2], midiChannel);
      else
        MIDI.sendNoteOn(incomingData[0], incomingData[2], incomingData[3]);

      digitalWrite(9, LOW);
    }
    else
    {
      if(midiChannel)
        MIDI.sendNoteOff(incomingData[0], incomingData[2], midiChannel);
      else
        MIDI.sendNoteOff(incomingData[0], incomingData[2], incomingData[3]);

      digitalWrite(9, HIGH);
    }
    
    if(len == 8 || len == 12) // second note if any
    {
      if(incomingData[5])
      {
        if(midiChannel)
          MIDI.sendNoteOn(incomingData[4], incomingData[6], midiChannel);
        else
          MIDI.sendNoteOn(incomingData[4], incomingData[6], incomingData[7]);

        digitalWrite(9, LOW);
      }
      else
      {
        if(midiChannel)
          MIDI.sendNoteOff(incomingData[4], incomingData[6], midiChannel);
        else
          MIDI.sendNoteOff(incomingData[4], incomingData[6], incomingData[7]);

        digitalWrite(9, HIGH);
      }
    }
    
    if(len == 12) // third note if any
    {
      if(incomingData[9])
      {
        if(midiChannel)
          MIDI.sendNoteOn(incomingData[8], incomingData[10], midiChannel);
        else
          MIDI.sendNoteOn(incomingData[8], incomingData[10], incomingData[11]);

        digitalWrite(9, LOW);
      }
      else
      {
        if(midiChannel)  
          MIDI.sendNoteOff(incomingData[8], incomingData[10], midiChannel);
        else
          MIDI.sendNoteOff(incomingData[8], incomingData[10], incomingData[11]);

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
      MIDI.sendPitchBend(bendData, midiChannel);
    else
      MIDI.sendPitchBend(bendData, incomingData[8]);
    //Serial.println(bendData);
  }
  else if(len == 3) // CC
  {
    if(midiChannel)
      MIDI.sendControlChange(incomingData[0], incomingData[1], midiChannel);
    else
      MIDI.sendControlChange(incomingData[0], incomingData[1], incomingData[2]);
  }
}

void setup()
{
#ifdef ATOMS3
  AtomLed.begin();
  AtomLed.drawpix(0x00FF00); // initialize LED to red
  AtomLed.setBrightness(50);
  AtomLed.show();

  Serial.begin(115200);

  delay(2000);

  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress()); 

  Serial.println("starting...");

  if(esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-Now");
  }
  else
  {
    esp_now_register_recv_cb(data_receive);
    Serial.println("ESP-Now OK");
  }

  pinMode(41, INPUT_PULLUP);

  delay(100);

  if(!digitalRead(41)) // Is button pushed on startup?
  {
    AtomLed.drawpix(0x0000FF); // initialize LED to blue
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
  while(!TinyUSBDevice.mounted()) 
    delay(1);

  delay(500); // need this or you get a squeal to start that doesn't go away!

  Serial.println("MIDI starting...");

#ifdef ATOMS3
    AtomLed.drawpix(0xFF0000); // initialize LED to green
    AtomLed.show();
#else
#if RGBLED
  pixels.setPixelColor(0, 0x00FF00); // set to green
  pixels.show();   
#else
  digitalWrite(9, HIGH);
#endif
#endif
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
      MIDI.sendNoteOn(note_sequence[position] - 12, 127, 1);

      Serial.println(position);
    }
  
    // Send Note Off for previous note.
    if(position > 0)
      MIDI.sendNoteOff(note_sequence[previous] - 12, 0, 1);
  
    // Increment position
    position++;
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
