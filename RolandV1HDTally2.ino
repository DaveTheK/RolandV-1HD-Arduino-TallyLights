/******************************************************************************

                  ROLAND V1HD TALLY LIGHT SYSTEM

   This Arduino Sketch is designed to take MIDI state changes informtion from a Roland V-1HD Video Switcher and tunn
   on Tally lights for each camera.   Tally light systems are used in braoacast studios to indicate to the talent which camera
   is on so they know which one to look at. Since we use the V-1HD for live streaming it's very helpful to know which cameras is "live"

   The design is built around the Sparkfun MIDI Shield hardware and MIDI data anlyzer code below.  We don't use much of that code
   as it didnt handle the Running Status data as we expected so wrote our own message handling.

   There are several compoenents of the system. The Main Controller sits near the V-1HD and connects by MIDI calble (not USB). The V-1HD setting
   must be set to MIDI Out NOT MIDI Thru. The main controller requries DC power, has a 3-pin XLR connector, and 4 LEDs on the front panel

   The Camera Controller connects with the Main Controller, which monitors as to what buttons have been
   pressed on the video switcher and relays the calculated cameras tally light colors to the Camera Controller. It uses a serial protocol
   at 9600 baud, which is slow enough to travel well over standard balanced mic cables.  The serial protocol sent to the camera controller
   is a printable 7-bit ASCII character, encoding the bank number of the switcher, the camera number and the color (including OFF).
   By sending a single serial character we have maximum reliability over the mic cable. However, since only one camera color is sent per byte,
   at startup time the Main Contoller may not be in synch with the video switcher, so the user has to press butttons and adjust the fader until
   they are in synch. Interestingly when the switcher powers up it does send these status: the bank 0 button, and the bank 1 button and the state of the
   fade-to-black controller, and the t-bar fader, so if you power on the switcher after the main controller it may synch properly...

   The Main controller connectors include an XLR output, and a 9-12V DC input.
   The Camera controller connectors include an XLR input, a 9-12V DC Input, and 4 mini-xlr outputs, one for each tally light. Each tally light is an
   Adafruit Neopixel which is linked by shielded 2-conductor cable.

   Designed by David Kauffman davekauffman.ca October 2020.

  Use SparkFun MIDI Shield as a MIDI data analyzer.
  Byron Jacquot, SparkFun Electronics
  October 8, 2015
  https://github.com/sparkfun/MIDI_Shield/tree/V_1.5/Firmware/MIDI-sniffer
  Reads all events arriving over MIDI, and turns them into descriptive text.
  If you hold the button on D2, it will switch to display the raw hex values arriving,
  which can be useful for viewing incomplete messages and running status.
  Resources:
  Requires that the MIDI Sheild be configured to use soft serial on pins 8 & 9,
  so that debug text can be printed to the hardware serial port.
  This code is dependent on the FortySevenEffects MIDI library for Arduino.
  https://github.com/FortySevenEffects/arduino_midi_library
  This was done using version 4.2, hash fb693e724508cb8a473fa0bf1915101134206c34
  This library is now under the MIT license, as well.
  You'll need to install that library into the Arduino IDE before compiling.

  Development environment specifics:
  It was developed for the Arduino Uno compatible SparkFun RedBoard, with a  SparkFun
  MIDI Shield.

  Written, compiled and loaded with Arduino 1.6.5
  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
  Please review the LICENSE.md file included with this example. If you have any questions
  or concerns with licensing, please contact techsupport@sparkfun.com.
  Distributed as-is; no warranty is given.
******************************************************************************/


#include <SoftwareSerial.h>
#include <MsTimer2.h>
#include <MIDI.h>
#include <avr/wdt.h>          // watchdog timer

#define PIN_RAW_INPUT 2

#define PIN_POT_A0 0
#define PIN_POT_A1 1

#define MDEBUG 3
#define SERIALSPEED 115200

// These variable help monitor the MIDI keep alive messages from the switcher to know it is up, running, and connected.
unsigned long LastHeardFrom;
int LinkStatus = false;

// keep the last camera control code each time it is sent
int ccu_code = 0;


// keep the last standby control code each time it is sent
int scu_code = 0;

static const uint16_t DEBOUNCE_COUNT = 50;

// Need to use soft serial for MIDI monitoring, so we can report what's happening
// via messages on hard serial.
SoftwareSerial SoftSerial(8, 9);
// The Camera Controller is connected serially on this port.
SoftwareSerial ArduinoSlave(2, 3);

#define BANKS 2
#define CAMERAS 4

// The TALLY LIGHT Codes are defined here.
#define TALLY_OFF 0
#define TALLY_GREEN 1
#define TALLY_RED 2
#define TALLY_WHITE 3
char *TallyColours [4] = {"TALLY_OFF", "TALLY_GREEN", "TALLY_RED", "TALLY_WHITE"};

// The ButtonStates array represents the state and colours of the buttons on the V-1HD console.
// The fucntion PrintButtonStates puts them on the Serial line, e.g.
//    R---
//    -G--
int ButtonStates [BANKS][CAMERAS];
int TallyStates[CAMERAS];
bool ReadyStates[CAMERAS];            // holds the status of whether any camera is "green" ie on standby

static int ArduinoSignal = 0;
static int RunningStatus = 0;
static int inSysEx = 0;
// use these values to update each camera as the active sense message arrives approx. every 250ms
static int nextCamUpdate = 0;

#define BANK0 0
#define BANK1 1
#define BANKBOTH 2
static int SelectedBank = 1;    // 0 = top 1 = bottom, 2 = both
static int BankButton[2] = {0, 1};
static int PressedBank;

#define WIPE 0
#define MIX 1
#define CUT 2
static int VideoTransition;      // 3 transition modes on the switcher affect the t-bar tally.
static int LastFaderVal;         // track the direction of the fader
static int LastUpdatedCCU;       // how long since we sent a command to the camera control unit

// Digital Pins for DTMF Tone Genrator
#define D0 4
#define D1 5
#define D2 6
#define D3 7

// Digital pins for ready light control
#define G0 10
#define G1 11
#define GS 12

// CIRCULAR QUEUE
// The Q is empty when the front index == next index
// The Q is full when the next index is one lower then front index

#define QSIZE 256
unsigned char queue[QSIZE];
unsigned int qFront, qNext;

// CIRCULAR QUEUE - SIZEQ
// return the currebt size of the queue by subtracting the head from the tail with modulu arithmetic
int sizeQ()
{
  int size = ((qNext - qFront + QSIZE) % QSIZE);
  return size;
}

// CIRCULAR QUEUE - PRINTQ
// print out the queue from head to tail
void printQ()
{
  int qlen = sizeQ();
  for (int i = 0; i < qlen; i++) {
    Serial.print (queue [(qFront + i) % QSIZE], HEX);
    Serial.print (" ");
  }
  Serial.println();
}

// CIRCULAR QUEUE - FULLQ
// queue is full when the next item would overwrite the head
int fullQ()
{
  int full = (((qNext + 1) % QSIZE) == qFront);
  return full;
}

// CIRCULAR QUEUE - MTQ
// queue is empty when the head == tail
int MTQ() {
  return (qFront == qNext);
}

// CIRCULAR QUEUE - NQ
// add an intem to the end of the queue, increment the end counter mod qsize
int NQ (int item)
{
  if (!fullQ()) {
    queue[qNext] = item;
    qNext = (qNext + 1) % QSIZE;
    if (MDEBUG > 3) {
      Serial.print ("NQ: ("); Serial.print(qNext); Serial.print(") "); Serial.print (sizeQ()); Serial.print (" :");
      Serial.println (item, HEX);
    }
    return 1;
  } else {
    Serial.print ("Queue full");
    qFront = qNext = 0;
    return 0;
  }
}

// CIRCULAR QUEUE - DQ
// dequeue the item at the front of the queue, move the head pointer along one mod qsize
int DQ () {
  if (!MTQ()) {
    int j = queue [qFront];
    qFront = ((qFront + 1) % QSIZE);
    //    Serial.println(j, HEX);
    return j;
  }
  else
  {
    Serial.println ("Buffer underflow!");
    return -1;
  }
}

// CIRCULAR QUEUE - PEEKQ
// this is the reason to build a circular serial buffer queue, so you can peak at the next character without
// committing to de-queueing it.
int peekQ() {
  return queue [qFront];
}

/*
   PRINT TALLY STATE
*/
void PrintTallyState()
{
  for (int i = 0; i < CAMERAS; i++)
  {
    Serial.print(TallyStates[i] == 2 ? 'R' : 'x');
  }
  Serial.println();
}

/*
    PRINT BUTTONS STATE
     Print the current model of the active buttons and their colors on the V-1HD
*/

void PrintButtonsState()
{
  for (int i = 0; i < BANKS; i++)
  {
    for (int j = 0; j < CAMERAS; j++)
    {
      switch (ButtonStates[i][j])
      {
        case 0: {
            Serial.print('-');
          } break;
        case 1: {
            Serial.print('G');
          } break;
        case 2: {
            Serial.print('R');
          } break;
        case 3: {
            Serial.print('W');
          } break;
      }
    }
    Serial.println();
  }
}

/*
    SEND TALLY
      This function calculates the minimum number of messages to send to the Camera Controller to set the Tally lights correctly.
      It is the only fucntion that actually sends serial data to the Camera Controller.
*/
void SetTally (int bank, int cam)
{

  bool setStdby;

  // Clever trick here to determine the color of a tally, sum the camera settings value in the two banks.
  // 0 is off, 1 is green, 2 is red.

  /* If the Column Sum is:
        0: turn that tally light off
        1: turn that tally light green
        2: turn that tally light red
        3: both standby and live are aciive, sum is 3
  */

  for (int j = 0; j < CAMERAS; j++)
  {
    int colSum = ButtonStates[0][j] + ButtonStates[1][j];
    if (colSum >= TALLY_RED)
      TallyStates[j] = TALLY_RED;
    else
      TallyStates[j] = TALLY_OFF;

    ReadyStates[j] = ((colSum == 1) || (colSum == 3));
  }

  // encode the camera tally bits into a nibl and write them out to digital pins D0-D3
  // These pins are what is displayed by the four leds on the controller console, and
  // also picke up by the dtmf-generator arduino to determine which dtmf code to send to the ccu.
  ccu_code = 0;
  for (int i = CAMERAS - 1; i >= 0; i--)
  {
    bool isitOn = (TallyStates[i] > 0);
    ccu_code = ((ccu_code * 2) + isitOn);
  }

  // Write to all output values at the same time avoiding race conditions from the other Arduino reading these values
  // asynchronously.
  PORTD = (ccu_code << 3) | 4; // pin 2 is a debug port


  // Now look to see if there are any cameras an :standby, i.e. their camera light is green
  // if there is one, signal that by GS going high, and G0/G1 being the binary number of the standby camera.
  scu_code = 0;
  setStdby = false;
  digitalWrite(GS, LOW);
  for (int i = 0; i < CAMERAS; i++)
  {
    if (ReadyStates[i])   // there is a green button on the console set that camera id on the Gx pins
    {
      scu_code = i;
      setStdby = true;
      digitalWrite(GS, HIGH);
      digitalWrite(G0, scu_code & 1);
      digitalWrite(G1, scu_code >> 1);
    }
  }

  if (MDEBUG > 2)
  {
    PrintButtonsState();
    PrintTallyState();

    Serial.print ("Set Tally code: ");
    Serial.print (ccu_code, BIN);
    Serial.print(" GS-G1-G0 ");
    Serial.print(setStdby, BIN);
    Serial.print(scu_code >> 1, BIN);
    Serial.println(scu_code & 1, BIN);
  }

  LastUpdatedCCU = millis();        // note the time when we last sent a command to the ccu

}

/*
    BUTTON PRESSED
      This routine is called when we received the MIDI command Program Change with Channel 0
      We know which button has been pressed but it;s color is detemined by which video buss is active,
      which relies on control information from the fader stored in the value Selected Bank. Since both banks can be
      active at teh same time (during video transitions e.g. wipes and some effects Selected Bank can be 0, 1 or both (3).
*/
void ButtonPressed (int button, int bank)
{
  /*
    Serial.print ("Button Pressed "); Serial.print (button); Serial.print (", on bank: ");
    Serial.println(bank);
  */

  if (MDEBUG > 0)
  {
    Serial.print ("Bank "); Serial.print(bank); Serial.print (", camera: "); Serial.println (button);
  }

  //No matter which button was pressed the one that was on now has to be turned off. Do this now so we
  // know which camera was on before we overwrite it with the new info
  ButtonStates [bank][BankButton[bank]] = TALLY_OFF;
  // SetTally (bank, BankButton[bank]);


  // Whether the button pressed is red or green depends on the last fader position, which selects the bank
  if (SelectedBank == bank || SelectedBank == 2)
  {
    ButtonStates [bank][button] = TALLY_RED;
  }
  else
  {
    ButtonStates [bank][button] = TALLY_GREEN;
  }

  SetTally (bank, button);

  BankButton [bank] = button;
  if (MDEBUG > 2)
  {
    PrintButtonsState();
    PrintTallyState();
  }

}

void HandleFader (int val)
{
  /*
       HANDLE FADER
         this is the t-bar fader
       This code is somewhat convoluted because we received many io buffer overflows and lock-ups trying to update the tally each time the fader was moved.
       The solution is to divide the fader range into 3 regions. and only send Tally results when the fader enters a new region.
  */


  if (val < 3)
  { // Fader is near the top
    SelectedBank = 0;
    // bank 0 is live, bank 1 is preview
    ButtonStates [1][BankButton[1]] = TALLY_GREEN;
    SetTally (1, BankButton[1]);

    ButtonStates [0][BankButton[0]] = TALLY_RED;
    SetTally (0, BankButton[0]);

  } else

    if (val > 125)
    { // Fader is near the bottom
      SelectedBank = 1;
      ButtonStates [0][BankButton[0]] = TALLY_GREEN;
      SetTally (0, BankButton[0]);

      ButtonStates [1][BankButton[1]] = TALLY_RED;
      SetTally (1, BankButton[1]);
    } else
    {
      if (SelectedBank != 2)  // IF we haven't already sent a red light to that camera while the fader is transitioning, mark both busses as red
      {
        SelectedBank = 2;
        ButtonStates [0][BankButton[0]] = TALLY_RED;
        SetTally (0, BankButton[0]);

        ButtonStates [1][BankButton[1]] = TALLY_RED;
        SetTally (1, BankButton[1]);
      }
    }

  LastFaderVal = val;
}



/* Args:
  - type of port to use (hard/soft)
  - port object name
  - name for this midi instance
*/
MIDI_CREATE_INSTANCE(SoftwareSerial, SoftSerial, MIDI);
// The MIDI Library can be told to use the soft serial line so we can use the main one for debug print statements.

/*************************
 * ******** S E T U P ****
 * ***********************
*/
void setup()
{
  // put your setup code here, to run once

  // LED outputs
  Serial.begin(SERIALSPEED);
  Serial.println();
  Serial.println("ROLAND V-1HD TALLY LIGHT SYSTEM V1.1");

  // do I need to init the soft serial port?
  // No - MIDI Lib will do it.
  // We want to receive messages on all channels
  MIDI.begin(MIDI_CHANNEL_OMNI);

  // We also want to echo the input to the output,
  // so the sniffer can be dropped inline when things misbehave.
  MIDI.turnThruOn();

  pinMode(PIN_RAW_INPUT, INPUT_PULLUP);

  // Initialize values for the circular queue manager
  qFront = qNext = 0;

  //The ButtonStates models the buttons on the front panel of V-1HD
  for (int i = 0; i < BANKS; i++)
  {
    for (int j = 0; j < CAMERAS; j++)
    {
      ButtonStates [i][j] = TALLY_OFF;
      TallyStates[j] = TALLY_RED;         // set all leds to on to start
      ReadyStates[j] = false;             // no cameras on "ready" light (green)
    }
  }

  // Set digital pins for Camera Tally Status LEDs
  // and these are also encoded as a 4-bit number and sent as a DTMF tone to the CCU

  DDRD = DDRD | B11111000;  // this is safer as it sets pins 3 to 7 as outputs
  // without changing the value of pins 0 & 1, which are RX & TX

  PORTD = B00000100; // pin 2 is a debug port

  // Pins G0 and G1 inform the CCU which camera is on standby, i.e. it's tally light is green to reflect the console.
  pinMode(G0, OUTPUT);
  pinMode(G1, OUTPUT);
  pinMode(GS, OUTPUT);

  digitalWrite(GS, LOW);
  digitalWrite(G0, LOW);
  digitalWrite(G1, LOW);

  // Active Sensing is a short MIDI message sent periodiclly to let you know the midi device is up and running
  // if we stop hearing them then the device is off or the midi cable is disconnected
  LastHeardFrom = millis();    // heartbeat for active sensing
}

/*************************
 * ********  LOOP  *******
 * ***********************
*/
void loop()
{

  static uint8_t  ticks = 0;
  static uint8_t  old_ticks = 0;
  int cmd, siz;

  if (digitalRead(PIN_RAW_INPUT) == LOW)
  {
    // If you hold button D2 on the shield, we'll print
    // the raw hex values from the MIDI input.
    //
    // This can be useful if you need to troubleshoot issues with
    // running status

    Serial.println("Button D2 pressed on MIDI board");

    byte input;
    while (SoftSerial.available() != 0)
    {
      input = SoftSerial.read();
      if (input != midi::ActiveSensing )
        NQ (input);
    }

    int len = sizeQ();
    for (int i = 0; i < len; i++) {
      if (peekQ() & 0x80)
      {
        Serial.println();
      }

      Serial.print(DQ(), HEX);
      Serial.print(' ');
    }
  }
  else
  {

    /* Here is the main MIDI Loop for the Roland V1-HD
        We look for messages coming from the button presses and fader movement,
        anything that will affect the decision of which camera is preview and which is live.
    */
    unsigned int inp, data1, data2, bank, prog;

    // Empty the serial buffer and put it in the circular queue
    while (SoftSerial.available() != 0)
    {
      inp = SoftSerial.read();
      LastHeardFrom = millis();
      NQ (inp);
    }


    // If we havent't received an AciveSense Midi message in 2 seconds, the connection must be down
    if (millis() - LastHeardFrom > 2000)
    {
      LinkStatus  = false;
      PORTD = B00000100; // pin 2 is a debug port
      digitalWrite(GS, LOW);
      digitalWrite(G0, LOW);
      digitalWrite(G1, LOW);
    }

    // if the queue is empty, just loop back and await and character
    if (sizeQ() < 1) {
      return;
    }

    // Running Status Mode - Midi can skip a command code if its the same command..
    // http://midi.teragonaudio.com/tech/midispec/run.htm
    // Command codes are > 127 so if we receive a command code than that terminates the Running Status Mode
    if (peekQ() > 127)
    {
      inp = DQ();         // got a commmand code, so rs is finished
      RunningStatus = 0;
    }
    else
    {
      inp = RunningStatus;  // this looks like a data byte so that's the sign we are in Running Status mode, set the input as if we received that command again
    }


    // System Exclusive are unique commands to this make and model of Midi device. Our appraoch at this point is to toss them away as quickly as possible as they
    // come from the Roland V-1HD very quickly when it is speaking with the Mac App
    // One day it would be good to decode the SysEx message and use it to the set the tally lights
    if (inSysEx)
      inp = midi::SystemExclusive;

    // Now look for commands, and execute the appropriate MIDI response.
    switch (inp) {

      // Control Change is basically a knob or button that changes value
      case midi::ControlChange :
        {

          if (sizeQ() < 2)
          {
            // Control Change has two data bytes, and this message might get broken across multiple loops
            // (In testing this was indeed true, hence this kludge)
            // If we don't have two characters queud at this point for the Control Change message, avoid buffer underflow, go back and get more characters
          } else
          {

            // have two data bytes, we can go ahead and process this Control Change MIDI message
            data1 = DQ();
            data2 = DQ();

            if (RunningStatus != 0) Serial.print("(");
            Serial.print("Controller, chan: ");
            Serial.print(inp & 0xF);
            Serial.print(" Controller#: ");
            Serial.print(data1);
            Serial.print(" Value: ");
            Serial.println(data2);

            // Fader Bar Handling
            int controller = data1;
            int value = data2;

            // Controller 17 is the t-bar fader, so as it transitions video frm one back
            // to another you need to update the tally lights
            if (controller == 17) {
              //           Serial.print('.');
              HandleFader (value);
            }

            // Controller 0 is a meta control that indicates which bank the next ProgramChange commeand is from
            // Button Press Handling, ie the Program Change button that is about to arrive tells us which of the 4
            // buttons was pressed, but THIS message tell us whether it is upper or lower bank of buttons
            if (controller == 0)
            {
              PressedBank = value;
            }

            // Controller 13 are the Mix/Wipe/Cut buttons are
            if (controller == 13)
            {
              VideoTransition = value;
            }
          }

          // prepare for running status by setting the RunningStatus variable to the last know status, this one
          RunningStatus = midi::ControlChange;
        }
        break;
      case midi::ProgramChange :
        {
          // The COntrol Changed message told us which bank the button was pressed, the Program change message tells
          // us which f the 4 buttons it was..

          prog = DQ();


          Serial.print("PropChange, chan: ");
          Serial.print(inp & 0xF);
          Serial.print(" program: ");
          Serial.println(prog & 0x7);


          if ((inp & 0xF) == 0)     // channel is 0
            /*
               So we now know everything we need to know which button is pressed on which bank
               Which color it is is determined by the slider position
               We also have to turn off the camera that was on before this was button was pushed
              // this array by bank number tells which button is active
              // 0 0 R 0  BankButton [0] = 2;
              // 0 0 0 G  BankButton [1] = 3;
            */
            ButtonPressed (prog, PressedBank);

          RunningStatus = midi::ProgramChange;
          //          Serial.println("RunningStatus Program Change");
        }
        break;
      //      case midi::SystemExclusive :
      case midi::SystemExclusive :
        {
          if (inSysEx == false)
          {
            inSysEx = true;
            Serial.println("Start Sysex");
          }

          while (sizeQ() > 0) {
            unsigned c;
            Serial.print (c = DQ(), HEX); Serial.print (" ");
            if (c == 0xf7)
            {
              inSysEx = false;
              Serial.println("SysEx end");
              break;
            }
          }
        }
        break;
      case midi::ActiveSensing :
        {
          // The ROLAND V-1HD sends an Active Sensing message every 250ms when there is otherwise no other MIDI traffic
        }
        break;

      default: {
          Serial.print("Midi case default "); Serial.println (inp, HEX);
          // reset
          wdt_disable();
          wdt_enable(WDTO_15MS);
          while (1) {}
        }
    }
  }
}
