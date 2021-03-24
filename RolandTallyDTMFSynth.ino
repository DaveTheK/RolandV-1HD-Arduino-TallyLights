// Arduino synth library modulation example
// Library by DZL:  https://github.com/dzlonline/the_synth
//
// Arduino Uno Hardware Connections:
//
//                        +10µF
// PIN 11 ---[ 1kohm ]--+---||--->> Audio out
//                      |
//                     === 10nF
//                      |
//                     GND
//
// This demo uses the_synth library to generate two simultaneous tones
// in frequency pairs that represent various DTMF combinations including
// North American touch tone keypad tones and other call progress tones.
//
//     DTMF keypad frequencies
//             1209Hz 1336Hz 1477Hz 1633Hz
//      697Hz    1      2      3      A
//      770Hz    4      5      6      B
//      852Hz    7      8      9      C
//      941Hz    *      0      #      D
//
//  DTMF keypad optimized function provided by Tengelgeer
//
//  Gadget Reboot

/*
   ROLAND TALLY DTMF SYNTH

   This Arduino monitors the 4 data values calculated by the Arduino attached to teh Roland V-1HD's MIDI output
   and then sends a synthesized DTMF tone of that value to the CCU, where an MT8870 DTMF decoder turns on the appropriate Tally light.
   The Low Pass Filter above is used to limit the bandwith (to approx 16KHz) 1/2piRC and to remove the DC bias that comes from Arduino PWM output.

   The DTMF Synth watches the 4 parallel lines from the MIDI-interfaces Arduino to see if they have changed.
   If they have, then it updates the remote Camera Control Unit CCU within onTime+offTime milliseconds.

   
  read from pins 3,4,5,6, num is a binary map of which of the red leds are on:
    R--- -R-- --R- ---R              // 4!1 possibilities for when just 1 LED is red      1, 2, 4, 8
    RR-- R-R- R--R -RR- -R-R --RR    // 4!2 possibilities = 6 for when 2 LEDs are red     3, 5, 9, 6, 10, 12
    ---- // all off                  0  // all off
    RRRR // all on                   15 // avoid this if the NEOPIXLEL Leds strips are powered solely through the Arduino as this will exceed its power limit and reset
  
  The codes above use 4+6+1+1 = 12 codes of the possible 16 DTM tones, leaving 4 more to use for signalling the standby (green) lights, which can only be 1 on at a time. 
  We send this second tone (standby) to tell teh CCU to turn that camera's tally to green, with one exception if the tally light is already red.

*/


#include <synth.h>
#include <Timer.h>

#define MDEBUG 0

// status lines in parallele with console LEDs - indicate which camera(s) tally lights are red 
// There can be 1 or 2 cameras on a time e.g. during transitions
#define D0  3
#define D1  4
#define D2  5
#define D3  6

// status lines for green led - GS is high if G0/G1 are valid and we should signal the CCU to turn on a green tally.
#define G0  8
#define G1  9
#define GS  10

static int num = 0;     // the bit pattern that represents which cameras have red tally
static int grn = 0;     // the number of the camera that can have its standby light on (green)
static int lastnum = 0; 
static int lastgrn = 0;
int grncodes[] = {7, 11, 13, 14};   // These four codes are made up of 3 on-bits so reserved for the standby leds. 7 - camera 1, 11 camera 2, 13 camera 3, 14 camera 4.
char Tally[4] = {'-', '-', '-', '-'};

// Set a timer to continously be able to send a dtmf code as quickly as possible frequency is based on onTime plus offTime.
Timer t;
int DTMFOnEvent;

// Synthesizer to create the DTMF tones
synth pwmSynth;    // create pwmSynth object
int onTime = 40;        // time in mS to keep playing DTMF tones
int offTime = 40;       // time in mS to stay silent after playing a DTMF tone



// SETUP
void setup() {
  Serial.begin(9600);

  // init synth with sine wave audio
  pwmSynth.begin();
  pwmSynth.setupVoice(0, SINE, 60, ENVELOPE1, 127, 64); //-Set up voice 0
  pwmSynth.setupVoice(1, SINE, 60, ENVELOPE1, 127, 64); //-Set up voice 1

  Serial.println("DTMF Generator\n");

  // The pins 3-6 are the same as the LED values on the main console
  pinMode(D0, INPUT);
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  pinMode(D3, INPUT);

  // These pins determine if a camera should have it's green (standby) light on
  pinMode(G0, INPUT);
  pinMode(G1, INPUT);
  pinMode(GS, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  // Set the time to repeat at the highest frequency we can send ftmf tones.
  DTMFOnEvent = t.every(onTime + offTime, SetDTMF);
}

// Send a keypad tone for onTime and wait for offTime.
// There's nothing else this device can do in the meantime so delay is ok to use.
void setTone(int code)
{
  keypadTone (code);
  delay (onTime);
  // turn off the synth sounds by setting frequency to zero.
  pwmSynth.setFrequency(0, 0);
  pwmSynth.setFrequency(1, 0);
  delay(offTime);
}


void SetDTMF()
/*
   SetDTMF is a timer routine that starts the DTMF tone based on the varaibla read into to PORTD
*/
{
  bool StandbyLED;

  // Using the Arduio Port read command, read all 3 bits at once
  num = (PIND >> 3) & 0xf;
  
  // these two pins tell us which camera is in standby so its tally light might be green.
  grn = digitalRead(G1) * 2 + digitalRead(G0);

  // StandbyLED is true if there is a green button showing on the Roland V-HD console.  We send an additional code to the CCU if we want that camera to show green
  // The only exception is when a camera is both green and red, in which we only light the red led, and avoid sending the command to turn it to green.
  StandbyLED = digitalRead(GS);

  if ((num != lastnum) || (grn != lastgrn))   // some value has changed so update the tally system
  {
    Tally[0] = (num & 1) ? 'R' : (grn == 0) ? 'G' : '-';
    Tally[1] = ((num / 2) & 1 ? 'R' : (grn == 1) ? 'G' : '-');
    Tally[2] = ((num / 4) & 1 ? 'R' : (grn == 2) ? 'G' : '-');
    Tally[3] = ((num / 8) & 1 ? 'R' : (grn == 3) ? 'G' : '-');

    for (int i = 0; i < 4; i++)
    {
      Serial.print(Tally[i]);
    }
    Serial.println();
  }

  digitalWrite(LED_BUILTIN, HIGH);

  // Send this tone if something has changed
  if ((num != lastnum)  || (lastgrn != grn))
  {
      setTone(num);
      Serial.print ("Set code: ");
      Serial.println (num);
      lastnum = num;

    // IF A CAMERA IS RED AND GREEN JUST MAKE IT RED IE DONT SEND THE STDBY CODE
    if ((StandbyLED) && (Tally[grn] != 'R'))
    {
      setTone (grncodes[grn]);        // we use unused dtmf tones 7, 11, 13, and 14 to signal which camera tally should be standby mode.
      Serial.print ("Set stby: ");
      Serial.println (grncodes[grn]);
    }
    lastgrn = grn;
  }
  digitalWrite(LED_BUILTIN, LOW);
}

// LOOP
void loop()
{
  // we are just timer driven, so just update the timer
  t.update();
}


// keypad row/column frequency selection functions by Tengelgeer
// play a tone identified as an integer 0..15
// key order 0123456789*#ABCD (0 = 0, D = 15)
// binary order D1234567890*#ABC

void keypadTone(int n) {
  const unsigned int ColumnFrequencies[] = {1209, 1336, 1477, 1633};
  const unsigned int RowFrequencies[] = {697, 770, 852, 941};
  //     DTMF keypad frequencies
  //     Column    0      1      2      3
  //             1209Hz 1336Hz 1477Hz 1633Hz
  //      697Hz    1      2      3      A     Row 0
  //      770Hz    4      5      6      B     Row 1
  //      852Hz    7      8      9      C     Row 2
  //      941Hz    *      0      #      D     Row 3

  const unsigned int RowCols[16][2] = {   // this array orders the keys in binary order and selects the index into the row/column arrays.
    {3, 3}, //0 "D"
    {0, 0}, //1 "1"
    {0, 1}, //2 "2"
    {0, 2}, //3 "3"
    {1, 0}, //4 "4"
    {1, 1}, //5 "5"
    {1, 2}, //6 "6"
    {2, 0}, //7 "7"
    {2, 1}, //8 "8"
    {2, 2}, //9 "9"
    {3, 1}, //10 "0"
    {3, 0}, //11 "*"
    {3, 2}, //12 "#"
    {0, 3}, //13 "A"
    {1, 3}, //14 "B"
    {2, 3}  //15 "C"
  };

  pwmSynth.setFrequency(0, (float)ColumnFrequencies[RowCols[n][1]]);
  pwmSynth.setFrequency(1, (float)RowFrequencies[RowCols[n][0]]);

  pwmSynth.trigger(0);
  pwmSynth.trigger(1);
}
