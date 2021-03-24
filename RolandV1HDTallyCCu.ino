#include <Adafruit_NeoPixel.h>
#include <Wire.h>


/*
   This code is for the Arduino in the Roland V-1HD Tally Light Camera Control Unit CCU

   The CCU receives DTM commands over the mic line from the Main unit and sets the appropriate tally lights for each camera, 
   Code assumes the Tally Lights are Adafruit NEOPIXEL devices

   Note the model here is built on 4 15-pixel neopixel strips. Lighting them all with full brightness exceeds the power of the Arduino, so 
   make sure to supplement the neopixel power.

   DTMF Code thanks to Gadget Reboot!
   https://github.com/GadgetReboot/MT8870_DTMF
   https://youtu.be/Wx6C4k_xxz0

   Pinout:

   Pin 2-6    from MT8870 Decoder board
   Pin 8-11   controls the 4 adafruit neipoxel strips for the camera tally lights
   Uses interrupt on pin 2
   
*/

// Using a Mitel MT8870 board, it sets STQ high when it detects a valid digital, then read its code from D0-D3. 
#define STQ 2
#define D0 3
#define D1 4
#define D2 5
#define D3 6

// The board builtin LED is echoed out to the panel so you can tell the Arduino has power
#define ALED 13

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN    8

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 15

// Declare our NeoPixel strip objects, we have one strip per camera
// Pins 8, 9, 10 and 11 control neopixels
Adafruit_NeoPixel strip0(LED_COUNT, LED_PIN, NEO_GRB);
Adafruit_NeoPixel strip1(LED_COUNT, LED_PIN + 1, NEO_GRB);
Adafruit_NeoPixel strip2(LED_COUNT, LED_PIN + 2, NEO_GRB);
Adafruit_NeoPixel strip3(LED_COUNT, LED_PIN + 3, NEO_GRB);

// Create an array of neopixel controllers so we can loop through them.
Adafruit_NeoPixel nps[] = {strip0, strip1, strip2, strip3};
bool cameraOn[] = {false, false, false, false};

// preset some colors in advance
uint32_t colourRed, colourGreen, colourBlue, colourWhite;

// this is an interrupt variable indicating a new DTMF tone has been located/received
// a variable is marked volatile to tell the compiler not to assume its state and try to optimize it, since the isr can set this variable asynchronously.
// e.g. an optimizing compiler might say "toneLoc" will never be true since newDTMF is never "called" and decide to take out all the code in the loop!
volatile bool toneLoc = false;

// Interrupt Sevice Routine
// isr to set data received flag upon interrupt. While this could be just tested each time through the loop, it's good to know about interrupts!
void newDTMF() {
  // just set this variable to true and we wil pick it up next time trough the loop
  toneLoc = true;
}

void setStripColour (Adafruit_NeoPixel *strip, uint32_t col)
{
  for (int i = 0; i < LED_COUNT; i++)
  {
    strip->setPixelColor(i, col);
  }
  strip->show();
}


void setup() {

  // Turn Power LED ON
  pinMode (LED_BUILTIN, OUTPUT);
  digitalWrite (LED_BUILTIN, HIGH);

// set the pins coming from the MT8870 board as input so we can read status and which dtmf tone arrived.
  pinMode(D0, INPUT);
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  pinMode(D3, INPUT);
  pinMode(STQ, INPUT);

  // if you are not familiar with interrupts, this is a command that runs the function newDTMF whenever the 8870 raises the STQ voltage signallng
  // a new DTMF tone has been decoded.
  // run isr to set data received flag when interrupt occurs on pin 2
  attachInterrupt(digitalPinToInterrupt(STQ), newDTMF, RISING);

  colourBlue = strip0.Color (0, 0, 8);      // make blue low intensity
  colourGreen = strip0.Color (0, 8, 0);     // make green low intensity
  colourRed = strip0.Color (255, 0, 0);     // make red high intensity
  colourWhite = strip0.Color (16, 16, 16);  // make white med intensity

  Serial.begin(9600);
  Serial.println("ROLAND V-1HD Camera Control Unit");

  for (int i = 0; i < 4; i++)
  {
    // initialize each neopixel strip
    nps[i].begin();
    nps[i].show();
    nps[i].setBrightness(250);
  }

  for (int i = 0; i < 4; i++)
  {
    nps[i].clear();         //   Set all pixels in RAM to 0 (off)
    for (int j = 0; j < LED_COUNT; j++)
    {
      // initialize all strips to aqua/cyan
      uint32_t colour = strip0.Color (0, 16, 16);
      nps[i].setPixelColor (j, colour);
      nps[i].show();
      delay (25);
    }
  }
}

void loop() {

  uint32_t colour;
  int val;

  // We wait until the 8870 detects a tone, which creates an interrupt which then sets this variable toneLoc to true;
  if (toneLoc)
  {
    digitalWrite (ALED, LOW);
    toneLoc = false;

    cameraOn[0] = digitalRead(D3);     // camera 1
    cameraOn[1] = digitalRead(D2);     // camera 2
    cameraOn[2] = digitalRead(D1);     // camera 3
    cameraOn[3] = digitalRead(D0);     // camera 4

    // compose val into a value from 0 to 15 so we can determine if this is a red tally command or a green tally command
    val = (cameraOn[3] << 3 | cameraOn[2] << 2 | cameraOn[1] << 1 | cameraOn[0]);

    // The special values 7, 11, 13, and 14 are used to set the green standby tally
    if ((val == 7) || (val == 11) || (val == 13) || (val == 14))
    {
      switch (val) {
        case 7: {
            setStripColour (&nps[0], colourGreen);
          } break;
        case 11: {
            setStripColour (&nps[1], colourGreen);
          } break;
        case 13: {
            setStripColour (&nps[2], colourGreen);
          } break;
        case 14: {
            setStripColour (&nps[3], colourGreen);
          } break;
      }
    }
    else
      for (int i = 4; i >= 0; i--)
      // some weird behavior in the neopixel timimg made it more reliable to go this direction. don't know why.
      {
        setStripColour (&nps[i], cameraOn[i] ? colourRed : colourBlue);
      }

    digitalWrite (ALED, HIGH);

  }
}
