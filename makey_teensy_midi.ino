int bounceThreshold = 0; // a press must last this many loop cycles (about 20ms each) before triggering. 
// higher values make it less sensitive, fewer false triggers, higher latency

#define NUM_INPUTS 44

int channel = 1;

// edit this array to change the MIDI notes sent 
//int pitches[NUM_INPUTS] = {
//  79, 84, 75, 67, 65, 63,           // top of board (up, left, down, right, space, click)
//  60, 62, 77, 68, 58, 56,           // left side of female header, w,a,s,d,f,g
//  51, 53, 48, 55, 72, 74            // right side of female header, up,down,left,right,left-click,right-click
//};

// teensy makey pin numbers
int pinNumbers[NUM_INPUTS] = {
  0,1,2,3,4,5,7,8,9,10,11,12,13,14,15,16,17,         // left side starting at USB connector, pin D0, skipping D6 
  26,25,24,23,22,21,20,19,18,38,39,40,41,42,43,44,45, // right side starting at USB connector, pin B6, skipping gnd and aref
  36,37,32,33,34,35,28,29,30,31                      // these are "interior" through-holes, in the center of the board
};

// cap sense thresholds for each pin
// this is proportional to the capacitance on the pin that will count as a press
// it is units of a very small unit of time, in iterations through an unrolled loop
// higher values make it less sensitive (i.e. require larger capacitance)
//int capThresholds[NUM_INPUTS] = {
//  1, 1, 1, 1, 1, 2,
//  2, 2, 1, 1, 1, 1,
//  2, 1, 1, 1, 1, 1,
//};

int bounceCounter[NUM_INPUTS];    
boolean pressed[NUM_INPUTS];

void setup(){
  for (int i=0; i<NUM_INPUTS; i++) {
    bounceCounter[i] = 0;
    pressed[i] = false;
  }
}

void loop() { 
  for (int i=0; i<NUM_INPUTS; i++) {                      // for each pin
    int c = readCapacitivePin(pinNumbers[i]);             // check capacitance
//    Serial.print(c);
//    Serial.print("\t");
    if (c>1){                                                       // if we detect a touch on the pin
      if (!pressed[i]) {                                          // and if we're not already pressed
        bounceCounter[i]++;                                           // increment the bounce counter
        if(bounceCounter[i] > bounceThreshold){                       // if we're over the bounce threshold
          usbMIDI.sendNoteOn(60+i,127,channel);                      // send a MIDI note on
          pressed[i] = true;                                              // remember it was pressed
          bounceCounter[i]=0;                                             // reset the bounce counter
        }
      }
    } 
    else {                                                  // if we don't a detect touch on the pin
      if (pressed[i]) {                                           // if this key was pressed before
        usbMIDI.sendNoteOff(60+i,127,channel);                    // send a MIDI note off
        pressed[i] = false;                                          // remember we are not pressed
        bounceCounter[i] = 0;                                        // reset the bounce counter
      }        
    }
  }
  //Serial.println(" ");
}  




// CapacitiveSensor tutorial from http://www.arduino.cc/playground/Code/CapacitiveSensor
// readCapacitivePin
//  Input: Arduino pin number
//  Output: A number, from 0 to 17 expressing
//  how much capacitance is on the pin
//  When you touch the pin, or whatever you have
//  attached to it, the number will get higher

uint8_t readCapacitivePin(int pinToMeasure) {
  // Variables used to translate from Arduino to AVR pin naming
  volatile uint8_t* port;
  volatile uint8_t* ddr;
  volatile uint8_t* pin;
  // Here we translate the input pin number from
  //  Arduino pin number to the AVR PORT, PIN, DDR,
  //  and which bit of those registers we care about.
  byte bitmask;
  port = portOutputRegister(digitalPinToPort(pinToMeasure));
  ddr = portModeRegister(digitalPinToPort(pinToMeasure));
  bitmask = digitalPinToBitMask(pinToMeasure);
  pin = portInputRegister(digitalPinToPort(pinToMeasure));
  // Discharge the pin first by setting it low and output
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  delay(1);
  // Make the pin an input with the internal pull-up on
  *ddr &= ~(bitmask);
  *port |= bitmask;

  // Now see how long the pin to get pulled up. This manual unrolling of the loop
  // decreases the number of hardware cycles between each read of the pin,
  // thus increasing sensitivity.
  uint8_t cycles = 17;
  if (*pin & bitmask) { 
    cycles =  0;
  }
  else if (*pin & bitmask) { 
    cycles =  1;
  }
  else if (*pin & bitmask) { 
    cycles =  2;
  }
  else if (*pin & bitmask) { 
    cycles =  3;
  }
  else if (*pin & bitmask) { 
    cycles =  4;
  }
  else if (*pin & bitmask) { 
    cycles =  5;
  }
  else if (*pin & bitmask) { 
    cycles =  6;
  }
  else if (*pin & bitmask) { 
    cycles =  7;
  }
  else if (*pin & bitmask) { 
    cycles =  8;
  }
  else if (*pin & bitmask) { 
    cycles =  9;
  }
  else if (*pin & bitmask) { 
    cycles = 10;
  }
  else if (*pin & bitmask) { 
    cycles = 11;
  }
  else if (*pin & bitmask) { 
    cycles = 12;
  }
  else if (*pin & bitmask) { 
    cycles = 13;
  }
  else if (*pin & bitmask) { 
    cycles = 14;
  }
  else if (*pin & bitmask) { 
    cycles = 15;
  }
  else if (*pin & bitmask) { 
    cycles = 16;
  }

  // Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to 
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors.
  *port &= ~(bitmask);
  *ddr  |= bitmask;

  return cycles;
}



