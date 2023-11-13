/* Encoder Knob for laser Cutter
    by: Gal Arbel, 2023
*/

#include "lasercutter.h"

Lasercutter mylaser(
8, //step
9, //dir
10, //encoderA
11, //encoderB
5, // times (how many pulses the stepper gets)
700);  //stepper speed (delay in uS)

void setup() {
  mylaser.begin(115200);
   
}
void loop() {
  mylaser.run();
}

