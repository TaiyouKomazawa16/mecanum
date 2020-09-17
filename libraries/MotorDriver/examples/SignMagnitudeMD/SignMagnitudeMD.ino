
#include <SignMagnitudeMD.h>

#define PWM_PIN   9
#define DIR_PIN   8

SignMagnitudeMD md(DIR_PIN, PWM_PIN);
int i = 0;

void setup() {
  md.free();
}

void loop() {
  while (i <= 255) {
    md.drive(i);
    delay(10);
    i++;
  }
  while (i >= -255) {
    md.drive(i);
    delay(10);
    i--;
  }
}
