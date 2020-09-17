
#include <LockedAntiphaseMD.h>

#define PWM_PIN     9   // Connect to DIR pin
#define ENABLE_PIN  8   // Connect to PWM pin

LockedAntiphaseMD md(PWM_PIN);
int i = 0;

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  md.free();

  digitalWrite(ENABLE_PIN, HIGH);
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
