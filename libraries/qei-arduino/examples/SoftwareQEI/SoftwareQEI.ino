
#include <SoftwareQEI.h>

SoftwareQEI qei(6, 7);

void setup() {
  Serial.begin(9600);
  qei.reset();
}

void loop() {
  delay(1000);
  Serial.println(qei.read());
  qei.reset();
}
