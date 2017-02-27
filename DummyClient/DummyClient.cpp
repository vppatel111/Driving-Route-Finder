#include <Arduino.h>

int main() {

  init();

  Serial.begin(9600);
  Serial.println("Starting...");
  Serial.flush();    // There can be nasty leftover bits.

    // Hitting server with dummy requests
    delay(1000);

    // Test case 1
    Serial.println("R 5365486 -11333915 5364728 -11335891");

    for (int16_t i=0; i < 9; i++) {
        Serial.println("A");
        delay(10);
      }

    delay(1000);

    // Test case 2
    Serial.println("R 5353683 -11350844 5352487 -11345827");

    for (int16_t i=0; i < 92; i++) {
        Serial.println("A");
        delay(10);
      }

    delay(1000);

    // Test case 3
    Serial.println("R 5365486 -11333915 5365486 -11333915");

    for (int16_t i=0; i < 2; i++) {
        Serial.println("A");
        delay(10);
      }

    delay(1000);

  return 0;
}
