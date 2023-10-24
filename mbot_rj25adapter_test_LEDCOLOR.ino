#include <Wire.h>
#include <MeMCore.h>
#include <MeRGBLed.h>
#define LED_PIN         7
// Define colour sensor LED pins
int ledArray[] = { A2, A3 };
int truth[][2] = {{0,0},{0,1},{1,0},{1,1}};

MeRGBLed                led(LED_PIN);
void setup() {
  // put your setup code here, to run once:
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // A2 LOW GIVES RED
  // A2 HIGH GIVES GREEN
  digitalWrite(A2, HIGH);
  digitalWrite(A3, LOW);
  delay(1000);
  digitalWrite(A2, LOW);
  digitalWrite(A3, HIGH);
  delay(1000);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  delay(1000);
  // digitalWrite(A2, LOW);
  // digitalWrite(A3, LOW);
  // delay(1000);
}
