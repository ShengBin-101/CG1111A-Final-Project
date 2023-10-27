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
  pinMode(A0, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  // for (int c = 0; c < 2; c++)
  // {
  //   // go through each port (c = 0 => A2, c = 1 => A3)
  //   // A2 port's state will correspond with truth[x][0]
  //   // A3 port's state will correspond with truth[x][1]
  // }
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  Serial.println("00");
  delay(5000);

  digitalWrite(A2, LOW);
  digitalWrite(A3, HIGH);
    Serial.println("01");
  delay(5000);

  digitalWrite(A2, HIGH);
  digitalWrite(A3, LOW);
    Serial.println("10");
  delay(5000);

  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  Serial.println("11");
  delay(5000);
}
