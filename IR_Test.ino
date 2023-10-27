#define IR A1   //IR R sensor pin at A0
#define LED 13  //Check Indicator to signal Calibration Completed

// Define colour sensor LED pins
int ledArray[] = { A2, A3 };
int truth[][2] = {{0,0},{1,1}};

void setup() {
  // put your setup code here, to run once:
  for (int i=0;i<2;i++){
    pinMode(ledArray[i],OUTPUT);
  }
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(ledArray[0],LOW);
  digitalWrite(ledArray[1],LOW);
  Serial.println(analogRead(IR));
  delay(1000);
  digitalWrite(ledArray[0],HIGH);
  digitalWrite(ledArray[1],HIGH);
  Serial.println(analogRead(IR));
  delay(1000);


}