#include "Wire.h"
#include "MeMCore.h"
#include "MeRGBLed.h"
#include "Notes.h"

/********** Settings **********/

#define MUSIC_PIN         8
// #define ULTRASONIC        12
MeDCMotor                 leftWheel(M1); // assigning leftMotor to port M1
MeDCMotor                 rightWheel(M2); // assigning RightMotor to port M2
MeLineFollower            lineFinder(PORT_2);
MeUltrasonicSensor        ultraSensor(PORT_1);  

MeRGBLed                  led(0,30);
MeBuzzer                  buzzer;

// Ultrasound
#define TIMEOUT           2000 // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND    340 // Update according to your own experimen

// Movement
#define MOTORSPEED        230
#define TURNING_TIME_MS   (75000/MOTORSPEED) // The time duration (ms) for turning 90 degrees
#define TIMEDELAY         20 // delay time before checking colour of waypoint
#define D_FRONT           10 // side distance threshold in cm
#define TIME_FOR_1_GRID   (180000/MOTORSPEED) // TO BE TESTED

// Color
// to be updated from colourcalibration.ino file after calibration
/*
#define COL_DIST        5000                    // 10000
#define WHI_VAL         {375, 335, 380}         // from LDR b4 normalisation
#define BLA_VAL         {255, 217, 243}
#define GRE_VAL         {116, 108, 130}

#define RED_ARR         {185,35,35}             // normalised rgb vals
#define GRE_ARR         {45, 100, 60}
#define YEL_ARR         {255, 175, 100}         //325,230,135
#define PUR_ARR         {155,150,200}
#define BLU_ARR         {175,240,240}
#define BLA_ARR         {0,0,0}
#define NUMCOL          6                       // black, red, green, yellow, purple, blue

// Calibration
#define CALLIBRATE_SEC  3                       // delay b4 calibration
#define COLOUR_NO       50                      // 50
#define SOUND_NO        50                      // no of measurements
#define IR_WAIT         100                     // delay btw measurements. IMPT!
#define RGB_WAIT        100                     // 200
#define LDR_WAIT        10
#define MIC_WAIT        100
#define LED_MAX         255
#define IR_MAX          1023

*/

/********** Global Variables **********/
bool stop = false;
bool status = false; // global status; 0 = do nothing, 1 = mBot runs

void setup()
{
  // Any setup code here runs only once:
  led.setpin(13);
  pinMode(MUSIC_PIN, OUTPUT);
  Serial.begin(9600);
  pinMode(A7, INPUT); // Setup A7 as input for the push button
  led.setColorAt(0, 0, 255, 0); // set Right LED to Red
  led.setColorAt(1, 0, 255, 0); // set Left LED to Blue
  led.show();
  delay(2000); // Do nothing for 10000 ms = 10 seconds
}

void stopMove(const int i);
void turnLeft(void);
void turnRight(void);
void moveForward(void);

void loop()
{
  if (status == true) { // run mBot only if status is 1
    int sensorState = lineFinder.readSensors(); // read the line sensor's state
    if (sensorState != S1_IN_S2_IN) { // situation 1
      led.setColor(255, 255, 255);
      led.show();
      moveForward();

      Serial.print("Distance : ");
      Serial.print(ultraSensor.distanceCm() );
      Serial.println(" cm");
      if (ultraSensor.distanceCm() < 7)
      {
        stop = true;
        status = false;
      }
    }
    else{
      stop = true;
      status = false;
    }
  }
  else{
    if (stop == true)
    {
      led.setColorAt(0, 255, 0, 0); // set Right LED to Red
      led.setColorAt(1, 0, 0, 255); // set Left LED to Blue
      led.show();
      stopMove(0);
      finishWaypoint();
      stop = false;
    }
    
    if (analogRead(A7) < 100) { // If push button is pushed
      status = !status; // Toggle status
      delay(500); // Delay 500ms so that a button push won't be counted multiple times.
    }
  delay(1);
  }  
}

void stopMove(const int i = 100) {
  rightWheel.stop();
  leftWheel.stop();
  if (i) delay(TIMEDELAY * i);
}

void moveForward() {
  // if (ultraSensor.distanceCm() < 3) return;
  leftWheel.run(-MOTORSPEED);
  rightWheel.run(MOTORSPEED);
  delay(1);
  // stopMove();
}

void forwardGrid() {
  // for (int i = 0; i < TIME_FOR_1_GRID; ++i) { 
  //   if (ultraSensor.distanceCm() < D_FRONT) break;
    leftWheel.run(-MOTORSPEED);
    rightWheel.run(MOTORSPEED);
    delay(TIMEDELAY*100);
  
  stopMove();
}

void turnRight() {
  leftWheel.run(-MOTORSPEED);
  rightWheel.run(-MOTORSPEED);
  delay(TURNING_TIME_MS);
  stopMove();
}

void turnLeft() {
  leftWheel.run(MOTORSPEED);
  rightWheel.run(MOTORSPEED);
  delay(TURNING_TIME_MS);
  stopMove();
}

void doubleRight() {
  turnRight();
  forwardGrid();
  turnRight();
}

void doubleLeft() {
  turnLeft();
  forwardGrid();
  turnLeft();
}

void uTurn() {
  turnRight();
  turnRight();
}



void finishWaypoint() {
  // keys and durations found in NOTES.h
  for (int i = 0; i < sizeof(music_key) / sizeof(int); ++i) {
    // quarter note = 1000 / 4, eighth note = 1000/8, etc. (Assuming 1 beat per sec)
    const int duration = 1000 / music_duration[i];
    buzzer.tone(MUSIC_PIN, music_key[i], duration);

    // to distinguish notes
    delay(duration * 1.30);
    buzzer.noTone(MUSIC_PIN);
  }
}