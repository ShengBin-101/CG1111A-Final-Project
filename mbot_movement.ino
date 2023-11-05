#include "Wire.h"
#include "MeMCore.h"
#include "MeRGBLed.h"

/********** Define Ports **********/
#define MUSIC_PIN         8
#define ULTRASONIC        12
#define LDR               A0
MeDCMotor                 leftWheel(M1);        // assigning LeftMotor to port M1
MeDCMotor                 rightWheel(M2);       // assigning RightMotor to port M2
MeLineFollower            lineFinder(PORT_2);
MeUltrasonicSensor        ultraSensor(PORT_1);  
MeRGBLed                  led(0,30);
MeBuzzer                  buzzer;
// Define time delay before the next RGB colour turns ON to allow LDR to stabilize
#define RGBWait 500 //in milliseconds 

// Define time delay before taking another LDR reading
#define LDRWait 20 //in milliseconds 
/********** Constants **********/
// Ultrasound
#define TIMEOUT           2000 // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND    340 // Update according to your own experimen
#define OUT_OF_RANGE      100

// Movement
#define MOTORSPEED        255
#define TURNING_TIME_MS   (75000/MOTORSPEED) // The time duration (ms) for turning 90 degrees
#define TIMEDELAY         20 // delay time before checking colour of waypoint
#define SIDE_MAX          10 // side distance threshold in cm
#define TIME_FOR_1_GRID   (180000/MOTORSPEED) // TO BE TESTED

/********** Variables for PID Controller **********/
const double kp = 30;            //   - For P component of PID
 const double ku = 100;          //  - For D component of PID
 const double tu = 10.5 / 17;    //  - For D component of PID
 const double kd = 0.1 * ku * tu;//  - For D component of PID
int L_motorSpeed;
int R_motorSpeed;

const double desired_dist = 10.5; // desired distance between ultrasound sensor and wall to keep mBot centered in tile
double error;
double correction_dble; //  - For calculation for correction
int correction;         //  - To be used as input to motor
double prev_error = 0;  // - For D component of PID
double error_delta;    // - For D component of PID

/********** Color Detection Parameters - To be updated from coloudcalibration.ino file after calibration **********/
// Define colour sensor LED pins
int ledArray[] = { A2, A3 };
int truth[][2] = { { 0, 1 }, { 1, 0 }, { 1, 1 } };

char colourStr[3][5] = {"B = ", "G = ", "R = "};

int color; // colour detected [0 - white, 1 - red, 2 - blue, 3 - green, 4 - orange, 5 - purple, 6 - black]

int blue = 0;
int green = 0;
int red = 0;

//floats to hold colour arrays
float colourArray[] = {0,0,0};
float whiteArray[] = {893.00,941.00,714.00};
float blackArray[] = {412.00,624.00,394.00};
float greyDiff[] = {481.00,317.00,320.00};
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

Calibration
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
bool stop = false;    // global ; 0 = haven't reach end of maze, 1 = reach end of maze, stop all motor and play buzzer
bool status = false;  // global status; 0 = do nothing, 1 = mBot runs
int sensorState;      // to keep track of whether waypoint is detected (ie. black line detected by line follower)
double dist;          // to keep track of distance between wall and ultrasound sensor

/********** Function Declarations **********/
void stopMove(const int i);
void turnLeft(void);
void turnRight(void);
void moveForward(void);
void finishWaypoint(void);


/********** Setup & Loop **********/
void setup()
{
  // Any setup code here runs only once:
  led.setpin(13);
  pinMode(MUSIC_PIN, OUTPUT);
  Serial.begin(9600);
  // pinMode(A7, INPUT); // Setup A7 as input for the push button
  led.setColor(128, 255, 0); // set LED to Green

  led.show();
  for(int c = 0;c < 2;c++){
    pinMode(ledArray[c],OUTPUT);  
  }
  for (int zz = 0; zz < 2; zz++) {
    digitalWrite(ledArray[zz], 0);
  }

  delay(2000); // Do nothing for 10000 ms = 10 seconds
}

void loop()
{
  turnLeft(320);
    delay(2000);   

}

/********** Functions (Movement) **********/

/**
 * This function is a general movement function used to move robot forward.
 * Takes in values from -255 to 255 as input and writes it to the motors as PWM values.
 */
void move(int L_spd, int R_spd) {
  if (stop == false) {
    leftWheel.run(-L_spd);
    rightWheel.run(R_spd);
  } else {
    leftWheel.run(0);
    rightWheel.run(0);
  }
}

/**
 * This function is called to stop both motors.
 */
void stopMove() {
  rightWheel.stop();
  leftWheel.stop();
}

void forwardGrid() {
  leftWheel.run(-MOTORSPEED);
  rightWheel.run(MOTORSPEED);
  delay(TIME_FOR_1_GRID);
  stopMove();
}

void turnRight(int speed) {
  leftWheel.run(-MOTORSPEED);
  rightWheel.run(-MOTORSPEED);
  delay(speed); // 310 for normal
  stopMove();
}

void turnLeft(int speed) {
  leftWheel.run(MOTORSPEED);
  rightWheel.run(MOTORSPEED);
  delay(speed);
  stopMove();
}

void doubleRight() {
  turnRight(310);
  forwardGrid();
  turnRight(400);
}

void doubleLeft() {
  turnLeft(310);
  forwardGrid();
  turnLeft(420);
}

void uTurn() {
  turnRight(590);
}
