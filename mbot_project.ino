#include "Wire.h"
#include "MeMCore.h"
#include "MeRGBLed.h"
#include "Notes.h"

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
const double kp = 50;            //   - For P component of PID
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
float whiteArray[] = {1000.00,994.00,984.00};
float blackArray[] = {939.00,811.00,943.00};
float greyDiff[] = {61.00,183.00,41.00};
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
  if (status == true) { // run mBot only if status is 1
    ultrasound(); // updates global variable dist 
    if (!on_line()) { // situation 1

      if (dist!= OUT_OF_RANGE)
      {
        led.setColor(255, 255, 255);
        led.show();
        pd_control();
      }
      else
      {
        led.setColor(0, 255, 255);
        led.show();
        // no wall detected, try to move straight
        move(MOTORSPEED, 240);
      }
    }
    else{
      Serial.println("Stopping");
      stopMove();
      Serial.println("Reading Colour");
      read_color();
      Serial.println("Classifying Color");
      color = classify_color();
      Serial.println("Executing Waypoint");
      execute_waypoint(color);
      
      
    }
  }
  else{
    if (stop == true)
    {
      led.setColor(255, 0, 0); // set Right LED to Red
      led.show();
      stopMove();
      finishMaze();
      stop = false;
    }
    
    if (analogRead(A7) < 100) { // If push button is pushed
      status = !status; // Toggle status
      delay(500); // Delay 500ms so that a button push won't be counted multiple times.
    }
  delay(1);
  }  
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

/**
 * This function calculates the new PWM values for left motor and right motor. 
 * This is done by calculating the error between the robot's current position and the desired position of the robot.
 * After finding the error, we multiply the error by a the proportional gain which helps us minimise this error.
 * The D component of this PID controller can be commented out as it seems that P alone is enough.
 */
void pd_control() {
  if (dist != OUT_OF_RANGE) {
    error = desired_dist - dist;
    error_delta = error - prev_error; //   - For D Component of PID
    correction_dble = kp * error + (kd*error_delta);
    correction = (int)correction_dble;

    // Determine direction of correction and execute movement
    if (correction < 0) {
      L_motorSpeed = 255 + correction;
      R_motorSpeed = 255;
    } else {
      L_motorSpeed = 255;
      R_motorSpeed = 255 - correction;
    }
    move(L_motorSpeed, R_motorSpeed);

    //Initialise current error as new previous error (For D Component of PID)
    prev_error = error; 
  } 
  else {
    // Re-initialise previous error to zero to prevent past interference if ultrasonic sensor
    // goes out of range
    prev_error = 0;
  }
}

/********** Functions (Sensors) **********/

/**
 * This function sets the global variable dist to correspond to the distance between the
 * ultrasonic sensor and the closest object (wall) to it. Sets dist to OUT_OF_RANGE if out of range.
 */
void ultrasound() {

  dist = ultraSensor.distanceCm();
  if (dist > 15)
  {

    dist = OUT_OF_RANGE;
  }
  // pinMode(ULTRASONIC, OUTPUT);
  // digitalWrite(ULTRASONIC, LOW);
  // delayMicroseconds(2);
  // digitalWrite(ULTRASONIC, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(ULTRASONIC, LOW);
  // pinMode(ULTRASONIC, INPUT);

  // long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
  // if (duration > 0) {
  //   dist = duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100;
  // } else {
  //   dist = OUT_OF_RANGE;
  // }
  Serial.println(dist);
}

/**
 * This function returns a boolean value for whether the line tracking sensor detects
 * that the robot is fully on the line (i.e. reached a waypoint)
 */
bool on_line() {
  sensorState = lineFinder.readSensors();
  if (sensorState != S1_OUT_S2_OUT) {
    return true;
  }
  return false;
}

/**
 * This function finds the average reading of LDR for greater accuracy of LDR readings
 *
 * @param[in] times Number of times to iterate and repeat before finding average
 *
 * @return Returns the averaged LDR values across times iterations
 */
int getAvgReading(int times) {
  //find the average reading for the requested number of times of scanning LDR
  int reading;
  int total = 0;
  //take the reading as many times as requested and add them up
  for (int i = 0; i < times; i++) {
    reading = analogRead(LDR);
    total = reading + total;
    delay(LDRWait);
  }
  //calculate the average and return it
  return total / times;
}

/**
 * This function sets the global array colourArray to reflect the RGB values sensed by the LDR with correspondence to
 * lighting up of the red, green and blue lights
 */
void read_color() {
  // turn on one colour at a time and LDR reads 5 times
//turn on one colour at a time and LDR reads 5 times
  for(int c = 0;c <= 2;c++){    
    Serial.print(colourStr[c]);
    // TURN ON LIGHT
    for (int zz = 0; zz < 2; zz++) {
      digitalWrite(ledArray[zz], truth[c][zz]);
    }
    delay(RGBWait);
//get the average of 5 consecutive readings for the current colour and return an average 
    colourArray[c] = getAvgReading(5);
//the average reading returned minus the lowest value divided by the maximum possible range, multiplied by 255 will give a value between 0-255, representing the value for the current reflectivity (i.e. the colour LDR is exposed to)
    int result = (colourArray[c] - blackArray[c])/(greyDiff[c])*255;
    if (result > 255)
    {
      result = 255;
    }
    else if (result < 0)
    {
      result = 0;
    }
    colourArray[c] = result;
    // TURN OFF LIGHT
     for (int zz = 0; zz < 2; zz++) {
      digitalWrite(ledArray[zz], 0);
    }
    delay(RGBWait);
    Serial.println(int(colourArray[c])); //show the value for the current colour LED, which corresponds to either the R, G or B of the RGB code
  } 

}

int classify_color() {
  // turn on one colour at a time and LDR reads 5 times
  // colour detected [0 - white, 1 - red, 2 - blue, 3 - green, 4 - orange, 5 - purple, 6 - black]

  blue = colourArray[0];
  green = colourArray[1];
  red = colourArray[2];
  
  // Serial.print("B: ");
  // Serial.print(blue);
  // Serial.print(" G: ");
  // Serial.print(green);
  // Serial.print(" R: ");
  // Serial.print(red);
  
  if (withinWhite())
  {
    return 0;
  }
  if (withinRed())
  {
    return 1;
  }
  if (withinBlue())
  {
    return 2;
  }
  if (withinGreen())
  {
    return 3;
  }
  if (withinOrange())
  {
    return 4;
  }
  if (withinPurple())
  {
    return 5;
  }
  return 6;
}

bool withinWhite()
{
  if ( (240 <= blue && blue <= 255) && (240 <= green && green <= 255) &&(240 <= red && red <= 255)  )
  {
    // Serial.println("White Detected.");
    return true;
  }
  return false;
}

bool withinRed()
{
  if ( (70 <= blue && blue <= 110) && (85 <= green && green <= 150) &&(230 <= red && red <= 255)  )
  {
    // Serial.println("Red Detected.");
    return true;
  }
  return false;
}

bool withinBlue()
{
    if ( (240 <= blue && blue <= 255) && (195 <= green && green <= 240) &&(85 <= red && red <= 105)  )
    {
      // Serial.println("Blue Detected.");
      return true;
    }
    return false;
}

bool withinGreen()
{
    if ( (80 <= blue && blue <= 140) && (150 <= green && green <= 210) &&(30 <= red && red <= 80)  )
    {
      // Serial.println("Green Detected.");
      return true;
    }
    return false;
}

bool withinOrange()
{
    if ( (80 <= blue && blue <= 115) && (150 <= green && green <= 205) &&(240 <= red && red <= 255)  )
    {
      // Serial.println("Orange Detected.");
      return true;
    }
    return false;
}

bool withinPurple()
{
    if ( (190 <= blue && blue <= 230) && (140 <= green && green <= 200) &&(150 <= red && red <= 180)  )
    {
      // Serial.println("Purple Detected.");
      return true;
    }
    return false;
}

/********** Functions (Waypoints) **********/

void execute_waypoint(const int color)
{
  switch(color) {
  case 0:
    // code block for white
    stop = true;
    status = false;
    break;
  case 1:
    // code block for red
    led.setColor(255, 0, 0); // set Right LED to Red
    led.show();
    turnLeft();
    break;
  case 2:
    // code block for blue
    led.setColor(0, 0, 255); // set Right LED to Red
    led.show();
    doubleRight();
    break;
  case 3:
    // code block for green
    led.setColor(0, 255, 0); // set Right LED to Red
    led.show();
    turnRight();
    break;
  case 4: 
    // code block for orange
    led.setColor(153, 76, 0); // set Right LED to Red
    led.show();
    uTurn();
    break;
  case 5:
    // code block for purple
    led.setColor(153, 51, 255); // set Right LED to Red
    led.show();
    doubleLeft();
    break;
  default:
    // code block for black or no color classified
    led.setColor(0, 0, 0); // set Right LED to Red
    led.show();
    break;
  }
}

void forwardGrid() {
  leftWheel.run(-MOTORSPEED);
  rightWheel.run(MOTORSPEED);
  delay(TIME_FOR_1_GRID);
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

/**
 * To be executed when maze is completed (ie. White Coloured Paper Detected).
 * This function plays a music tune of our choice.
 */
void finishMaze() {
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