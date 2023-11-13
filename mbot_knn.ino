/********** Include Header Files **********/
#include "Wire.h"
#include "MeMCore.h"
#include "MeRGBLed.h"
#include "Notes.h"    // Self-made header file to contain Victory Tune for Buzzer

/********** Define Ports **********/
#define MUSIC_PIN         8                     
#define ULTRASONIC        12
#define LDR               A0
#define IR                A1
MeDCMotor                 leftWheel(M1);        // assigning LeftMotor to port M1
MeDCMotor                 rightWheel(M2);       // assigning RightMotor to port M2
MeLineFollower            lineFinder(PORT_2);
MeUltrasonicSensor        ultraSensor(PORT_1);  
MeRGBLed                  led(0,30);
MeBuzzer                  buzzer;

/********** Define Delay Constants **********/
// Define time delay constants before taking analogue readings (to let voltage stabalise)
#define RGBWait   60    // Time delay (in ms) before taking LDR reading 
#define IRWait    20    // Time delay (in ms) before taking IR reading 
#define LDRWait   20    // Time delay (in ms) before taking LDR reading 

/********** Constants **********/
// Ultrasound
#define OUT_OF_RANGE                100

// Movement
#define MOTORSPEED                  255
#define SIDE_MAX                    18    // side distance threshold in cm
#define TIME_FOR_LEFT_TURN          315   // The time duration (ms) for turning 90 degrees counter-clockwise      (for red waypoint)
#define TIME_FOR_RIGHT_TURN         310   // The time duration (ms) for turning 90 degrees clockwise              (for green waypoint)
#define TIME_FOR_1_GRID_PURPLE      680   // The time duration (ms) for moving forward by 1 grid                  (for purple waypoint)
#define TIME_FOR_1_GRID_BLUE        750   // The time duration (ms) for moving forward by 1 grid                  (for blue waypoint)
#define TIME_FOR_SECOND_LEFT_TURN   330   // The time duration (ms) for second 90 degrees counter-clockwise turn  (for purple waypoint) 
#define TIME_FOR_SECOND_RIGHT_TURN  320   // The time duration (ms) for second 90 degrees clockwise turn          (for blue waypoint)
#define TIME_FOR_UTURN              560   // The time duration (ms) for turning 180 degrees clockwise             (for orange waypoint)

/********** Constants & Variables for PID Controller (only PD is used) **********/

const double kp = 20;  //25              // Proportional Gain/Constant  (P component of PID)
const double kd = 20;  //20              // Derivative Constant         (D component of PID)

// Variables to hold final motorspeed calculated for each motor
int L_motorSpeed;                       
int R_motorSpeed;

const double desired_dist = 10.75;  // Desired distance between ultrasound sensor and wall to keep mBot centered in tile
double error;                       // Difference between current position and our desired distance
double prev_error = 0;              // Variable to store previous error, to be used to calculate change in error (For D component of PID)
double error_delta;                 // Difference between current error and previous error (For D component of PID)
double correction_dble;             // For calculation of correction for motors
int correction;                     // To be used to adjust input to motor

/********** Color Detection Parameters **********/
// Pins controlling 2-4 Decoder
int ledArray[2] = { A2, A3 };

// Truth Table to control 2-4 Decoder (for LED Control):
int truth[3][2] =  { { 0, 1 },      // Blue LED ON 
                     { 1, 0 },      // Green LED ON
                     { 1, 1 }       // Red LED ON
                   };

int ir_state[2][2] = { { 0, 0 },  // IR Emitter ON
                       { 1, 1}       // IR Emitter OFF (same as Red LED ON)
                     };

char colourStr[3][5] = {"B = ", "G = ", "R = "};  // 

int color; // to store classified color [0 - white, 1 - red, 2 - blue, 3 - green, 4 - orange, 5 - purple, 6 - black]

// variables to store measured intensity for each color
int measured_blue = 0;
int measured_green = 0;
int measured_red = 0;

//floats to store calibrated values for color arrays
float colourArray[] = {0,0,0};
float whiteArray[] = {928.00,908.00,766.00};
float blackArray[] = {566.00,473.00,308.00};
float greyDiff[] = {362.00,435.00,458.00};

// struct to create class for each label for color classification
struct Color {
  String name;
  int id; // 0 - white, 1 - red, 2 - blue, 3 - green, 4 - orange, 5 - purple, 6 - unknown
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

// dataset for each color to be used in KNN classification, average of all collected samples
Color colors[] = {
  //  Label -   id - R  -  G  -  B
  {   "Red",    1,  220,  130,  135},
  {   "Blue",   2,  130,  212,  225},
  {   "Green",  3,  130,  169,  128},
  {   "Orange", 4,  195,  170,  120},
  {   "Purple", 5,  137,  163,  188},
  {   "White",  0,  255,  255,  255}  
};

/********** Global Variables **********/
bool stop = false;    // variable to tell mBot to halt, used for white waypoint (end of maze) [0 = haven't reach end of maze, 1 = reach end of maze, stop all motor and play buzzer]
bool status = false;  // variable to tell mBot when to start maze solving algorithm, used to run mBot after pressing button [0 = do nothing, 1 = mBot runs]
int sensorState;      // to keep track of whether black line is detected by line follower
double dist;          // to keep track of distance between wall and ultrasound sensor
int ambientIR;        // to keep track of measured analog voltage for ambient IR  

/********** Function Declarations **********/
void stopMove(const int i);
void turnLeft(void);
void turnRight(void);
void moveForward(void);
void finishWaypoint(void);


/********** Setup & Loop **********/
void setup()
{
  led.setpin(13);
  pinMode(MUSIC_PIN, OUTPUT);
  pinMode(A7, INPUT); // Setup A7 as input for the push button
  pinMode(IR, INPUT);


  for(int c = 0;c < 2;c++){
    pinMode(ledArray[c],OUTPUT);  
  }
  
  for (int zz = 0; zz < 2; zz++) {
    digitalWrite(ledArray[zz], 0);
  }
  // Serial.begin(9600);
  
  // setup complete
  led.setColor(128, 255, 0); // set LED to Green 
  led.show();

  delay(2000); // Do nothing for 1000 ms = 1 second
}

void loop()
{
  if (status == true) { // run mBot only if status is 1
    // update global variable dist
    ultrasound();
    // check if on black line
    if (!on_line()) {
      // check for presence of wall based on ultrasound dist
      if (dist!= OUT_OF_RANGE)
      {
        // wall present, run pd_control
        led.setColor(255, 255, 255);
        led.show();
        pd_control();
      }
      else
      {
        // call IR to check right side (nudge left if we determine robot is too close to wall on right)  
        checkRight();
        // no wall detected, move straight
        led.setColor(0, 255, 255);
        led.show();
        move(MOTORSPEED, 240);
      }
    }
    else{
      // stop all motors
      stopMove();
      // read color
      read_color();
      // classify color after reading color
      int color = classify_color();
      // execute waypoint objectives
      execute_waypoint(color);
      // measure and update reading of ambient IR
      // updateAmbient();
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
      // updateAmbient();
      status = !status; // Toggle status
      delay(500); // Delay 500ms so that a button push won't be counted multiple times.
    }
  }  
}

// function updates voltage value for ambient IR
void updateAmbient(){
  digitalWrite(A2, 1);
  digitalWrite(A3, 1);
  delay(IRWait);
  ambientIR = analogRead(IR);
  Serial.print("Ambient: ");
  Serial.println(ambientIR);
}

void checkRight() {
  // Take raw value and threshold
  digitalWrite(A2, 0);
  digitalWrite(A3, 0);
  delay(IRWait);
  int irVolt = analogRead(IR);
  digitalWrite(A2, 1);
  digitalWrite(A3, 1);
  // delay(IRWait);
  Serial.print("Measured: ");
  Serial.println(irVolt);
  int difference = ambientIR - irVolt;
  Serial.print("Difference: ");
  Serial.println(difference);
  updateAmbient();
  if (difference > 480)
  {
    led.setColor(0, 0, 255); // set Right LED to Red
    led.show();
    // nudge left
    move(215,255);
    delay(3);
  }
  else
  {
    led.setColor(255, 255, 255); // set Right LED to White
    led.show();
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
  if (dist > SIDE_MAX)
  {
    dist = OUT_OF_RANGE;
  }
  // Serial.println(dist);
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
  for(int c = 0; c <= 2; c++){    
    // Serial.print(colourStr[c]);
    // TURN ON LIGHT
    for (int zz = 0; zz < 2; zz++) {
      digitalWrite(ledArray[zz], truth[c][zz]);
    }
    delay(RGBWait);
    //get the average of 5 consecutive readings for the current colour and return an average 
    colourArray[c] = getAvgReading(1);
    //the average reading returned minus the lowest value divided by the maximum possible range, multiplied by 255 will give a value between 0-255, representing the value for the current reflectivity (i.e. the colour LDR is exposed to)
    int result = (colourArray[c] - blackArray[c])/(greyDiff[c])*255;
    if (result > 255) {
      result = 255;
    }
    else if (result < 0) {
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
  int classified = 6;
  measured_blue = colourArray[0];
  measured_green = colourArray[1];
  measured_red = colourArray[2];
  
  // Calculate Euclidean distances for each known color
  double minDistance = 9999;  // Initialize with a large value
  String classifiedColor;

  for (int i = 0; i < 6; i++) { // 6 is the number of known colors
    double distance = sqrt(pow(colors[i].red - measured_red, 2) + pow(colors[i].green - measured_green, 2) + pow(colors[i].blue - measured_blue, 2));
    if (distance < minDistance) {
      minDistance = distance;
      classifiedColor = colors[i].name;
      classified = colors[i].id;
    }
  }
    // Serial.println("Classified as: " + classifiedColor);
  return classified;
}

/********** Functions (Waypoints) **********/

/**
 * Function takes in the color of waypoint classified by mBot and calls respective functions to move mBot to complete waypoint objective. 
 * 
 */
void execute_waypoint(const int color)
{
  switch(color) {
  case 0:
    // code block for white
    led.setColor(255, 255, 255); // set both LED to WHITE
    led.show();
    stop = true;
    status = false;
    break;
  case 1:
    // code block for red
    led.setColor(255, 0, 0); // set both LED to RED
    led.show();
    turnLeft(TIME_FOR_LEFT_TURN);
    break;
  case 2:
    // code block for blue
    led.setColor(0, 0, 255); // set both LED to BLUE
    led.show();
    doubleRight(TIME_FOR_1_GRID_BLUE);
    break;
  case 3:
    // code block for green
    led.setColor(0, 255, 0); // set both LED to GREEN
    led.show();
    turnRight(TIME_FOR_RIGHT_TURN);
    break;
  case 4: 
    // code block for orange
    led.setColor(153, 76, 0); // set both LED to ORANGE
    led.show();
    uTurn();
    break;
  case 5:
    // code block for purple
    led.setColor(153, 51, 255); // set both LED to PURPLE
    led.show();
    doubleLeft(TIME_FOR_1_GRID_PURPLE);
    break;
  default:
    // code block for no color classified
    led.setColor(0, 0, 0); // set both LED to NONE
    led.show();
    break;
  }
}

/**
 * Function moves mBot forward by a distance of approximately one tile. 
 * 
 */
void forwardGrid(int time) {
  leftWheel.run(-MOTORSPEED);
  rightWheel.run(MOTORSPEED);
  delay(time);
  stopMove();
}

/**
 * Function allows mBot to make a rough 90 degrees clockwise turn. 
 * 
 */
void turnRight(int time) {
  leftWheel.run(-MOTORSPEED);
  rightWheel.run(-MOTORSPEED);
  delay(time);
  stopMove();
}

/**
 * Function allows mBot to make a rough 90 degrees counter-clockwise turn. 
 * 
 */
void turnLeft(int time) {
  leftWheel.run(MOTORSPEED);
  rightWheel.run(MOTORSPEED);
  delay(time);
  stopMove();
}

/**
 * Function . 
 * 
 */
void doubleRight(int time) {
  turnRight(TIME_FOR_RIGHT_TURN);
  delay(10);
  forwardGrid(time);
  delay(100);
  turnRight(TIME_FOR_SECOND_RIGHT_TURN);

}

void doubleLeft(int time) {
  turnLeft(TIME_FOR_LEFT_TURN);
  delay(10);
  forwardGrid(time);
  delay(100);
  turnLeft(TIME_FOR_SECOND_LEFT_TURN);
}

void uTurn() {
  turnRight(TIME_FOR_UTURN);
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