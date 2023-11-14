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
#define PUSHBUTTON        A7
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
#define IR_THRESHOLD                480   // Amount of dip in value to determine proximity of wall on right side. (Difference between AmbientIR and Measured IR Detector Voltage)
#define SIDE_MAX                    18    // Side distance threshold (cm)
#define TIME_FOR_LEFT_TURN          290   // The time duration (ms) for turning 90 degrees counter-clockwise      (for red waypoint)
#define TIME_FOR_RIGHT_TURN         280   // The time duration (ms) for turning 90 degrees clockwise              (for green waypoint)
#define TIME_FOR_1_GRID_PURPLE      640   // The time duration (ms) for moving forward by 1 grid                  (for purple waypoint)
#define TIME_FOR_1_GRID_BLUE        720   // The time duration (ms) for moving forward by 1 grid                  (for blue waypoint)
#define TIME_FOR_SECOND_LEFT_TURN   320   // The time duration (ms) for second 90 degrees counter-clockwise turn  (for purple waypoint) 
#define TIME_FOR_SECOND_RIGHT_TURN  300   // The time duration (ms) for second 90 degrees clockwise turn          (for blue waypoint)
#define TIME_FOR_UTURN              560   // The time duration (ms) for turning 180 degrees clockwise             (for orange waypoint)

/********** Constants & Variables for PID Controller (only PD is used) **********/

const double kp = 19;  // Proportional Gain/Constant  (P component of PID)
const double kd = 19;  // Derivative Constant         (D component of PID)

// Variables to hold final motorspeed calculated for each motor
int L_motorSpeed;                       
int R_motorSpeed;

const double desired_dist = 10.50;  // Desired distance between ultrasound sensor and wall to keep mBot centered in tile
double error;                       // Difference between current position and our desired distance
double prev_error = 0;              // Variable to store previous error, to be used to calculate change in error (For D component of PID)
double error_delta;                 // Difference between current error and previous error (For D component of PID)
double correction_dble;             // For calculation of correction for motors
int correction;                     // To be used to adjust input to motor

/********** Color Detection **********/
// Pins controlling 2-4 Decoder
int ledArray[2] = { A2, A3 };

// Truth Table to control 2-4 Decoder (for LED Control):
int truth[3][2] =  { { 0, 1 },      // Blue LED ON 
                     { 1, 0 },      // Green LED ON
                     { 1, 1 }       // Red LED ON, also used for IR Emitter OFF
                   };

char colourStr[3][5] = {"B = ", "G = ", "R = "};  // array of strings to aid debugging of measured RGB values 

// Variable to store resulting classified color [0 - white, 1 - red, 2 - blue, 3 - green, 4 - orange, 5 - purple, 6 - no color classified]
int color; 

// Variables to store measured intensity for each color
int measured_blue = 0;
int measured_green = 0;
int measured_red = 0;

// Float arrays to store calibrated values for color arrays
float colourArray[] = {0,0,0};
float whiteArray[] = {928.00,908.00,766.00};
float blackArray[] = {566.00,473.00,308.00};
float greyDiff[] = {362.00,435.00,458.00};


// Struct for each label to be used for color classification
struct Color {
  String name;
  int id; // 0 - white, 1 - red, 2 - blue, 3 - green, 4 - orange, 5 - purple
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

// Data points for each color, obtained by taking average of collected color samples.
// To be used in KNN classification using euclidean distance. 
Color colors[] = {
  //  Label/name -   id - R  -  G  -  B
  {   "Red",    1,  220,  130,  135 },
  {   "Blue",   2,  180,  212,  225 },
  {   "Green",  3,  160,  169,  128 },
  {   "Orange", 4,  195,  170,  120 },
  {   "Purple", 5,  160,  163,  188 },
  {   "White",  0,  255,  255,  255 }  
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
  led.setpin(13);             // led
  pinMode(MUSIC_PIN, OUTPUT); // buzzer
  pinMode(PUSHBUTTON, INPUT); // push button
  pinMode(IR, INPUT);         // ir
  // 2-4 decoder 
  for(int color = 0; color < 2; color++){
    pinMode(ledArray[color],OUTPUT);  
  }
  // set all to LOW
  for (int i = 0; i < 2; i++) {
    digitalWrite(ledArray[i], 0);
  }
    
  // setup complete
  led.setColor(128, 255, 0); // set LED to Green 
  led.show();

  delay(2000); // Do nothing for 2000 ms = 2 seconds
}

void loop()
{
  // run mBot only if status is 1
  if (status) { 
    ultrasound();             // update global variable dist
    if (!on_line()) {         // check if on black line
      if (dist!= OUT_OF_RANGE)  
      {
        // wall present, run pd_control
        led.setColor(255, 255, 255);
        led.show();
        pd_control();
      }
      else
      {
        // Re-initialise previous error to zero to prevent past interference if ultrasonic sensor goes out of range
        prev_error = 0;
        // no wall present on leftside, call IR to check right side (nudge left if we determine robot is too close to wall on right)  
        checkRight();
        // no wall detected, move straight
        led.setColor(0, 255, 255);
        led.show();
        move(MOTORSPEED, 240);
      }
    }
    else{
      // black line detected, stop all motors
      stopMove();
      // read color
      read_color();
      // classify color
      int color = classify_color();
      // execute waypoint objectives
      execute_waypoint(color);
      // measure and update reading of ambient IR
      updateAmbient();
    }
  }
  else{
    // entered if status == false, ie. robot is not running maze-solving algorithm 
    if (stop == true)
    {
      // entered after white waypoint is executed
      led.setColor(255, 0, 0); // set mBot LED to Red
      led.show();
      stopMove();
      finishMaze();
      stop = false;
    }
    // check if push button is pressed, 
    if (analogRead(PUSHBUTTON) < 100) { 
      updateAmbient();    // measure and update reading of ambient IR before moving
      status = true;      // Toggle status
      delay(500);         // Delay 500ms so that a button push won't be counted multiple times.
    }
  }  
}

/********** Functions (Movement, not including waypoint movement) **********/

/**
 * This function is a general movement function used to move robot forward.
 * Takes in values from -255 to 255 as input and writes it to the motors as PWM values.
 * @param[in] L_spd Left Motorspeed to be passed to left motor, positive value indicate forward, negative value indicate reverse
 * @param[in] R_spd Reft Motorspeed to be passed to right motor, positive value indicate forward, negative value indicate reverse
 */
void move(int L_spd, int R_spd) {
  if (stop == false) {
    leftWheel.run(-L_spd);
    rightWheel.run(R_spd);
  } 
  else {
    stopMove();
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
 * Function used P and D components of PID to calculate the new PWM values for left motor and right motor. 
 * Error is the difference between the robot's current position and the desired position of the robot.
 * After finding the error, we multiply the error by the proportional gain which helps us minimise this error.
 * Change of error (error_delta) helps us prevent oscillation by adding "damping" effect to our correction.
 */
void pd_control() {
  error = desired_dist - dist;                      // P Component of PID
  error_delta = error - prev_error;                 // D Component of PID
  correction_dble = (kp * error) + (kd * error_delta);
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

  //Initialise current error as new previous error (D Component of PID)
  prev_error = error; 
}

/********** Functions (IR) **********/

/**
 * This function updates measured voltage value for ambient IR.
 * Ambient IR is measured by first turning OFF the IR Emitter and measuring the voltage of IR Detector.
 */
void updateAmbient(){
  // Turn OFF IR Emitter
  for(int i = 0; i < 2; i++){  
    digitalWrite(ledArray[i], 1);
  }
  // wait for voltage to stabilise before reading
  delay(IRWait);
  ambientIR = analogRead(IR);
  // Serial.print("Ambient: ");
  // Serial.println(ambientIR);
}

/**
 * This function turns ON IR emitter and measures voltage across Detector.
 * Distance is approximated by finding the difference between this measured voltage with baseline voltage(ambientIR).
 * If difference in voltage is beyond IR_THRESHOLD, nudge mBot left and away from right side.
 */
void checkRight() {
  // Turn ON IR Emitter
  for(int i = 0; i < 2; i++){  
    digitalWrite(ledArray[i], 0);
  }
  delay(IRWait);
  // Measure voltage across IR Detector
  int irVolt = analogRead(IR);
  // TURN OFF IR Emitter
  for(int i = 0; i < 2; i++){  
    digitalWrite(ledArray[i], 1);
  }
  // Serial.print("Measured: ");
  // Serial.println(irVolt);
  int difference = ambientIR - irVolt;
  // Serial.print("Difference: ");
  // Serial.println(difference);
  if (difference > IR_THRESHOLD)
  {
    // nudge left
    move(215,255);
    delay(5);
  }
}

/********** Functions (Ultrasound & Line Sensor) **********/

/**
 * This function updates the global variable dist to current distance between the ultrasonic sensor and the closest object (wall) to it. 
 * Sets dist to OUT_OF_RANGE if measured distance is out of threshold SIDE_MAX.
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
 * This function detects for black line (waypoint)
 * @return Returns true if black line is detected, false if not detected.
 */
bool on_line() {
  sensorState = lineFinder.readSensors();
  if (sensorState != S1_OUT_S2_OUT) {
    return true;
  }
  return false;
}

/********** Functions (Color Detection) **********/

/**
 * Function turns on one colour at a time and measure LDR voltage for each colour to estimate respective R/G/B values. 
 * Estimated RGB values are stored in global array colourArray.
 */
void read_color() {
  for(int c = 0; c <= 2; c++){    
    // Serial.print(colourStr[c]);
    // TURN ON LIGHT
    for (int i = 0; i < 2; i++) {
      digitalWrite(ledArray[i], truth[c][i]);
    }
    delay(RGBWait);
    // 1 Average Reading is taken for color measurement as our color detection has rather high success rate
    colourArray[c] = getAvgReading(1);
    //the difference between average reading and calibrated values for black sample, multiplied by 255 will give a value between 0-255, representing the value for the current reflectivity
    int result = (colourArray[c] - blackArray[c])/(greyDiff[c])*255;
    if (result > 255) {
      result = 255;
    }
    else if (result < 0) {
      result = 0;
    }
    colourArray[c] = result;
    // TURN OFF LIGHT
     for (int i = 0; i < 2; i++) {
      digitalWrite(ledArray[i], 0);
    }
    delay(RGBWait);
    // Serial.println(int(colourArray[c])); //show the value for the current colour LED, which corresponds to either the R, G or B of the RGB code
  } 

}

/**
 * Function looks at RGB values stored in colourArray[] and compare it with defined points for each color stored in colors[].
 * Color is classified by finding calculating the Euclidean distance for each known color and select the one with the minimum distance. 
 * @return Returns the color id of classified color. [0 - white, 1 - red, 2 - blue, 3 - green, 4 - orange, 5 - purple, 6 - unknown]
 */
int classify_color() {
  int classified = 6;
  measured_blue = colourArray[0];
  measured_green = colourArray[1];
  measured_red = colourArray[2];
  
  // Calculate Euclidean distances for each known color
  double minDistance = 9999;  // Initialize with a large value
  //String classifiedColor;

  for (int i = 0; i < 6; i++) { // 6 is the number of known colors
    double distance = sqrt(pow(colors[i].red - measured_red, 2) + pow(colors[i].green - measured_green, 2) + pow(colors[i].blue - measured_blue, 2));
    if (distance < minDistance) {
      minDistance = distance;
      //classifiedColor = colors[i].name;
      classified = colors[i].id;
    }
  }
  // Serial.println("Classified as: " + classifiedColor);
  return classified;
}

/**
 * This function finds the average reading of LDR for greater accuracy of LDR readings.
 * @param[in] times Number of times to iterate and repeat before finding average
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

/********** Functions (Waypoints) **********/

/**
 * Function takes in the color of waypoint classified by mBot and calls respective functions to move mBot to complete waypoint objective. 
 * @param[in] color Classified color id
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
    led.setColor(0, 0, 0); // set both LED to OFF
    led.show();
    break;
  }
}

/**
 * Function moves mBot forward by a distance of approximately one tile. 
 * @param[in] time delay time in ms. 
 */
void forwardGrid(int time) {
  leftWheel.run(-MOTORSPEED);
  rightWheel.run(MOTORSPEED);
  delay(time);
  stopMove();
}

/**
 * Function allows mBot to make a 90 degrees clockwise turn. 
 * @param[in] time delay time in ms. 
 */
void turnRight(int time) {
  leftWheel.run(-MOTORSPEED);
  rightWheel.run(-MOTORSPEED);
  delay(time);
  stopMove();
}

/**
 * Function allows mBot to make a 90 degrees counter-clockwise turn. 
 * @param[in] time delay time in ms. 
 */
void turnLeft(int time) {
  leftWheel.run(MOTORSPEED);
  rightWheel.run(MOTORSPEED);
  delay(time);
  stopMove();
}

/**
 * Function allows mBot to make a 90 degrees clockwise turn, followed by moving forward by 1 tile, lastly following with another 90 degrees clockwise turn.
 * @param[in] time delay time in ms. 
 */
void doubleRight(int time) {
  turnRight(TIME_FOR_RIGHT_TURN);
  delay(10);
  forwardGrid(time);
  delay(100);
  turnRight(TIME_FOR_SECOND_RIGHT_TURN);

}

/**
 * Function allows mBot to make a 90 degrees counter-clockwise turn, followed by moving forward by 1 tile, lastly following with another 90 degrees counter-clockwise turn.
 * @param[in] time delay time in ms. 
 */
void doubleLeft(int time) {
  turnLeft(TIME_FOR_LEFT_TURN);
  delay(10);
  forwardGrid(time);
  delay(100);
  turnLeft(TIME_FOR_SECOND_LEFT_TURN);
}

/**
 * Function allows mBot to make a 180 degrees clockwise turn. 
 */
void uTurn() {
  turnRight(TIME_FOR_UTURN);
}

/**
 * This function plays a music tune of our choice.
 */
void finishMaze() {
  // keys and durations found in NOTES.h
  for (int i = 0; i < sizeof(music_key) / sizeof(int); ++i) {
    // quarter note = 1000 / 4, eighth note = 1000/8, etc. (Assuming 1 beat per sec)
    const int duration = 1000 / music_duration[i];
    buzzer.tone(MUSIC_PIN, music_key[i], duration);
    delay(duration * 1.30);   
    buzzer.noTone(MUSIC_PIN);  
  }
}