/*-------------INCLUDE LIBRARIES-------------*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

/*-------------DEFINE CONFIGS-------------*/
/*DEFINE MOTOR PINS*/
#define mtrLpos 3
#define mtrLneg 2
#define mtrRpos 5
#define mtrRneg 4

/*DEFINE PID CONFIGS*/
#define Kp 10
#define Ki 40
#define Kd 30
#define initial_motor_speed 60

/*DEFINE CE CSN PIN FOR NRF24*/
#define CE 6
#define CSN 7

/*DEFINE SERVO PIN*/
#define servoPin 9

/*FOLOWING TOP SPEED AND DEGREE OFFSET*/
#define topSpeed 75
#define slowSpeed 55
#define degreeOffset 20

/*TURNING TOP SPEED*/
#define turningMaxSpeed 100

/*ROTATION RESOLUTION*/
#define rotres 1

/*DEFINE ULTRASONIC PINS*/
#define trigPin 10
#define echoPin 8

/*DEFINE DISTANCE THRESHOLDS*/
#define distanceTresh 15
#define moveBackTresh 20

/*IR SENSOR PINS*/
int sensors[5] = {A0, A1, A2, A3, A6};

/*SERVO LEFT RIGHT ANGLES*/
const int servo_right_angles[5] = {72, 54, 36, 18, 0};
const int servo_left_angles[5] = {108, 126, 144, 162, 180};

/*-------------DEFINE GLOBAL VARIABLES-------------*/
/*MODE (MODE 1 => manual, MODE = 2 => follow line)*/
int MODE = 2;

/*keeps the last state of button pressed*/
int btnLastState = 1;

/*INITIAL SERVO*/
Servo servomtr;

/*INITIAL MPU*/
Adafruit_MPU6050 mpu;

/*INITIAL OLED*/
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

/*INITIAL NRF24*/
RF24 radio(CE, CSN);
const byte address[6] = "00001";

/*DEFINE RADIO MESSAGE STRUCT*/
struct Data_frame{
  int XPOS;
  int YPOS;
  int MODE;
};
Data_frame msg;

/*DEFINE IR SENSOR OUTPUT ARRAY*/
bool flgs[5] = {false, false, false, false, false};

/*STABLIZING VARIABLES*/
double timeStep, time = millis(), timePrev;
double angle = 0, gyroSpeedDeg;
int speedleft = 0, speedright = 0;
double degree = 0;
double current_angle = 0, current_gyroSpeedDeg, lastStraightAngle = 0, error_angle = 0;
double current_timeStep, current_time = millis(), current_timePrev;
int initialTime;
int timer;

/*PID VARIABLES*/
float error=0, P=0, I=0, D=0, PID_value=0;
float previous_error=0, previous_I=0;


/*-------------SETUP FUNCTIONS-------------*/
void setup() {
  /*SETUP IR SENSOR PINS*/
  for (int i ; i<5; i++) {
    pinMode(sensors[i], INPUT);
  }

  /*SETUP SERVO*/
  servomtr.attach(servoPin);
  servomtr.write(90);

  /*SETUP NRF24*/
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  msg.XPOS = msg.YPOS = 0;
  msg.MODE = MODE;

  /*SETUP MPU6050*/
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  /*SETUP OLED*/
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextSize(4);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.display();
  delay(500);
  display.clearDisplay();
  display.display();

  /*SETUP MOTOR PINS*/
  pinMode(mtrRpos, OUTPUT);
  pinMode(mtrRneg, OUTPUT);
  pinMode(mtrLpos, OUTPUT);
  pinMode(mtrLneg, OUTPUT);

  /*SETUP ULTRASONIC PINS*/
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  /*SETUPT SERIAL PORT*/
  Serial.begin(9600);
}

/*-------------LOOP FUNCTIONS-------------*/
void loop() {
  /*CHECK FOR CONTROLLER SIGNAL*/
  if (radio.available())
  {
    radio.read(&msg, sizeof(Data_frame));
    Serial.print("X POS: ");
    Serial.print(msg.XPOS);
    Serial.print("\tY POS: ");
    Serial.print(msg.YPOS);
    Serial.print("\tMODE: ");
    Serial.println(msg.MODE);
    MODE = msg.MODE;
  }
  display.clearDisplay();
  display.setCursor(64,0);
  display.print(MODE);
  display.display();

  /*MODE 0 => HULT*/
  if (MODE == 0) {
    movement(0, 0);
    error = 0;
  }
  /*MODE 1 => MANUAL CONTROL*/
  else if (MODE == 1){
    int right = msg.XPOS + msg.YPOS;
    int left = msg.XPOS - msg.YPOS;
    left = left > 127 ? 127 : left;
    left = left < -127 ? -127 : left;
    right = right > 127 ? 127 : right;
    right = right < -127 ? -127 : right;
    error = 0;
    movement(left, right);
  }
  /*MODE 2 => AUTONOMOUS CONTROL*/
  else if (MODE == 2)
  {

    current_timePrev = current_time;
    current_time = millis();
    current_timeStep = (current_time - current_timePrev) / 1000;
    current_gyroSpeedDeg = read_gyroSpeed();
    current_angle = current_angle + (current_timeStep * round(current_gyroSpeedDeg));
    error_angle = lastStraightAngle - current_angle;
    Serial.println(error_angle);
    int servoangle = 90 + error_angle;
    constrain(servoangle, 0, 180);
    servomtr.write(servoangle);
    /*READ DISTANCE INFO*/
    // servomtr.write(90);
    double distance = get_distance(1);

    if (distance < distanceTresh){
      /*====MENUVER STARTS====*/
      /*STEP0: STABALIZE*/
      movement(0, 0);
      delay(500);
      /*STEP1: STOP AND CHECK LEFT AND RIGHT*/
      movement(0, 0);
      delay(1000);
      delay(1000);
      int rotation = leftOrRight();
      servomtr.write(90);

      /*STEP2: INITIAL TURNING TO 45 DEGREE*/
      rotate(rotation / 2);
      delay(800);
      initialTime = millis();
      angle = 0;
      do {
        timer = millis();
        stable_movement(0);
      }while (timer - initialTime < 1000);
      movement(0, 0);

      /*STEP3: TURN 45 DEGREE PARALLEL TO THE LINE*/
      rotate(-rotation / 2);
      delay(800);
      movement(0, 0);
      initialTime = millis();
      angle = 0;
      do {
        timer = millis();
        stable_movement(0);
      }while (timer - initialTime < 700);
      movement(0, 0);

      /*STEP4: TURN 45 DEGREE TOWARDS THE LINE AND START MOVING AND LOOKING FOR LINE*/
      rotate(-rotation * 0.50);
      delay(800);
      angle = 0;
      do {
        readSensors();
        stable_movement(0);
      }while ((rotation > 0 ? (!flgs[1] and !flgs[0]) :(!flgs[3] and !flgs[4])));
      error = 0;
    }

    /*DECIDE MOVEMENT BASE ON IR INFO*/
    readPosition();
    calculate_pid();
    motor_control();
  }
}


/*-------------CUSTOME FUNCTIONS-------------*/

/*MOVEMENT FUNCTION (input left and right speed)*/
void movement(int left, int right){
  if (right > 0 and right > 30){
    analogWrite(mtrRpos, right);
    digitalWrite(mtrRneg, LOW);
    // Serial.print("RIGHT: ");
    // Serial.print(right);
  }else if (right < -30){
    analogWrite(mtrRpos, 255 + right);
    digitalWrite(mtrRneg, HIGH);
    // Serial.print("RIGHT: ");
    // Serial.print(right);
  }else{
    digitalWrite(mtrRpos, LOW);
    digitalWrite(mtrRneg, LOW);
  }
  if (left > 0 and left > 30){
    analogWrite(mtrLpos, left);
    digitalWrite(mtrLneg, LOW);
    // Serial.print(" LEFT: ");
    // Serial.println(left);
  }else if (left < -30){
    analogWrite(mtrLpos, 255 + left);
    digitalWrite(mtrLneg, HIGH);
    // Serial.print(" LEFT: ");
    // Serial.println(left);
  }else{
    digitalWrite(mtrLpos, LOW);
    digitalWrite(mtrLneg, LOW);
  }
}

/*CALCULATE PID FUNCTION*/
void calculate_pid(){
  P = error;
  I = I + previous_I;
  D = error-previous_error;
  
  PID_value = (Kp*P) + (Ki*I) + (Kd*D);
  
  previous_I=I;
  previous_error=error;
}
/*MOTOR CONTROL BASE ON PID FUNCTION*/
void motor_control()
{
    // Calculating the effective motor speed:
    int left_motor_speed = initial_motor_speed-PID_value;
    int right_motor_speed = initial_motor_speed+PID_value;
    
    // The motor speed should not exceed the max PWM value
    constrain(left_motor_speed,50,140);
    constrain(right_motor_speed,50,140);
    // Serial.print("ERROR:");
    // Serial.print(error );
    // Serial.print("/tPID:");
    // Serial.print(PID_value );
    // Serial.print("\tleft_motor_speed:");
    // Serial.print(left_motor_speed);
    // Serial.print("\tright_motor_speed:");
    // Serial.println(right_motor_speed);
    movement(left_motor_speed, right_motor_speed);
}
/*READ SENSORS FUNCTION*/
int readSensors(){
  /*READ ALL IR SENSORS*/
  // Serial.print("[");
  for (int i; i<5; i++) {
    flgs[i] = analogRead(sensors[i]) > 512 ? 1 : 0;
    // Serial.print(flgs[i]);
    // Serial.print(" ");
  }
  // Serial.println("]");
}
/*READ POSITION ERROR FUNCTION*/
int readPosition(){
  readSensors();

  if (flgs[0]) {
    if (flgs[1])
      error = 3;
    else
      error = 4;
  }else if (flgs[4]) {
    if (flgs[3])
      error = -3;
    else
      error = -4;
  }else if (flgs[1]){
    if (flgs[2])
      error = 1;
    else
      error = 2;
  }else if (flgs[3]){
    if (flgs[2])
      error = -1;
    else
      error = -2;
  }else if (flgs[2]){
    error = 0;
    lastStraightAngle = current_angle;
  }else {
    if (error == -4) {
      error=-5;
    }
    else if (error == 4) {
     error = 5;
    }
  }
}

/*ULTRASONIC DISTANCE (gets avarage of n sample)*/
double get_distance(int n){
  long duration;
  int distance;
  double avg = 0;
  for (int i = 0; i < n; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration*.0343)/2;
    avg += distance;
    delay(2);
  }
  avg /= n;
  return avg;
}

/*READ GYRO SPEED IN DEG/S */
double read_gyroSpeed(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return (180/3.141592) * g.gyro.z;
}

/*ROTATION FUNCTION (input betwenn 90 and -90)*/
void rotate(int degree){
  double timeStep, time, timePrev;
  time = millis();
  double angle = 0, gyroSpeedDeg;
  int speed = 0;

  while (!(angle < degree + rotres and angle > degree - rotres and gyroSpeedDeg < 1 and gyroSpeedDeg > -1)) {

    timePrev = time;
    time = millis();
    timeStep = (time - timePrev) / 1000;
    gyroSpeedDeg = read_gyroSpeed();
    angle = angle + (timeStep * round(gyroSpeedDeg));

    // Serial.print(angle);
    // Serial.print(" DEG\t");
    // Serial.print(gyroSpeedDeg);
    // Serial.print(" DEG/s\t");

  
    if(angle < degree - rotres){
      if (gyroSpeedDeg > 10) {
        if (degree < 0) {
          speed = map(angle, -180, -90, turningMaxSpeed, 10);
        }else {
          speed = map(angle, 0, 90, turningMaxSpeed, 10);
        }
      }else {
        speed += 3;
      }
      // Serial.print("LEFT\tSPEED: ");
      // Serial.println(speed);
      movement(-speed, speed);
    }else if (angle > degree + rotres) {
      if (gyroSpeedDeg < -10) {
        if (degree < 0) {
          speed = map(angle, 0, -90, turningMaxSpeed, 10);
        }else {
          speed = map(angle, 180, 90, turningMaxSpeed, 10);
        }
      }else {
        speed += 5;
      }
      // Serial.print("RIGHT\tSPEED: ");
      // Serial.println(speed);
      movement(speed, -speed);
    }else {
      movement(0, 0);
    }
  }
  movement(0, 0);
}

/*STABALIZED MOVEMENT*/
void stable_movement(double degree){
  timePrev = time;
  time = millis();
  timeStep = (time - timePrev) / 1000;
  gyroSpeedDeg = read_gyroSpeed();
  angle = angle + (timeStep * round(gyroSpeedDeg));
  if (angle >= degree) {
    speedright = map(angle, degree, degree + degreeOffset, topSpeed, 0);
    speedleft = topSpeed;
  } else if (angle < degree){
    speedleft = map(angle, degree, degree - degreeOffset, topSpeed, 0);
    speedright = topSpeed;
    }
  movement(speedleft, speedright);
  // Serial.print("DEGREE = ");
  // Serial.print(degree);
  // Serial.print("\tANGLE = ");
  // Serial.print(angle);
  // Serial.print("\tspeedleft: ");
  // Serial.print(speedleft);
  // Serial.print("\tspeedright: ");
  // Serial.println(speedright);
}

void stable_movement_back(double degree){
  timePrev = time;
  time = millis();
  timeStep = (time - timePrev) / 1000;
  gyroSpeedDeg = read_gyroSpeed();
  angle = angle + (timeStep * round(gyroSpeedDeg));
  if (angle >= degree) {
    speedright = map(angle, degree, degree + degreeOffset, topSpeed, 0);
    speedleft = topSpeed;
  } else if (angle < degree){
    speedleft = map(angle, degree, degree - degreeOffset, topSpeed, 0);
    speedright = topSpeed;
    }
  movement(-speedright, -speedleft);
  // Serial.print("DEGREE = ");
  // Serial.print(degree);
  // Serial.print("\tANGLE = ");
  // Serial.print(angle);
  // Serial.print("\tspeedleft: ");
  // Serial.print(speedleft);
  // Serial.print("\tspeedright: ");
  // Serial.println(speedright);
}

void turnLeft(int speed){
  movement(topSpeed - speed, topSpeed);
}
void turnRight(int speed){
  movement(topSpeed, topSpeed - speed);
}

/*LEFTRIGHT FUNCTION*/
int leftOrRight(){
    int right_score = 0;
    int left_score = 0;
    double right[5] = {0};
    double left[5] = {0};
    for (int i = 0; i<5; i++) {
      servomtr.write(servo_right_angles[i]);
      delay(50);
      right[i] = get_distance(10);
    }
    // Serial.println();
    // Serial.print("LEFT:\t");
    for (int i = 0; i<5; i++) {
      servomtr.write(servo_left_angles[i]);
      delay(50);
      left[i] = get_distance(10);
    }
    // Serial.println();
    for (int i = 0; i < 5; i++) {
      if (right[i] > left[i]) 
        right_score++;
      else
        left_score++;
    }
    if (right_score > left_score) {
      return -90;
    }else {
      return 90;
    }
}