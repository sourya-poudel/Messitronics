#include "WiFiEsp.h"
#include "WiFiEspUDP.h"
#include <Servo.h>

namespace constants {
// Speed constants
const int speed = 85;
const int turnSpeed = 90;
const int shiftSpeed = 130;
const int midSpeed = 100;
const int highSpeed = 120;
const int lowSpeed = 80;
const int backSpeed1 = 160;
const int backSpeed2 = 90;
// Delay times
const int longDelayTime = 70;
const int delayTime = 40;
const int shortDelayTime = 300;

// Sensor pins
const int sensor1 = A4;             // left most sensor
const int sensor2 = A3;             // 2nd left sensor
const int sensor3 = A2;             // center sensor
const int sensor4 = A1;             // 2nd right most sensor
const int sensor5 = A0;             // right most sensor
const int rightObstacleSensor = 2;  // right obstacle sensor
const int leftObstacleSensor = 3;   // left obstacle sensor
const int servoPin = 13;            // servo pin
const int echoPin = 31;             // ultrasonic echo pin
const int trigPin = 30;             // ultrasonic trig pin

// Configuration constants
const int distanceLimit = 30;    // distance threshold for ultrasonic in cm
const int maxWifiAttempts = 5;   // maximum WiFi connection attempts
const int wifiRetryDelay = 500;  // delay between WiFi attempts in ms
const int servoCenter = 90;      // center position for servo
const int servoLeft = 0;         // left position for servo
const int servoRight = 180;      // right position for servo
const int servoUp = 135;         // up position for servo
const int servoDown = 45;        // down position for servo
const int LPT = 2;
// Motor control pins
const int speedPinR = 9;
const int rightMotorDirPin1 = 22;
const int rightMotorDirPin2 = 24;
const int leftMotorDirPin1 = 26;
const int leftMotorDirPin2 = 28;
const int speedPinL = 10;
const int speedPinRB = 11;
const int rightMotorDirPin1B = 5;
const int rightMotorDirPin2B = 6;
const int leftMotorDirPin1B = 7;
const int leftMotorDirPin2B = 8;
const int speedPinLB = 12;

// Buffer sizes
const int packetBufferSize = 256;
const int ssidBufferSize = 12;

//Display battery percentage
const float max_voltage = 8.05;
const float min_voltage = 5.00;
}
// servo object
Servo ourServo;
bool isObstacleTracking = false;
int servoCurrentAngle = 90;
bool isObstacleFollowing = false;
bool isLineTracking = false;
class Car {
private:
  char ssid[constants::ssidBufferSize] = "Transformer";
  int status = WL_IDLE_STATUS;
  char packetBuffer[constants::packetBufferSize];
  unsigned int localPort = 8888;
  WiFiEspUDP Udp;

  bool isValidSpeed(int speed) {
    return speed >= 0 && speed <= 255;
  }

protected:
  void set_Motorspeed(int leftFront,int rightFront,int leftBack,int rightBack)
  {
  analogWrite(constants::speedPinL,leftFront); 
  analogWrite(constants::speedPinR,rightFront); 
  analogWrite(constants::speedPinLB,leftBack);  
  analogWrite(constants::speedPinRB,rightBack);
  }

  void setMotorForward(int leftSpeed, int rightSpeed) {
    if (!isValidSpeed(leftSpeed) || !isValidSpeed(rightSpeed)) {
      Serial.println("Invalid speed value");
      return;
    }

    // forward motors
    digitalWrite(constants::leftMotorDirPin1, HIGH);
    digitalWrite(constants::leftMotorDirPin2, LOW);
    analogWrite(constants::speedPinL, leftSpeed);

    digitalWrite(constants::rightMotorDirPin1, HIGH);
    digitalWrite(constants::rightMotorDirPin2, LOW);
    analogWrite(constants::speedPinR, rightSpeed);

    // rear motors
    digitalWrite(constants::leftMotorDirPin1B, HIGH);
    digitalWrite(constants::leftMotorDirPin2B, LOW);
    analogWrite(constants::speedPinLB, leftSpeed);

    digitalWrite(constants::rightMotorDirPin1B, HIGH);
    digitalWrite(constants::rightMotorDirPin2B, LOW);
    analogWrite(constants::speedPinRB, rightSpeed);
  }

  void setMotorBackward(int leftSpeed, int rightSpeed) {
    if (!isValidSpeed(leftSpeed) || !isValidSpeed(rightSpeed)) {
      Serial.println("Invalid speed value");
      return;
    }

    // forward motors
    digitalWrite(constants::leftMotorDirPin1, LOW);
    digitalWrite(constants::leftMotorDirPin2, HIGH);
    analogWrite(constants::speedPinL, leftSpeed);

    digitalWrite(constants::rightMotorDirPin1, LOW);
    digitalWrite(constants::rightMotorDirPin2, HIGH);
    analogWrite(constants::speedPinR, rightSpeed);

    // rear motors
    digitalWrite(constants::leftMotorDirPin1B, LOW);
    digitalWrite(constants::leftMotorDirPin2B, HIGH);
    analogWrite(constants::speedPinLB, leftSpeed);

    digitalWrite(constants::rightMotorDirPin1B, LOW);
    digitalWrite(constants::rightMotorDirPin2B, HIGH);
    analogWrite(constants::speedPinRB, rightSpeed);
  }

  void stopMotors() {
    analogWrite(constants::speedPinL, 0);
    analogWrite(constants::speedPinR, 0);
    analogWrite(constants::speedPinLB, 0);
    analogWrite(constants::speedPinRB, 0);
  }

  void FrBck(int speed) {
    if (!isValidSpeed(speed))
      return;
    digitalWrite(constants::rightMotorDirPin1, LOW);
    digitalWrite(constants::rightMotorDirPin2, HIGH);
    analogWrite(constants::speedPinR, speed);
  }

  void FrFwd(int speed) {
    if (!isValidSpeed(speed))
      return;
    digitalWrite(constants::rightMotorDirPin1, HIGH);
    digitalWrite(constants::rightMotorDirPin2, LOW);
    analogWrite(constants::speedPinR, speed);
  }

  void FlBck(int speed) {
    if (!isValidSpeed(speed))
      return;
    digitalWrite(constants::leftMotorDirPin1, LOW);
    digitalWrite(constants::leftMotorDirPin2, HIGH);
    analogWrite(constants::speedPinL, speed);
  }

  void FlFwd(int speed) {
    if (!isValidSpeed(speed))
      return;
    digitalWrite(constants::leftMotorDirPin1, HIGH);
    digitalWrite(constants::leftMotorDirPin2, LOW);
    analogWrite(constants::speedPinL, speed);
  }

  void RrBck(int speed) {
    if (!isValidSpeed(speed))
      return;
    digitalWrite(constants::rightMotorDirPin1B, LOW);
    digitalWrite(constants::rightMotorDirPin2B, HIGH);
    analogWrite(constants::speedPinRB, speed);
  }

  void RrFwd(int speed) {
    if (!isValidSpeed(speed))
      return;
    digitalWrite(constants::rightMotorDirPin1B, HIGH);
    digitalWrite(constants::rightMotorDirPin2B, LOW);
    analogWrite(constants::speedPinRB, speed);
  }

  void RlBck(int speed) {
    if (!isValidSpeed(speed))
      return;
    digitalWrite(constants::leftMotorDirPin1B, LOW);
    digitalWrite(constants::leftMotorDirPin2B, HIGH);
    analogWrite(constants::speedPinLB, speed);
  }

  void RlFwd(int speed) {
    if (!isValidSpeed(speed))
      return;
    digitalWrite(constants::leftMotorDirPin1B, HIGH);
    digitalWrite(constants::leftMotorDirPin2B, LOW);
    analogWrite(constants::speedPinLB, speed);
  }

  void leftShift(int speedFlBck, int speedRlFwd, int speedRrBck, int speedFrFwd) {
    if (!isValidSpeed(speedFlBck) || !isValidSpeed(speedRlFwd) || !isValidSpeed(speedRrBck) || !isValidSpeed(speedFrFwd))
      return;

    FlBck(speedFlBck);
    RlFwd(speedRlFwd);
    FrFwd(speedFrFwd);
    RrBck(speedRrBck);
  }

  void rightShift(int speedFlFwd, int speedRlBck, int speedRrFwd, int speedFrBck) {
    if (!isValidSpeed(speedFlFwd) || !isValidSpeed(speedRlBck) || !isValidSpeed(speedRrFwd) || !isValidSpeed(speedFrBck))
      return;

    FlFwd(speedFlFwd);
    RlBck(speedRlBck);
    FrBck(speedFrBck);
    RrFwd(speedRrFwd);
  }

  void goForward(int speed) {
    if (!isValidSpeed(speed))
      return;
    setMotorForward(speed, speed);
    ourServo.write(constants::servoCenter);
  }

  void goBack(int speed) {
    if (!isValidSpeed(speed))
      return;
    setMotorBackward(speed, speed);
  }

  void leftTurn(int speed) {
    if (!isValidSpeed(speed))
      return;
    setMotorForward(0, speed);
    ourServo.write(constants::servoLeft);
  }

  void rightTurn(int speed) {
    if (!isValidSpeed(speed))
      return;
    setMotorForward(speed, 0);
    ourServo.write(constants::servoRight);
  }

  void initializePins() {
    // Motor control pins
    pinMode(constants::rightMotorDirPin1, OUTPUT);
    pinMode(constants::rightMotorDirPin2, OUTPUT);
    pinMode(constants::rightMotorDirPin1B, OUTPUT);
    pinMode(constants::rightMotorDirPin2B, OUTPUT);
    pinMode(constants::leftMotorDirPin1, OUTPUT);
    pinMode(constants::leftMotorDirPin2, OUTPUT);
    pinMode(constants::leftMotorDirPin1B, OUTPUT);
    pinMode(constants::leftMotorDirPin2B, OUTPUT);

    // Speed control pins
    pinMode(constants::speedPinL, OUTPUT);
    pinMode(constants::speedPinLB, OUTPUT);
    pinMode(constants::speedPinR, OUTPUT);
    pinMode(constants::speedPinRB, OUTPUT);

    // Sensor pins
    pinMode(constants::sensor1, INPUT);
    pinMode(constants::sensor2, INPUT);
    pinMode(constants::sensor3, INPUT);
    pinMode(constants::sensor4, INPUT);
    pinMode(constants::sensor5, INPUT);
    pinMode(constants::leftObstacleSensor, INPUT);
    pinMode(constants::rightObstacleSensor, INPUT);

    // Ultrasonic sensor pins
    pinMode(constants::trigPin, OUTPUT);
    pinMode(constants::echoPin, INPUT);

    stopMotors();
  }

  void moveServo(char direction) {
    switch (direction) {
      case 'L':
        servoCurrentAngle = constants::servoLeft;
        ourServo.write(constants::servoLeft);
        break;
      case 'R':
        servoCurrentAngle = constants::servoRight;
        ourServo.write(constants::servoRight);
        break;
      case 'F':
        servoCurrentAngle = constants::servoCenter;
        ourServo.write(constants::servoCenter);
        break;
      case 'D':
        servoCurrentAngle = constants::servoDown;
        ourServo.write(constants::servoDown);
        break;
      case 'U':
        servoCurrentAngle = constants::servoUp;
        ourServo.write(constants::servoUp);
        break;
      default:
        break;
    }
  }

  void forward(int speedLeft, int speedRight) {
    if (!isValidSpeed(speedLeft) || !isValidSpeed(speedRight))
      return;
    RlFwd(speedLeft);
    FlFwd(speedLeft);
    RrFwd(speedRight);
    FrFwd(speedRight);
  }

  void reverse(int speed) {
    if (!isValidSpeed(speed))
      return;
    RlBck(speed);
    RrBck(speed);
    FrBck(speed);
    FlBck(speed);
  }

  void right(int speed) {
    if (!isValidSpeed(speed))
      return;
    FrFwd(0);
    RrFwd(0);
    RlFwd(speed);
    FlFwd(speed);
  }

  void left(int speed) {
    if (!isValidSpeed(speed))
      return;
    RlFwd(0);
    FlFwd(0);
    RrFwd(speed);
    FrFwd(speed);
  }

  void sharpRightTurn(int speedLeft, int speedRight) {
    if (!isValidSpeed(speedLeft) || !isValidSpeed(speedRight))
      return;
    RlFwd(speedLeft);
    RrBck(speedRight);
    FrBck(speedRight);
    FlFwd(speedLeft);
  }

  void sharpLeftTurn(int speedLeft, int speedRight) {
    if (!isValidSpeed(speedLeft) || !isValidSpeed(speedRight))
      return;
    RlBck(speedLeft);
    RrFwd(speedRight);
    FrFwd(speedRight);
    FlBck(speedLeft);
  }

  void stopCar() {
    stopMotors();
    digitalWrite(constants::rightMotorDirPin1, LOW);
    digitalWrite(constants::rightMotorDirPin2, LOW);
    digitalWrite(constants::rightMotorDirPin1B, LOW);
    digitalWrite(constants::rightMotorDirPin2B, LOW);
    digitalWrite(constants::leftMotorDirPin1, LOW);
    digitalWrite(constants::leftMotorDirPin2, LOW);
    digitalWrite(constants::leftMotorDirPin1B, LOW);
    digitalWrite(constants::leftMotorDirPin2B, LOW);
    delay(constants::delayTime);
  }
  


  void printWifiStatus() {
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
    Serial.flush();
  }

public:
int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
const int distancelimit = 30; //distance limit for obstacles in front           
const int sidedistancelimit = 30; //minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)
int distance;
int numcycles = 0;
const int turntime = 250; //Time the robot spends turning (miliseconds)
const int backtime = 300; //Time the robot spends turning (miliseconds)

int thereis;
Servo ourServo;
  void auto_following() {
    int IRvalueLeft = digitalRead(constants::rightObstacleSensor);
    int IRvalueRight = digitalRead(constants::leftObstacleSensor);

    Serial.println("R sensorValue");
    Serial.println(IRvalueLeft);
    Serial.println("L sensorValue");
    Serial.println(IRvalueRight);

    if (IRvalueLeft == HIGH && IRvalueRight == HIGH) {
        // No obstacle detected, move forward
        forward(constants::speed, constants::speed);
    } else if (IRvalueLeft == LOW && IRvalueRight == LOW) {
        // Both sensors detect obstacles, stop and reverse slightly
        stopCar();
        reverse(constants::speed);
        delay(300);  // A short delay to back up safely
        stopCar();
    } else if (IRvalueLeft == LOW && IRvalueRight == HIGH) {
        // Obstacle detected by the left sensor, turn right
        stopCar();
        reverse(constants::speed);
        delay(300);
        right(constants::turnSpeed);
        delay(500);
    } else if (IRvalueLeft == HIGH && IRvalueRight == LOW) {
        // Obstacle detected by the right sensor, turn left
        stopCar();
        reverse(constants::speed);
        delay(300);
        left(constants::turnSpeed);
        delay(500);
    }
}


    int watch(){
      long echo_distance;
      digitalWrite(constants::trigPin,LOW);
      delayMicroseconds(5);                                                                              
      digitalWrite(constants::trigPin,HIGH);
      delayMicroseconds(15);
      digitalWrite(constants::trigPin,LOW);
      echo_distance=pulseIn(constants::echoPin,HIGH);
      echo_distance=echo_distance*0.01657; //how far away is the object in cm
      return round(echo_distance);
    }

  String watchsurrounding(){
  int obstacle_status =B100000;
  centerscanval = watch();
  if(centerscanval<distancelimit){
    stopCar();
    obstacle_status  =obstacle_status | B100;
    }
  ourServo.write(120);
  ldiagonalscanval = watch();
  if(ldiagonalscanval<distancelimit){
    stopCar();
    
     obstacle_status  =obstacle_status | B1000;
    }
  ourServo.write(170); //Didn't use 180 degrees because my servo is not able to take this angle
  leftscanval = watch();
  if(leftscanval<sidedistancelimit){
    stopCar();
    
     obstacle_status  =obstacle_status | B10000;
    }

  ourServo.write(90); //use 90 degrees if you are moving your servo through the whole 180 degrees
  centerscanval = watch();
  if( centerscanval<distancelimit){
    stopCar();
    
    obstacle_status  =obstacle_status | B100;
    }
  ourServo.write(40);
  rdiagonalscanval = watch();
  if(rdiagonalscanval<distancelimit){
    stopCar();
    
    obstacle_status  =obstacle_status | B10;
    }
  ourServo.write(0);
  rightscanval = watch();
  if(rightscanval<sidedistancelimit){
    stopCar();
    
    obstacle_status  =obstacle_status | 1;
    }
  ourServo.write(90); //Finish looking around (look forward again)
   String obstacle_str= String(obstacle_status,BIN);
  obstacle_str= obstacle_str.substring(1,6);
  
  return obstacle_str; //return 5-character string standing for 5 direction obstacle status
  }

  void trackLine() {

    int s0 = !digitalRead(constants::sensor1);
    int s1 = !digitalRead(constants::sensor2);
    int s2 = !digitalRead(constants::sensor3);
    int s3 = !digitalRead(constants::sensor4);
    // Ignore s4 (sensor5), treat as always OFF
    int sensorvalue = 16 + (s0 * 8 + s1 * 4 + s2 * 2 + s3);
    String senstr = String(sensorvalue, BIN).substring(1, 5); // Exclude s4

    Serial.print("Sensor values (ignoring last sensor): ");
    Serial.println(senstr);
    Serial.print("\t");
    if (senstr == "1000" || senstr == "0100" || senstr == "1100") {
      Serial.println("Shift Left");
      sharpLeftTurn(constants::lowSpeed, constants::midSpeed);
      delay(constants::delayTime);
      stopCar();
    } else if (senstr == "1110" || senstr == "1010") {
      Serial.println("Slight Shift Left");
      forward(0, constants::highSpeed);
      delay(constants::delayTime);
      stopCar();
    } else if (senstr == "0110" || senstr == "1111" || senstr == "1001" || senstr == "1011" || senstr == "1101") {
      Serial.println("Slight Left");
      forward(constants::lowSpeed, constants::midSpeed);
      delay(constants::delayTime);
    } else if (senstr == "0111" || senstr == "0101" || senstr == "0010" || senstr == "1000" || senstr == "1010" || senstr == "1001" || senstr == "1101" || senstr == "1011" || senstr == "1101" || senstr == "1100") {
      Serial.println("Forward");
      forward(constants::midSpeed, constants::midSpeed);
      delay(constants::delayTime);
      stopCar();
    } else if (senstr == "0011" || senstr == "0111" || senstr == "0100" || senstr == "0101" || senstr == "0110") {
      Serial.println("Slight Right");
      forward(constants::midSpeed, constants::midSpeed);
      delay(constants::delayTime);
      stopCar();
    } else if (senstr == "0010" || senstr == "0011") {
      Serial.println("Slight Shift to Right");
      forward(constants::midSpeed, 0);
      delay(constants::delayTime);
      stopCar();
    } else if (senstr == "0001") {
      Serial.println("Shift to Right");
      sharpRightTurn(constants::midSpeed, constants::lowSpeed);
      delay(constants::delayTime);
      stopCar();
    } else if (senstr == "0000") {
      stopCar();
    } else if (senstr == "1111") {
      Serial.println("Sharp Right U-Turn");
      sharpRightTurn(constants::midSpeed, constants::midSpeed);
      delay(constants::delayTime);
      stopCar();
    }
}
  void auto_avoidance(){
    ++ numcycles;
    String obstacle_sign=watchsurrounding();
    if(numcycles>=constants::LPT){
      stopCar();
      String obstacle_sign=watchsurrounding(); // 5 digits of obstacle_sign binary value means the 5 direction obstacle status
        Serial.print("begin str=");
          Serial.println(obstacle_sign);
          if(obstacle_sign)
          {
            stopCar();
          }
      if( obstacle_sign=="10000"){
      Serial.println("SLIT right");
            set_Motorspeed(constants::highSpeed,constants::speed,constants::highSpeed,constants::speed);
      forward(constants::speed,constants::speed);
  
        delay(turntime);
        stopCar();
      }
          else if( obstacle_sign=="00001"  ){
      Serial.println("SLIT LEFT");
        set_Motorspeed(constants::speed,constants::highSpeed,constants::speed,constants::highSpeed);
        forward(constants::speed,constants::speed);
    
        delay(turntime);
        stopCar();
      }
      else if( obstacle_sign=="11100" || obstacle_sign=="01000" || obstacle_sign=="11000"  || obstacle_sign=="10100"  || obstacle_sign=="01100" ||obstacle_sign=="00100"  ||obstacle_sign=="01000" ){
      Serial.println("hand right");
        right(constants::speed);
        set_Motorspeed(constants::turnSpeed,constants::turnSpeed,constants::turnSpeed,constants::turnSpeed);
        delay(turntime);
        stopCar();
      } 
      else if( obstacle_sign=="00010" || obstacle_sign=="00111" || obstacle_sign=="00011"  || obstacle_sign=="00101" || obstacle_sign=="00110" || obstacle_sign=="01010" ){
      Serial.println("hand left");
      left(constants::speed);//Turn left
      set_Motorspeed(constants::turnSpeed,constants::turnSpeed,constants::turnSpeed,constants::turnSpeed);
        delay(turntime);
        stopCar();
      }
  
      else if(  obstacle_sign=="01111" ||  obstacle_sign=="10111" || obstacle_sign=="11111"  ){
      Serial.println("hand back left");
      reverse(constants::speed);
      set_Motorspeed(constants::backSpeed1,constants::backSpeed2,constants::backSpeed1,constants::backSpeed2);
        delay(backtime);
            stopCar();
          } 
          else if( obstacle_sign=="11011"  ||    obstacle_sign=="11101"  ||  obstacle_sign=="11110"  || obstacle_sign=="01110"  ){
      Serial.println("hand back right");
    reverse(constants::speed);
      set_Motorspeed(constants::backSpeed2,constants::backSpeed1,constants::backSpeed2,constants::backSpeed1);
        delay(backtime);
            stopCar();
          }    
    
          else Serial.println("no handle");
      numcycles=0; //Restart count of cycles
    } else {
      set_Motorspeed(constants::speed,constants::speed,constants::speed,constants::speed);
      forward(constants::speed,constants::speed);  // if nothing is wrong go forward using go() function above.
          delay(backtime);
            stopCar();
    }
    

    distance =watch(); 
    if (distance<distancelimit){ 
  Serial.println("final go back");
    reverse(constants::speed);
    set_Motorspeed(constants::backSpeed1,constants::backSpeed2,constants::backSpeed1,constants::backSpeed2);
    delay(backtime);
        ++thereis;}
    if (distance>distancelimit){
        thereis=0;} //Count is restarted
    if (thereis > 25){
    Serial.println("final stop");
      stopCar(); // Since something is ahead, stop moving.
      thereis=0;
    }
  }
  void setup() {
    initializePins();
    ourServo.attach(constants::servoPin);
    ourServo.write(constants::servoCenter);

    // Initialize Serial communications
    Serial.begin(9600);
    while (!Serial) {
      ;  // Wait for serial port to connect
    }

    // Initialize ESP8266 Serial communication
    Serial1.begin(115200);
    Serial1.write("AT+UART_DEF=9600,8,1,0,0\r\n");
    delay(200);
    Serial1.write("AT+RST\r\n");
    delay(200);
    Serial1.begin(9600);
    WiFi.init(&Serial1);

    if (WiFi.status() == WL_NO_SHIELD) {
      Serial.println("WiFi shield not present");
      while (true) {
      };
    }

    status = WiFi.beginAP(ssid, 10, "", 0);
    Serial.println("You're connected to the network");
    printWifiStatus();

    // Start UDP
    Udp.begin(localPort);
    Serial.print("Listening on port ");
    Serial.println(localPort);
    Serial.flush();
  }

  void run() {
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      if (packetSize > constants::packetBufferSize - 1) {
        Serial.println("Packet too large");
        return;
      }

      Serial.print("Received packet of size ");
      Serial.println(packetSize);

      int len = Udp.read(packetBuffer, constants::packetBufferSize - 1);
      if (len > 0) {
        packetBuffer[len] = 0;  // Null terminate the string

        char command = packetBuffer[0];
        Serial.print("Command received: ");
        Serial.println(command);

        switch (command) {
          case 'A':
          isObstacleFollowing = false;
            isObstacleTracking = false;
            isLineTracking = false;
            goForward(constants::speed);
            moveServo('F');
            break;
          case 'L':
          isObstacleFollowing = false;
            isObstacleTracking = false;
            isLineTracking = false;
            leftTurn(constants::turnSpeed);
            moveServo('R');
            break;
          case 'R':
          isObstacleFollowing = false;
            isObstacleTracking = false;
            isLineTracking = false;
            rightTurn(constants::turnSpeed);
            moveServo('L');
            break;
          case 'B':
          isObstacleFollowing = false;
            isObstacleTracking = false;
            isLineTracking = false;
            goBack(constants::speed);
            moveServo('F');
            break;
          case 'E':
            isObstacleFollowing = false;
            isObstacleTracking = false;
            isLineTracking = false;
            stopMotors();
            break;
          case 'F':
          isObstacleFollowing = false;
            isObstacleTracking = false;
            isLineTracking = false;
            leftShift(0, 150, 0, 150);
            moveServo('U');
            break;
          case 'H':
          isObstacleFollowing = false;
            isObstacleTracking = false;
            isLineTracking = false;
            rightShift(150, 0, 150, 0);
            moveServo('D');
            break;
          case 'I':
          isObstacleFollowing = false;
            isObstacleTracking = false;
            isLineTracking = false;
            leftShift(150, 0, 150, 0);
            moveServo('U');
            break;
          case 'K':
          isObstacleFollowing = false;
            isObstacleTracking = false;
            isLineTracking = false;
            rightShift(0, 130, 0, 130);
            moveServo('D');
            break;
          case 'G':
          isObstacleFollowing = false;
            isObstacleTracking = false;
            isLineTracking = false;
            leftShift(200, 150, 150, 200);
            moveServo('R');
            break;
          case 'J':
          isObstacleFollowing = false;
            isObstacleTracking = false;
            isLineTracking = false;
            rightShift(200, 150, 150, 200);
            moveServo('L');
            break;
          case 'O':
          isObstacleTracking = false;
            isObstacleFollowing = true;
            isLineTracking = false;
            break;
          case 'T':
          isObstacleFollowing = false;
            isObstacleTracking = false;
            isLineTracking = true;
            trackLine();
            break;
          case 'Q':
          isObstacleTracking = true;
          isObstacleFollowing = false;
          isLineTracking = false;
            auto_avoidance();
            break;
          default:
            Serial.println("Unknown command received");
            break;
        }
      }
    }
  }
};

Car car;

void setup() {
  car.setup();
}

void loop() {
  car.run();
  if (isObstacleTracking) {
    car.auto_avoidance();}
  if (isObstacleFollowing) {
    car.auto_following();}
  if (isLineTracking) {
    car.trackLine();
  }
}
