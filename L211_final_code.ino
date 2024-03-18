#include <Adafruit_MotorShield.h>
#include "DFRobot_TCS34725.h"
DFRobot_TCS34725 tcs = DFRobot_TCS34725(&Wire, TCS34725_ADDRESS, TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#include <Servo.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

int lineSensorPin1 = 2; //store the location of the line sensors. The numbers show the location counting from the left
int lineSensorPin2 = 3;
int lineSensorPin3 = 4;
int lineSensorPin4 = 5;

int crashSwitchPin = 6; //used for detecting the block/platform

int led2 = 7; //green led is pin 7
int led3 = 8; //red led is pin 8

int button = 10; //for starting the program



//store the information regarding the next action
int goStraight = 0;
int turnLeft = 0;
int turnRight = 0;
int lineEnd = 0;
int stopMotor = 0;
int atDelivery = 0;
int turnLeftW = 0;
int turnRightW = 0;

int skip = 0; //whether the current junction should be skipped

int hold = 0; //0 if no block held, 1 if block held
int colour = 0; //0 for green, 1 for red
int delivered = 0; //no blocks delivered

int started = 0; //1, if the button has been pressed

int step = 0; //which step of the path is the AGV currently on
int pathLength = 0; //length of the current path
int nextMove; //next step according to the bath

const double orgRatio = 1.25; //ratio between the wheel speeds. It was determined experimentally.
double ratio = 1.25;

int time = 0; //time since the last change in blue diode
int diode = 0; //status of the blue diode
int ledPin = 9; //location of the blue diode

Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);

Servo myservo1; //create servo object to control a servo
Servo myservo2; //create servo object to control a servo

int servoLift = 0; //variable to store the servo position
int servoLift1 = 0; 

int lastSpeed1 = 120; //stores the last speed of the motors
int lastSpeed2 = 120;

void pick() {
  releaseBlock(); //open the claws
  delay(1000);
  dropBlock(); //tilt the grabber box down
  delay(1000);  
  grabBlock(); //close the claws
  delay(1000);
  liftBlock(); //lift the grabber box
}

void dropBlock(){ //tilts the grabber down
  for (int i = 0; i < 1; i++) {
    for (servoLift1 = 22; servoLift1 <= 50 ; servoLift1 += 1) {//angle goes from 22 degrees to 50 degrees
      myservo2.write(servoLift1);
      delay(100);
    } 
  } 
}


void liftBlock(){
  for (int l = 0; l < 1; l++) {
    for (servoLift1 = 50; servoLift1 >= 22 ; servoLift1 -= 1) {//angle goes from 50 degrees to 22 degrees
      myservo2.write(servoLift1);
      delay(100);
    } 
  }   
}

void grabBlock(){
  for (int j = 0; j < 1; j++) {
    for (servoLift = 165; servoLift >= 110 ; servoLift -= 1) {//angle goes from 165 degrees to 110 degrees
      myservo1.write(servoLift); 
      delay(50);
    } 
  } 
}

void releaseBlock(){ 
  for (int k = 0; k < 1; k++) {
    for (servoLift = 110; servoLift <= 165 ; servoLift += 1) {//angle goes from 110 degrees to 65 degrees
      myservo1.write(servoLift);
      delay(50);
    } 
  }
}

void flashDelay(int del) {
  //the function replaces delay while travelling. It splits the delay time into intervals of 10ms and switches the LED every time the "time" variable reaches 500
  int intervals = del/10;
  int i = 0;
  for (int i = 0; i<intervals; i++) {
    time += 10;
    delay(10);
    if (time == 500) {
      diode = (diode+1)%2;
      time = 0;
      digitalWrite(ledPin,diode);
    }
  }
}

void stop() {
  //turns the diode on, used when the agv stops
  diode = 0;
  time = 0;
  digitalWrite(ledPin,diode);
}

void chooseDirection(int valSens1,int valSens2,int valSens3,int valSens4, int nextMove, int crash) {
  //the function uses the values from the sensors and the predetermined path to choose what action should the AGV take
  if (valSens1 == 1 && (nextMove == 1 || nextMove == 3)) {
    lineEnd = 0;
    goStraight = 0;
    turnRightW = 0;
    turnRight = 0;
    if (nextMove == 1) {
      turnLeft = 1;
      turnLeftW = 0;
    } else {
      turnLeftW = 1;
      turnLeft = 0;
    }
  } else if (valSens4 == 1 && (nextMove == 2 || nextMove == 4)) {
    lineEnd = 0;
    goStraight = 0;
    turnLeft = 0;
    turnLeftW = 0;
    if (nextMove == 2) {
      turnRight = 1;
      turnRightW = 0;
   } else {
      turnRightW = 1;
      turnRight = 0;
    }
  }  else if (((step >= pathLength -1) && crash == 0)) {
    //this means that the robot has reached the block/platform
    lineEnd = 1;
    goStraight = 0;
    turnLeft = 0;
    turnRight = 0;
    turnLeftW = 0;
    turnRightW = 0;
  } else if (delivered == 4 && crash == 0) {
    //this means the AGV finished the task and is back at the starting point
    stopMotor = 1;
  } else {
    lineEnd = 0;
    goStraight = 1;
    turnLeft = 0;
    turnRight = 0;
    turnLeftW = 0;
    turnRightW = 0;
    if(nextMove == 0) {
      skip = 1;
    } else {
      skip = 0;
    }
  }
}

void forward(int valSens2, int valSens3) {
  //function for making the AGV move forward
  myMotor1->run(BACKWARD);
  myMotor2->run(BACKWARD);
  if (valSens2 == 1 && valSens3 == 1) {
    if ((step >= pathLength -1)&& hold == 0) {
    //the AGV moves at a slower speed when approaching the block, this is in order to give it more time to correct its path
      if (delivered == 3 && hold == 0) {
        //the exception is due to the step present on the board. In order to be able to go over it, the AGV needed to move at a higher speed.
        myMotor1->setSpeed(200);
        myMotor2->setSpeed(200*ratio);
        lastSpeed1 = 200;
        lastSpeed2 = 200*ratio;
      } else {
        myMotor1->setSpeed(130);
        myMotor2->setSpeed(130*ratio);
        lastSpeed1 = 130;
        lastSpeed2 = 130*ratio;
      }

    } else {
      myMotor1->setSpeed(180);
      myMotor2->setSpeed(180*ratio);
      lastSpeed1 = 200;
      lastSpeed2 = 200*ratio;
    }

  //following two cases are for when the AGV starts going off the line and needs to correct
  } else if (valSens2 == 0 && valSens3 == 1) {
    myMotor1->setSpeed(140+40);
    myMotor2->setSpeed((140-40)*ratio);
    lastSpeed1 = 140+40;
    lastSpeed2 = (140-40)*ratio;
  } else if (valSens2 == 1 && valSens3 == 0) {
    myMotor1->setSpeed(140-40);
    myMotor2->setSpeed((140+40)*ratio);
    lastSpeed1 = 140-40;
    lastSpeed2 = (140+40)*ratio;
  } else {
    //if both the sensors are off the line the AGV continues correcting in the same way as before, giving it a chance to return to the line
    myMotor1->setSpeed(lastSpeed1);
    myMotor2->setSpeed(lastSpeed2);
  }
  flashDelay(10);
}

void correction() {
  //used after turns, to prevent the AGV from making a second turn due to one of the edge sensors ending up over the line
  int valSens1 = digitalRead(lineSensorPin1);
  int valSens4 = digitalRead(lineSensorPin4);  
  myMotor1->run(FORWARD);
  myMotor2->run(BACKWARD);
  myMotor1->setSpeed(100);
  myMotor2->setSpeed(100*ratio);
  while (valSens1 == 1) {
    flashDelay(10);
    valSens1 = digitalRead(lineSensorPin1);  
  }
  myMotor1->run(BACKWARD);
  myMotor2->run(FORWARD);
  while (valSens4 == 1) {
    flashDelay(10);
    valSens4 = digitalRead(lineSensorPin4);  
  }
}

void right() {
  //used for turning right. The AGV first moves forward, then makes a predetermined turn to get the central sensors off the line and then continues to turn until they reach the line it should follow after the turn
  int valSens2 = digitalRead(lineSensorPin2);
  int valSens3 = digitalRead(lineSensorPin3);
  myMotor1->run(BACKWARD);
  myMotor2->run(BACKWARD);
  myMotor1->setSpeed(120);
  myMotor2->setSpeed(120*ratio);
  flashDelay(750);
  //predetermined turn
  myMotor1->run(BACKWARD);
  myMotor2->run(FORWARD);
  myMotor1->setSpeed(150);
  myMotor2->setSpeed(150*ratio);
  flashDelay(500);
  //turning until the line is reached
  myMotor1->setSpeed(150);
  myMotor2->setSpeed(150*ratio);
  valSens2 = digitalRead(lineSensorPin2);
  valSens3 = digitalRead(lineSensorPin3);
  while (valSens2 == 0) {
    valSens2 = digitalRead(lineSensorPin2);
    flashDelay(10);
  }
  flashDelay(50);
  correction();
  myMotor1->setSpeed(0);
  myMotor2->setSpeed(0);
}

void left() {
  //as above, but turning left instead of right
  int valSens2 = digitalRead(lineSensorPin2);
  int valSens3 = digitalRead(lineSensorPin3);
  myMotor1->run(BACKWARD);
  myMotor2->run(BACKWARD);
  myMotor1->setSpeed(120);
  myMotor2->setSpeed(120*ratio);
  flashDelay(500);
  myMotor1->run(FORWARD);
  myMotor2->run(BACKWARD);
  myMotor1->setSpeed(150);
  myMotor2->setSpeed(150*ratio);
  flashDelay(500);
  myMotor1->setSpeed(150);
  myMotor2->setSpeed(150*ratio);
  valSens2 = digitalRead(lineSensorPin2);
  valSens3 = digitalRead(lineSensorPin3);
  while (valSens3 == 0) {
    valSens2 = digitalRead(lineSensorPin2);
    valSens3 = digitalRead(lineSensorPin3);
    flashDelay(10);
  }
  flashDelay(50);
  correction();
  myMotor1->setSpeed(0);
  myMotor2->setSpeed(0);
}

void wallLeft() {
  //used for turning left at the wall. The initial reversing and turning in several stages prevent the AGV from crashing at the wall
  int valSens2 = digitalRead(lineSensorPin2);
  int valSens3 = digitalRead(lineSensorPin3);

  //reversing
  ratio = 1.6; //it was determined experimentally that such a ratio works better in the case of reversing
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  myMotor1->setSpeed(100);
  myMotor2->setSpeed(100*ratio);
  flashDelay(530);
  ratio = orgRatio;

  //turning
  myMotor1->run(FORWARD);
  myMotor2->run(BACKWARD);
  myMotor1->setSpeed(140);
  myMotor2->setSpeed(140*ratio);
  flashDelay(600);

  //forward
  myMotor1->run(BACKWARD);
  myMotor2->run(BACKWARD);
  myMotor1->setSpeed(140);
  myMotor2->setSpeed(140*ratio);
  flashDelay(500);

  //turning
  myMotor1->run(FORWARD);
  myMotor2->run(BACKWARD);
  myMotor1->setSpeed(160);
  myMotor2->setSpeed(160*ratio);
  valSens2 = digitalRead(lineSensorPin2);
  valSens3 = digitalRead(lineSensorPin3);
  while (valSens3 == 0) {
    valSens2 = digitalRead(lineSensorPin2);
    valSens3 = digitalRead(lineSensorPin3);
    flashDelay(20);
  }

  //forward
  myMotor1->run(BACKWARD);
  myMotor2->run(BACKWARD);
  myMotor1->setSpeed(160);
  myMotor2->setSpeed(160*ratio);
  flashDelay(300);

  //turning
  myMotor1->run(FORWARD);
  myMotor2->run(BACKWARD);
  myMotor1->setSpeed(160);
  myMotor2->setSpeed(160*ratio);
  valSens2 = digitalRead(lineSensorPin2);
  valSens3 = digitalRead(lineSensorPin3);
  while (valSens3 == 0) {
    valSens2 = digitalRead(lineSensorPin2);
    valSens3 = digitalRead(lineSensorPin3);
    flashDelay(20);
  }

  flashDelay(150);
  correction();
  myMotor1->setSpeed(0);
  myMotor2->setSpeed(0);
  stop();
  delay(1000);

  
}

void wallRight() { 
  //same as above, but turning right instead
  int valSens2 = digitalRead(lineSensorPin2);
  int valSens3 = digitalRead(lineSensorPin3);

  //reversing
  ratio = 1.6;
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  myMotor1->setSpeed(100);
  myMotor2->setSpeed(100*ratio);
  flashDelay(530);
  ratio = orgRatio;

  //turning
  myMotor1->run(BACKWARD);
  myMotor2->run(FORWARD);
  myMotor1->setSpeed(140);
  myMotor2->setSpeed(140*ratio);
  flashDelay(600);

  //forward
  myMotor1->run(BACKWARD);
  myMotor2->run(BACKWARD);
  myMotor1->setSpeed(140);
  myMotor2->setSpeed(140*ratio);
  flashDelay(500);

  //turning
  myMotor1->run(BACKWARD);
  myMotor2->run(FORWARD);
  myMotor1->setSpeed(160);
  myMotor2->setSpeed(160*ratio);
  valSens2 = digitalRead(lineSensorPin2);
  valSens3 = digitalRead(lineSensorPin3);
  while (valSens2 == 0) {
    valSens2 = digitalRead(lineSensorPin2);
    valSens3 = digitalRead(lineSensorPin3);
    flashDelay(20);
  }

  //forward
  myMotor1->run(BACKWARD);
  myMotor2->run(BACKWARD);
  myMotor1->setSpeed(160);
  myMotor2->setSpeed(160*ratio);
  flashDelay(300);

  //turning
  myMotor1->run(BACKWARD);
  myMotor2->run(FORWARD);
  myMotor1->setSpeed(160);
  myMotor2->setSpeed(160*ratio);
  valSens2 = digitalRead(lineSensorPin2);
  valSens3 = digitalRead(lineSensorPin3);
  while (valSens2 == 0) {
    valSens2 = digitalRead(lineSensorPin2);
    valSens3 = digitalRead(lineSensorPin3);
    flashDelay(20);
  }

  flashDelay(150);
  correction();
  myMotor1->setSpeed(0);
  myMotor2->setSpeed(0);
  stop();
  delay(1000);
}

void reverse() {
  //used after a block or a platform is encountered
  int valSens1 = digitalRead(lineSensorPin1);
  int valSens2 = digitalRead(lineSensorPin2);
  int valSens3 = digitalRead(lineSensorPin3);
  int valSens4 = digitalRead(lineSensorPin4);
  ratio = 1.6; //it was determined experimentally that such a ratio works better in the case of reversing
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  myMotor1->setSpeed(120);
  myMotor2->setSpeed(120*ratio);
  flashDelay(100);
  myMotor1->setSpeed(0);
  myMotor2->setSpeed(0);
  stop();
  
  if (hold == 0) {
    //if no block is held (meaning that a block was just approached) the AGV first determines the colour of the new block, signals it with the light and then picks it up
    colour = getColour();
    delay(5000); //waiting for 5000ms after detecting the block was a requirement
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
    dropBlock();
    delay(100);
    grabBlock();
    //the AGV reverses slightly after grabbing to prevent the block from hitting the pavement while being lifted
    myMotor1->setSpeed(100);
    myMotor2->setSpeed(100*ratio);
    flashDelay(250);
    myMotor1->setSpeed(0);
    myMotor2->setSpeed(0);
    liftBlock();
  } else {
    //if a block is held, it is released onto the platform
    releaseBlock();
    delay(100);
  }
  //used when the AGV returns to the starting point
  if (delivered == 4) {
    stopMotor == 1;
    Serial.println("Task completed");
  } else {
  step = 0;
  delivered += hold;
  hold = (hold+1)%2;
  }  
  nextMove = moveCheck(step);


  //the AGV moves backwards until it encounters a line on which it could turn
  while (!(valSens1 == 1 && nextMove == 1 || valSens4 == 1 && nextMove == 2)) {
      myMotor1->setSpeed(140);
      myMotor2->setSpeed(140*ratio);

    flashDelay(50); 
    valSens1 = digitalRead(lineSensorPin1);
    valSens2 = digitalRead(lineSensorPin2);
    valSens3 = digitalRead(lineSensorPin3);
    valSens4 = digitalRead(lineSensorPin4);
  }
  myMotor1->setSpeed(0);
  myMotor2->setSpeed(0);
  stop();
  delay(500);
  //the wheel ratio is returned to it's original value, which is more suitable for moving forwards
  ratio = orgRatio;
}

int moveCheck(int step) {
  //this function returns the movement the block should take at the next junction
  if (delivered == 0) {
    if (hold == 0) {
      int path[4] = {0, 1,2,0}; //there is a group of predetermined paths. 0 means skipping the junction,1 means turning left, 2 right, 3 left at a wall and 4- right at a wall.
      pathLength = sizeof(path)/sizeof(path[0]);
      return (path[step]);
    } else {
      if (colour == 0) { 
        int path[3] = {1,3,0};
        pathLength = sizeof(path)/sizeof(path[0]);
        return (path[step]);
      } else {
        int path[4] = {2,0,4,0};
        pathLength = sizeof(path)/sizeof(path[0]);
        return (path[step]);
      }
    }
  } else if (delivered == 3) {
    if (hold == 0) {
      if (colour == 0) { //when no block is held, the colour refers to the colour of the last delivered block
        int path[7] = {1,0,0,3,1,1,0};
        pathLength = sizeof(path)/sizeof(path[0]);
        return (path[step]);
      } else {
        int path[8] = {2,0,0,4,2,0,2,0};
        pathLength = sizeof(path)/sizeof(path[0]);
        return (path[step]);
      }
    } else {
      if (colour == 0) {
        int path[5] = {2,0,3,0,0};
        pathLength = sizeof(path)/sizeof(path[0]);
        return (path[step]);
      } else {
        int path[4] = {1,4,0,0};
        pathLength = sizeof(path)/sizeof(path[0]);
        return (path[step]);
      }
    }
  } else if (delivered == 2) {
    if (hold == 0) {
      if (colour == 0) {
        int path[11] = {1,0,0,3,1,0,2,0,4,2,0};
        pathLength = sizeof(path)/sizeof(path[0]);
        return (path[step]);
      } else {
        int path[10] = {2,0,0,4,2,1,0,4,2,0};
        pathLength = sizeof(path)/sizeof(path[0]);
        return (path[step]);
      }
    } else {
      if (colour == 0) {
        int path[7] = {2,1,0,4,3,0,0};
        pathLength = sizeof(path)/sizeof(path[0]);
        return (path[step]);
      } else {
        int path[9] = {2,1,0,4,3,1,0,0,4};
        pathLength = sizeof(path)/sizeof(path[0]);
        return (path[step]);
      }
    }
  } else if (delivered == 1) {
    if (hold == 0) {
      if (colour == 0) {
        int path[9] = {1,0,0,3,1,0,2,1,0};
        pathLength = sizeof(path)/sizeof(path[0]);
        return (path[step]);       
      } else {
        int path[7] = {2,0,0,4,2,1,1};
        pathLength = sizeof(path)/sizeof(path[0]);
        return (path[step]);
      }
    } else {
      if (colour == 0) {
        int path[5] = {1,4,3,0,0};
        pathLength = sizeof(path)/sizeof(path[0]);
        return (path[step]);
      } else {
        int path[8] = {1,4,3,1,0,0,4,0};
        pathLength = sizeof(path)/sizeof(path[0]);
        return (path[step]);
      }
    }
  } else if (delivered == 4) {
      if (colour == 0) {
        int path[4] = {1,0,2,0};
        pathLength = sizeof(path)/sizeof(path[0]);
        return (path[step]);
      } else {
        int path[3] = {2,1,0};
        pathLength = sizeof(path)/sizeof(path[0]);
        return (path[step]);
      }
  }
  Serial.println(pathLength);
}

int getColour()  {
  //uses the colour sensor to return the
  uint16_t clear, red, green, blue;
  tcs.getRGBC(&red, &green, &blue, &clear);
  // tcs.lock(); // What does this line do?
  /*
  Serial.print("C:\t"); Serial.print(clear);
  Serial.print("\tR:\t"); Serial.print(red);
  Serial.print("\tG:\t"); Serial.print(green);
  Serial.print("\tB:\t"); Serial.print(blue);
  Serial.println("\t");
*/
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  /*
  Serial.print("\t");
  Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
  Serial.println();
*/
  // Check conditions and return color value
  if ( red > 0.55 * clear) // Change the parameter here after testing
  {
    digitalWrite(led2, LOW);
    digitalWrite(led3, HIGH);
    return 1; // Colour is red
  }
  else {
    digitalWrite(led3, LOW);
    digitalWrite(led2, HIGH);
    return 0; // Colour is green
  }
}
void setup() {
  //sets the serial output, sensors and servos
  Serial.begin(9600);
  Serial.println("in setup");
  pinMode(lineSensorPin1, INPUT);
  pinMode(lineSensorPin2, INPUT);
  pinMode(lineSensorPin3, INPUT);
  pinMode(lineSensorPin4, INPUT);
  pinMode(crashSwitchPin, INPUT);

  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);

  pinMode(button, INPUT);
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }

  while (!tcs.begin())
  {
    Serial.println("No TCS34725 found ... check your connections");
    delay(1000);
  }
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  myservo1.attach(13); // attaches the servo on pin 9 to the servo object. servo 9 is attached to box
  myservo2.attach(11); // attaches the servo on pin 10 to the servo object. servo 10 is attached to grabber
  
  //sets the initial positions of the servos
  myservo2.write(22);
  myservo1.write(165);

}



void loop() {
  //Serial.println("IN the loop");
  //no action is taken until the button has been pressed
  if (started == 0) {
    //when the button is pressed, the started function permanently becomes 1 and the button is no longer used
    int pressed = digitalRead(button);
    if (pressed == 1) {
      started = 1;
    }
    delay(10);
  } else {
    int valSens1 = digitalRead(lineSensorPin1);
    int valSens2 = digitalRead(lineSensorPin2);
    int valSens3 = digitalRead(lineSensorPin3);
    int valSens4 = digitalRead(lineSensorPin4);
    int crash = digitalRead(crashSwitchPin);

    //the function first runs previous functions to determine the next step and then acts according to their output 
    nextMove = moveCheck(step);
    chooseDirection(valSens1,valSens2,valSens3,valSens4,nextMove,crash);

    Serial.println(step);
    //Serial.println("moving");
    if (turnLeft == 1 && nextMove == 1) {
      left();
      step +=1; //the AGV moves to the next step of it's path
      Serial.println(step);
    } else if (turnRight == 1 && nextMove == 2) {
      right();
      step +=1;
      Serial.println(step);
    } else if (goStraight == 1){
      forward(valSens2,valSens3);
      if ((valSens1 == 1 || valSens4 == 1) && skip == 1) {
        step += 1;
        Serial.println(step);
        flashDelay(350); //this is done so that the AGV doesn't immediately perform the next step on the junction it is skipping
      }
    } else if (lineEnd == 1) {
      reverse();
    } else if (turnRightW == 1 && nextMove == 4) {
      step+=1;
      wallRight();
    } else if (turnLeftW == 1 && nextMove == 3) {
      step+=1;
      wallLeft();
    } 
    //Serial.println(nextMove);

  }
}

