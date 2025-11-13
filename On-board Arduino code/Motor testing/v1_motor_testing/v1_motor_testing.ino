#include <AccelStepper.h>
#include <RPLidar.h>
RPLidar lidar;

// SoftwareSerial BT(10, 11); // RX, TX (connect HC-05 TX to pin 10, RX to pin 11)

// === Stepper setup (A4988/DRV8825 drivers) ===
#define stepPinX 2
#define dirPinX 5
#define stepPinY 3
#define dirPinY 6
#define stepPinZ 4
#define dirPinZ 7
#define RPLIDAR_MOTOR 12

AccelStepper motorX(AccelStepper::DRIVER, stepPinX, dirPinX);
AccelStepper motorY(AccelStepper::DRIVER, stepPinY, dirPinY);
AccelStepper motorZ(AccelStepper::DRIVER, stepPinZ, dirPinZ);

// === Motion settings ===
float stepDistance = 4*300;   // steps to move per command
float motorSpeed = 200;     // steps per second (adjust for speed)
float motorAccel = 300;    // acceleration (smooth start/stop)

int commandOLD = 'N';
int instant_command_array[] = {"a","d","q","e","w","s"};

void setup() {
// Serial.begin(115200);
  lidar.begin(Serial1);
  //Serial2.begin(115200);
  Serial3.begin(115200);
  pinMode(RPLIDAR_MOTOR, OUTPUT);

  analogWrite(RPLIDAR_MOTOR, 80); // spin motor
  delay(1500);                     // let RPM stabilize
  lidar.startScan();

  rplidar_response_device_health_t h;
  //while (!IS_OK(lidar.getHealth(h)) || h.status != 0) {
    if(!IS_OK(lidar.getHealth(h)) || h.status != 0){
    Serial.println("Health not OK, attempting restart");
    lidar.stop();
    delay(200);
    lidar.startScan();
    }
   else {
    lidar.startScan();
  }
  
  
  // MOTORS
  motorX.setMaxSpeed(motorSpeed);
  motorY.setMaxSpeed(motorSpeed);
  motorZ.setMaxSpeed(motorSpeed);

  motorX.setAcceleration(motorAccel);
  motorY.setAcceleration(motorAccel);
  motorZ.setAcceleration(motorAccel);

  Serial.println("Ready for Bluetooth commands: w/s/a/d/q/e");
}

void loop() {
  if (commandOLD =="N"){
      if (Serial3.available()) { // checks if HC-05 has sent any data to the arduino
        commandOLD = Serial3.read(); // reads one character at a time
  }} 
  
  else{
    readBT();
    if (commandOLD == 'x'){}
    else if ((commandOLD=='a') or (commandOLD=='d')){
      processCommand('x');
      processCommand(commandOLD);
      // processCommand('x');
      commandOLD = 'x';
      }
    else{
      processCommand(commandOLD);}
  }
  
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; // mm
    int angle = lidar.getCurrentPoint().angle;       // degrees
    int quality = lidar.getCurrentPoint().quality;
    
    if (angle % 5 == 0){
      if(quality == 15){
        if (distance != 0){
          Serial3.print(angle);
          Serial3.print(",");
          //Serial.print(quality);
          Serial3.println((distance/10.0)*0.394,2); // convert to in
          //delay(50);
       }
    }
    }
  }
  
  // Run all motors simultaneously (non-blocking)
  motorX.run();
  motorY.run();
  motorZ.run();
}

void readBT(){
    if (Serial3.available()!=0){
      if (commandOLD!="none"){
        commandOLD = Serial3.read();
      }
    }
  }

void processCommand(char cmd) {
  if (cmd == 'w') { // Forward
    motorY.setMaxSpeed(motorSpeed);
    motorX.setMaxSpeed(motorSpeed);
    motorZ.setMaxSpeed(motorSpeed*1.05);
    motorX.move(-stepDistance/20);
    motorZ.move(stepDistance/20);
  }

  else if (cmd == 's') { // Backward
    motorY.setMaxSpeed(motorSpeed);
    motorX.setMaxSpeed(motorSpeed);
    motorZ.setMaxSpeed(motorSpeed*1.05);
    motorX.move(stepDistance/20);
    motorZ.move(-stepDistance/20);
  }

 else if (cmd == 'q') { // Rotate Left
    // motorX.setAcceleration(500);
    // motorY.setAcceleration(500);
    // motorZ.setAcceleration(500);

    motorY.setMaxSpeed(motorSpeed/2);
    motorX.setMaxSpeed(motorSpeed/2);
    motorZ.setMaxSpeed(motorSpeed/2);

    motorX.move(-stepDistance/24);
    motorY.move(-stepDistance/24);
    motorZ.move(-stepDistance/24);
  }

  else if (cmd == 'e') { // Rotate Right
    // motorX.setAcceleration(1000);
    // motorY.setAcceleration(1000);
    // motorZ.setAcceleration(1000);

    motorY.setMaxSpeed(motorSpeed/2);
    motorX.setMaxSpeed(motorSpeed/2);
    motorZ.setMaxSpeed(motorSpeed/2);

    motorX.move(stepDistance/24);
    motorY.move(stepDistance/24);
    motorZ.move(stepDistance/24);
  }

  else if (cmd == 'a') { // Slide Left
  // y goes 2x faster (not farther)
  motorY.setMaxSpeed(motorSpeed * 2);
  motorX.setMaxSpeed(motorSpeed);
  motorZ.setMaxSpeed(motorSpeed);

  motorY.move(stepDistance/4);
  motorX.move(-stepDistance/8);
  motorZ.move(-stepDistance/8);

}

else if (cmd == 'd') { // Slide Right
  // y goes 2x faster (not farther)
  motorY.setMaxSpeed(motorSpeed * 2);
  motorX.setMaxSpeed(motorSpeed);
  motorZ.setMaxSpeed(motorSpeed);

  motorY.move(-stepDistance/4);
  motorX.move(stepDistance/8);
  motorZ.move(stepDistance/8);
}

else if (cmd == 'x') { // stop
    motorX.stop();
    motorY.stop();
    motorZ.stop();
  }

 
  else {
    Serial.println("Unknown command.");
  }
}
