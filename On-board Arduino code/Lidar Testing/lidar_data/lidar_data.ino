#include <RPLidar.h>
RPLidar lidar;
#define RPLIDAR_MOTOR 8
// #include <SoftwareSerial.h>
// SoftwareSerial BT(10, 11); // RX, TX

void setup() {
  // Serial.begin(115200);
  lidar.begin(Serial1);
  Serial2.begin(115200);
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
}

void loop() {
  if Serial2.available(){
    Serial3.print(Serial2.read()); // pass on information received from HC-05 to arduino uno
  }
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; // mm
    int angle = lidar.getCurrentPoint().angle;       // degrees
    int quality = lidar.getCurrentPoint().quality;
    
    if (angle % 5 == 0){
      if(quality == 15){
        if (distance != 0){
          Serial2.print(angle);
          Serial2.print(",");
          //Serial.print(quality);
          Serial2.println((distance/10.0)*0.394,2); // convert to in
          delay(50);
       }
    }
    }
    

  }
}
