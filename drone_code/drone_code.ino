#include <MPU6050_tockn.h>
#include <Wire.h>
#include <WiFi.h>

IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);
WiFiServer tcpServer(8080);
WiFiClient client;

MPU6050 mpu6050(Wire);

const String FIRMWARE_VERSION = "3.0";

byte pinA = 4;
byte pinB = 5;
byte pinC = 3;
byte pinD = 6;

byte mode = 0;

const float MAX_ANGLE = 800;
const byte TURNING_THRUST_LIMIT = 120;
float P = 0.02;
float I = 0.00001;
float D = 5;

float yaw = 0;
float cmdYaw = 0;

float targetGyroX = 0;
float targetGyroY = 0;

float I_valX = 0;
float I_valY = 0;

int thrustA = 0;
int thrustB = 0;
int thrustC = 0;
int thrustD = 0;

bool propLock = false;

unsigned long lastTime = 0;
unsigned long lastGyroTime = 0;
unsigned long lastCom = 0;


void recalibrate(){
  digitalWrite(pinA, LOW); // ensure motors off
  digitalWrite(pinB, LOW);
  digitalWrite(pinC, LOW);
  digitalWrite(pinD, LOW);

  digitalWrite(7, HIGH); // LED blue
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);

  cmdYaw = 0;
  yaw = 0;

  Serial.println("Callibrating, please wait");

  mpu6050.calcGyroOffsets(true);

  digitalWrite(7, LOW); // LED green
  digitalWrite(8, LOW);
  digitalWrite(9, HIGH);
}
 
void setup() {
  Serial.begin(115200);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(7, HIGH); // LED blue
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);

  delay(3000);

  Wire.begin(11,10);
  mpu6050.begin();
  
  /* if failed (need to find new check condition) {
    Serial.println("Failed to find MPU6050 chip");
    digitalWrite(7, LOW); // LED red
    digitalWrite(8, HIGH);
    delay(2000);
    ESP.restart();
  }*/
  recalibrate();

  WiFi.softAPConfig(apIP, apIP, netMsk);
  WiFi.softAP("AeroHacks Drone 9", "skibidi123");
  tcpServer.begin();

  /*timer = timerBegin(1000000);   

  if (timer == NULL) {
      Serial.println("Error with the start of the timer");
      digitalWrite(7, LOW); // LED red
      digitalWrite(8, HIGH);
      while (1);
  }
  timerAttachInterrupt(timer, &gyro_update);
  timerStart(timer);*/

  Serial.println("ready");

  lastTime = micros();
  lastGyroTime = micros();
}








void loop() {
  mpu6050.update();

  float gyroX = mpu6050.getAngleX();
  float gyroY = mpu6050.getAngleY();
  float gyroVX = mpu6050.getGyroX();
  float gyroVY = mpu6050.getGyroY();
  float gyroVZ = mpu6050.getGyroZ();

  unsigned long newTime = micros();
  unsigned int dt_1000 = (newTime - lastTime);
  float dt = dt_1000;
  dt = dt / 1000;
  lastTime = newTime;
  

  if (gyroX > MAX_ANGLE or gyroX < -MAX_ANGLE or gyroY > MAX_ANGLE or gyroY < -MAX_ANGLE) {
    mode = 0;
    digitalWrite(8, HIGH);
  }

  if (!client) {client = tcpServer.available();}
  else if (!client.connected()) {
    client.stop();
    mode = 0;
  }

  if (client.available()) {
    String instruct = client.readStringUntil('\n');

    if (instruct == "ping") {
      client.print("ping");
    }

    else if (instruct == "angX") {client.print(String(gyroX));}
    else if (instruct == "angY") {client.print(String(gyroY));}
    else if (instruct == "gyroX") {client.print(String(gyroVX));}
    else if (instruct == "gyroY") {client.print(String(gyroVY));}
    else if (instruct == "gMode") {client.print(String(mode));}
    else if (instruct == "vers") {client.print(FIRMWARE_VERSION);}
    else if (instruct == "lb1") {digitalWrite(7, HIGH);}
    else if (instruct == "lb0") {digitalWrite(7, LOW);}
    else if (instruct == "lr1") {digitalWrite(8, HIGH);}
    else if (instruct == "lr0") {digitalWrite(8, LOW);}
    else if (instruct == "lg1") {digitalWrite(9, HIGH);}
    else if (instruct == "lg0") {digitalWrite(9, LOW);}
    else if (instruct == "rst") {recalibrate();}
    else if (instruct == "lck") {propLock = true;}
    
    else if (instruct.startsWith("mode")) {
      instruct.remove(0, 4);
      mode = instruct.toInt();
      Serial.print("New Mode: ");
      Serial.print(mode);
    }
    
    else if (instruct.startsWith("gx")) {
      instruct.remove(0, 2);
      targetGyroX = instruct.toInt();
    }
    
    else if (instruct.startsWith("gy")) {
      instruct.remove(0, 2);
      targetGyroY = instruct.toInt();
    }
    
    else if (instruct.startsWith("gainP")) {
      instruct.remove(0, 5);
      P = instruct.toFloat();
    }
    
    else if (instruct.startsWith("gainI")) {
      instruct.remove(0, 5);
      I = instruct.toFloat();
    }
    
    else if (instruct.startsWith("gainD")) {
      instruct.remove(0, 5);
      D = instruct.toFloat();
    }
    
    else if (instruct.startsWith("yaw")) {
      instruct.remove(0, 3);
      cmdYaw = instruct.toFloat();
    }
    
    else if (instruct == "irst") {
      I_valX = 0;
      I_valY = 0;
    }

    else if (instruct == "geti"){
      client.print(I_valX);
      client.print(',');
      client.print(I_valY);
    }

    else if (instruct == "manT") {
      thrustA = client.readStringUntil(',').toInt();
      thrustB = client.readStringUntil(',').toInt();
      thrustC = client.readStringUntil(',').toInt();
      thrustD = client.readStringUntil('\n').toInt();
    }

    else if (instruct == "incT") {
      thrustA += client.readStringUntil(',').toInt();
      thrustB += client.readStringUntil(',').toInt();
      thrustC += client.readStringUntil(',').toInt();
      thrustD += client.readStringUntil('\n').toInt();
    }




    client.print("\n");
    lastCom = millis();
  }

  //if (millis() - lastCom > 4000) {
  //  mode = 0;
  //}

  float thrustOffA = 0;
  float thrustOffB = 0;
  float thrustOffC = 0;
  float thrustOffD = 0;

  if (mode == 2){
    if (gyroVZ > cmdYaw) {yaw += 0.1;}
    else if (gyroVZ < cmdYaw) {yaw -= 0.1;}

    I_valX += (gyroX - targetGyroX) * dt;
    I_valY += (gyroY - targetGyroY) * dt;

    thrustOffA -= P * (gyroX - targetGyroX) * dt;
    thrustOffB -= P * (gyroX - targetGyroX) * dt;
    thrustOffC += P * (gyroX - targetGyroX) * dt;
    thrustOffD += P * (gyroX - targetGyroX) * dt;

    thrustOffA -= I * I_valX * dt;
    thrustOffB -= I * I_valX * dt;
    thrustOffC += I * I_valX * dt;
    thrustOffD += I * I_valX * dt;

    thrustOffA += D * gyroVX * dt;
    thrustOffB += D * gyroVX * dt;
    thrustOffC -= D * gyroVX * dt;
    thrustOffD -= D * gyroVX * dt;


    thrustOffA -= P * (gyroY - targetGyroY) * dt;
    thrustOffB += P * (gyroY - targetGyroY) * dt;
    thrustOffC -= P * (gyroY - targetGyroY) * dt;
    thrustOffD += P * (gyroY - targetGyroY) * dt;

    thrustOffA -= I * I_valY * dt;
    thrustOffB += I * I_valY * dt;
    thrustOffC -= I * I_valY * dt;
    thrustOffD += I * I_valY * dt;

    thrustOffA += D * gyroVY * dt;
    thrustOffB -= D * gyroVY * dt;
    thrustOffC += D * gyroVY * dt;
    thrustOffD -= D * gyroVY * dt;
  }


  if (thrustA < 0) {thrustA = 0;}
  if (thrustB < 0) {thrustB = 0;}
  if (thrustC < 0) {thrustC = 0;}
  if (thrustD < 0) {thrustD = 0;}
  if (thrustA > 200) {thrustA = 200;}
  if (thrustB > 200) {thrustB = 200;}
  if (thrustC > 200) {thrustC = 200;}
  if (thrustD > 200) {thrustD = 200;}


  if (mode == 0) {
    yaw = 0;
    thrustA = 0;
    thrustB = 0;
    thrustC = 0;
    thrustD = 0;
  }

  if (mode <= 1){
    thrustOffA = 0;
    thrustOffB = 0;
    thrustOffC = 0;
    thrustOffD = 0;
  }

  if (thrustOffA < -TURNING_THRUST_LIMIT) {thrustOffA = -TURNING_THRUST_LIMIT;}
  if (thrustOffB < -TURNING_THRUST_LIMIT) {thrustOffB = -TURNING_THRUST_LIMIT;}
  if (thrustOffC < -TURNING_THRUST_LIMIT) {thrustOffC = -TURNING_THRUST_LIMIT;}
  if (thrustOffD < -TURNING_THRUST_LIMIT) {thrustOffD = -TURNING_THRUST_LIMIT;}
  if (thrustOffA > TURNING_THRUST_LIMIT) {thrustOffA = TURNING_THRUST_LIMIT;}
  if (thrustOffB > TURNING_THRUST_LIMIT) {thrustOffB = TURNING_THRUST_LIMIT;}
  if (thrustOffC > TURNING_THRUST_LIMIT) {thrustOffC = TURNING_THRUST_LIMIT;}
  if (thrustOffD > TURNING_THRUST_LIMIT) {thrustOffD = TURNING_THRUST_LIMIT;}

  if (yaw > 80) {yaw = 80;}
  if (yaw < -80) {yaw = -80;}

  int newThrustA = thrustA + thrustOffA - yaw;
  int newThrustB = thrustB + thrustOffB + yaw;
  int newThrustC = thrustC + thrustOffC + yaw;
  int newThrustD = thrustD + thrustOffD - yaw;


  if (newThrustA < 0) {newThrustA = 0;}
  if (newThrustB < 0) {newThrustB = 0;}
  if (newThrustC < 0) {newThrustC = 0;}
  if (newThrustD < 0) {newThrustD = 0;}
  if (newThrustA > 250) {newThrustA = 250;}
  if (newThrustB > 250) {newThrustB = 250;}
  if (newThrustC > 250) {newThrustC = 250;}
  if (newThrustD > 250) {newThrustD = 250;}

  if (propLock) {
    newThrustA = 0;
    newThrustB = 0;
    newThrustC = 0;
    newThrustD = 0;
  }

  analogWrite(pinA, newThrustA);
  analogWrite(pinB, newThrustB);
  analogWrite(pinC, newThrustC);
  analogWrite(pinD, newThrustD);

  //Serial.println(dt);
}
