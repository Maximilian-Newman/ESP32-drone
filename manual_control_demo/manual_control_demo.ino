#include <WiFi.h>

String ssid = "";
String password = "skibidi123";
WiFiClient client;

bool rightIsClicked = false;
bool leftIsClicked = false;
bool yawMode = false;

int yaw = 0;
int targetX = 0;
int targetY = 0;

unsigned int PERIOD = 200;
unsigned long nextTime = 0;

long joyOffsetX = 0;
long joyOffsetY = 0;

void recalibrate_joystick() {
  joyOffsetX = 0;
  joyOffsetX = 0;

  for (byte i=0; i<200; i++) {
    joyOffsetX += analogRead(35);
    joyOffsetY += analogRead(32);
    delay(1);
  }

  joyOffsetX /= 200;
  joyOffsetY /= 200;
}

String msg(String tx) {
  while (client.available()) {client.read();}
  client.print(tx + "\n");
  Serial.println("sent: " + tx);
  while (!client.available()) {delay(1);};
  return client.readStringUntil('\n');
}

void increment_base_thrust(int val) {
  String sval = String(val);
  msg("incT\n" + sval + "," + sval + "," + sval + "," + sval);
}

void set_base_thrust(int val) {
  String sval = String(val);
  msg("manT\n" + sval + "," + sval + "," + sval + "," + sval);
}


void setup() {
  Serial.begin(115200);
  Serial.println("\n\n");

  pinMode(27, INPUT);
  pinMode(33, INPUT);
  pinMode(35, INPUT);
  pinMode(32, INPUT);
  pinMode(34, INPUT);

  Serial.println("calibrating joystick ...");
  recalibrate_joystick();
  Serial.println("done joystick calibration");

  Serial.println("Enter ssid  toconnect");
  while (Serial.available()) {Serial.read();}
  while (!Serial.available()) {delay(10);}
  ssid = Serial.readStringUntil('\n');
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.println("...");
  }

  Serial.print("WiFi connected with IP:");
  Serial.println(WiFi.localIP());

  delay(2000);

  if(!client.connect(IPAddress(192, 168, 4, 1), 8080)){
    Serial.println("Connection to host failed");
    while (true);
  }

  Serial.println("Connected to server");
  Serial.print("Drone Firmware version: ");
  Serial.println(msg("vers"));

  msg("rst"); // recalibrate
  delay(1000);

  Serial.println("calib done");
  msg("gainI0");
  msg("mode2");
  //set_base_thrust(20);

  nextTime = millis() + PERIOD;
}

unsigned long nextPing = 0;

void loop() {

  if (Serial.available()){
    String instruct = Serial.readStringUntil('\n');


    if (instruct == "r"){
      msg("mode0");
      msg("rst");
      delay(500);
      msg("angX0");
      msg("angY0");
      msg("irst");
      msg("yaw" + String(yaw));
      targetX = 0;
      targetY = 0;
      msg("mode2");
    }

    else if (instruct == "i"){
      msg("irst");
    }

    else if (instruct.startsWith("t")){
      instruct.remove(0, 1);
      msg("manT\n" + instruct + "," + instruct + "," + instruct + "," + instruct);
    }

    else if (instruct.startsWith("y")){
      instruct.remove(0, 1);
      yaw = instruct.toInt();
      msg("yaw" + String(yaw));
    }

    else if (instruct.startsWith("-")){
      instruct.remove(0, 1);
      Serial.println(msg(instruct));
    }

    else if (instruct == "") {
      yawMode = !yawMode;
      Serial.print("yaw mode: ");
      Serial.println(yawMode);
    }
  }

  int vx = (vx + analogRead(35) - joyOffsetX) / 2;
  int vy = (vy + analogRead(32) - joyOffsetY) / 2;

  if (vx > -15 and vx < 15) {vx = 0;}
  if (vy > -15 and vy < 15) {vy = 0;}

  if (digitalRead(34) == LOW) {rightIsClicked = true;}
  if (digitalRead(33) == LOW) {leftIsClicked = true;}

  if (digitalRead(27) == LOW) {
    Serial.println("EMERGENCY STOP");
    msg("mode0");
    delay(1000);
  }
  



  if (millis() > nextTime){
    nextTime = millis() + PERIOD;

    if (rightIsClicked) {
      Serial.println("right");
      if (yawMode) {
        yaw += 2;
        msg("yaw" + String(yaw));
      }
      else {increment_base_thrust(5);}
    }
    if (leftIsClicked) {
      Serial.println("left");
      if (yawMode) {
        yaw -= 2;
        msg("yaw" + String(yaw));
      }
      else {increment_base_thrust(-5);}
    }

    targetX = -vy / 3;
    targetY = vx / 3;

    msg("gx" + String(targetX));
    msg("gy" + String(targetY));

    //print(msg("angX"));

    rightIsClicked = false;
    leftIsClicked = false;
  }
}
