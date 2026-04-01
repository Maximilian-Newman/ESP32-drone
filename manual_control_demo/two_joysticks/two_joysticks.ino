#include <WiFi.h>

const byte joyPinRX = 35;
const byte joyPinRY = 32;
const byte joyPinRC = 27;
const byte joyPinLX = 39;
const byte joyPinLY = 36;
const byte joyPinLC = 14;

String ssid = "";
String password = "skibidi123";
WiFiClient client;

bool rightIsClicked = false;
bool leftIsClicked = false;
bool leftJoyIsClicked = false;
bool yawMode = false;

int yaw = 0;
int targetX = 0;
int targetY = 0;

unsigned int PERIOD = 300;
unsigned long nextTime = 0;

long joyOffsetRX = 0;
long joyOffsetRY = 0;
long joyOffsetLX = 0;
long joyOffsetLY = 0;

bool verboseComs = false;

void recalibrate_joystick() {
  joyOffsetRX = 0;
  joyOffsetRX = 0;

  for (byte i=0; i<200; i++) {
    joyOffsetRX += analogRead(joyPinRX);
    joyOffsetRY += analogRead(joyPinRY);
    joyOffsetLX += analogRead(joyPinLX);
    joyOffsetLY += analogRead(joyPinLY);
    delay(1);
  }

  joyOffsetRX /= 200;
  joyOffsetRY /= 200;
  joyOffsetLX /= 200;
  joyOffsetLY /= 200;
}

String msg(String tx) {
  while (client.available()) {client.read();}
  client.print(tx + "\n");
  if (verboseComs) {Serial.println("sent: " + tx);}
  while (!client.available()) {delay(1);};
  String rx = client.readStringUntil('\n');
  if (verboseComs) {Serial.println("recv: " + rx);}
  return rx;
}

void increment_base_thrust(int val) {
  String sval = String(val);
  msg("incT\n" + sval + "," + sval + "," + sval + "," + sval);
}

void set_base_thrust(int val) {
  String sval = String(val);
  msg("manT\n" + sval + "," + sval + "," + sval + "," + sval);
  //Serial.print("set thrust: ");
  //Serial.println(val);
}

void set_yaw(int y) {
  if (y == yaw) {return;}
  yaw = y;
  msg("yaw" + String(yaw));
  Serial.print("set yaw: ");
  Serial.println(yaw);
}


void setup() {
  Serial.begin(115200);
  Serial.println("\n\n");

  pinMode(joyPinRX, INPUT);
  pinMode(joyPinRY, INPUT);
  pinMode(joyPinRC, INPUT);
  pinMode(joyPinLX, INPUT);
  pinMode(joyPinLY, INPUT);
  pinMode(joyPinLC, INPUT);
  pinMode(33, INPUT);
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
      set_base_thrust(instruct.toInt());
    }

    else if (instruct.startsWith("y")){
      instruct.remove(0, 1);
      set_yaw(instruct.toInt());
    }

    else if (instruct.startsWith("-")){
      instruct.remove(0, 1);
      Serial.println(msg(instruct));
    }

    else if (instruct == "v") {
      verboseComs = !verboseComs;
    }

    else if (instruct == "") {
      yawMode = !yawMode;
      Serial.print("yaw mode: ");
      Serial.println(yawMode);
    }
  }






  int vrx = analogRead(joyPinRX) - joyOffsetRX;
  int vry = analogRead(joyPinRY) - joyOffsetRY;
  int vlx = analogRead(joyPinLX) - joyOffsetLX;
  int vly = analogRead(joyPinLY) - joyOffsetLY;

  if (vly < 0) {
    vly *= 2;
  }

  if (digitalRead(34) == LOW) {rightIsClicked = true;}
  if (digitalRead(33) == LOW) {leftIsClicked = true;}
  if (digitalRead(joyPinRC) == LOW) {leftJoyIsClicked = true;}

  if (digitalRead(joyPinRC) == LOW) {
    Serial.println("EMERGENCY STOP");
    msg("mode0");
    delay(1000);
  }
  



  if (millis() > nextTime){
    nextTime = millis() + PERIOD;

    if (leftJoyIsClicked) {msg("irst");}

/*    if (rightIsClicked) {
      if (yawMode) {
        set_yaw(yaw+1);
      }
      else {increment_base_thrust(5);}
    }
    if (leftIsClicked) {
      if (yawMode) {
        set_yaw(yaw-1);
      }
      else {increment_base_thrust(-5);}
    }*/

    targetX = -vry / 70;
    targetY = vrx / 70;

    msg("gx" + String(targetX));
    msg("gy" + String(targetY));

    /* derivative controls were too dificult
    if (abs(vlx/ 150) >= 1) {
      set_yaw(yaw + vlx / 150);
    }
    if (abs(vly/ 70) >= 1) {
      increment_base_thrust(vly / 70);
    }
    */

    if (abs(vlx) / 30 > 10){
      set_yaw(vlx / 30);
    }
    else{
      set_yaw(0);
    }
    set_base_thrust(vly / 15);


    rightIsClicked = false;
    leftIsClicked = false;
    leftJoyIsClicked = false;

    Serial.print(targetX);
    Serial.print("\t");
    Serial.println(targetY);
  }
}
