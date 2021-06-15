#include <WiFiEsp.h>
#include <NTPClient.h>
#include <WiFiEspUdp.h>
#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Arduino.h>
#include <MsTimer2.h>     //定时器库的头文件


#define DRAW_TIME
#define INTERVAL
Adafruit_VL6180X vl = Adafruit_VL6180X();
SoftwareSerial mySerial(10, 11); // RX, TX


enum LIFT_STATE
{
  ON_DRAW,
  BETWEEN_NUMS,
  LIFTED,
  CLEAR
};

class Control
{
public:
  MControl(int pin_lift, int pin_left, int pin_right);
  void Calibrate();
  void DetachServo();
  void DrawNumber(float bx, float by, int num, float scale);
  void DrawWeatherIcon(float bx, float by, int type, float scale);
  
  void Lift(LIFT_STATE s);
  void GoTo(float pX, float pY);
  void Clear();
  void GoHome();
  uint8_t  rubberx = 82, rubbery = 48; //【此数值可能需要调节】
  
  Servo servo_lift;
  Servo servo_left;
  Servo servo_right;
private:
  // When in calibration mode, adjust the following factor until the servos move exactly 90 degrees
  // int SERVOFAKTOR = 550;
  int SERVOFAKTORLEFT = 670; //数值加大，顺时针旋转，减小则逆时针旋转 默认650
  int SERVOFAKTORRIGHT = 650; //数值减小，顺时针旋转，加大则逆时针旋转 默认650

  // Zero-position of left and right servo
  // When in calibration mode, adjust the NULL-values so that the servo arms are at all times parallel
  // either to the X or Y axis
  int SERVOLEFTNULL = 2070; //数值减小，顺时针旋转，加大则逆时针旋转 默认2000
  int SERVORIGHTNULL = 1000; //数值减小，顺时针旋转，加大则逆时针旋转  默认1000

  int LIFT0 = 2090; //落笔写字 
  int LIFT1 = 1950; //写字时抬臂动作 一般比lift0 小100左右
  int LIFT2 = 1850; //高抬笔架
  int LIFT3 = 2010; // erase


  uint8_t SERVOPINLIFT = 2; //抬笔舵机
  uint8_t SERVOPINLEFT = 3; //左臂舵机
  uint8_t SERVOPINRIGHT = 4; //右臂舵机

  // speed of lifting arm, higher is slower
  int LIFTSPEED = 2000;

  //摆臂的长度，根据图纸测量，无需改变
  int L1 = 35;
  int L2 = 57.1;
  int L3 = 14;

  //左右舵机轴心的位置，根据图纸测量，无需改变
  int O1X = 28.7; //box版 28.7，普通版22
  int O1Y = -25;
  int O2X = 53.9; //box版53.9， 普通版47
  int O2Y = -25;

  volatile float lastX = rubberx;
  volatile float lastY = rubbery;
  volatile int servoLift = 2000;
  
  volatile float inkr = -0.05;
  volatile float count = 0;

  void circleClockwise(float bx, float by, float radius, int start, int ende, float sqee);

  void circleAntiClockwise(float bx, float by, float radius, int start, int ende, float sqee);

  float return_angle(float a, float b, float c);

  void set_XY(float Tx, float Ty);
};



char buf[20];
const char ssid[] = "fuckzju";            // your network SSID (name)
const char pass[] = "maurice910";        // your network password

//const char ssid[] = "cjmhaoshuai";            // your network SSID (name)
//const char pass[] = "cjmztms5753";        // your network password

const char CITY[] PROGMEM = "shanghai";
int status = WL_IDLE_STATUS;     // the Wifi radio's status

WiFiEspUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp1.aliyun.com",60*60*8, 30*60*1000);

struct TimeData
{
  int day;
  int hours;
  int minutes;
  int seconds;
};
const char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
int32_t t = -300000;
int32_t last_sec = 0;
int32_t interval = 300000; //  5 min
bool flag = true;

volatile int state = LOW;
Control control;
TimeData CurrentTime, LastTime, TimeCount;



Control::MControl(int pin_lift, int pin_left, int pin_right)
{ 
  if (!servo_lift.attached()) servo_lift.attach(pin_lift);  //  lifting servo
  if (!servo_left.attached()) servo_left.attach(pin_left);  //  left servo
  if (!servo_right.attached()) servo_right.attach(pin_right);  //  right servo
  Lift(LIFT_STATE::LIFTED); delay(50);
  GoTo(rubberx, rubbery); delay(50);
}

void Control::DetachServo()
{
  servo_lift.detach();
  servo_left.detach();
  servo_right.detach();
}

void Control::Calibrate()
{
  while (true)
  {
//    GoTo(6.1, 30.7);
//    delay(1000);
//    GoTo(83.3, 26.7);
//    delay(1000);
    Lift(LIFT_STATE::LIFTED); delay(50);
    delay(500);
    Lift(LIFT_STATE::CLEAR); delay(50);
  }
}

void Control::Clear()
{
  DrawNumber(3, 3, 111, 1);
}

void Control::GoHome()
{
  Lift(LIFT_STATE::LIFTED);
  GoTo(rubberx, rubbery);
  Lift(LIFT_STATE::BETWEEN_NUMS);
  Serial.println(F("Ok, homed"));
  delay(50);
}


void Control::DrawNumber(float bx, float by, int num, float scale)
{
  switch (num)
  {
  case 0:
    GoTo(bx + 12 * scale, by + 6 * scale);
    Lift(LIFT_STATE::ON_DRAW);
    circleAntiClockwise(bx + 7 * scale, by + 10 * scale, 10 * scale, -0.8, 6.7, 0.5);
    delay(100);
    
    Lift(LIFT_STATE::BETWEEN_NUMS);
    break;
  case 1:
    GoTo(bx + 3 * scale, by + 15 * scale);
    Lift(LIFT_STATE::ON_DRAW);
    GoTo(bx + 10 * scale, by + 20 * scale);
    GoTo(bx + 10 * scale, by + 0 * scale);
    delay(100);
    Lift(LIFT_STATE::BETWEEN_NUMS);
    break;
  case 2:
    GoTo(bx + 2 * scale, by + 12 * scale);
    Lift(LIFT_STATE::ON_DRAW);
    circleClockwise(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -1.6, 1);
    GoTo(bx + 1 * scale, by + 0 * scale);
    GoTo(bx + 12 * scale, by + 0 * scale);
    delay(100);
    Lift(LIFT_STATE::BETWEEN_NUMS);
    break;
  case 3:
    GoTo(bx + 2 * scale, by + 17 * scale);
    Lift(LIFT_STATE::ON_DRAW);
    circleClockwise(bx + 5 * scale, by + 15 * scale, 5 * scale, 3, -2, 1);
    circleClockwise(bx + 5 * scale, by + 5 * scale, 5 * scale, 1.57, -3, 1);
    delay(100);
    Lift(LIFT_STATE::BETWEEN_NUMS);
    break;
  case 4:
    GoTo(bx + 10 * scale, by + 0 * scale);
    Lift(LIFT_STATE::ON_DRAW);
    GoTo(bx + 10 * scale, by + 20 * scale);
    GoTo(bx + 2 * scale, by + 6 * scale);
    GoTo(bx + 12 * scale, by + 6 * scale);
    delay(100);
    Lift(LIFT_STATE::BETWEEN_NUMS);
    break;
  case 5:
    GoTo(bx + 2 * scale, by + 5 * scale);
    Lift(LIFT_STATE::ON_DRAW);
    circleAntiClockwise(bx + 5 * scale, by + 6 * scale, 6 * scale, -2.5, 2, 1);
    GoTo(bx + 5 * scale, by + 20 * scale);
    GoTo(bx + 12 * scale, by + 20 * scale);
    delay(100);
    Lift(LIFT_STATE::BETWEEN_NUMS);
    break;
  case 6:
    GoTo(bx + 2 * scale, by + 10 * scale);
    Lift(LIFT_STATE::ON_DRAW);
    circleClockwise(bx + 7 * scale, by + 6 * scale, 6 * scale, 2, -4.4, 1);
    GoTo(bx + 11 * scale, by + 20 * scale);
    delay(100);
    Lift(LIFT_STATE::BETWEEN_NUMS);
    break;
  case 7:
    GoTo(bx + 2 * scale, by + 20 * scale);
    Lift(LIFT_STATE::ON_DRAW);
    GoTo(bx + 12 * scale, by + 20 * scale);
    GoTo(bx + 2 * scale, by + 0);
    delay(100);
    Lift(LIFT_STATE::BETWEEN_NUMS);
    break;
  case 8:
    GoTo(bx + 5 * scale, by + 10 * scale);
    Lift(LIFT_STATE::ON_DRAW);
    circleClockwise(bx + 5 * scale, by + 15 * scale, 5 * scale, 4.7, -1.6, 1);
    circleAntiClockwise(bx + 5 * scale, by + 5 * scale, 5 * scale, -4.7, 2, 1);
    delay(100);
    Lift(LIFT_STATE::BETWEEN_NUMS);
    break;

  case 9:
    GoTo(bx + 9 * scale, by + 11 * scale);
    Lift(LIFT_STATE::ON_DRAW);
    circleClockwise(bx + 7 * scale, by + 15 * scale, 5 * scale, 4, -0.5, 1);
    GoTo(bx + 5 * scale, by + 0);
    delay(100);
    Lift(LIFT_STATE::BETWEEN_NUMS);
    break;

  case 111:
    GoTo(rubberx, rubbery);
    Lift(LIFT_STATE::ON_DRAW); delay(100);
    GoTo(70, rubbery); delay(50);
    GoTo(65, 45); delay(50);

    GoTo(5, 45); delay(50);
    GoTo(5, 41); delay(50);
    GoTo(65, 41); delay(50);
    GoTo(65, 37); delay(50);

    GoTo(5, 37); delay(50);
    GoTo(5, 33); delay(50);
    GoTo(65, 33); delay(50);
    GoTo(65, 29); delay(50);

    GoTo(5, 29); delay(50);
    GoTo(5, 25); delay(50);
    GoTo(65, 25); delay(50);
    GoTo(65, 20); delay(50);

    GoTo(7, 20); delay(50);
    GoTo(7, rubbery); delay(50);

    GoTo(58, rubbery); delay(50);
    Lift(LIFT_STATE::CLEAR); delay(100);
    GoTo(rubberx, rubbery-1); delay(50);
    delay(200);
    Lift(LIFT_STATE::LIFTED);
    delay(300);
    break;

  case 11:
    GoTo(bx + 5 * scale, by + 15 * scale);
    Lift(LIFT_STATE::ON_DRAW);
    circleAntiClockwise(bx + 4 * scale, by + 15 * scale, 0.1 * scale, 1, -1, 1);
    delay(200);
    Lift(LIFT_STATE::BETWEEN_NUMS);
    GoTo(bx + 5 * scale, by + 5 * scale);
    Lift(LIFT_STATE::ON_DRAW);
    circleAntiClockwise(bx + 5 * scale, by + 5 * scale, 0.1 * scale, 1, -1, 1);
    delay(200);
    Lift(LIFT_STATE::BETWEEN_NUMS);
    break;

  }
}

void Control::Lift(LIFT_STATE s)
{
  int target;

  switch (s)
  {
  case LIFT_STATE::ON_DRAW:
    target = LIFT0;
    break;
  case LIFT_STATE::BETWEEN_NUMS:

    target = LIFT1;
    break;
  case LIFT_STATE::LIFTED:
    target = LIFT2;
    break;
  case LIFT_STATE::CLEAR:
    target = LIFT3;
  }

  if (servoLift > target)
  {
    while (servoLift > target)
    {
      servoLift--;
      servo_lift.writeMicroseconds(servoLift);
      delayMicroseconds(LIFTSPEED);
      if (s == LIFT_STATE::BETWEEN_NUMS){
         float px = lastX;
         float py = lastY;
         py += 0.05;
         GoTo(px, py);
      }
    }
    delay(50);
  }
  else
  {
    while (servoLift < target)
    {
      servoLift++;
      servo_lift.writeMicroseconds(servoLift);
      delayMicroseconds(LIFTSPEED);
    }
    delay(50);
  }
}

void Control::circleClockwise(float bx, float by, float radius, int start, int ende, float sqee)
{
  inkr = -0.05;
  count = 0;

  do
  {
    GoTo(sqee * radius * cos(start + count) + bx,
      radius * sin(start + count) + by);
    count += inkr;
  } while ((start + count) > ende);
  delay(50);
}

void Control::circleAntiClockwise(float bx, float by, float radius, int start, int ende, float sqee)
{
  inkr = 0.05;
  count = 0;

  do
  {
    GoTo(sqee * radius * cos(start + count) + bx,
      radius * sin(start + count) + by);
    count += inkr;
  } while ((start + count) <= ende);
  delay(50);
}

void Control::GoTo(float pX, float pY)
{
  float dx, dy, c;
  int i;

  // dx dy of new point
  dx = pX - lastX;
  dy = pY - lastY;
  //path lenght in mm, times 4 equals 4 steps per mm
  c = (float)floor(3 * sqrt(dx * dx + dy * dy));

  if (c < 1) c = 1;

  for (i = 0; i <= c; i++)
  {
    // draw line point by point
    set_XY(lastX + (i * dx / c), lastY + (i * dy / c));
  }

  lastX = pX;
  lastY = pY;
}


float Control::return_angle(float a, float b, float c)
{
  // cosine rule for angle between c and a
  return acos((a * a + c * c - b * b) / (2 * a * c));
}

void Control::set_XY(float Tx, float Ty)
{
  delay(10);
  float dx, dy, c, a1, a2, Hx, Hy;

  // calculate triangle between pen, servoLeft and arm joint
  // cartesian dx/dy
  dx = Tx - O1X;
  dy = Ty - O1Y;

  // polar lemgth (c) and angle (a1)
  c = sqrt(dx * dx + dy * dy); // 
  a1 = atan2(dy, dx); //
  a2 = return_angle(L1, L2, c);

  servo_left.writeMicroseconds(floor(((a2 + a1 - M_PI) * SERVOFAKTORLEFT) + SERVOLEFTNULL));

  // calculate joinr arm point for triangle of the right servo arm
  a2 = return_angle(L2, L1, c);
  Hx = Tx + L3 * cos((a1 - a2 + 0.621) + M_PI); 
  Hy = Ty + L3 * sin((a1 - a2 + 0.621) + M_PI);

  // calculate triangle between pen joint, servoRight and arm joint
  dx = Hx - O2X;
  dy = Hy - O2Y;

  c = sqrt(dx * dx + dy * dy);
  a1 = atan2(dy, dx);
  a2 = return_angle(L1, (L2 - L3), c);

  servo_right.writeMicroseconds(floor(((a1 - a2) * SERVOFAKTORRIGHT) + SERVORIGHTNULL));
}

void setup() {   
   // uncommet this to do calibration, code will be blocked here
//    control.MControl(2, 3, 4);delay(50);
//    control.Calibrate();
  
  Serial.begin(9600); 
  while (!Serial) {;}
  Serial.println(F("hardware serial connected."));
  mySerial.begin(9600);
  Serial.println(F("software serial connected."));
  WiFi.init(&mySerial); // initialize ESP module
  
  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println(F("WiFi shield not present"));
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while (status != WL_CONNECTED) {
    Serial.print(F("Attempting to connect to WPA SSID: "));
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  Serial.println(F("You're connected to the network"));
  Serial.println("Adafruit VL6180x test!");
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");
  printWifiData();
  printCurrentNet();
  timeClient.begin();  
}

void loop() {

    UpdateTime(CurrentTime, TimeCount);
    Serial.println(String(daysOfTheWeek[TimeCount.day]) + \
    " " + String(timeClient.getFormattedTime()));
#ifdef DRAW_TIME
  if (CurrentTime.minutes != LastTime.minutes || CurrentTime.hours != LastTime.hours){
    if(flag)
    {    
      control.MControl(2, 3, 4);delay(50);
      flag = false;
    }
    if (!control.servo_lift.attached()) control.servo_lift.attach(2);  //  lifting servo
    if (!control.servo_left.attached()) control.servo_left.attach(3);  //  left servo
    if (!control.servo_right.attached()) control.servo_right.attach(4);  //  right servo
    
    // control.Clear(); delay(50);
    control.DrawNumber(8, 25, CurrentTime.hours / 10, 1); delay(50);
    control.DrawNumber(22, 25, CurrentTime.hours % 10, 1); delay(50);
    control.DrawNumber(33, 25, 11, 0.9); delay(50);
    control.DrawNumber(40, 25, CurrentTime.minutes / 10, 1); delay(50);
    control.DrawNumber(54, 25, CurrentTime.minutes % 10, 1); delay(50);
    control.Lift(LIFT_STATE::LIFTED); delay(50);
    control.GoTo(control.rubberx, control.rubbery); delay(50);
    control.Lift(LIFT_STATE::CLEAR); delay(50);
    LastTime = CurrentTime;
    control.DetachServo();    
  }
#endif 
  delay(1000);
}

void UpdateTime(TimeData& curr, TimeData& count)
{
  timeClient.update();
  count.day = timeClient.getDay();
  count.hours = timeClient.getHours();
  count.minutes = timeClient.getMinutes();
  count.seconds = timeClient.getSeconds();
#ifdef INTERVAL
  if (millis() - t > interval)
  {
    curr.day = timeClient.getDay();
    curr.hours = timeClient.getHours();
    curr.minutes = timeClient.getMinutes();
    curr.seconds = timeClient.getSeconds();
    t = millis();
  }
#else
  curr = count;
#endif
}


void printWifiData()
{
  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print(F("IP Address: "));
  Serial.println(ip);

  // print your MAC address
  byte mac[6];
  WiFi.macAddress(mac);
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
  Serial.print(F("MAC address: "));
  Serial.println(buf);
}

void printCurrentNet()
{
  // print the SSID of the network you're attached to
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to
  byte bssid[6];
  WiFi.BSSID(bssid);
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", bssid[5], bssid[4], bssid[3], bssid[2], bssid[1], bssid[0]);
  Serial.print(F("BSSID: "));
  Serial.println(buf);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print(F("Signal strength (RSSI): "));
  Serial.println(rssi);
}
