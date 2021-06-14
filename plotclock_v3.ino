#include <WiFiEsp.h>
#include <NTPClient.h>
#include <WiFiEspUdp.h>
#include <SoftwareSerial.h>
#include "Control.h"

#define DRAW_TIME
#define INTERVAL
SoftwareSerial mySerial(10, 11); // RX, TX
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

Control control;
TimeData CurrentTime, LastTime, TimeCount;

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
  printWifiData();
  printCurrentNet();
  
  timeClient.begin();  
}

void loop() {
  UpdateTime(CurrentTime, TimeCount);
  Serial.println(String(daysOfTheWeek[TimeCount.day]) + \
  " " + String(timeClient.getFormattedTime()));
#ifdef DRAW_TIME
/***********************only for debug, to be deleted**********************/
  control.Clear(); delay(50);
  control.DrawNumber(8, 25, CurrentTime.hours / 10, 1); delay(50);
/***********************only for debug, to be deleted*******************/
  if (CurrentTime.minutes != LastTime.minutes || CurrentTime.hours != LastTime.hours){
    if(flag)
    {    
      control.MControl(2, 3, 4);delay(50);
      flag = false;
    }
    if (!control.servo_lift.attached()) control.servo_lift.attach(2);  //  lifting servo
    if (!control.servo_left.attached()) control.servo_left.attach(3);  //  left servo
    if (!control.servo_right.attached()) control.servo_right.attach(4);  //  right servo
    
    control.Clear(); delay(50);
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
