#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WS2812FX.h>
#include <Preferences.h>

/* To store the calibration value for each servo motor */
Preferences preferences;

/* led DECLARATION START */
#define LED_COUNT 8
#define LED_PIN 21
#define TIMER_MS 5000

WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);
unsigned long last_change = 0;
unsigned long now = 0;

/* led DECLARATION END */

/* PWM DECLARATION START*/
// use 12 bit precission for LEDC timer
#define LEDC_TIMER_12_BIT  12

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     50

#define MIN 50
#define MAX 550

#define ARM_1 0
#define ARM_2 1
#define ARM_3 2
#define ARM_4 3
#define LEG_1 4
#define LEG_2 5
#define LEG_3 6
#define LEG_4 7
/* PWM DECLARATION END */

/* SERVER DECLARATION START */
/* Put your SSID & Password */
const char* ssid = "iSEB Spider";  // Enter SSID here
const char* password = "12345678";  //Enter Password here

/* Put IP Address details */
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

WebServer server(80);

/* SERVER DECLARATION END */

/* MOTOR DECLARATION START */
// Motion data index
int Servo_PROGRAM = 0;

// Servos matrix
const int ALLMATRIX = 9; // GPIO14 + GPIO12 + GPIO13 + GPIO15 + GPIO16 + GPIO5 + GPIO4 + GPIO2 + Run Time
const int ALLSERVOS = 8; // GPIO14 + GPIO12 + GPIO13 + GPIO15 + GPIO16 + GPIO5 + GPIO4 + GPIO2

// Servo delay base time
const int BASEDELAYTIME = 50; // 10 ms

// Backup servo value
int Running_Servo_POS [ALLMATRIX] = {};

// Servo zero position 歸零位置
int Servo_Act_0 [ ] PROGMEM = {  90,  90, 90,  90,  90, 90, 90, 90,  500  };

// Standby 待機
int Servo_Prg_1_Step = 1;
int Servo_Prg_1 [][ALLMATRIX] PROGMEM = {
  //  ARM1, ARM2, ARM3, ARM4, LEG1, LEG2, LEG3, LEG4, ms
  {     90,   90,    90,   90,  155,   15,   15,  155, 500  }, // servo center point
};

// Forward 前行
int Servo_Prg_2_Step = 12;
int Servo_Prg_2 [][ALLMATRIX] PROGMEM = {
 //  ARM1, ARM2, ARM3, ARM4, LEG1, LEG2, LEG3, LEG4, ms
  {    90,   90,   90,   90,  155,   15,   15,  155, 500  }, // standby
  {   135,   90,   90,   90,  135,   15,   15,  155, 500  }, // leg4,1 up; leg1 fw
  {   135,   90,   90,  110,  155,   15,   15,  155, 500  }, // leg4,1 dn
  {   135,   90,   90,  110,  155,   60,   15,  135, 500  }, // leg3,2 up
  {    90,   90,  135,  110,  155,   60,   60,  135, 500  }, // leg4,1 bk; leg3 fw
  {    90,   90,  135,  110,  155,   15,   60,  155, 500  }, // leg3,2 dn
  {    90,   90,   90,   90,  135,   15,   60,  155, 500  }, // leg4,1 up; leg4 fw
  {    90,   45,   90,   90,  135,   15,   15,  155, 500  }, // leg3,1 bk
  {    90,   45,   90,  110,  155,   15,   15,  155, 500  }, // leg4,1 dn
  {    90,   45,   90,  110,  155,   15,   15,  135, 500  }, // leg2 up
  {    90,   90,   90,  110,  155,   15,   15,  155, 500  }, // leg2 fw dn
  {    90,   90,   90,   90,  155,   15,   15,  155, 500  }, // standby
};

// Backward 退後
int Servo_Prg_3_Step = 11;
int Servo_Prg_3 [][ALLMATRIX] PROGMEM = {
  // P16, P05, P04, P00, P02, P14, P12, P13,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   90,  90,  90, 110, 110,  90,  45,  90,  100  }, // leg1,4 up; leg4 fw
  {   70,  90,  90, 110, 110,  90,  45,  70,  100  }, // leg1,4 dn
  {   70,  90,  90,  90,  90,  90,  45,  70,  100  }, // leg2,3 up
  {   70,  45, 135,  90,  90,  90,  90,  70,  100  }, // leg1,4 bk; leg2 fw
  {   70,  45, 135, 110, 110,  90,  90,  70,  100  }, // leg2,3 dn
  {   90,  90, 135, 110, 110,  90,  90,  90,  100  }, // leg1,4 up; leg1 fw
  {   90,  90,  90, 110, 110, 135,  90,  90,  100  }, // leg2,3 bk
  {   70,  90,  90, 110, 110, 135,  90,  70,  100  }, // leg1,4 dn
  {   70,  90,  90, 110,  90, 135,  90,  70,  100  }, // leg3 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg3 fw dn
};

// Left shift 左移
int Servo_Prg_4_Step = 12;
int Servo_Prg_4 [][ALLMATRIX] PROGMEM = {
  //  ARM1, ARM2, ARM3, ARM4, LEG1, LEG2, LEG3,LEG4,  ms
  //   P05,  P04,  P12,  P13,  P16,  P02,  P14, P00,  ms
  {     90,   90,   90,   90,  155,   15,   15,  155, 500  }, // standby
  {     90,   90,   90,   90,  155,   90,   45,  90,  100  }, // leg2,3 up; leg3 fw
  {     90,   90,   90,   90,  155,  110,   45, 110,  100  }, // leg2,3 dn
  {     90,   90,   90,  110,  135,  110,   45, 110,  100  }, // leg4,1 up
  {     90,   45,  135,  110,  135,  110,   90, 110,  100  }, // leg2,3 bk; leg4 fw
  {     90,   45,  135,   90,   70,  110,   90, 110,  100  }, // leg4,1 dn
  {     90,   90,  135,   90,   70,   90,   90,  90,  100  }, // leg2,3 up; leg2 fw
  {    135,   90,   90,   90,   70,   90,   90,  90,  100  }, // leg4,1 bk
  {    135,   90,   90,   90,   70,  110,   90, 110,  100  }, // leg2,3 dn
  {    135,   90,   90,   90,   90,  110,   90, 110,  100  }, // leg1 up
  {     90,   90,   90,   90,   70,  110,   90, 110,  100  }, // leg1 fw dn
  {     90,   90,   90,   90,  155,   15,   15,  155, 500  }, // standby
};

// Right shift 右移
int Servo_Prg_5_Step = 11;
int Servo_Prg_5 [][ALLMATRIX] PROGMEM = {
  // P16, P05, P04, P00, P02, P14, P12, P13,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   70,  90,  45,  90,  90,  90,  90,  70,  100  }, // leg3,2 up; leg2 fw
  {   70,  90,  45, 110, 110,  90,  90,  70,  100  }, // leg3,2 dn
  {   90,  90,  45, 110, 110,  90,  90,  90,  100  }, // leg1,4 up
  {   90, 135,  90, 110, 110,  45,  90,  90,  100  }, // leg3,2 bk; leg1 fw
  {   70, 135,  90, 110, 110,  45,  90,  70,  100  }, // leg1,4 dn
  {   70, 135,  90,  90,  90,  90,  90,  70,  100  }, // leg3,2 up; leg3 fw
  {   70,  90,  90,  90,  90,  90, 135,  70,  100  }, // leg1,4 bk
  {   70,  90,  90, 110, 110,  90, 135,  70,  100  }, // leg3,2 dn
  {   70,  90,  90, 110, 110,  90, 135,  90,  100  }, // leg4 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg4 fw dn
};

// Turn left 左轉leg
int Servo_Prg_6_Step = 8;
int Servo_Prg_6 [][ALLMATRIX] PROGMEM = {
  // P16, P05, P04, P00, P02, P14, P12, P13,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   90,  90,  90, 110, 110,  90,  90,  90,  100  }, // leg1,4 up
  {   90, 135,  90, 110, 110,  90, 135,  90,  100  }, // leg1,4 turn
  {   70, 135,  90, 110, 110,  90, 135,  70,  100  }, // leg1,4 dn
  {   70, 135,  90,  90,  90,  90, 135,  70,  100  }, // leg2,3 up
  {   70, 135, 135,  90,  90, 135, 135,  70,  100  }, // leg2,3 turn
  {   70, 135, 135, 110, 110, 135, 135,  70,  100  }, // leg2,3 dn
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg1,2,3,4 turn
};

// Turn right 右轉
int Servo_Prg_7_Step = 8;
int Servo_Prg_7 [][ALLMATRIX] PROGMEM = {
  // P16, P05, P04, P00, P02, P14, P12, P13,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   70,  90,  90,  90,  90,  90,  90,  70,  100  }, // leg2,3 up
  {   70,  90,  45,  90,  90,  45,  90,  70,  100  }, // leg2,3 turn
  {   70,  90,  45, 110, 110,  45,  90,  70,  100  }, // leg2,3 dn
  {   90,  90,  45, 110, 110,  45,  90,  90,  100  }, // leg1,4 up
  {   90,  45,  45, 110, 110,  45,  45,  90,  100  }, // leg1,4 turn
  {   70,  45,  45, 110, 110,  45,  45,  70,  100  }, // leg1,4 dn
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg1,2,3,4 turn
};

// Lie 趴地
int Servo_Prg_8_Step = 2;
int Servo_Prg_8 [][ALLMATRIX] PROGMEM = {
  {     90,   90,    90,   90,  155,   15,   15,  155, 200  }, // servo center point
  {     90,   90,    90,   90,  116,   63,   63,  116, 500  }, // leg1,4 up
};

// Say Hi 打招呼
int Servo_Prg_9_Step = 7;
int Servo_Prg_9 [][ALLMATRIX] PROGMEM = {
  // P16, P05, P04, P00, P02, P14, P12, P13,  ms
  {  120,  90,  90, 110,  60,  90,  90,  70,  200  }, // leg1, 3 down
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, // standby
  {  120,  90,  90, 110,  60,  90,  90,  70,  200  }, // leg1, 3 down
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, // standby
};

// Fighting 戰鬥姿態
int Servo_Prg_10_Step = 11;
int Servo_Prg_10 [][ALLMATRIX] PROGMEM = {
  // P16, P05, P04, P00, P02, P14, P12, P13,  ms
  {  120,  90,  90, 110,  60,  90,  90,  70,  200  }, // leg1, 2 down
  {  120,  70,  70, 110,  60,  70,  70,  70,  200  }, // body turn left
  {  120, 110, 110, 110,  60, 110, 110,  70,  200  }, // body turn right
  {  120,  70,  70, 110,  60,  70,  70,  70,  200  }, // body turn left
  {  120, 110, 110, 110,  60, 110, 110,  70,  200  }, // body turn right
  {   70,  90,  90,  70, 110,  90,  90, 110,  200  }, // leg1, 2 up ; leg3, 4 down
  {   70,  70,  70,  70, 110,  70,  70, 110,  200  }, // body turn left
  {   70, 110, 110,  70, 110, 110, 110, 110,  200  }, // body turn right
  {   70,  70,  70,  70, 110,  70,  70, 110,  200  }, // body turn left
  {   70, 110, 110,  70, 110, 110, 110, 110,  200  }, // body turn right
  {   70,  90,  90,  70, 110,  90,  90, 110,  200  }  // leg1, 2 up ; leg3, 4 down
};

// Push up 掌上壓
int Servo_Prg_11_Step = 11;
int Servo_Prg_11 [][ALLMATRIX] PROGMEM = {
  //  ARM1, ARM2,  ARM3, ARM4, LEG1, LEG2, LEG3, LEG4,  ms
  {     90,   90,    90,   90,  155,   15,   15,  155, 200  }, // start
  {     90,   90,    90,   90,  116,   63,   63,  116, 300  }, // down
  {     90,   90,    90,   90,  155,   15,   15,  155, 400  }, // up
  {     90,   90,    90,   90,  116,   63,   63,  116, 500  }, // down
  {     90,   90,    90,   90,  155,   15,   15,  155, 600  }, // up
  {     90,   90,    90,   90,  116,   63,   63,  116, 700  }, // down
  {     90,   90,    90,   90,  155,   15,   15,  155, 1300 }, // up
  {     90,   90,    90,   90,  116,   63,   63,  116, 200  },  // down
  {     90,   90,    90,   90,  155,   63,   63,  116, 300  }, // leg 1 up
  {     90,   90,    90,   90,  155,   15,   63,  116, 300  }, // leg 2 up
  {     90,   90,    90,   90,  155,   15,   15,  155, 200  }, // leg3, leg4 up
};

// Sleep 睡眠姿勢
int Servo_Prg_12_Step = 2;
int Servo_Prg_12 [][ALLMATRIX] PROGMEM = {
  {     90,   90,    90,   90,  155,   15,   15,  155, 200  }, // servo center point
  {     55,   115,  115,   55,  155,   15,   15,  155, 500  }, // leg1,4 up
};

// 舞步 1
int Servo_Prg_13_Step = 10;
int Servo_Prg_13 [][ALLMATRIX] PROGMEM = {
  // P16, P05, P04, P00, P02, P14, P12, P13,  ms
  {   90,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg1,2,3,4 up
  {   50,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg1 dn
  {   90,  90,  90, 130,  90,  90,  90,  90,  300  }, // leg1 up; leg2 dn
  {   90,  90,  90,  90,  90,  90,  90,  50,  300  }, // leg2 up; leg4 dn
  {   90,  90,  90,  90, 130,  90,  90,  90,  300  }, // leg4 up; leg3 dn
  {   50,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg3 up; leg1 dn
  {   90,  90,  90, 130,  90,  90,  90,  90,  300  }, // leg1 up; leg2 dn
  {   90,  90,  90,  90,  90,  90,  90,  50,  300  }, // leg2 up; leg4 dn
  {   90,  90,  90,  90, 130,  90,  90,  90,  300  }, // leg4 up; leg3 dn
  {   90,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg3 up
};

// 舞步 2
int Servo_Prg_14_Step = 9;
int Servo_Prg_14 [][ALLMATRIX] PROGMEM = {
  // P16, P05, P04, P00, P02, P14, P12, P13,  ms
  {   70,  45, 135, 110, 110, 135,  45,  70,  300  }, // leg1,2,3,4 two sides
  {  115,  45, 135,  65, 110, 135,  45,  70,  300  }, // leg1,2 up
  {   70,  45, 135, 110,  65, 135,  45, 115,  300  }, // leg1,2 dn; leg3,4 up
  {  115,  45, 135,  65, 110, 135,  45,  70,  300  }, // leg3,4 dn; leg1,2 up
  {   70,  45, 135, 110,  65, 135,  45, 115,  300  }, // leg1,2 dn; leg3,4 up
  {  115,  45, 135,  65, 110, 135,  45,  70,  300  }, // leg3,4 dn; leg1,2 up
  {   70,  45, 135, 110,  65, 135,  45, 115,  300  }, // leg1,2 dn; leg3,4 up
  {  115,  45, 135,  65, 110, 135,  45,  70,  300  }, // leg3,4 dn; leg1,2 up
  {   75,  45, 135, 105, 110, 135,  45,  70,  300  }, // leg1,2 dn
};

// 舞步 3
int Servo_Prg_15_Step = 10;
int Servo_Prg_15 [][ALLMATRIX] PROGMEM = {
  // P16, P05, P04, P00, P02, P14, P12, P13,  ms
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,2,3,4 bk
  {  110,  45,  45,  60,  70, 135, 135,  70,  300  }, // leg1,2,3 up
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,2,3 dn
  {  110,  45,  45, 110,  70, 135, 135, 120,  300  }, // leg1,3,4 up
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,3,4 dn
  {  110,  45,  45,  60,  70, 135, 135,  70,  300  }, // leg1,2,3 up
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,2,3 dn
  {  110,  45,  45, 110,  70, 135, 135, 120,  300  }, // leg1,3,4 up
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,3,4 dn
  {   70,  90,  90, 110, 110,  90,  90,  70,  300  }, // standby
};

/* MOTOR DECLARATION START */

/* PWM CODE START */
void motorInit()
{
  // Setup timer 
  ledcSetup(ARM_1, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(ARM_2, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(ARM_3, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(ARM_4, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(LEG_1, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(LEG_2, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(LEG_3, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(LEG_4, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
 
  // Attach timer to a led pin
  ledcAttachPin(13, ARM_1);  /* ARM 1 *//* CN1  *//* PIN 13*/
  delay(50);
  ledcAttachPin(33, ARM_2);  /* ARM 2 *//* CN7  *//* PIN 33*/
  delay(50);
  ledcAttachPin(15, ARM_3);  /* ARM 3 *//* CN9  *//* PIN 15*/
  delay(50);
  ledcAttachPin(19, ARM_4);  /* ARM 4 *//* CN15 *//* PIN 19*/
  delay(50);
  ledcAttachPin(12, LEG_1);  /* LEG 1 *//* CN2  *//* PIN 12*/
  delay(50);
  ledcAttachPin(32, LEG_2);  /* LEG 2 *//* CN8  *//* PIN 32*/
  delay(50);
  ledcAttachPin( 4, LEG_3);  /* LEG 3 *//* CN10 *//* PIN  4*/
  delay(50);
  ledcAttachPin(23, LEG_4);  /* LEG 4 *//* CN16 *//* PIN 23*/
  delay(50);

}
/* PWM CODE END */

/* SERVER CODE START */
// Servo calibration
void handleSetting()
{
  double servo7Val = preferences.getDouble("7", 0);
  String servo7ValStr = String(servo7Val);
  double servo6Val = preferences.getDouble("6", 0);
  String servo6ValStr = String(servo6Val);
  double servo5Val = preferences.getDouble("5", 0);
  String servo5ValStr = String(servo5Val);
  double servo4Val = preferences.getDouble("4", 0);
  String servo4ValStr = String(servo4Val);
  double servo3Val = preferences.getDouble("3", 0);
  String servo3ValStr = String(servo3Val);
  double servo2Val = preferences.getDouble("2", 0);
  String servo2ValStr = String(servo2Val);
  double servo1Val = preferences.getDouble("1", 0);
  String servo1ValStr = String(servo1Val);
  double servo0Val = preferences.getDouble("0", 0);
  String servo0ValStr = String(servo0Val);
  String content = "";

  content += "<html>";
  content += "<head>";
  content += "<title>Servo calibration</title>";
  content += "<meta charset=UTF-8>";
  content += "<meta name=viewport content=width=device-width>";
  content += "<style type=text/css>";
  content += "body {";
  content += "margin: 0px;";
  content += "backgound-color: #FFFFFF;";
  content += "font-family: helvetica, arial;";
  content += "font-size: 100%;";
  content += "color: #555555;";
  content += "}";
  content += "td {";
  content += "text-align: center;";
  content += "}";
  content += "span {";
  content += "font-family: helvetica, arial;";
  content += "font-size: 70%;";
  content += "color: #777777;";
  content += "}";
  content += "input[type=text] {";
  content += "width: 40%;";
  content += "font-family: helvetica, arial;";
  content += "font-size: 90%;";
  content += "color: #555555;";
  content += "text-align: center;";
  content += "padding: 3px 3px 3px 3px;";
  content += "}";
  content += "button {";
  content += "width: 40%;";
  content += "font-family: helvetica, arial;";
  content += "font-size: 90%;";
  content += "color: #555555;";
  content += "background: #BFDFFF;";
  content += "padding: 5px 5px 5px 5px;";
  content += "border: none;";
  content += "}";
  content += "</style>";
  content += "</head>";
  content += "<body>";
  content += "<br>";
  content += "<table width=100% height=90%>";
  content += "<tr>";
  content += "<td width=50%>ARM1<br/><input type=text id=servo_0 value=\"" + servo0ValStr + "\"><button type=button style=background:#FFE599 onclick=saveServo(0,'servo_0')>SET</button></td>";
  content += "<td width=50%>LEG1<br/><input type=text id=servo_4 value=\"" + servo4ValStr + "\"><button type=button style=background:#FFE599 onclick=saveServo(4,'servo_4')>SET</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td>ARM2<br/><input type=text id=servo_1 value=\"" + servo1ValStr + "\"><button type=button onclick=saveServo(1,'servo_1')>SET</button></td>";
  content += "<td>LEG2<br/><input type=text id=servo_5 value=\"" + servo5ValStr + "\"><button type=button onclick=saveServo(5,'servo_15')>SET</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td>ARM3<br/><input type=text id=servo_2 value=\"" + servo2ValStr + "\"><button type=button onclick=saveServo(2,'servo_2')>SET</button></td>";
  content += "<td>LEG3<br/><input type=text id=servo_6 value=\"" + servo6ValStr + "\"><button type=button onclick=saveServo(6,'servo_6')>SET</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td>ARM4<br/><input type=text id=servo_3 value=\"" + servo3ValStr + "\"><button type=button style=background:#FFE599 onclick=saveServo(3,'servo_3')>SET</button></td>";
  content += "<td>LEG4<br/><input type=text id=servo_7 value=\"" + servo7ValStr + "\"><button type=button style=background:#FFE599 onclick=saveServo(7,'servo_7')>SET</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td colspan=2><button type=button style=background:#FFBFBF onclick=saveServo(100,0)>RESET ALL</button></td>";
  content += "</tr>";
  content += "</table>";
  content += "</body>";
  content += "<script>";
  content += "function saveServo(id, textId) {";
  content += "var xhttp = new XMLHttpRequest();";
  content += "var value = \"0\";";
  content += "if(id==100){";
  content += "document.getElementById(\"servo_7\").value = \"0\";";
  content += "document.getElementById(\"servo_6\").value = \"0\";";
  content += "document.getElementById(\"servo_5\").value = \"0\";";
  content += "document.getElementById(\"servo_4\").value = \"0\";";
  content += "document.getElementById(\"servo_3\").value = \"0\";";
  content += "document.getElementById(\"servo_2\").value = \"0\";";
  content += "document.getElementById(\"servo_1\").value = \"0\";";
  content += "document.getElementById(\"servo_0\").value = \"0\";";
  content += "}else{";
  content += "value = document.getElementById(textId).value;";
  content += "}";
  content += "xhttp.onreadystatechange = function() {";
  content += "if (xhttp.readyState == 4 && xhttp.status == 200) {";
  content += "document.getElementById(\"demo\").innerHTML = xhttp.responseText;";
  content += "}";
  content += "};";
  content += "xhttp.open(\"GET\",\"save?key=\"+id+\"&value=\"+value, true);";
  content += "xhttp.send();";
  content += "}";
  content += "</script>";
  content += "</html>";

  server.send(200, "text/html", content);
}

void handleController()
{
  String pm = server.arg("pm");
  String servo = server.arg("servo");
  String value = server.arg("value");
  Serial.println("Controller pm: "+pm+" servo: "+servo);
  if (pm != "") {
    Servo_PROGRAM = pm.toInt();
    server.send(200, "text/html", "(pm)=(" + pm + ")");
  }

  if (servo != "" && value!= "") {
    Set_PWM_to_Servo(servo.toInt(),value.toInt());
    server.send(200, "text/html", "servo =" + servo + " value =" + value);
  }
  server.send(200, "text/html", "Input invalid");
}

void handleOnLed()
{
  String m0 = server.arg("m0");
  String m1 = server.arg("m1");
  String m2 = server.arg("m2");
  String m3 = server.arg("m3");
  String m4 = server.arg("m4");
  String m5 = server.arg("m5");
  String m6 = server.arg("m6");
  String m7 = server.arg("m7");
  String t1 = server.arg("t1");

  Serial.println("Online");
  // int Servo_Prg_tmp [][ALLMATRIX] = {
  //   // GPIO14,     GPIO12,    GPIO13,     GPIO15,     GPIO16,     GPIO5,      GPIO4,      GPIO2,      Run Time
  //   { m0.toInt(), m1.toInt(), m2.toInt(), m3.toInt(), m4.toInt(), m5.toInt(), m6.toInt(), m7.toInt(), t1.toInt() }
  // };

  // Servo_PROGRAM_Run(Servo_Prg_tmp, 1);

  server.send(200, "text/html", "(m0, m1)=(" + m0 + "," + m1 + ")");
}

void handleZero()
{
  String content = "";

  content += "<html>";
  content += "<head>";
  content += "<title>Zero check</title>";
  content += "<meta charset=UTF-8>";
  content += "<meta name=viewport content=width=device-width>";
  content += "<style type=text/css>";
  content += "body {";
  content += "margin: 0px;";
  content += "backgound-color: #FFFFFF;";
  content += "font-family: helvetica, arial;";
  content += "font-size: 100%;";
  content += "color: #555555;";
  content += "}";
  content += "td {";
  content += "text-align: center;";
  content += "}";
  content += "span {";
  content += "font-family: helvetica, arial;";
  content += "font-size: 80%;";
  content += "color: #777777;";
  content += "}";
  content += "button {";
  content += "width: 40%;";
  content += "font-family: helvetica, arial;";
  content += "font-size: 90%;";
  content += "color: #555555;";
  content += "background: #BFDFFF;";
  content += "padding: 5px 5px 5px 5px;";
  content += "border: none;";
  content += "}";
  content += "</style>";
  content += "</head>";
  content += "<body>";
  content += "<br>";
  content += "<table width=100% height=90%>";
  content += "<tr>";
  content += "<td width=50%><button type=button style=background:#FFE599 onclick=controlServo(4,45)>D16</button></td>";
  content += "<td width=50%><button type=button style=background:#FFE599 onclick=controlServo(0,135)>D14</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td><button type=button onclick=controlServo(5,135)>D05</button></td>";
  content += "<td><button type=button onclick=controlServo(1,45)>D12</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td><button type=button onclick=controlServo(6,45)>D04</button></td>";
  content += "<td><button type=button onclick=controlServo(2,135)>D13</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td><button type=button style=background:#FFE599 onclick=controlServo(7,135)>D02</button></td>";
  content += "<td><button type=button style=background:#FFE599 onclick=controlServo(3,45)>D15</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td colspan=2><button type=button style=background:#FFBFBF onclick=controlPm(100)>ZERO Postition</button></td>";
  content += "</tr>";
  content += "</table>";
  content += "</body>";
  content += "<script>";
  content += "function controlServo(id, value) {";
  content += "var xhttp = new XMLHttpRequest();";
  content += "xhttp.onreadystatechange = function() {";
  content += "if (xhttp.readyState == 4 && xhttp.status == 200) {";
  content += "document.getElementById(\"demo\").innerHTML = xhttp.responseText;";
  content += "}";
  content += "};";
  content += "xhttp.open(\"GET\", \"controller?servo=\"+id+\"&value=\"+value, true);";
  content += "xhttp.send();";
  content += "}";
  content += "function controlPm(value) {";
  content += "var xhttp = new XMLHttpRequest();";
  content += "xhttp.onreadystatechange = function() {";
  content += "if (xhttp.readyState == 4 && xhttp.status == 200) {";
  content += "document.getElementById(\"demo\").innerHTML = xhttp.responseText;";
  content += "}";
  content += "};";
  content += "xhttp.open(\"GET\", \"controller?pm=\"+value, true);";
  content += "xhttp.send();";
  content += "}";
  content += "</script>";
  content += "</html>";

  server.send(200, "text/html", content);
}

void handleIndex()
{
  String content = "";

  content += "<html>";
  content += "<head>";
  content += "<title>iSEB Spider</title>";
  content += "<meta charset=UTF-8>";
  content += "<meta name=viewport content=width=device-width>";
  content += "<style type=text/css>";
  content += "body {";
  content += "margin: 0px;";
  content += "backgound-color: #FFFFFF;";
  content += "font-family: helvetica, arial;";
  content += "font-size: 100%;";
  content += "color: #555555;";
  content += "text-align: center;";
  content += "}";
  content += "td {";
  content += "text-align: center;";
  content += "}";
  content += "span {";
  content += "font-family: helvetica, arial;";
  content += "font-size: 70%;";
  content += "color: #777777;";
  content += "}";
  content += "button {";
  content += "width: 90%;";
  content += "height: 90%;";
  content += "font-family: helvetica, arial;";
  content += "font-size: 100%;";
  content += "color: #555555;";
  content += "background: #BFDFFF;";
  content += "border-radius: 4px;";
  content += "padding: 2px 2px 2px 2px;";
  content += "border: none;";
  content += "}";
  content += "</style>";
  content += "</head>";
  content += "<body><h1>iSEB Spider</h1>";
  content += "<table width=100% height=90%>";
  content += "<tr height=19%>";
  content += "<td width=33%><button type=button style=background:#BFFFCF onclick=controlPm(6)>Turn left</button></td>";
  content += "<td width=33%><button type=button onclick=controlPm(2)>Forward</button></td>";
  content += "<td width=33%><button type=button style=background:#BFFFCF onclick=controlPm(7)>Turn right</button></td>";
  content += "</tr>";
  content += "<tr height=19%>";
  content += "<td><button type=button onclick=controlPm(4)>Left shift</button></td>";
  content += "<td><button type=button onclick=controlPm(3)>Backward</button></td>";
  content += "<td><button type=button onclick=controlPm(5)>Right shift</button></td>";
  content += "</tr>";
  content += "<tr height=5%><td colspan=3><span><br></span></td></tr>";
  content += "<tr height=20%>";
  content += "<td><button type=button style=background:#FFCC99 onclick=controlPm(1)>Stand </button></td>";
  content += "<td><button type=button style=background:#FFE599 onclick=controlPm(9)>Say Hi</button></td>";
  content += "<td><button type=button style=background:#FFE599 onclick=controlPm(11)>Push up</button></td>";
  content += "</tr>";
  content += "<tr height=19%>";
  content += "<td><button type=button style=background:#FFE599 onclick=controlPm(8)>Lie</button></td>";
  content += "<td><button type=button style=background:#FFE599 onclick=controlPm(10)>Fighting</button></td>";
  content += "<td><button type=button style=background:#FFBFBF onclick=controlPm(12)>Sleep</button></td>";
  content += "</tr>";
  content += "<tr height=19%>";
  content += "<td><button type=button style=background:#CFBFFF onclick=controlPm(13)>Dance1</button></td>";
  content += "<td><button type=button style=background:#CFBFFF onclick=controlPm(14)>Dance2</button></td>";
  content += "<td><button type=button style=background:#CFBFFF onclick=controlPm(15)>Dance3</button></td>";
  content += "</tr>";
  content += "</table>";
  content += "</body>";
  content += "<script>";
  content += "function controlPm(id) {";
  content += "var xhttp = new XMLHttpRequest();";
  content += "xhttp.onreadystatechange = function() {";
  content += "if (xhttp.readyState == 4 && xhttp.status == 200) {";
  content += "}";
  content += "};";
  content += "xhttp.open(\"GET\", \"controller?pm=\"+id, true);";
  content += "xhttp.send();";
  content += "}";
  content += "</script>";
  content += "</html>";

  server.send(200, "text/html", content);
}

void handleSave()
{
  String key = server.arg("key");
  String value = server.arg("value");
  int keyInt = key.toInt();
  double valueDouble = value.toDouble();
  if(100 == keyInt){
    preferences.putDouble("7", 0);
    preferences.putDouble("6", 0);
    preferences.putDouble("5", 0);
    preferences.putDouble("4", 0);
    preferences.putDouble("3", 0);
    preferences.putDouble("2", 0);
    preferences.putDouble("1", 0);
    preferences.putDouble("0", 0);
    
    Serial.println("Reset all Done");
  }
  else{
    preferences.putDouble(key.c_str(), valueDouble); // 儲存校正值
    Serial.print(F("Servo: "));
    Serial.print(key); 
    Serial.print(F(" value: "));
    Serial.println(value); 
  }
  server.send(200, "text/html", "(key, value)=(" + key + "," + value + ")");
}

void handleEditor()
{
  String content = "";

  content += "<html>";
  content += "<head>";
  content += "<title>Motion editor</title>";
  content += "<meta charset=UTF-8>";
  content += "<meta name=viewport content=width=device-width>";
  content += "<style type=text/css>";
  content += "body {";
  content += "margin: 0px;";
  content += "backgound-color: #FFFFFF;";
  content += "font-family: helvetica, arial;";
  content += "font-size: 100%;";
  content += "color: #555555;";
  content += "}";
  content += "td {";
  content += "text-align: center;";
  content += "}";
  content += "span {";
  content += "font-family: helvetica, arial;";
  content += "font-size: 70%;";
  content += "color: #777777;";
  content += "}";
  content += "input[type=range] {";
  content += "-webkit-appearance: none;";
  content += "background-color: #CCCCCC;";
  content += "width: 70%;";
  content += "height: 20px;";
  content += "}";
  content += "input[type=range]::-webkit-slider-thumb {";
  content += "-webkit-appearance: none;";
  content += "background-color: #4DA6FF;";
  content += "opacity: 0.9;";
  content += "width: 12px;";
  content += "height: 20px;";
  content += "}";
  content += "input[type=text] {";
  content += "width: 40%;";
  content += "font-family: helvetica, arial;";
  content += "font-size: 90%;";
  content += "color: #555555;";
  content += "text-align: center;";
  content += "padding: 3px 3px 3px 3px;";
  content += "}";
  content += "button {";
  content += "width: 40%;";
  content += "font-family: helvetica, arial;";
  content += "font-size: 90%;";
  content += "color: #555555;";
  content += "padding: 5px 5px 5px 5px;";
  content += "border: none;";
  content += "}";
  content += "</style>";
  content += "</head>";
  content += "<body onload='actionCode()'>";
  content += "<br>";
  content += "<table width=100% height=90%>";

  content += "<tr>";
  content += "<td width=50%>ARM1 <span>Default 90<br>0 <input type=range id=range_0 min=0 max=180 value=90 onchange=controlServo(0,'range_0')> 180</span>";
  content += "<br><input type=text id=servo_0 value=90> <button type=button style=background:#FFE599 onclick=controlServo(0,'servo_0')>Send</button></td>";
  content += "<td width=50%>LEG1 <span>Default 90<br>0 <input type=range id=range_4 min=0 max=180 value=90 onchange=controlServo(4,'range_4')> 180</span>";
  content += "<br><input type=text id=servo_4 value=90> <button type=button style=background:#FFE599 onclick=controlServo(4,'servo_4')>Send</button></td>";
  content += "</tr>";

  content += "<tr><td colspan=4><span><br></span></td></tr>";

  content += "<tr>";
  content += "<td>ARM2 <span>Default 90<br>0 <input type=range id=range_1 min=0 max=180 value=90 onchange=controlServo(1,'range_1')> 180</span>";
  content += "<br><input type=text id=servo_1 value=90> <button type=button style=background:#BFDFFF onclick=controlServo(1,'servo_1')>Send</button></td>";
  content += "<td>LEG2 <span>Default 90<br>0 <input type=range id=range_5 min=0 max=180 value=90 onchange=controlServo(5,'range_5')> 180</span>";
  content += "<br><input type=text id=servo_5 value=90> <button type=button style=background:#BFDFFF onclick=controlServo(5,'servo_5')>Send</button></td>";
  content += "</tr>";

  content += "<tr><td colspan=4><span><br></span></td></tr>";

  content += "<tr>";
  content += "<td>ARM3 <span>Default 90<br>0 <input type=range id=range_2 min=0 max=180 value=90 onchange=controlServo(2,'range_2')> 180</span>";
  content += "<br><input type=text id=servo_2 value=90> <button type=button style=background:#BFDFFF onclick=controlServo(2,'servo_2')>Send</button></td>";
  content += "<td>LEG3 <span>Default 90<br>0 <input type=range id=range_6 min=0 max=180 value=90 onchange=controlServo(6,'range_6')> 180</span>";
  content += "<br><input type=text id=servo_6 value=90> <button type=button style=background:#BFDFFF onclick=controlServo(6,'servo_6')>Send</button></td>";
  content += "</tr>";

  content += "<tr><td colspan=4><span><br></span></td></tr>";

  content += "<tr>";
  content += "<td>ARM4 <span>Default 90<br>0 <input type=range id=range_3 min=0 max=180 value=90 onchange=controlServo(3,'range_3')> 180</span>";
  content += "<br><input type=text id=servo_3 value=110> <button type=button style=background:#FFE599 onclick=controlServo(3,'servo_3')>Send</button></td>";
  content += "<td>LEG4 <span>Default 90<br>0 <input type=range id=range_7 min=0 max=180 value=90 onchange=controlServo(7,'range_7')> 180</span>";
  content += "<br><input type=text id=servo_7 value=70> <button type=button style=background:#FFE599 onclick=controlServo(7,'servo_7')>Send</button></td>";
  content += "</tr>";

  content += "<tr>";
  content += "<td colspan=4><br><span>Action Code:<br><output id=actionCode></output></span></font></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td colspan=2><button type=button style=background:#FFCC99 onclick=controlPm(1)>Standby</button></td>";
  content += "</tr>";
  content += "</body>";
  content += "<script>";
  content += "function controlServo(id, textId) {";
  content += "var xhttp = new XMLHttpRequest();";
  content += "var value = document.getElementById(textId).value;";
  content += "document.querySelector('#range_' + id).value = value;";
  content += "document.querySelector('#servo_' + id).value = value;";
  content += "actionCode();";
  content += "xhttp.onreadystatechange = function() {";
  content += "if (xhttp.readyState == 4 && xhttp.status == 200) {";
  content += "document.getElementById(\"demo\").innerHTML = xhttp.responseText;";
  content += "}";
  content += "};";
  content += "xhttp.open(\"GET\",\"controller?servo=\"+id+\"&value=\"+value, true);";
  content += "xhttp.send();";
  content += "}";
  content += "function controlPm(value) {";
  content += "var xhttp = new XMLHttpRequest();";
  content += "xhttp.onreadystatechange = function() {";
  content += "if (xhttp.readyState == 4 && xhttp.status == 200) {";
  content += "document.getElementById(\"demo\").innerHTML = xhttp.responseText;";
  content += "}";
  content += "};";
  content += "xhttp.open(\"GET\", \"controller?pm=\"+value, true);";
  content += "xhttp.send();";
  content += "}";
  content += "function actionCode() {";
  content += "document.querySelector('#actionCode').value =";
  content += "document.getElementById('servo_0').value + ', '";
  content += "+ document.getElementById('servo_1').value + ', '";
  content += "+ document.getElementById('servo_2').value + ', '";
  content += "+ document.getElementById('servo_3').value + ', '";
  content += "+ document.getElementById('servo_4').value + ', '";
  content += "+ document.getElementById('servo_5').value + ', '";
  content += "+ document.getElementById('servo_6').value + ', '";
  content += "+ document.getElementById('servo_7').value;";
  content += "}";
  content += "</script>";
  content += "</html>";

  server.send(200, "text/html", content);
}

/* SERVER CODE END */

/* MOTOR CODE START */
void Set_PWM_to_Servo(int iServo, int iValue)
{
  // 讀取 EEPROM 修正誤差
  iValue = (iValue*MAX/180.0)+MIN;
  double NewPWM = iValue + preferences.getDouble((String(iServo)).c_str(),0);
  Serial.print(F("iServo: "));
  Serial.print(iServo); 
  Serial.print(F(" iValue: "));
  Serial.print(iValue);
  Serial.print(F(" NewPWM: "));
  Serial.println(NewPWM); 
  /* 0 = zero degree 550 = 180 degree*/
  ledcWrite(iServo,NewPWM);
}

void Servo_PROGRAM_Zero()
{
  // 清除備份目前馬達數值
  for (int Index = 0; Index < ALLMATRIX; Index++) {
    Running_Servo_POS[Index] = Servo_Act_0[Index];
  }

  // 重新載入馬達預設數值
  for (int iServo = 0; iServo < ALLSERVOS; iServo++) {
    Set_PWM_to_Servo(iServo,Running_Servo_POS[iServo]);
    delay(50);
  }
}

void Servo_PROGRAM_Run(int iMatrix[][ALLMATRIX], int iSteps)
{
  int INT_TEMP_A, INT_TEMP_B, INT_TEMP_C;

  for (int MainLoopIndex = 0; MainLoopIndex < iSteps; MainLoopIndex++) { // iSteps 步驟主迴圈

    int InterTotalTime = iMatrix[MainLoopIndex][ALLMATRIX - 1]; // InterTotalTime 此步驟總時間

    int InterDelayCounter = InterTotalTime / BASEDELAYTIME; // InterDelayCounter 此步驟基本延遲次數

    for (int InterStepLoop = 0; InterStepLoop < InterDelayCounter; InterStepLoop++) { // 內差次數迴圈

      for (int ServoIndex = 0; ServoIndex < ALLSERVOS; ServoIndex++) { // 馬達主迴圈

        INT_TEMP_A = Running_Servo_POS[ServoIndex]; // 馬達現在位置
        INT_TEMP_B = iMatrix[MainLoopIndex][ServoIndex]; // 馬達目標位置

        if (INT_TEMP_A == INT_TEMP_B) { // 馬達數值不變
          INT_TEMP_C = INT_TEMP_B;
        } else if (INT_TEMP_A > INT_TEMP_B) { // 馬達數值減少
          INT_TEMP_C =  map(BASEDELAYTIME * InterStepLoop, 0, InterTotalTime, 0, INT_TEMP_A - INT_TEMP_B); // PWM內差值 = map(執行次數時間累加, 開始時間, 結束時間, 內差起始值, 內差最大值)
          if (INT_TEMP_A - INT_TEMP_C >= INT_TEMP_B) {
            Set_PWM_to_Servo(ServoIndex, INT_TEMP_A - INT_TEMP_C);
          }
        } else if (INT_TEMP_A < INT_TEMP_B) { // 馬達數值增加
          INT_TEMP_C =  map(BASEDELAYTIME * InterStepLoop, 0, InterTotalTime, 0, INT_TEMP_B - INT_TEMP_A); // PWM內差值 = map(執行次數時間累加, 開始時間, 結束時間, 內差起始值, 內差最大值)
          if (INT_TEMP_A + INT_TEMP_C <= INT_TEMP_B) {
            Set_PWM_to_Servo(ServoIndex, INT_TEMP_A + INT_TEMP_C);
          }
        }

      }

      delay(BASEDELAYTIME);
    }

    // 備份目前馬達數值
    for (int Index = 0; Index < ALLMATRIX; Index++) {
      Running_Servo_POS[Index] = iMatrix[MainLoopIndex][Index];
    }
  }
}
/* MOTOR CODE END */

void setup()
{
  Serial.begin(115200);
  WiFi.softAP(ssid);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);
  if(!MDNS.begin("esp32")) {
     Serial.println("Error starting mDNS");
     return;
  }
  server.on("/",handleIndex);
  server.on("/editor", handleEditor);
  server.on("/controller", handleController);
  server.on("/zero", handleZero);
  server.on("/setting",handleSetting);
  server.on("/save", handleSave);
  // server.on("/led", );

  server.begin();
  Serial.println("HTTP server started");
  MDNS.addService("http", "tcp", 80);

  delay(50);
  motorInit();
  Servo_PROGRAM_Zero();

  ws2812fx.init();
  ws2812fx.setBrightness(1);
  ws2812fx.setSegment(0, 0,LED_COUNT, FX_MODE_CUSTOM,  RED, 500, false);
  ws2812fx.setCustomMode(myCustomEffect);
  ws2812fx.start();

 // Open Preferences with my-app namespace. Each application module, library, etc
  // has to use a namespace name to prevent key name collisions. We will open storage in
  // RW-mode (second parameter has to be false).
  // Note: Namespace name is limited to 15 chars.
  preferences.begin("iSEBSpider", false);
}

void loop() 
{
  server.handleClient();

  if (Servo_PROGRAM >= 1 ) {
    switch (Servo_PROGRAM) {
      case 1: // Standby 待機
        Serial.println("Standby Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_1, Servo_Prg_1_Step);
        break;
      case 2: // Forward 前行
        Serial.println("Forward Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_2, Servo_Prg_2_Step);
        break;
      case 3: // Backward 退後
        Serial.println("Backward Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_3, Servo_Prg_3_Step);
        break;
      case 4: // Left shift 左移
        Serial.println("Left shift Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_4, Servo_Prg_4_Step);
        break;
      case 5: // Right shift 右移
        Serial.println("Right shift Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_5, Servo_Prg_5_Step);
        break;
      case 6: // Turn left 左轉
        Serial.println("Turn left Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_6, Servo_Prg_6_Step);
        break;
      case 7: // Turn right 右轉
        Serial.println("Turn right Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_7, Servo_Prg_7_Step);
        break;
      case 8: // Lie 趴地
        Serial.println("Lie Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_8, Servo_Prg_8_Step);
        break;
      case 9: // Say Hi 打招呼
        Serial.println("Say Hi Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_9, Servo_Prg_9_Step);
        Servo_PROGRAM_Run(Servo_Prg_1, Servo_Prg_1_Step);
        break;
      case 10: // Fighting 戰鬥姿態
        Serial.println("Fighting Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_10, Servo_Prg_10_Step);
        break;
      case 11: // Push up 掌上壓
        Serial.println("Push up Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_11, Servo_Prg_11_Step);
        break;
      case 12: // Sleep 睡眠姿勢
        Serial.println("Sleep Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_1, Servo_Prg_1_Step);
        Servo_PROGRAM_Run(Servo_Prg_12, Servo_Prg_12_Step);
        break;
      case 13: // 舞步 1
        Serial.println("Dance 1 Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_13, Servo_Prg_13_Step);
        break;
      case 14: // 舞步 2
        Serial.println("Dance 2 Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_14, Servo_Prg_14_Step);
        break;
      case 15: // 舞步 3
        Serial.println("Dance 3 Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_15, Servo_Prg_15_Step);
        break;
      case 100: // 歸零姿勢
        Serial.println("Zero");
        Servo_PROGRAM_Zero();
        delay(300);
        break;
    }
    Servo_PROGRAM = 0;
  }

  ws2812fx.service();
}

uint16_t myCustomEffect(void) { // random chase
  WS2812FX::Segment* seg = ws2812fx.getSegment(); // get the current segment
  for(uint16_t i=seg->stop; i>seg->start; i--) {
    ws2812fx.setPixelColor(i, ws2812fx.getPixelColor(i-1));
  }
  uint32_t color = ws2812fx.getPixelColor(seg->start + 1);
  int r = random(6) != 0 ? (color >> 16 & 0xFF) : random(256);
  int g = random(6) != 0 ? (color >> 8  & 0xFF) : random(256);
  int b = random(6) != 0 ? (color       & 0xFF) : random(256);
  ws2812fx.setPixelColor(seg->start, r, g, b);
  return seg->speed; // return the delay until the next animation step (in msec)
}
