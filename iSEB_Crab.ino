#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WS2812FX.h>
#include <Preferences.h>
#include "pitches.h"

/* buzzer */
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

/* To store the calibration value for each servo motor */
Preferences preferences;

/* led DECLARATION START */
#define LED_COUNT 8
#define LED_PIN 21
#define TIMER_MS 5000

#define buzzerPin 22

WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);
unsigned long last_change = 0;
unsigned long now = 0;

/* led DECLARATION END */

/* PWM DECLARATION START*/
// use 12 bit precission for LEDC timer
#define LEDC_TIMER_12_BIT  12

// use 50 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     50

#define MIN 50
#define MAX 550
// why it is (_1 0) space?
#define WALKING_1 0 /* Chanel 0 */
#define WALKING_2 1 /* Chanel 1 */
#define WALKING_3 2 /* Chanel 2 */
#define WALKING_4 3 /* Chanel 3 */
#define MERUS_1 4 /* Chanel 4 */
#define MERUS_2 5 /* Chanel 5 */
#define MERUS_3 6 /* Chanel 6 */
#define MERUS_4 7 /* Chanel 7 */
#define BUZZER_PWM 8 /* Channel 8 */
/* PWM DECLARATION END */

/* SERVER DECLARATION START */
/* Put your SSID & Password */
const char* ssid = "iSEB Crab";  // Enter SSID here
const char* password = "12345678";  //Enter Password here

/* Put IP Address details */
//From libray, variable decralation - IPAddress - data class. 
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
const int BASEDELAYTIME = 20; // 10 ms

// Backup servo value
int Running_Servo_POS [ALLMATRIX] = {};

// Servo zero position 
//Array 
int Servo_Act_0 [ ] PROGMEM = {  90,  90, 90,  90,  15, 155, 155, 15,  500  };

// Standby 
int Servo_Act_1 [ ] PROGMEM = {  90,  90, 90,  90,  70, 100, 100, 70,  500  };

// Forward 
int Servo_Prg_2_Step = 11;
int Servo_Prg_2 [][ALLMATRIX] PROGMEM = {
 //WALKING1,WALKING2,WALKING3,WALKING4,MERUS1,MERUS2,MERUS3,MERUS4, ms
  {  90,  90,  90,  90,  80,  85,  85, 80, 200  }, // standby
  {  90,  90,  90,  50,  55,  85,  85, 60, 200  }, // MERUS1,4 up; WALKING4 fw
  {  90,  90,  90,  50,  80,  85,  85, 80, 200  }, // MERUS1,4 dn
  {  90,  90,  90,  50,  80, 115, 115, 80, 200  }, // MERUS2,3 up
  {  50, 120,  90,  90,  80, 115, 115, 80, 200  }, // WALKING1,4 bk; WALKING2 fw
  {  50, 120,  90,  90,  80,  85,  85, 80, 200  }, // MERUS2,3 dn
  {  50, 120,  90,  90,  55,  85,  85, 55, 200  }, // MERUS1,4 up; WALKING1 fw
  {  90,  90, 120,  90,  55,  85,  85, 55, 200  }, // WALKING2,3 bk
  {  90,  90, 120,  90,  80,  85,  85, 80, 200  }, // MERUS1,4 dn
  {  90,  90, 120,  90,  80,  85, 115, 80, 200  }, // MERUS3 up 
  {  90,  90,  90,  90,  80,  85,  85, 80, 200  }, // MERUS3 dn WALKING3 fw
};

	// Backward 
int Servo_Prg_3_Step = 11;
int Servo_Prg_3 [][ALLMATRIX] PROGMEM = {
  //WALKING1,WALKING2,WALKING3,WALKING4,MERUS1,MERUS2,MERUS3,MERUS4, ms
  {   90,  90,  90,  90,  80,  85,  85, 80, 200  }, // standby
  {   60,  90,  90,  90,  55,  85,  85, 50, 200  }, // MERUS4,1 up; WALKING1 fw
  {   60,  90,  90,  90,  80,  85,  85, 80, 200  }, // MERUS4,1 dn
  {   60,  90,  90,  90,  80, 115, 115, 80, 200  }, // MERUS3,2 up
  {   90,  90, 120,  60,  80, 115, 115, 80, 200  }, // WALKING4,1 bk; WALKING3 fw
  {   90,  90, 120,  60,  80,  85,  85, 80, 200  }, // MERUS3,2 dn
  {   90,  90, 120,  90,  55,  85,  85, 55, 200  }, // MERUS4,1 up; WALKING4 fw
  {   90, 120,  90,  90,  55,  85,  85, 55, 200  }, // WALKING3,2 bk
  {   90, 120,  90,  90,  80,  85,  85, 80, 200  }, // MERUS4,1 dn
  {   90, 120,  90,  90,  80, 115,  85, 80, 200  }, // MERUS2 up WALKING2 fw
  {   90,  90,  90,  90,  80,  85,  85, 80, 200  }, // MERUS2 dn 
};

// Left shift 
int Servo_Prg_4_Step = 11;
int Servo_Prg_4 [][ALLMATRIX] PROGMEM = {
  //WALKING1,WALKING2,WALKING3,WALKING4,MERUS1,MERUS2,MERUS3,MERUS4,  ms
  {   90,  90,  90,  90,  80,  85,  85,  80, 300  }, // standby
  {   90,  90,  45,  90,  80, 120, 120,  80, 300  }, // MERUS2,3 up; WALKING3 fw
  {   90,  90,  45,  90,  80,  85,  85,  80, 300  }, // MERUS2,3 dn
  {   90,  90,  45,  90,  50,  85,  85,  50, 300  }, // MERUS4,1 up
  {   90,  45,  90, 120,  50,  85,  85,  50, 300  }, // WALKING2,3 bk; WALKING4 fw
  {   90,  45,  90, 120,  80,  85,  85,  80, 300  }, // MERUS4,1 dn
  {   90,  90,  90, 120,  80, 120, 120,  80, 300  }, // MERUS2,3 up; WALKING2 fw
  {  120,  90,  90,  90,  80, 120, 120,  80, 300  }, // WALKING4,1 bk
  {  120,  90,  90,  90,  80,  85,  85,  80, 300  }, // MERUS2,3 dn
  {  120,  90,  90,  90,  50,  85,  85,  80, 300  }, // MERUS1 up
  {   90,  90,  90,  90,  80,  85,  85,  80, 300  }, // WALKING1 fw MERUS1 dn
};

// Right shift 
int Servo_Prg_5_Step = 11;
int Servo_Prg_5 [][ALLMATRIX] PROGMEM = {
  //WALKING1,WALKING2,WALKING3,WALKING4,MERUS1,MERUS2,MERUS3,MERUS4,  ms
  {   90,  90,  90,  90,  80,  85,  85,  80, 300  }, // standby
  {   90,  45,  90,  90,  80, 120, 120,  80, 300  }, // MERUS3,2 up; WALKING2 fw
  {   90,  45,  90,  90,  80,  85,  85,  80, 300  }, // MERUS3,2 dn
  {   90,  45,  90,  90,  50,  85,  85,  50, 300  }, // MERUS1,4 up
  {  120,  90,  45,  90,  50,  85,  85,  50, 300  }, // WALKING3,2 bk; WALKING1 fw
  {  120,  90,  45,  90,  80,  85,  85,  80, 300  }, // MERUS1,4 dn
  {  120,  90,  90,  90,  80, 120, 120,  80, 300  }, // MERUS3,2 up; WALKING3 fw
  {   90,  90,  90, 120,  80, 120, 120,  80, 300  }, // WALKING1,4 bk
  {   90,  90,  90, 120,  80,  85,  85,  80, 300  }, // MERUS3,2 dn
  {   90,  90,  90, 120,  80,  85,  85,  50, 300  }, // MERUS4 up
  {   90,  90,  90,  90,  80,  85,  85,  80, 300  }, // WALKING4 fw MERUS4 dn

};

// Turn left 
int Servo_Prg_6_Step = 8;
int Servo_Prg_6 [][ALLMATRIX] PROGMEM = {
  //WALKING1,WALKING2,WALKING3,WALKING4,MERUS1,MERUS2,MERUS3,MERUS4,ms
  {   90,  90,  90,  90,  80,  85,  85,  80, 300  }, // standby
  {   90,  90,  90,  90,  50,  85,  85,  50, 300  }, // MERUS4,1 up
  {  135,  90,  90, 135,  50,  85,  85,  50, 300  }, // WALKING4,1 turn
  {  135,  90,  90, 135,  80,  85,  85,  80, 300  }, // MERUS4,1 dn
  {  135,  90,  90, 135,  80, 120, 120,  80, 300  }, // MERUS3,2 up
  {  135, 135, 135, 135,  80, 120, 120,  80, 300  }, // WALKING3,2 turn
  {  135, 135, 135, 135,  80,  85,  85,  80, 300  }, // MERUS3,2 dn
  {   90,  90,  90,  90,  80,  85,  85,  80, 300  }, // WALKING1,2,3,4 turn
};

// Turn right 
int Servo_Prg_7_Step = 8;
int Servo_Prg_7 [][ALLMATRIX] PROGMEM = {
  //WALKING1,WALKING2,WALKING3,WALKING4,MERUS1,MERUS2,MERUS3,MERUS4,  ms
  {   90,  90,  90,  90,  80,  85,  85,  80, 300  }, // standby
  {   90,  90,  90,  90,  80, 120, 120,  80, 300  }, // MERUS3,2 up
  {   90,  45,  45,  90,  80, 120, 120,  80, 300  }, // WALKING3,2 turn
  {   90,  45,  45,  90,  80,  85,  85,  80, 300  }, // MERUS3,2 dn
  {   90,  45,  45,  90,  50,  85,  85,  50, 300  }, // MERUS4,1 up
  {   45,  45,  45,  45,  50,  85,  85,  50, 300  }, // WALKING4,1 turn
  {   45,  45,  45,  45,  80,  85,  85,  80, 300  }, // MERUS4,1 dn
  {   90,  90,  90,  90,  80,  85,  85,  80, 300  }, // WALKING1,2,3,4 turn
};

// Lie 
int Servo_Prg_8_Step = 2;
int Servo_Prg_8 [][ALLMATRIX] PROGMEM = {
  //WALKING1,WALKING2,WALKING3,WALKING4,MERUS1,MERUS2,MERUS3,MERUS4,  ms
  {   90,  90,  90,  90,  80,  85,  85, 80, 200 }, // center
  {   90,  90,  90,  90,  40, 125, 125, 40, 400 }, // lir
};

// Say Hi 
int Servo_Prg_9_Step = 5;
int Servo_Prg_9 [][ALLMATRIX] PROGMEM = {
  //WALKING1,WALKING2,WALKING3,WALKING4,MERUS1,MERUS2,MERUS3,MERUS4,  ms
  {   90,  90,  90,  90,  80,  85,  85, 80, 200 }, // center
  {   90,  90,  90,  90,  80, 120,  85, 50, 200 }, // MERUS2, 4 down
  {   90,  90,  90,  90,  80,  85,  85, 80, 200 }, // standby
  {   90,  90,  90,  90,  80, 120,  85, 50, 200 }, // MERUS2, 4 down
  {   90,  90,  90,  90,  80,  85,  85, 80, 200 }, // center
};

// Fighting 
int Servo_Prg_10_Step = 12;
int Servo_Prg_10 [][ALLMATRIX] PROGMEM = {
  //WALKING1,WALKING2,WALKING3,WALKING4,MERUS1,MERUS2,MERUS3,MERUS4,ms
  {   90,  90,  90,  90,  80,  85,  85,  80,200 }, // center
  {   90,  90,  90,  90,  50, 85,  120,  80,200 }, // MERUS4, 2 up ; MERUS1, 3 down
  {   70,  70,  70,  70,  50, 85,  120,  80,200 }, // body turn left
  {  110, 110, 110, 110,  50, 85,  120,  80,200 }, // body turn right
  {   70,  70,  70,  70,  50, 85,  120,  80,200 }, // body turn left
  {  110, 110, 110, 110,  50, 85,  120,  80,200 }, // body turn right
  {   90,  90,  90,  90,  80, 120,  85,  50,200 }, // MERUS4, 2 down
  {   70,  70,  70,  70,  80, 120,  85,  50,200 }, // body turn left
  {  110, 110, 110, 110,  80, 120,  85,  50,200 }, // body turn right
  {   70,  70,  70,  70,  80, 120,  85,  50,200 }, // body turn left
  {  110, 110, 110, 110,  80, 120,  85,  50,200 }, // body turn right
  {   90,  90,  90,  90,  50, 85,  120,  80,200 }  // MERUS4, 2 up ; MERUS1, 3 down
};

// Push up 
int Servo_Prg_11_Step = 11;
int Servo_Prg_11 [][ALLMATRIX] PROGMEM = {
  //WALKING1,WALKING2,WALKING3,WALKING4,MERUS1,MERUS2,MERUS3,MERUS4, ms
  {   90,  90,  90,  90,  80,  85,  85,  80,200 }, // center
  {   90,  90,  90,  90,  50, 120, 120,  50,400 }, // down
  {   90,  90,  90,  90,  80,  85,  85,  80,500 }, // up
  {   90,  90,  90,  90,  50, 120, 120,  50,600 }, // down
  {   90,  90,  90,  90,  80,  85,  85,  80,700 }, // up
  {   90,  90,  90,  90,  50, 120, 120,  50,1300 }, // down
  {   90,  90,  90,  90,  80,  85,  85,  80,1800 }, // up
  {   90,  90,  90,  90,  50, 120, 120,  50,200 }, // down
  {   90,  90,  90,  90,  80, 120, 120,  50,500 }, // MERUS 1 up
  {   90,  90,  90,  90,  80,  85, 120,  50,500 }, // MERUS 2 up
  {   90,  90,  90,  90,  80,  85,  85,  80,500 }, // MERUS3, MERUS4 up
};

// Sleep 
int Servo_Prg_12_Step = 2;
int Servo_Prg_12 [][ALLMATRIX] PROGMEM = {
  //WALKING1,WALKING2,WALKING3,WALKING4,MERUS1,MERUS2,MERUS3,MERUS4, ms
  {   90,  90,  90,  90,  70, 100, 100,  70, 200 }, // standby
  {   65, 115, 115,  65,  90,  70,  70,  90, 500 }, // MERUS1,4 up
};

// Dance 1
int Servo_Prg_13_Step = 10;
int Servo_Prg_13 [][ALLMATRIX] PROGMEM = {
  //WALKING1,WALKING2,WALKING3,WALKING4,MERUS1,MERUS2,MERUS3,MERUS4, ms
  {   90,  90,  90,  90,  80,  85,  85,  80, 200 }, // center
  {   90,  90,  90,  90,  80,  85,  85,  50, 300 }, // MERUS4 dn
  {   90,  90,  90,  90,  80,  85, 120,  80, 300 }, // MERUS4 up; MERUS3 dn
  {   90,  90,  90,  90,  50,  85,  85,  80, 300 }, // MERUS3 up; MERUS1 dn
  {   90,  90,  90,  90,  80, 120,  85,  80, 300 }, // MERUS1 up; MERUS2 dn
  {   90,  90,  90,  90,  80,  85,  85,  50, 300 }, // MERUS2 up; MERUS4 dn
  {   90,  90,  90,  90,  80,  85, 120,  80, 300 }, // MERUS4 up; MERUS3 dn
  {   90,  90,  90,  90,  50,  85,  85,  80, 300 }, // MERUS3 up; MERUS1 dn
  {   90,  90,  90,  90,  80, 120,  85,  80, 300 }, // MERUS1 up; MERUS2 dn
  {   90,  90,  90,  90,  80,  85,  85,  80, 300 }, // MERUS2 up
};

// Dance 2
int Servo_Prg_14_Step = 10;
int Servo_Prg_14 [][ALLMATRIX] PROGMEM = {
  //WALKING1,WALKING2,WALKING3,WALKING4,MERUS1,MERUS2,MERUS3,MERUS4,  ms
  {   90,  90,  90,  90,  80,  85,  85,  80, 200 },  // standby
  {  120,  60,  60, 120,  50, 120, 120,  50, 300  }, // MERUS1,2,3,4 two sides
  {  120,  60,  60, 120,  80,  85, 120,  50, 300  }, // MERUS1,2 up
  {  120,  60,  60, 120,  50, 120,  85,  80, 300  }, // MERUS1,2 dn; MERUS3,4 up
  {  120,  60,  60, 120,  80,  85, 120,  50, 300  }, // MERUS3,4 dn; MERUS1,2 up
  {  120,  60,  60, 120,  50, 120,  85,  80, 300  }, // MERUS1,2 dn; MERUS3,4 up
  {  120,  60,  60, 120,  80,  85, 120,  50, 300  }, // MERUS3,4 dn; MERUS1,2 up
  {  120,  60,  60, 120,  50, 120,  85,  80, 300  }, // MERUS1,2 dn; MERUS3,4 up
  {  120,  60,  60, 120,  80,  85, 120,  50, 300  }, // MERUS3,4 dn; MERUS1,2 up
  {  120,  60,  60, 120,  50, 120, 120,  50, 300  }, // MERUS1,2 dn
  {   90,  90,  90,  90,  80,  85,  85,  80, 200 },  // standby
};


// Dance 3
int Servo_Prg_15_Step = 9;
int Servo_Prg_15 [][ALLMATRIX] PROGMEM = {
   //WALKING1,WALKING2,WALKING3,WALKING4,MERUS1,MERUS2,MERUS3,MERUS4,  ms
  {   140,  90,  40,  90, 50,  120, 120,  50,  300  }, // MERUS1,2,3,4 bk
  {   140,  90,  40,  90, 80,   85,  85,  50,  300  }, // MERUS1,2,3 up
  {   140,  90,  40,  90, 50,  120, 120,  50,  300  }, // MERUS1,2,3 dn
  {   140,  90,  40,  90, 80,  120,  85,  80,  300  }, // MERUS1,3,4 up
  {   140,  90,  40,  90, 50,  120, 120,  50,  300  }, // MERUS1,3,4 dn
  {   140,  90,  40,  90, 80,   85,  85,  50,  300  }, // MERUS1,2,3 up
  {   140,  90,  40,  90, 50,  120, 120,  50,  300  }, // MERUS1,2,3 dn
  {   140,  90,  40,  90, 80,  120,  85,  80,  300  }, // MERUS1,3,4 up
  {   140,  90,  40,  90, 50,  120, 120,  50,  300  }, // MERUS1,3,4 dn
};

/* MOTOR DECLARATION START */

/* PWM CODE START */
// From ESP 32 library - sharing led and pwm function. 
// input channel / frequency / resolution
// base on servo spec: frequency - 50hz 
void motorInit()
{
  // Setup timer 
  ledcSetup(WALKING_1, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(WALKING_2, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(WALKING_3, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(WALKING_4, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(MERUS_1, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(MERUS_2, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(MERUS_3, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(MERUS_4, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(BUZZER_PWM, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);


  // Attach timer to a led pin
  ledcAttachPin(19, WALKING_1);  /* WALKING_1 *//* CN15 *//* PIN 19*/
  ledcAttachPin(15, WALKING_2);  /* WALKING_2 *//* CN9  *//* PIN 15*/
  ledcAttachPin(33, WALKING_3);  /* WALKING_3 *//* CN7  *//* PIN 33*/
  ledcAttachPin(13, WALKING_4);  /* WALKING_4*//* CN1  *//* PIN 13*/
  ledcAttachPin(23, MERUS_1);  /* MERUS_1 *//* CN16 *//* PIN 23*/
  ledcAttachPin( 4, MERUS_2);  /* MERUS_2 *//* CN10 *//* PIN  4*/
  ledcAttachPin(32, MERUS_3);  /* MERUS_3 *//* CN8  *//* PIN 32*/
  ledcAttachPin(12, MERUS_4);  /* MERUS_4 *//* CN2  *//* PIN 12*/
  ledcAttachPin(buzzerPin,BUZZER_PWM);
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
  content += "<td width=50%>WALKING1<br/><input type=text id=servo_0 value=\"" + servo0ValStr + "\"><button type=button style=background:#FFE599 onclick=saveServo(0,'servo_0')>SET</button></td>";
  content += "<td width=50%>MERUS1<br/><input type=text id=servo_4 value=\"" + servo4ValStr + "\"><button type=button style=background:#FFE599 onclick=saveServo(4,'servo_4')>SET</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td>WALKING2<br/><input type=text id=servo_1 value=\"" + servo1ValStr + "\"><button type=button onclick=saveServo(1,'servo_1')>SET</button></td>";
  content += "<td>MERUS2<br/><input type=text id=servo_5 value=\"" + servo5ValStr + "\"><button type=button onclick=saveServo(5,'servo_15')>SET</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td>WALKING3<br/><input type=text id=servo_2 value=\"" + servo2ValStr + "\"><button type=button onclick=saveServo(2,'servo_2')>SET</button></td>";
  content += "<td>MERUS3<br/><input type=text id=servo_6 value=\"" + servo6ValStr + "\"><button type=button onclick=saveServo(6,'servo_6')>SET</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td>WALKING4<br/><input type=text id=servo_3 value=\"" + servo3ValStr + "\"><button type=button style=background:#FFE599 onclick=saveServo(3,'servo_3')>SET</button></td>";
  content += "<td>MERUS4<br/><input type=text id=servo_7 value=\"" + servo7ValStr + "\"><button type=button style=background:#FFE599 onclick=saveServo(7,'servo_7')>SET</button></td>";
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
  content += "width: 60%;";
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
  content += "<td width=50%><button type=button style=background:#FFE599 onclick=controlServo(5,155)>MERUS_2</button></td>";
  content += "<td width=50%><button type=button style=background:#FFE599 onclick=controlServo(7,15)>MERUS_4</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td><button type=button onclick=controlServo(1,90)>WALKING_2</button></td>";
  content += "<td><button type=button onclick=controlServo(3,90)>WALKING_4</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td><button type=button onclick=controlServo(0,90)>WALKING_1</button></td>";
  content += "<td><button type=button onclick=controlServo(2,90)>WALKING_3</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td><button type=button style=background:#FFE599 onclick=controlServo(4,15)>MERUS_1</button></td>";
  content += "<td><button type=button style=background:#FFE599 onclick=controlServo(6,155)>MERUS_3</button></td>";
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
  content += "<title>SMLab iCrab ™</title>";
  content += "<meta charset=UTF-8>";
  content += "<meta name=viewport content=width=device-width>";
  content += "<style type=text/css>";
  content += "body {";
  content += "margin:0px;";
  content += "backgound-color:#FFFFFF;";
  content += "font-family:helvetica,arial;";
  content += "font-size:100%;";
  content += "color: #555555;";
  content += "text-align: center;";
  content += "}";
  content += "td {";
  content += "text-align: center;";
  content += "}";
  content += "span {";
  content += "font-family:helvetica,arial;";
  content += "font-size:70%;";
  content += "color:#777777;";
  content += "}";
  content += ".button{";
  content += "width:90%;";
  content += "height:90%;";
  content += "font-family:helvetica,arial;";
  content += "font-size:100%;";
  content += "color:#555555;";
  content += "background:#BFDFFF;";
  content += "border-radius:4px;";
  content += "padding: 2px 2px 2px 2px;";
  content += "border:none;}";
  content += ".button:active{";
  content += "background-color:#999;";
  content += "color:white;}";
  content += ".button2{background-color:#BFFFCF;}";
  content += ".button3{background-color:#FFBFBF;}"; 
  content += ".button4{background-color:#FFCC99;}";
  content += ".button5{background-color:#FFE599;}";
  content += ".button6{background-color:#CFBFFF;}";
  content += "</style>";
  content += "</head>";
  content += "<body><h1>SMLab iCrab ™</h1>";
  content += "<table width=100% height=90%>";
  content += "<tr height=19%>";
  content += "<td width=33%><button class=\"button button2\" onclick=controlPm(6)>Turn left</button></td>";
  content += "<td width=33%><button class=\"button\" onclick=controlPm(2)>Forward</button></td>";
  content += "<td width=33%><button class=\"button button2\" onclick=controlPm(7)>Turn right</button></td>";
  content += "</tr>";
  content += "<tr height=19%>";
  content += "<td><button class=\"button\" onclick=controlPm(4)>Left shift</button></td>";
  content += "<td><button class=\"button\" onclick=controlPm(3)>Backward</button></td>";
  content += "<td><button class=\"button\" onclick=controlPm(5)>Right shift</button></td>";
  content += "</tr>";
  content += "<tr height=5%><td colspan=3><span><br></span></td></tr>";
  content += "<tr height=20%>";
  content += "<td><button class=\"button button4\" onclick=controlPm(1)>Stand </button></td>";
  content += "<td><button class=\"button button5\" onclick=controlPm(9)>Say Hi</button></td>";
  content += "<td><button class=\"button button5\" onclick=controlPm(11)>Push up</button></td>";
  content += "</tr>";
  content += "<tr height=19%>";
  content += "<td><button class=\"button button5\" onclick=controlPm(8)>Lie</button></td>";
  content += "<td><button class=\"button button5\" onclick=controlPm(10)>Fighting</button></td>";
  content += "<td><button class=\"button button3\" onclick=controlPm(12)>Sleep</button></td>";
  content += "</tr>";
  content += "<tr height=19%>";
  content += "<td><button class=\"button button6\" onclick=controlPm(13)>Dance1</button></td>";
  content += "<td><button class=\"button button6\" onclick=controlPm(14)>Dance2</button></td>";
  content += "<td><button class=\"button button6\" onclick=controlPm(15)>Dance3</button></td>";
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
  content += "<td width=50%>WALKING1 <span>Default 90<br>0 <input type=range id=range_0 min=0 max=180 value=90 onchange=controlServo(0,'range_0')> 180</span>";
  content += "<br><input type=text id=servo_0 value=90> <button type=button style=background:#FFE599 onclick=controlServo(0,'servo_0')>Send</button></td>";
  content += "<td width=50%>MERUS1 <span>Default 90<br>0 <input type=range id=range_4 min=0 max=180 value=90 onchange=controlServo(4,'range_4')> 180</span>";
  content += "<br><input type=text id=servo_4 value=90> <button type=button style=background:#FFE599 onclick=controlServo(4,'servo_4')>Send</button></td>";
  content += "</tr>";

  content += "<tr><td colspan=4><span><br></span></td></tr>";

  content += "<tr>";
  content += "<td>WALKING2 <span>Default 90<br>0 <input type=range id=range_1 min=0 max=180 value=90 onchange=controlServo(1,'range_1')> 180</span>";
  content += "<br><input type=text id=servo_1 value=90> <button type=button style=background:#BFDFFF onclick=controlServo(1,'servo_1')>Send</button></td>";
  content += "<td>MERUS2 <span>Default 90<br>0 <input type=range id=range_5 min=0 max=180 value=90 onchange=controlServo(5,'range_5')> 180</span>";
  content += "<br><input type=text id=servo_5 value=90> <button type=button style=background:#BFDFFF onclick=controlServo(5,'servo_5')>Send</button></td>";
  content += "</tr>";

  content += "<tr><td colspan=4><span><br></span></td></tr>";

  content += "<tr>";
  content += "<td>WALKING3 <span>Default 90<br>0 <input type=range id=range_2 min=0 max=180 value=90 onchange=controlServo(2,'range_2')> 180</span>";
  content += "<br><input type=text id=servo_2 value=90> <button type=button style=background:#BFDFFF onclick=controlServo(2,'servo_2')>Send</button></td>";
  content += "<td>MERUS3 <span>Default 90<br>0 <input type=range id=range_6 min=0 max=180 value=90 onchange=controlServo(6,'range_6')> 180</span>";
  content += "<br><input type=text id=servo_6 value=90> <button type=button style=background:#BFDFFF onclick=controlServo(6,'servo_6')>Send</button></td>";
  content += "</tr>";

  content += "<tr><td colspan=4><span><br></span></td></tr>";

  content += "<tr>";
  content += "<td>WALKING4 <span>Default 90<br>0 <input type=range id=range_3 min=0 max=180 value=90 onchange=controlServo(3,'range_3')> 180</span>";
  content += "<br><input type=text id=servo_3 value=110> <button type=button style=background:#FFE599 onclick=controlServo(3,'servo_3')>Send</button></td>";
  content += "<td>MERUS4 <span>Default 90<br>0 <input type=range id=range_7 min=0 max=180 value=90 onchange=controlServo(7,'range_7')> 180</span>";
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
  Serial.print(F("iServo: "));
  Serial.print(iServo); 
  Serial.print(F(" iValue: "));
  Serial.println(iValue);
  // Read from EEPROM to fix zero error reading
  iValue = (iValue*MAX/180.0)+MIN; /* convertion to pwm value */
  double NewPWM = iValue + preferences.getDouble((String(iServo)).c_str(),0);

  /* 0 = zero degree 550 = 180 degree*/
  ledcWrite(iServo,NewPWM);
}

void Servo_PROGRAM_Standby()
{
  /* Update standby value to servo motor variable */
  for (int Index = 0; Index < ALLMATRIX; Index++) {
    Running_Servo_POS[Index] = Servo_Act_1[Index];
  }

  /* Update the servo motor to standby position */
  for (int iServo = 0; iServo < ALLSERVOS; iServo++) {
    Set_PWM_to_Servo(iServo,Running_Servo_POS[iServo]);
    delay(50);
  }
}

void Servo_PROGRAM_Zero()
{
  /* Update zero value to servo motor variable */
  for (int Index = 0; Index < ALLMATRIX; Index++) {
    Running_Servo_POS[Index] = Servo_Act_0[Index];
  }

    /* Update the servo motor to zero position */
  for (int iServo = 0; iServo < ALLSERVOS; iServo++) {
    Set_PWM_to_Servo(iServo,Running_Servo_POS[iServo]);
    delay(50);
  }
}

void Servo_PROGRAM_Run(int iMatrix[][ALLMATRIX], int iSteps)
{
  int INT_TEMP_A, INT_TEMP_B, INT_TEMP_C;

  for (int MainLoopIndex = 0; MainLoopIndex < iSteps; MainLoopIndex++) { // iSteps number of step

    int InterTotalTime = iMatrix[MainLoopIndex][ALLMATRIX - 1]; // InterTotalTime - total time needed

    int InterDelayCounter = InterTotalTime / BASEDELAYTIME; // InterDelayCounter time / step 

    for (int InterStepLoop = 0; InterStepLoop < InterDelayCounter; InterStepLoop++) { 

      for (int ServoIndex = 0; ServoIndex < ALLSERVOS; ServoIndex++) { 

        INT_TEMP_A = Running_Servo_POS[ServoIndex]; // servo motor current position
        INT_TEMP_B = iMatrix[MainLoopIndex][ServoIndex]; // servo motor next position

        if (INT_TEMP_A == INT_TEMP_B) { // no update in servo motor position
          INT_TEMP_C = INT_TEMP_B;
        } else if (INT_TEMP_A > INT_TEMP_B) { // servo motor position position reduce
          INT_TEMP_C =  map(BASEDELAYTIME * InterStepLoop, 0, InterTotalTime, 0, INT_TEMP_A - INT_TEMP_B); 
          if (INT_TEMP_A - INT_TEMP_C >= INT_TEMP_B) {
            Set_PWM_to_Servo(ServoIndex, INT_TEMP_A - INT_TEMP_C);
          }
        } else if (INT_TEMP_A < INT_TEMP_B) { /// servo motor position position increase
          INT_TEMP_C =  map(BASEDELAYTIME * InterStepLoop, 0, InterTotalTime, 0, INT_TEMP_B - INT_TEMP_A); 
          if (INT_TEMP_A + INT_TEMP_C <= INT_TEMP_B) {
            Set_PWM_to_Servo(ServoIndex, INT_TEMP_A + INT_TEMP_C);
          }
        }

      }
      delay(BASEDELAYTIME);
    }

    // back of current servo motor position
    for (int Index = 0; Index < ALLMATRIX; Index++) {
      Running_Servo_POS[Index] = iMatrix[MainLoopIndex][Index];
    }
  }
}
/* MOTOR CODE END */

void setup()
{
  Serial.begin(115200);/* start the serial monitor at 115200 baud rate */
  Serial.write("Hello World\n");


  WiFi.softAP(ssid);/* without password */
  // WiFi.softAP(ssid, password);;/* with password */
  WiFi.softAPConfig(local_ip, gateway, subnet); /* to add exception to server */

  /* to access Index page through iSEBCrab.local */
  if(!MDNS.begin("iSEBCrab")) { 
     Serial.println("Error starting mDNS");
     return;
  }
  server.on("/",handleIndex);
  server.on("/editor", handleEditor);
  server.on("/controller", handleController);
  server.on("/zero", handleZero);
  server.on("/setting",handleSetting);
  server.on("/save", handleSave);
  server.begin();
  Serial.println("HTTP server started");
  MDNS.addService("http", "tcp", 80);
  
  if(false == WiFi.setTxPower(WIFI_POWER_2dBm))/* control the wifi tx power */
  {
    Serial.println("Update Tx Power fail");
  } 
  motorInit();
  Servo_PROGRAM_Zero();

  // 100Hz tone for 1 second
  Serial.write("Buzzer\n"); 
  ledcWriteNote(BUZZER_PWM, NOTE_C, 4);
  delay(100);
  ledcWriteNote(BUZZER_PWM, NOTE_D, 4);
  delay(100);
  ledcWriteNote(BUZZER_PWM, NOTE_E, 4);
  delay(100);
  ledcWriteNote(BUZZER_PWM, NOTE_F, 4);
  delay(100);
  ledcWriteNote(BUZZER_PWM, NOTE_G, 4);
  delay(100);
  ledcWriteNote(BUZZER_PWM, NOTE_A, 4);
  delay(100);
  ledcWriteNote(BUZZER_PWM, NOTE_B, 4);
  delay(100);
  ledcWriteNote(BUZZER_PWM, NOTE_C, 5);
  delay(100);
  ledcWriteTone(BUZZER_PWM,0);

  ws2812fx.init();
  ws2812fx.setBrightness(1);
  ws2812fx.setSegment(0, 0,LED_COUNT, FX_MODE_CUSTOM,  RED, 200, false);
  ws2812fx.setCustomMode(myCustomEffect);
  ws2812fx.start();

 // Open Preferences with my-app namespace. Each application module, library, etc
  // has to use a namespace name to prevent key name collisions. We will open storage in
  // RW-mode (second parameter has to be false).
  // Note: Namespace name is limited to 15 chars.
  preferences.begin("iSEBCrab", false);
}

void loop() 
{
  server.handleClient();
  if (Servo_PROGRAM >= 1 ) {
    delay(500);
    switch (Servo_PROGRAM) {
      case 1: // Standby 
        Serial.println("Standby Button Pressed.");
        Servo_PROGRAM_Standby();
        break;
      case 2: // Forward 
        Serial.println("Forward Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_2, Servo_Prg_2_Step);
        break;
      case 3: // Backward 
        Serial.println("Backward Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_3, Servo_Prg_3_Step);
        break;
      case 4: // Left shift 
        Serial.println("Left shift Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_4, Servo_Prg_4_Step);
        break;
      case 5: // Right shift 
        Serial.println("Right shift Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_5, Servo_Prg_5_Step);
        break;
      case 6: // Turn left 
        Serial.println("Turn left Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_6, Servo_Prg_6_Step);
        break;
      case 7: // Turn right 
        Serial.println("Turn right Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_7, Servo_Prg_7_Step);
        break;
      case 8: // Lie 
        Serial.println("Lie Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_8, Servo_Prg_8_Step);
        break;
      case 9: // Say Hi 
        Serial.println("Say Hi Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_9, Servo_Prg_9_Step);
        break;
      case 10: // Fighting 
        Serial.println("Fighting Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_10, Servo_Prg_10_Step);
        break;
      case 11: // Push up 
        Serial.println("Push up Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_11, Servo_Prg_11_Step);
        Servo_PROGRAM_Standby();
        break;
      case 12: // Sleep 
        Serial.println("Sleep Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_12, Servo_Prg_12_Step);
        break;
      case 13: // Dance 1
        Serial.println("Dance 1 Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_13, Servo_Prg_13_Step);
        break;
      case 14: // Dance 2
        Serial.println("Dance 2 Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_14, Servo_Prg_14_Step);
        break;
      case 15: // Dance 3
        Serial.println("Dance 3 Button Pressed.");
        Servo_PROGRAM_Run(Servo_Prg_15, Servo_Prg_15_Step);
        break;
      case 100: // Zero
        Serial.println("Zero");
        Servo_PROGRAM_Zero();
        break;
    }
    Servo_PROGRAM = 0;
  }
  
  /* to keep update RBG LED */
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
