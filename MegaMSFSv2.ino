#include <Thread.h>
#include <StaticThreadController.h>
#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#include <QList.h>
#include "PinChangeInterrupt.h"

// Version
char version[] = "MegaMSFS v2.01";

// Threads
Thread* printLcdThread = new Thread();
Thread* readEncodersThread = new Thread();
Thread* readButtonssThread = new Thread();
Thread* processMessagesQueueThread = new Thread();
Thread* recvWithStartEndMarkersThread = new Thread();
Thread* parseDataThread = new Thread();
Thread* switchLedsThread = new Thread();
StaticThreadController<7> threadController (printLcdThread, 
  readEncodersThread,
  readButtonssThread, 
  processMessagesQueueThread, 
  recvWithStartEndMarkersThread,
  parseDataThread,
  switchLedsThread);

// The LCD object and other variables
hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip (way faster than the liquid library (does not block code execution))
String emptyLcdLine = "                    ";
int curPage = 0;

// List containing messages to be sent via serial
QList<String> commandsQueue;

// Variables for when we receive a message from serial port
const byte numChars = 80;
char receivedChars[numChars];
boolean newData = false;

// PINs
#define PIN_AUTO_PILOT  50
#define PIN_YAW_DAMP    51
#define PIN_HEADING     52
#define PIN_SY_HEADING  64 //A10
#define PIN_APPROACH    53
#define PIN_NAV         3
#define PIN_ALTITUDE    18
#define PIN_SY_ALTITUDE 65 //A11
#define PIN_VS          2
#define PIN_ROTARY1_CCK 30
#define PIN_ROTARY1_CLK 31
#define PIN_ROTARY2_CCK 32
#define PIN_ROTARY2_CLK 33
#define PIN_ROTARY3_CCK 34
#define PIN_ROTARY3_CLK 35
#define PIN_ROTARY4_CCK 36
#define PIN_ROTARY4_CLK 37

#define PIN_LED_AP      12
#define PIN_LED_YAWDAMP 11
#define PIN_LED_HEAD    10
#define PIN_LED_APPR    9
#define PIN_LED_NAV     8
#define PIN_LED_ALT     7
#define PIN_LED_VS      6

#define PIN_PREV_PAGE   62 //A8
#define PIN_NEXT_PAGE   63 //A9

// FlightSim variables
int cur_autopilot_act = 0;
int cur_yawdamp_act = 0;
int cur_heading_act = 0;
int cur_appr_act = 0;
int cur_nav_act = 0;
int cur_alt_act = 0;
int cur_vs_act = 0;

int cur_altitude = 0;
int cur_heading = 0;
double cur_altimeter = 0;
int cur_fuelp = 0;
volatile boolean autopilot = false;
int cur_wind_dir = 0;
int cur_wind_vel = 0;
int cur_flaps = 0;
String cur_aptext = "----------------";
int cur_heading_var = 0;
int cur_altitude_var = 0;
int cur_vs_var = 0;

// Custom chars for wind direction
byte windPointNorth[] = {B00000, B00100, B01110, B10101, B00100, B00100, B00100, B00000};
byte windPointSouth[] = {B00000, B00100, B00100, B00100, B10101, B01110, B00100, B00000};
byte windPointEast[] = {B00000, B00100, B00010, B11111, B00010, B00100, B00000, B00000};
byte windPointWest[] = {B00000, B00100, B01000, B11111, B01000, B00100, B00000, B00000};
byte windPointNorthWest[] = {B00000, B11110, B11000, B10100, B10010, B00001, B00000, B00000};
byte windPointNorthEast[] = {B00000, B01111, B00011, B00101, B01001, B10000, B00000, B00000};
byte windPointSouthEast[] = {B00000, B00000, B10000, B01001, B00101, B00011, B01111, B00000};
byte windPointSouthWest[] = {B00000, B00000, B00001, B10010, B10100, B11000, B11110, B00000};

// Debouncing Time in Milliseconds
long debouncing_time = 50;

// Variables for the encoders (rotaries)
uint8_t prevEncoderNextCode[4] = {0, 0, 0, 0};
uint16_t encoderStore[4] = {0, 0, 0, 0};

// Variables for the buttons
uint8_t prevButtonState[11] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; //PREV, NEXT, SY_HEADING, SY_ALTITUDE, AUTO_PILOT, YAW_DAMP, HEADING, APPROACH, NAV, ALTITUDE, VS

// Callback function that prints to LCD on every threadController run
void printLcd_cb(){
  
  char buf[10];
  String lcdLine;
    
  if (curPage == 0)
  {
    // Line 1
    lcd.setCursor(0, 0);
    lcdLine = "";
    lcdLine += "Alt: ";
    sprintf(buf, "%05d", cur_altitude);
    lcdLine += buf;
  
    lcdLine += " Hdg:  ";
    sprintf(buf, "%03d", cur_heading);
    lcdLine += buf;

    lcdLine = fillLine(lcdLine);   
    lcd.print(lcdLine);    

    // Line 2
    lcd.setCursor(0, 1);
    lcdLine = "";
    lcdLine += cur_altimeter;
    if (lcdLine.length() < 5)
      lcdLine = "0" + lcdLine;
    lcdLine = "Alm: " + lcdLine;
    lcd.print(lcdLine);
  
    lcd.print(" Wnd: ");
    lcd.setCursor(16, 1);

    int windDirRelative = cur_heading - cur_wind_dir;
    if (windDirRelative < 0)
      windDirRelative += 360;
      
    if ((windDirRelative > 337 && windDirRelative <= 360) || (windDirRelative >= 0 && windDirRelative <= 22)) //windPointNorth
      lcd.write(0);
    if (windDirRelative > 22 && windDirRelative <= 67) //windPointNorthEast
      lcd.write(5);
    if (windDirRelative > 67 && windDirRelative <= 112) //windPointEast
      lcd.write(2);
    if (windDirRelative > 112 && windDirRelative <= 157) //windPointSouthEast
      lcd.write(6);
    if (windDirRelative > 157 && windDirRelative <= 202) //windPointSouth
      lcd.write(1);
    if (windDirRelative > 202 && windDirRelative <= 247) //windPointSouthWest
      lcd.write(7);
    if (windDirRelative > 247 && windDirRelative <= 292) //windPointWest
      lcd.write(3);
    if (windDirRelative > 292 && windDirRelative <= 337) //windPointNorthWest
      lcd.write(4);
      
    lcd.setCursor(17, 1);
    sprintf(buf, "%03d", cur_wind_vel);
    lcd.print(buf);

    // Line 3
    lcd.setCursor(0, 2);
    lcdLine = "";
    lcdLine += "Fu : ";
    sprintf(buf, "% 4d", cur_fuelp);
    lcdLine += buf;
    lcdLine += "%";
  
    lcdLine += " Flp: ";
    lcdLine += cur_flaps;
    lcdLine += "%";

    lcdLine = fillLine(lcdLine);   
    lcd.print(lcdLine);

    // Line 4
    lcd.setCursor(0, 3);
    lcdLine = "";
    lcdLine += "  ";
    lcdLine += cur_aptext;
    lcdLine += " >";

    lcdLine = fillLine(lcdLine);   
    lcd.print(lcdLine);
  }
  else if (curPage == 1)
  {
    // Line 1
    lcd.setCursor(0, 0);
    lcdLine = "";
    lcdLine += "Hdg set : ";
    sprintf(buf, "%03d", cur_heading_var);
    lcdLine += buf;

    lcdLine = fillLine(lcdLine);   
    lcd.print(lcdLine);

    // Line 2
    lcd.setCursor(0, 1);
    lcdLine = "";
    lcdLine += "Alt set : ";
    sprintf(buf, "%05d", cur_altitude_var);
    lcdLine += buf;

    lcdLine = fillLine(lcdLine);   
    lcd.print(lcdLine);

    // Line 3
    lcd.setCursor(0, 2);
    lcdLine = "";
    lcdLine += "Vs set  : ";
    sprintf(buf, "%04d", cur_vs_var);
    lcdLine += buf;

    lcdLine = fillLine(lcdLine);   
    lcd.print(lcdLine);

    // Line 4
    lcd.setCursor(0, 3);
    lcdLine = "";
    lcdLine += "<     ";
    lcdLine += version;

    lcdLine = fillLine(lcdLine);   
    lcd.print(lcdLine);
  }
}

// Function that reads button state
// Returns 1 = DOWN
//         0 = UP
uint8_t readButton(uint8_t id, uint8_t pin)
{
  uint8_t curButtonState;

  curButtonState = digitalRead(pin);
  if (curButtonState == LOW && prevButtonState[id] != curButtonState)
  {
    prevButtonState[id] = curButtonState;
    return 1;
  }
  
  prevButtonState[id] = curButtonState;
  return 0;
}

// Callback function for the buttons thread
void readButtons_cb()
{
  if (readButton(0, PIN_PREV_PAGE))
  { 
    curPage -= 1;
    if (curPage < 0)
      curPage = 0;
  }

  if (readButton(1, PIN_NEXT_PAGE))
  { 
    curPage += 1;
    if (curPage > 1)
      curPage = 1;
  }

  if (readButton(2, PIN_SY_HEADING))
    commandsQueue.push_front("sync_heading");
    
  if (readButton(3, PIN_SY_ALTITUDE))
    commandsQueue.push_front("sync_altitude");

  if (readButton(4, PIN_AUTO_PILOT))
    commandsQueue.push_front("set_autopilot_onoff");

  if (readButton(5, PIN_YAW_DAMP))
    commandsQueue.push_front("set_yawdamp_onoff");

  if (readButton(6, PIN_HEADING))
    commandsQueue.push_front("set_heading_onoff");

  if (readButton(7, PIN_APPROACH))
    commandsQueue.push_front("set_approach_onoff");

  if (readButton(8, PIN_NAV))
    commandsQueue.push_front("set_nav_onoff");

  if (readButton(9, PIN_ALTITUDE))
    commandsQueue.push_front("set_altitude_onoff");

  if (readButton(10, PIN_VS))
    commandsQueue.push_front("set_vs_onoff");
}

// Callback function for the encoders thread
void readEncoders_cb()
{
  int8_t rr = 0;
  rr = readEncoder(PIN_ROTARY1_CCK, PIN_ROTARY1_CLK, 0);
  if (rr == -1)
    commandsQueue.push_front("set_altimeter_down");
  else if (rr == 1)
    commandsQueue.push_front("set_altimeter_up");

  rr = readEncoder(PIN_ROTARY2_CCK, PIN_ROTARY2_CLK, 1);
  if (rr == -1)
    commandsQueue.push_front("set_heading_left");
  else if (rr == 1)
    commandsQueue.push_front("set_heading_right");

  rr = readEncoder(PIN_ROTARY3_CCK, PIN_ROTARY3_CLK, 2);
  if (rr == -1)
    commandsQueue.push_front("set_altitude_down");
  else if (rr == 1)
    commandsQueue.push_front("set_altitude_up");

  rr = readEncoder(PIN_ROTARY4_CCK, PIN_ROTARY4_CLK, 3);
  if (rr == -1)
    commandsQueue.push_front("set_vertical_down");
  else if (rr == 1)
    commandsQueue.push_front("set_vertical_up");
}

// Read encoder status
// Returns  0 = not turning
//         -1 = counter clockwise
//          1 = clockwise
int8_t readEncoder(uint8_t DATA, uint8_t CLK, uint8_t id) {
  static int8_t rot_enc_table[] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};

  prevEncoderNextCode[id] <<= 2;
  if (digitalRead(DATA)) prevEncoderNextCode[id] |= 0x02;
  if (digitalRead(CLK)) prevEncoderNextCode[id] |= 0x01;
  prevEncoderNextCode[id] &= 0x0f;

  // If valid then store as 16 bit data.
  if  (rot_enc_table[prevEncoderNextCode[id]] ) {
    encoderStore[id] <<= 4;
    encoderStore[id] |= prevEncoderNextCode[id];
    if ((encoderStore[id] & 0xff) == 0x2b) return -1;
    if ((encoderStore[id] & 0xff) == 0x17) return 1;
  }
  return 0;
}

// Callback function for the message processing thread
// Sends command via the Serial port to the c# companion app
void processMessagesQueue_cb()
{
  noInterrupts();
  
  while (commandsQueue.size() > 0) {
    String str = commandsQueue.back();
    commandsQueue.pop_back();
    Serial.println(str);
  }

  interrupts();
}

// Callback function for the serial (read) thread
// Reads data from the c# app via the serial port
void recvWithStartEndMarkers_cb() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  noInterrupts();
  
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }

  interrupts();
}

// Callback function for the parse data thread
// Process input data from the serial port
void parseData_cb(){  
  noInterrupts();
  
  if (newData == true)
  {    
    //Handshake
    if (getValue(receivedChars, ';', 0) == "handshakepc")
    {
      commandsQueue.push_front("handshakearduino");
    }
    else if (getValue(receivedChars, ';', 0) == "DATA")
    {   
      cur_autopilot_act = getValue(receivedChars, ';', 1).toInt();
      cur_yawdamp_act = getValue(receivedChars, ';', 2).toInt();
      cur_heading_act = getValue(receivedChars, ';', 3).toInt();
      cur_appr_act = getValue(receivedChars, ';', 4).toInt();
      cur_nav_act = getValue(receivedChars, ';', 5).toInt();
      cur_alt_act = getValue(receivedChars, ';', 6).toInt();
      cur_vs_act = getValue(receivedChars, ';', 7).toInt();
      
      cur_altitude = getValue(receivedChars, ';', 8).toInt();
      cur_heading = getValue(receivedChars, ';', 9).toInt();
      cur_altimeter = getValue(receivedChars, ';', 10).toFloat();
      cur_fuelp = getValue(receivedChars, ';', 11).toInt();
      cur_wind_dir = getValue(receivedChars, ';', 12).toInt();
      cur_wind_vel = getValue(receivedChars, ';', 13).toInt();
      cur_flaps = getValue(receivedChars, ';', 14).toInt();
      cur_heading_var = getValue(receivedChars, ';', 15).toInt();
      cur_altitude_var = getValue(receivedChars, ';', 16).toInt();
      cur_vs_var = getValue(receivedChars, ';', 17).toInt();
      cur_aptext = getValue(receivedChars, ';', 18);
    }
    else
    {
      commandsQueue.push_front("UNKNOWN MESSAGE");
    }

    newData = false;
    commandsQueue.push_front("READY");
  }

  interrupts();
}

// Callback function that changes leds status
void switchLeds_cb()
{
  // LEDS
  if (cur_autopilot_act == 1)  
    digitalWrite(PIN_LED_AP, HIGH);
  else
    digitalWrite(PIN_LED_AP, LOW);

  if (cur_yawdamp_act == 1)
    digitalWrite(PIN_LED_YAWDAMP, HIGH);
  else
    digitalWrite(PIN_LED_YAWDAMP, LOW);

  if (cur_heading_act == 1)
    digitalWrite(PIN_LED_HEAD, HIGH);
  else
    digitalWrite(PIN_LED_HEAD, LOW);

  if (cur_appr_act == 1)
    digitalWrite(PIN_LED_APPR, HIGH);
  else
    digitalWrite(PIN_LED_APPR, LOW);

  if (cur_nav_act == 1)
    digitalWrite(PIN_LED_NAV, HIGH);
  else
    digitalWrite(PIN_LED_NAV, LOW);

  if (cur_alt_act == 1)
    digitalWrite(PIN_LED_ALT, HIGH);
  else
    digitalWrite(PIN_LED_ALT, LOW);

   if (cur_vs_act == 1)
    digitalWrite(PIN_LED_VS, HIGH);
  else
    digitalWrite(PIN_LED_VS, LOW);
}

void setup(){
  // INIT LCD
  lcd.begin(20, 4);
  lcd.backlight();

  Serial.begin(9600);

  // LEDS
  pinMode(PIN_LED_AP, OUTPUT);
  pinMode(PIN_LED_YAWDAMP, OUTPUT);
  pinMode(PIN_LED_HEAD, OUTPUT);
  pinMode(PIN_LED_APPR, OUTPUT);
  pinMode(PIN_LED_NAV, OUTPUT);
  pinMode(PIN_LED_ALT, OUTPUT);
  pinMode(PIN_LED_VS, OUTPUT);

  // BUTTONS
  pinMode(PIN_AUTO_PILOT, INPUT_PULLUP);
  pinMode(PIN_YAW_DAMP, INPUT_PULLUP);
  pinMode(PIN_HEADING, INPUT_PULLUP);
  pinMode(PIN_SY_HEADING, INPUT_PULLUP);
  pinMode(PIN_APPROACH, INPUT_PULLUP);
  pinMode(PIN_NAV, INPUT_PULLUP);
  pinMode(PIN_ALTITUDE, INPUT_PULLUP);
  pinMode(PIN_SY_ALTITUDE, INPUT_PULLUP);
  pinMode(PIN_VS, INPUT_PULLUP);
  pinMode(PIN_PREV_PAGE, INPUT_PULLUP);
  pinMode(PIN_NEXT_PAGE, INPUT_PULLUP);

  //attachPCINT(digitalPinToPCINT(PIN_AUTO_PILOT), processButton, CHANGE);
  //attachPCINT(digitalPinToPCINT(PIN_YAW_DAMP), processButton, CHANGE);
  //attachPCINT(digitalPinToPCINT(PIN_HEADING), processButton, CHANGE);
  //attachPCINT(digitalPinToPCINT(PIN_APPROACH), processButton, CHANGE);
  //attachPCINT(digitalPinToPCINT(PIN_PREV_PAGE), processButton, CHANGE);
  //attachPCINT(digitalPinToPCINT(PIN_NEXT_PAGE), processButton, CHANGE);
  //attachPCINT(digitalPinToPCINT(PIN_SY_HEADING), processButton, CHANGE);
  //attachPCINT(digitalPinToPCINT(PIN_SY_ALTITUDE), processButton, CHANGE);
  
  //attachInterrupt(digitalPinToInterrupt(PIN_NAV), processButtonNav, RISING);
  //attachInterrupt(digitalPinToInterrupt(PIN_ALTITUDE), processButtonAlt, RISING);
  //attachInterrupt(digitalPinToInterrupt(PIN_VS), processButtonVs, RISING);

  // ROTARY ENCODERS
  pinMode(PIN_ROTARY1_CCK, INPUT_PULLUP);
  pinMode(PIN_ROTARY1_CLK, INPUT_PULLUP);
  pinMode(PIN_ROTARY2_CCK, INPUT_PULLUP);
  pinMode(PIN_ROTARY2_CLK, INPUT_PULLUP);
  pinMode(PIN_ROTARY3_CCK, INPUT_PULLUP);
  pinMode(PIN_ROTARY3_CLK, INPUT_PULLUP);
  pinMode(PIN_ROTARY4_CCK, INPUT_PULLUP);
  pinMode(PIN_ROTARY4_CLK, INPUT_PULLUP);

  // Register Special chars
  lcd.createChar(0, windPointNorth);
  lcd.createChar(1, windPointSouth);
  lcd.createChar(2, windPointEast);
  lcd.createChar(3, windPointWest);
  lcd.createChar(4, windPointNorthWest);
  lcd.createChar(5, windPointNorthEast);
  lcd.createChar(6, windPointSouthEast);
  lcd.createChar(7, windPointSouthWest);

  // Define threads callback functions and interval
  printLcdThread->onRun(printLcd_cb);
  printLcdThread->setInterval(250);

  readEncodersThread->onRun(readEncoders_cb);
  readEncodersThread->setInterval(1);

  readButtonssThread->onRun(readButtons_cb);
  readButtonssThread->setInterval(50);

  processMessagesQueueThread->onRun(processMessagesQueue_cb);
  processMessagesQueueThread->setInterval(5);

  recvWithStartEndMarkersThread->onRun(recvWithStartEndMarkers_cb);
  recvWithStartEndMarkersThread->setInterval(5);

  parseDataThread->onRun(parseData_cb);
  parseDataThread->setInterval(5);

  switchLedsThread->onRun(switchLeds_cb);
  switchLedsThread->setInterval(5);
}

void loop(){
  // Run due threads inside the threadController
  threadController.run();
}

// Function that fills lcd line with up to 20 chars (prevents the use of lcd.clear() which stops flickering issue since we always override the line)
String fillLine(String l)
{
  String s = l;
  while(s.length() < 20)
    s += " ";

  return s;
}

// Function that return the Nth variable from the serial stream of the c# app
// Data is separated by ;
// <DATA;1;2;3;4;END;>
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {

    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
