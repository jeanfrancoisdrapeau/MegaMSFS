#include <LiquidCrystal_I2C.h>
#include "PinChangeInterrupt.h"
#include <QList.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

QList<String> commandsQueue;

#define PIN_AUTO_PILOT  50
#define PIN_YAW_DAMP    51
#define PIN_HEADING     52
#define PIN_APPROACH    53
#define PIN_NAV         3
#define PIN_ALTITUDE    18
#define PIN_VS          2
#define PIN_ROTARY1_CCK 30
#define PIN_ROTARY1_CLK 31
#define PIN_ROTARY2_CCK 32
#define PIN_ROTARY2_CLK 33
#define PIN_ROTARY3_CCK 34
#define PIN_ROTARY3_CLK 35
#define PIN_ROTARY4_CCK 36
#define PIN_ROTARY4_CLK 37

#define PIN_PREV_PAGE   62 //A8
#define PIN_NEXT_PAGE   63 //A9

String emptyLcdLine = "                    ";

uint8_t prevEncoderNextCode[4] = {0, 0, 0, 0};
uint16_t encoderStore[4] = {0, 0, 0, 0};

int curPage = 0;

const byte numChars = 80;
char receivedChars[numChars];
boolean newData = false;

int cur_autopilot = 0;
int cur_yawdamp = 0;
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

long debouncing_time = 100; //Debouncing Time in Milliseconds
volatile unsigned long lastMicrosButtons;
volatile unsigned long lastMicrosEncoders;
volatile unsigned long lastMicrosPrintLcd;

void setup() {
  //INIT LCD
  lcd.init();
  lcd.backlight();

  Serial.begin(9600);

  //LEDS
  pinMode(12, OUTPUT); //AUTOPILOT
  pinMode(11, OUTPUT); //YAW DAMPENER

  //BUTTONS
  pinMode(PIN_AUTO_PILOT, INPUT_PULLUP);
  pinMode(PIN_YAW_DAMP, INPUT_PULLUP);
  pinMode(PIN_HEADING, INPUT_PULLUP);
  pinMode(PIN_APPROACH, INPUT_PULLUP);
  pinMode(PIN_NAV, INPUT_PULLUP);
  pinMode(PIN_ALTITUDE, INPUT_PULLUP);
  pinMode(PIN_VS, INPUT_PULLUP);
  pinMode(PIN_PREV_PAGE, INPUT_PULLUP);
  pinMode(PIN_NEXT_PAGE, INPUT_PULLUP);

  attachPCINT(digitalPinToPCINT(PIN_AUTO_PILOT), processButton, CHANGE);
  attachPCINT(digitalPinToPCINT(PIN_YAW_DAMP), processButton, CHANGE);
  attachPCINT(digitalPinToPCINT(PIN_HEADING), processButton, CHANGE);
  attachPCINT(digitalPinToPCINT(PIN_APPROACH), processButton, CHANGE);
  attachPCINT(digitalPinToPCINT(PIN_PREV_PAGE), processButton, CHANGE);
  attachPCINT(digitalPinToPCINT(PIN_NEXT_PAGE), processButton, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_NAV), processButtonNav, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ALTITUDE), processButtonAlt, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_VS), processButtonVs, RISING);

  //ROTARY ENCODERS
  pinMode(PIN_ROTARY1_CCK, INPUT_PULLUP);
  pinMode(PIN_ROTARY1_CLK, INPUT_PULLUP);
  pinMode(PIN_ROTARY2_CCK, INPUT_PULLUP);
  pinMode(PIN_ROTARY2_CLK, INPUT_PULLUP);
  pinMode(PIN_ROTARY3_CCK, INPUT_PULLUP);
  pinMode(PIN_ROTARY3_CLK, INPUT_PULLUP);
  pinMode(PIN_ROTARY4_CCK, INPUT_PULLUP);
  pinMode(PIN_ROTARY4_CLK, INPUT_PULLUP);
}

void loop() {

  // RECEIVE
  recvWithStartEndMarkers();

  // PARSE DATA
  if (newData == true)
  {    
    //Handshake
    if (getValue(receivedChars, ';', 0) == "handshakepc")
    {
      commandsQueue.push_front("handshakearduino");
    }
    else if (getValue(receivedChars, ';', 0) == "DATA")
    {   
      cur_autopilot = getValue(receivedChars, ';', 1).toInt();
      cur_yawdamp = getValue(receivedChars, ';', 2).toInt();
      cur_altitude = getValue(receivedChars, ';', 3).toInt();
      cur_heading = getValue(receivedChars, ';', 4).toInt();
      cur_altimeter = getValue(receivedChars, ';', 5).toFloat();
      cur_fuelp = getValue(receivedChars, ';', 6).toInt();
      cur_wind_dir = getValue(receivedChars, ';', 7).toInt();
      cur_wind_vel = getValue(receivedChars, ';', 8).toInt();
      cur_flaps = getValue(receivedChars, ';', 9).toInt();
      cur_heading_var = getValue(receivedChars, ';', 10).toInt();
      cur_altitude_var = getValue(receivedChars, ';', 11).toInt();
      cur_vs_var = getValue(receivedChars, ';', 12).toInt();
      cur_aptext = getValue(receivedChars, ';', 13);
    }
    else
    {
      commandsQueue.push_front("UNKNOWN MESSAGE");
    }

    newData = false;
  }

  //ENCODERS
  int8_t rr = 0;
  rr = readRotary(PIN_ROTARY1_CCK, PIN_ROTARY1_CLK, 0);
  if (rr == -1)
    commandsQueue.push_front("set_altimeter_down");
  else if (rr == 1)
    commandsQueue.push_front("set_altimeter_up");

  rr = readRotary(PIN_ROTARY2_CCK, PIN_ROTARY2_CLK, 1);
  if (rr == -1)
    commandsQueue.push_front("set_heading_left");
  else if (rr == 1)
    commandsQueue.push_front("set_heading_right");

  rr = readRotary(PIN_ROTARY3_CCK, PIN_ROTARY3_CLK, 2);
  if (rr == -1)
    commandsQueue.push_front("set_altitude_down");
  else if (rr == 1)
    commandsQueue.push_front("set_altitude_up");

  rr = readRotary(PIN_ROTARY4_CCK, PIN_ROTARY4_CLK, 3);
  if (rr == -1)
    commandsQueue.push_front("set_vertical_down");
  else if (rr == 1)
    commandsQueue.push_front("set_vertical_up");

  //PROCESS MESSAGES QUEUE
  while (commandsQueue.size() > 0) {
    String str = commandsQueue.back();
    commandsQueue.pop_back();
    Serial.println(str);
  }

  // LEDS
  if (cur_autopilot == 1)
    digitalWrite(12, HIGH);
  else
    digitalWrite(12, LOW);

  if (cur_yawdamp == 1)
    digitalWrite(11, HIGH);
  else
    digitalWrite(11, LOW);

  if ((long)(micros() - lastMicrosPrintLcd) >= 500000) {
    print_lcd();
    commandsQueue.push_front("MESSAGE RECEIVED");
    lastMicrosPrintLcd = micros();
  }
}

void processButtonNav()
{
  if ((long)(micros() - lastMicrosButtons) >= debouncing_time * 1000) {
    commandsQueue.push_front("set_nav_onoff");
    lastMicrosButtons = micros();
  }
}

void processButtonAlt()
{
  if ((long)(micros() - lastMicrosButtons) >= debouncing_time * 1000) {
    commandsQueue.push_front("set_altitude_onoff");
    lastMicrosButtons = micros();
  }
}

void processButtonVs()
{
  if ((long)(micros() - lastMicrosButtons) >= debouncing_time * 1000) {
    commandsQueue.push_front("set_vs_onoff");
    lastMicrosButtons = micros();
  }
}

void processButton()
{
  if ((long)(micros() - lastMicrosButtons) >= debouncing_time * 1000) {
    uint8_t trigger1 = getPinChangeInterruptTrigger(digitalPinToPCINT(PIN_AUTO_PILOT));
    if (trigger1 == FALLING)
      commandsQueue.push_front("set_autopilot_onoff");

    uint8_t trigger2 = getPinChangeInterruptTrigger(digitalPinToPCINT(PIN_YAW_DAMP));
    if (trigger2 == FALLING)
      commandsQueue.push_front("set_yawdamp_onoff");

    uint8_t trigger3 = getPinChangeInterruptTrigger(digitalPinToPCINT(PIN_HEADING));
    if (trigger3 == FALLING)
      commandsQueue.push_front("set_heading_onoff");

    uint8_t trigger4 = getPinChangeInterruptTrigger(digitalPinToPCINT(PIN_APPROACH));
    if (trigger4 == FALLING)
      commandsQueue.push_front("set_approach_onoff");

    uint8_t trigger5 = getPinChangeInterruptTrigger(digitalPinToPCINT(PIN_PREV_PAGE));
    if (trigger5 == FALLING)
    {
      curPage -= 1;
      if (curPage < 0)
        curPage = 0;
      lastMicrosPrintLcd = 0;
    }
    
    uint8_t trigger6 = getPinChangeInterruptTrigger(digitalPinToPCINT(PIN_NEXT_PAGE));
    if (trigger6 == FALLING)
    {
      curPage += 1;
      if (curPage > 1)
        curPage = 1;
      lastMicrosPrintLcd = 0;
    }

    lastMicrosButtons = micros();
  }
}

int8_t readRotary(uint8_t DATA, uint8_t CLK, uint8_t id) {
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

void print_lcd()
{
  String lcdLine = "";
  char buf[10];
    
  if (curPage == 0)
  {
    //Output to LCD
    //lcd.clear();
  
    lcd.setCursor(0, 0);
    lcdLine = "Alt: ";
    sprintf(buf, "%05d", cur_altitude);
    lcdLine += buf;
  
    lcdLine += " Hdg:  ";
    sprintf(buf, "%03d", cur_heading);
    lcdLine += buf;
    printLcdLine(lcdLine);
  
    lcdLine = "";
    lcd.setCursor(0, 1);
    lcdLine += cur_altimeter;
    if (lcdLine.length() < 5)
      lcdLine = "0" + lcdLine;
    lcdLine = "Alm: " + lcdLine;
  
    lcdLine += " W: ";
    sprintf(buf, "%03d", cur_wind_dir);
    lcdLine += buf;
    lcdLine += "d";
    sprintf(buf, "%02d", cur_wind_vel);
    lcdLine += buf;
    
    printLcdLine(lcdLine);
  
    lcdLine = "";
    lcd.setCursor(0, 2);
    lcdLine += "Fu : ";
    sprintf(buf, "% 4d", cur_fuelp);
    lcdLine += buf;
    lcdLine += "%";
  
    lcdLine += " Flp: ";
    lcdLine += cur_flaps;
    lcdLine += "%";
    
    printLcdLine(lcdLine);
  
    lcdLine = "";
    lcd.setCursor(0, 3);

    lcdLine += "  ";
    lcdLine += cur_aptext;
    lcdLine += " >";
    
    printLcdLine(lcdLine);
  }
  else if (curPage == 1)
  {
    lcdLine = "";
    lcd.setCursor(0, 0);
    lcdLine += "Hdg set : ";
    sprintf(buf, "%03d", cur_heading_var);
    lcdLine += buf;

    printLcdLine(lcdLine);

    lcdLine = "";
    lcd.setCursor(0, 1);
    lcdLine += "Alt set : ";
    sprintf(buf, "%05d", cur_altitude_var);
    lcdLine += buf;

    printLcdLine(lcdLine);

    lcdLine = "";
    lcd.setCursor(0, 2);
    lcdLine += "Vs set  : ";
    sprintf(buf, "%04d", cur_vs_var);
    lcdLine += buf;

    printLcdLine(lcdLine);
    
    lcdLine = "<";
    lcd.setCursor(0, 3);
    
    printLcdLine(lcdLine);
  }
}

void printLcdLine(String s)
{
  String lcdLine = "";
  lcdLine = s;

  while (lcdLine.length() < 20)
  {
    lcdLine += " ";
  }

  lcd.print(lcdLine);
}

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

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

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
}
