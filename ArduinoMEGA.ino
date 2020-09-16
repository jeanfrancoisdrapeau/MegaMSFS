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
#define PIN_ROTARY1_CCK 62 //A8
#define PIN_ROTARY1_CLK 63 //A9
#define PIN_ROTARY2_CCK 64 //A10
#define PIN_ROTARY2_CLK 65 //A11
#define PIN_ROTARY3_CCK 66 //A12
#define PIN_ROTARY3_CLK 67 //A13
#define PIN_ROTARY4_CCK 68 //A14
#define PIN_ROTARY4_CLK 69 //A15

String emptyLcdLine = "                    ";

uint8_t prevEncoderNextCode[4] = {0, 0, 0, 0};
uint16_t encoderStore[4] = {0, 0, 0, 0};

const byte numChars = 100;
char receivedChars[numChars];
boolean newData = false;

int cur_autopilot = 0;
int cur_yawdamp = 0;
int cur_altitude = 0;
int cur_heading = 0;
float cur_altimeter = 0;
int cur_fuelp = 0;
volatile boolean autopilot = false;
String cur_aptext = "";

long debouncing_time = 100; //Debouncing Time in Milliseconds
volatile unsigned long lastMicrosButtons;
volatile unsigned long lastMicrosEncoders;

void setup() {
  //INIT LCD
  lcd.init();
  lcd.backlight();

  Serial.begin(9600);

  lcd.setCursor(3, 0);
  lcd.print("-- MegaMSFS --");
  lcd.setCursor(0, 1);
  lcd.print("Looking for host...");

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
  
  attachPCINT(digitalPinToPCINT(PIN_AUTO_PILOT), processButton, CHANGE);
  attachPCINT(digitalPinToPCINT(PIN_YAW_DAMP), processButton, CHANGE);
  attachPCINT(digitalPinToPCINT(PIN_HEADING), processButton, CHANGE);
  attachPCINT(digitalPinToPCINT(PIN_APPROACH), processButton, CHANGE);
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

void processButtonNav()
{
  if((long)(micros() - lastMicrosButtons) >= debouncing_time * 1000) {
    commandsQueue.push_front("set_nav_onoff");
    lastMicrosButtons = micros();
  }
}

void processButtonAlt()
{
  if((long)(micros() - lastMicrosButtons) >= debouncing_time * 1000) {
    commandsQueue.push_front("set_altitude_onoff");
    lastMicrosButtons = micros();
  }
}

void processButtonVs()
{
  if((long)(micros() - lastMicrosButtons) >= debouncing_time * 1000) {
    commandsQueue.push_front("set_vs_onoff");
    lastMicrosButtons = micros();
  }
}

void processButton()
{
  if((long)(micros() - lastMicrosButtons) >= debouncing_time * 1000) {
    uint8_t trigger1 = getPinChangeInterruptTrigger(digitalPinToPCINT(PIN_AUTO_PILOT));
    if(trigger1 == FALLING)
      commandsQueue.push_front("set_autopilot_onoff");

    uint8_t trigger2 = getPinChangeInterruptTrigger(digitalPinToPCINT(PIN_YAW_DAMP));
    if(trigger2 == FALLING)
      commandsQueue.push_front("set_yawdamp_onoff");

    uint8_t trigger3 = getPinChangeInterruptTrigger(digitalPinToPCINT(PIN_HEADING));
    if(trigger3 == FALLING)
      commandsQueue.push_front("set_heading_onoff");

    uint8_t trigger4 = getPinChangeInterruptTrigger(digitalPinToPCINT(PIN_APPROACH));
    if(trigger4 == FALLING)
      commandsQueue.push_front("set_approach_onoff");
      
    lastMicrosButtons = micros();
  }  
}

int8_t readRotary(uint8_t DATA, uint8_t CLK, uint8_t id) {
  static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  prevEncoderNextCode[id] <<= 2;
  if (digitalRead(DATA)) prevEncoderNextCode[id] |= 0x02;
  if (digitalRead(CLK)) prevEncoderNextCode[id] |= 0x01;
  prevEncoderNextCode[id] &= 0x0f;

   // If valid then store as 16 bit data.
   if  (rot_enc_table[prevEncoderNextCode[id]] ) {
      encoderStore[id] <<= 4;
      encoderStore[id] |= prevEncoderNextCode[id];
      if ((encoderStore[id]&0xff)==0x2b) return -1;
      if ((encoderStore[id]&0xff)==0x17) return 1;
   }
   return 0;
}

void print_lcd()
{
  //Output to LCD
  //lcd.clear();
  String lcdLine = "";
  
  lcd.setCursor(0, 0);
  lcdLine = "Alt : ";
  if (cur_altitude > 9999)
  {
    lcdLine += "FL";
    lcdLine += String(cur_altitude).substring(0, 3);
  }
  else
  {
    lcdLine += cur_altitude;
    lcdLine += " ";
  }
  lcdLine += " Hdg: ";
  lcdLine += cur_heading;
  printLcdLine(lcdLine);

  lcdLine = "";
  lcd.setCursor(0, 1);
  lcdLine += "inHg: ";
  lcdLine += cur_altimeter;
  printLcdLine(lcdLine);

  lcdLine = "";
  lcd.setCursor(0, 2);
  lcdLine += "Fuel: ";
  lcdLine += cur_fuelp;
  lcdLine += "%";
  printLcdLine(lcdLine);

  lcdLine = "";
  lcd.setCursor(0, 3);
  lcdLine += cur_aptext;
  printLcdLine(lcdLine);
}

void printLcdLine(String s)
{
  String lcdLine = "";
  lcdLine = s;

  while(lcdLine.length() < 20)
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
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void loop() {

  // RECEIVE
  recvWithStartEndMarkers();

  // PARSE DATA
  if (newData == true)
  {    
    //Handshake
    if (receivedChars == "handshake_pc") {
      Serial.print("handshake_arduino");
      Serial.println();

      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("-- MegaMSFS --");
      lcd.setCursor(0, 1);
      lcd.print("Found host!");
    }
    else
    {
      cur_autopilot = getValue(receivedChars, ';', 1).toInt();
      cur_yawdamp = getValue(receivedChars, ';', 2).toInt();
      cur_altitude = getValue(receivedChars, ';', 3).toInt();
      cur_heading = getValue(receivedChars, ';', 4).toInt();
      cur_altimeter = getValue(receivedChars, ';', 5).toFloat();
      cur_fuelp = getValue(receivedChars, ';', 6).toFloat();
      cur_aptext = getValue(receivedChars, ';', 7);
    
      print_lcd();

      Serial.print("MESSAGE RECEIVED");
      Serial.println();
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
