#include <SPI.h>
#include <mcp_can.h>

// Define which digital pins are for left and right side
const byte right_pin = 6;
const byte left_pin = 5;
static bool left_is_on = false;
static bool right_is_on = false;

const short SW_ANGLE_THRESHOLD = 40; // Bending light ON if SW_ANGLE > this

// PWM fade related vars.
// Looks ugly, I know.
static int onDelayLength = 200;
static int offDelayLength = 1000;

static byte rightBrightness = 0;
static byte leftBrightness = 0;

static unsigned long rightOffDelayStart = 0;
static unsigned long rightOffPassedTime = 0;
static bool rightOffDelayRunning = false;

static unsigned long rightOnDelayStart = 0;
static unsigned long rightOnPassedTime = 0;
static bool rightOnDelayRunning = false;


static unsigned long leftOffDelayStart = 0;
static unsigned long leftOffPassedTime = 0;
static bool leftOffDelayRunning = false;

static unsigned long leftOnDelayStart = 0;
static unsigned long leftOnPassedTime = 0;
static bool leftOnDelayRunning = false;

// Declare CAN as MCP_CAN
const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN); // Set CS pin used for CAN bus shield


static byte tmp_byte;           // temporary variable for bit grab
static short swAngleMessage;    // where to assemble Steering Wheel angle from 14-bit message
static short SW_ANGLE;          // final steering wheel angle value   

static short vssMessage;    // where to assemble Vehicle Speed from 16-bit message
static short VSS;           // final Vehicle Speed value

static bool WHEELS_DIR; // Where wheels are pointed to, 0=left, 1=right
static bool REVERSE = false; // Reverse gear engaged
static bool LIGHTS = false; // Low beam headlights are on

// enable LCD (for debugging)
#define _LCD_TYPE 1
#include <LCD_1602_RUS_ALL.h>
LCD_1602_RUS <LiquidCrystal_I2C> lcd(0x27, 16, 2);
boolean enableLCD = false;

// PWM fade-in functions

// direction-specific functions
void left_on() {
    byte my_pin = left_pin;
    if (!left_is_on){
        if (!leftOnDelayRunning) {
            leftOnDelayStart = millis(); // start delay
            leftOnDelayRunning = true;
            leftBrightness = 0;
        }
        leftOnPassedTime = millis() - leftOnDelayStart;

        leftBrightness = (leftOnPassedTime)/(round(onDelayLength/255.0)+1);
        analogWrite(my_pin, leftBrightness);

        if (leftOnDelayRunning && ( leftOnPassedTime >= onDelayLength)) {
            digitalWrite(my_pin, HIGH);
            left_is_on = true;
            leftOnDelayRunning = false;
            leftOnPassedTime = 0;
        }
    }
}

void right_on() {
    byte my_pin = right_pin;
    if (!right_is_on){
        if (!rightOnDelayRunning) {
            rightOnDelayStart = millis(); // start delay
            rightOnDelayRunning = true;
            rightBrightness = 0;
        }
        rightOnPassedTime = millis() - rightOnDelayStart;

        rightBrightness = (rightOnPassedTime)/(round(onDelayLength/255.0)+1);
        analogWrite(my_pin, rightBrightness);

        if (rightOnDelayRunning && ( rightOnPassedTime >= onDelayLength)) {
            digitalWrite(my_pin, HIGH);
            right_is_on = true;
            rightOnDelayRunning = false;
            rightOnPassedTime = 0;
        }
    }
}

// PWM fade-out functions

// direction-specific functions
void left_off() {
    byte my_pin = left_pin;
    if (left_is_on){
        if (!leftOffDelayRunning) {
            leftOffDelayStart = millis(); // start delay
            leftOffDelayRunning = true;
            leftBrightness = 255;
        }
        leftOffPassedTime = millis() - leftOffDelayStart;

        leftBrightness = (offDelayLength - leftOffPassedTime)/round(offDelayLength/255.0);
        analogWrite(my_pin, leftBrightness);

        if (leftOffDelayRunning && ( leftOffPassedTime >= offDelayLength)) {
            digitalWrite(my_pin, LOW);
            left_is_on = false;
            leftOffDelayRunning = false;
            leftOffPassedTime = 0;
        }
    }
}

void right_off() {
    byte my_pin = right_pin;
    if (right_is_on){
        if (!rightOffDelayRunning) {
            rightOffDelayStart = millis(); // start delay
            rightOffDelayRunning = true;
            rightBrightness = 255;
        }
        rightOffPassedTime = millis() - rightOffDelayStart;

        rightBrightness = (offDelayLength - rightOffPassedTime)/round(offDelayLength/255.0);
        analogWrite(my_pin, rightBrightness);

        if (rightOffDelayRunning && ( rightOffPassedTime >= offDelayLength)) {
            digitalWrite(my_pin, LOW);
            right_is_on = false;
            rightOffDelayRunning = false;
            rightOffPassedTime = 0;
        }
    }
}


// the setup function runs once when you press reset or power the board
void setup() {
    // initialize pins as an output.
    pinMode(right_pin, OUTPUT);
    pinMode(left_pin, OUTPUT);
    // immediately send LOW to those pins so that lights don't light up by mistake
    digitalWrite(right_pin, LOW);
    digitalWrite(left_pin, LOW);

    if(enableLCD){
        // LCD display will be used for debugging
        lcd.init();
        lcd.backlight();
    }

    while (CAN_OK != CAN.begin(CAN_125KBPS, MCP_8MHz)) // init CAN bus, make sure you select correct bus speed -- 125 kbit/s for MS-CAN
    {
        if(enableLCD){ lcd.setCursor(0, 0); lcd.print("CAN init FAILED"); }
    }
    if (enableLCD){ lcd.setCursor(0, 0); lcd.print("CAN init OK"); lcd.setCursor(0, 1); lcd.print(String(round(onDelayLength/255.0)+1));lcd.print("|"); lcd.print(String(round(offDelayLength/255.0))); }

    /*  Now I'll try to init CAN message masks and filters
     *  INFO: https://copperhilltech.com/content/MCP2515.pdf
     *  CAN message ID is 11 bits long, full 11-bit mask is 0x7FF.
     *  Note: HS-CAN ID 0x076 is 00001110110
     */

    /*  The following masks are for MS CAN messages with ID 433 and 480 and 0x08b.
     *  The mask tells the controller which specific bits to check,
     *  and the filter tells it how those bits should be set (or unset).
     *  See https://forum.arduino.cc/index.php?topic=156069.0 for explanation.
     *
     *  In this particular case, I want to configure Mask to include both
     *  ID 0x433 and ID 0x480 and ID 0x08b. I look at their binary form:
     *  0x433:  10000110011
     *  0x480:  10010000000
     *  0x08b:  00010001011
     *  so the resulting mask to include these all bits of three CAN IDs will be:
     *          10010111011 == 0x4BB
     *
     *  Then, I should set up Filters, so that the Mask + Filter combo
     *  becomes more precise.
     */
    // Two receiver buffers in MCP2515 have acceptance masks and filters assigned to each.
    // RX buffer 0 (higher priority). Let's grab Steering Wheel angle in it.
    CAN.init_Mask(0, false, 0x480);
    CAN.init_Filt(0, false, 0x480);
    CAN.init_Filt(1, false, 0x480);
    // RX buffer 1 (lower priority). Low beam status, Reverse gear status, and Vehicle speed go here.
    CAN.init_Mask(1, false, 0x4BB);
    CAN.init_Filt(2, false, 0x08b);
    CAN.init_Filt(3, false, 0x433);
    CAN.init_Filt(4, false, 0x433);
    CAN.init_Filt(5, false, 0x08b);
}

void loop() {
    // receive CAN messages
    unsigned char len = 0;
    unsigned char buf[8];
    if(CAN_MSGAVAIL == CAN.checkReceive())  // check if data coming
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        //unsigned long canId = CAN.getCanId();
        unsigned int canId = CAN.getCanId();

        // switch..case should be faster than if..else (not sure).
        switch (canId) {
            case 0x480:
                WHEELS_DIR = buf[6] & 0x80;
                tmp_byte = buf[6] & 0x7F;
                swAngleMessage = word(tmp_byte,buf[7]);
                SW_ANGLE = swAngleMessage * 0.04395;
                break;
            case 0x433:
                LIGHTS = buf[3] & 0x80;
                REVERSE = buf[3] & 0x2;
                break;
            case 0x08b:
                vssMessage = word(buf[1],buf[2]);
                VSS = vssMessage * 0.01072; // Value should be divided by 100, that's why *0.01. Also added small correction factor.
                break;
        }
        
        if(enableLCD){ 
            // print angle
            lcd.setCursor(0, 0);
            lcd.print(String(SW_ANGLE));
            lcd.print("°");
            if (SW_ANGLE < 10){
                lcd.print("  ");
            } else if (SW_ANGLE < 100){
                lcd.print(" ");
            }
            
            lcd.print("|VSS:");
            lcd.print(String(VSS));
            lcd.print("                ");

            // print wheels direction
            lcd.setCursor(0, 1);
            lcd.print("WH:");
            if (WHEELS_DIR == true){
                lcd.print("R");
            } else if (WHEELS_DIR == false) {
                lcd.print("L");
            }            
            if (LIGHTS){
                lcd.print("|LB"); // "Low Beam"
            }
            if (REVERSE){
                lcd.print("|REV"); // Reverse gear engaged
            }
            lcd.print("                ");
        }

        // Let's light up the lamps!
        if (LIGHTS && VSS < 80 ){
            if (int(SW_ANGLE) >= int(SW_ANGLE_THRESHOLD)){
                if (WHEELS_DIR == true){ // wheels are turned right
                    if (!REVERSE) {
                        // TODO: insert small delay here to prevent flashing lights
                        // when rapidly switching from N to P (through R).
                        right_on();
                        left_off();
                    } else if (REVERSE){
                        // or should this small delay ^
                        // go here (too)?
                        right_off();
                        left_on();
                    }
                } else if (WHEELS_DIR == false){ // wheels are turned left
                    if (!REVERSE) {
                        // TODO: insert small delay here to prevent flashing lights
                        // when rapidly switching from N to P (through R).
                        right_off();
                        left_on();
                    } else if (REVERSE){
                        right_on();
                        left_off();
                    }
                }
            } else if (int(SW_ANGLE) < int(SW_ANGLE_THRESHOLD)){ // steering wheel angle lower than threshold
                right_off();
                left_off();
            }
        }
        else { // if Low Beam is off then disable bending lights
            right_off();
            left_off();
        }
/*
        if (!LIGHTS && (SW_ANGLE < SW_ANGLE_THRESHOLD)){ // second time for failsafe operation?
            right_off();
            left_off();
        }
*/
    } // end CAN receive

}
