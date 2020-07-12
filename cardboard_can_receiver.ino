#include <SPI.h>
#include <mcp_can.h>

// Define which digital pins are for left and right side
const byte right_pin = 6;
const byte left_pin = 5;
static bool left_is_on = false;
static bool right_is_on = false;

const short SW_ANGLE_THRESHOLD = 33; // Bending light ON if SW_ANGLE > this


// Declare CAN as MCP_CAN
const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN); // Set CS pin used for CAN bus shield


static byte tmp_byte;           // temporary variable for bit grab
static short swAngleMessage;    // where to assemble Steering Wheel angle from 14-bit message
static short SW_ANGLE;          // final steering wheel angle value   

static short vssMessage;    // where to assemble Vehicle Speed from 16-bit message
static short VSS;           // final Vehicle Speed value

/*
// For calculate previous steering wheel angle, unused for now.
static short SW_ANGLE_PREVIOUS;
short ANGLE_DIFF;
static short ANGLE_DIFF_ABS;
*/
//static bool SW_ROTATION; // which side the steering wheel is being rotated to. Available on HS-CAN.

static bool WHEELS_DIR; // Where wheels are pointed to, 0=left, 1=right
static bool REVERSE = false; // Reverse gear engaged
static bool LIGHTS = false; // Low beam headlights are on

// enable LCD (for debugging)
#define _LCD_TYPE 1
#include <LCD_1602_RUS_ALL.h>
LCD_1602_RUS <LiquidCrystal_I2C> lcd(0x27, 16, 2);
boolean enableLCD = true;

// PWM fade-in functions
// generic fade-in function for code reuse
void pwm_on(byte pin){
    for (int i = 0; i <= 255; i=i+1) {
        analogWrite(pin, i);
        //delay(10);
        // classic blocking delay was here
    }

    // make sure it is fully on in the end
    digitalWrite(pin, HIGH);
}

// direction-specific functions
void left_on() {
    byte my_pin = left_pin;
    if (!left_is_on){
        pwm_on(my_pin);
        left_is_on = true;
    }
}

void right_on() {
    byte my_pin = right_pin;
    if (!right_is_on){
        pwm_on(my_pin);
        right_is_on = true;
    }
}

// PWM fade-out functions
// generic fade-out function for code reuse
void pwm_off(byte pin){
    /*
    for (int i = 255; i >= 0; i=i-1) {
        analogWrite(pin, i);
        delay(3);
    }
    */

    // make sure it is fully off in the end
    digitalWrite(pin, LOW);
}

// direction-specific functions
void left_off() {
    byte my_pin = left_pin;
    if (left_is_on){
        pwm_off(my_pin);
        left_is_on = false;
    }
}

void right_off() {
    byte my_pin = right_pin;
    if (right_is_on){
        pwm_off(my_pin);
        right_is_on = false;
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

    while (CAN_OK != CAN.begin(CAN_125KBPS, MCP_8MHz))              // init can bus, make sure you select correct bus speed
    {
        if(enableLCD){ lcd.setCursor(0, 0); lcd.print("CAN init FAILED"); }
    }
    if (enableLCD){ lcd.setCursor(0, 0); lcd.print("CAN init OK"); }

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
     *  ID 0x433 and ID 0x480. I look at their binary form:
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
        if (LIGHTS){
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

        if (!LIGHTS && (SW_ANGLE < SW_ANGLE_THRESHOLD)){ // second time for failsafe operation?
            right_off();
            left_off();
        }
        
    } // end CAN receive

}
