#include <SPI.h>
#include <mcp_can.h>

// Define which digital pins are for left and right side
const byte right_pin = 6;
const byte left_pin = 5;
short SW_ANGLE_THRESHOLD = 40; // light ON if SW_ANGLE > this

unsigned long counter = 0;  // for debugging CAN masks and filters

static bool left_is_on = false;
static bool right_is_on = false;

// Declare CAN as MCP_CAN
const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN); // Set CS pin used for CAN bus shield

// enable LCD (for debugging)
#define _LCD_TYPE 1
#include <LCD_1602_RUS_ALL.h>
LCD_1602_RUS <LiquidCrystal_I2C> lcd(0x27, 16, 2);
boolean enableLCD = true;

static byte tmp_byte;   // temporary variable for bit grab
static short swAngleMessage; // where to assemble Steering Wheel angle from 14-bit message
static short SW_ANGLE; // final steering wheel angle value   

/*
// For calculate previous steering wheel angle, unused for now.
static short SW_ANGLE_PREVIOUS;
short ANGLE_DIFF;
static short ANGLE_DIFF_ABS;
*/
//static bool SW_ROTATION; // which side the steering wheel is rotated to. Available on HS-CAN.

static bool WHEELS_DIR;
static bool REVERSE = false; // Reverse gear engaged
static bool LIGHTS = false; // Low beam headlights are on

/*
// temporary values used to assemble Reverse Gear boolean if using HS-CAN.
static bool REV_D2;
static bool REV_D3;
*/

// PWM fade-in functions
// generic fade-in function for code reuse
void pwm_on(byte pin){
    /*
    for (int i = 0; i <= 255; i=i+5) {
        analogWrite(pin, i);
        delay(10);
    }
    */
    // make sure it is fully on in the end
    //analogWrite(pin, 255);
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
    //analogWrite(pin, 0);
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
    // initialize digital pins as an output.
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

    /* Now I'll try to init CAN message masks and filters
     *  CAN message ID is 11 bits long, full 11-bit mask is 0x7FF.
     *  Note: HS-CAN ID 0x076 is 00001110110
     */

    /*  The following masks are for MS CAN messages with ID 433 and 480.
     *  The mask tells the controller which specific bits to check,
     *  and the filter tells it how those bits should be set (or unset).
     *  See https://forum.arduino.cc/index.php?topic=156069.0 for explanation.
     */

    CAN.init_Mask(0, false, 0x4B3);
    CAN.init_Mask(1, false, 0x4B3);

    CAN.init_Filt(0, false, 0x433);
    CAN.init_Filt(1, false, 0x480);
    CAN.init_Filt(2, false, 0x433);
    CAN.init_Filt(3, false, 0x480);
    CAN.init_Filt(4, false, 0x433);
    CAN.init_Filt(5, false, 0x480);
}

void loop() {
    // try to receive CAN messages and show them in Serial port
    unsigned char len = 0;
    unsigned char buf[8];

    
    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        unsigned long canId = CAN.getCanId();
        counter++;
        if (canId == 0x480){ // MS-CAN message which contains steering wheel angle
            WHEELS_DIR = buf[6] & 0x80;
            tmp_byte = buf[6] & 0x7F;
            swAngleMessage = word(tmp_byte,buf[7]);
            SW_ANGLE = swAngleMessage * 0.04395;
        } else if (canId == 0x433){ // this MS-CAN message contains Reverse gear and Lights on/off
            LIGHTS = buf[3] & 0x80;
            REVERSE = buf[3] & 0x2;
        }
/*        
        if (canId == 0x076){ // look for CAN messages from SASM module
            // get the direction where wheels are pointing at, left=0 right=1
            WHEELS_DIR = buf[0] & 0x40; // grab bit 1 from byte 00

            // get rotation direction of the steering wheel, left=0 right=1
            SW_ROTATION = buf[2] & 0x80; // grab bit 0 from byte 02

            // get the steering wheel angle
            /*  I must cut off first two bits of 6th (7th if counting from 1) byte of message with CAN ID 0x076
                in order to get 14-bit value of steering angle from bytes 6 and 7.
                That is why there is this mask 0x3f, it is 00111111 in binary,
                ones means "copy this bit", zeroes mean "ignore" */
/*            tmp_byte = buf[6] & 0x3F;
            swAngleMessage = word(tmp_byte,buf[7]);
            SW_ANGLE = swAngleMessage * 0.04395;

            /*  Try to at least see previous angle value
             *  probably can be used to smooth value changes */
/*            ANGLE_DIFF = SW_ANGLE - SW_ANGLE_PREVIOUS;
            ANGLE_DIFF_ABS = abs(ANGLE_DIFF);

        }   // end filtering for 0x076 message
        
        if (canId == 0x1BE){ // this must be TCM module
            /*  CD 58 5E 34 83 FF 00 00 means R is engaged
             *  but most of the bits do not change
             */
/*            REV_D2 = buf[2] & 0x2; // grab bit 6 from byte 02
            REV_D3 = buf[3] & 0x4; // grab bit 5 from byte 03
            if (REV_D2 && REV_D3){
                // if stars align then we are in reverse gear
                REVERSE = true;
            }
            else {
                REVERSE = false;
            }
        }
*/
        if(enableLCD){ 
            // print angle
            lcd.setCursor(0, 0);
            lcd.print(String(SW_ANGLE));
            lcd.print("°");
            /*lcd.print(" P:");
            lcd.print(String(SW_ANGLE_PREVIOUS));
            lcd.print("° D:");
            lcd.print(String(ANGLE_DIFF_ABS));
            */
            lcd.print("|C:"); // aka "Counter", this is for debugging CAN masks and filters when replaying logs from computer
            lcd.print(String(counter));
            lcd.print("                ");

            // print SW rotation and wheels direction
            lcd.setCursor(0, 1);
            /*lcd.print("ROT:");
            if (SW_ROTATION == true){
                lcd.print("R");
            } else if (SW_ROTATION == false) {
                lcd.print("L");
            }*/
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
                lcd.print("|REV");
            }
            lcd.print("                ");
        }

        // Let's light up the lamps!
        if (LIGHTS){
            if (int(SW_ANGLE) > int(SW_ANGLE_THRESHOLD)){
                if (WHEELS_DIR == true){ // wheels are turned right
                    if (!REVERSE) {
                        // TODO: insert small delay here to prevent flashing lights
                        // when rapidly switching from N to P (through R).
                        right_on();
                        left_off();
                    } else if (REVERSE){
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
        /*
         * TODO: delay turn off previous side like WV does; probably use SW_ROTATION?
         * Check if L or R lamp is still on when the steering wheel has already
         * turned through zero position and SW_ROTATION is opposite to the previous enabled lamp
         */
                right_off();
                left_off();
            }
        }
        else { // light is off
            right_off();
            left_off();
        }

        if (!LIGHTS && (SW_ANGLE < SW_ANGLE_THRESHOLD)){ // second time for failsafe operation?
            right_off();
            left_off();
        }
        
    } // end CAN receive
    
//    SW_ANGLE_PREVIOUS = SW_ANGLE; // save previous SW_ANGLE value
}
