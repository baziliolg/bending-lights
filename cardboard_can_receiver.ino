#include <SPI.h>
#include <mcp_can.h>

// Define which digital pins are for left and right side
const byte right_pin = 6;
const byte left_pin = 5;

bool left_already_on = false;
bool right_already_on = false;
bool left_already_off = false;
bool right_already_off = false;

// Declare CAN as MCP_CAN
const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN); // Set CS pin used for CAN bus shield

// enable LCD (for debugging)
#define _LCD_TYPE 1
#include <LCD_1602_RUS_ALL.h>
LCD_1602_RUS <LiquidCrystal_I2C> lcd(0x27, 16, 2);
boolean enableLCD = true;

// PWM fade-in functions
void left_on() {
   if (!left_already_on){
        
        for (int i = 0; i <= 255; i=i+5) {
            analogWrite(left_pin, i);
            delay(10);
        }
    // make sure it is fully on in the end
    analogWrite(left_pin, 255);
    left_already_on = true;
    left_already_off = false;
    }
}

void right_on() {
    if (!right_already_on){
    
    for (int i = 0; i <= 255; i=i+5) {
        analogWrite(right_pin, i);
        delay(10);
    }
    // make sure it is fully on in the end
    analogWrite(right_pin, 255);
    right_already_on = true;
    right_already_off = false;
    }
}

// PWM fade-out fun_ctions
void left_off() {
    if (!left_already_off) {
    left_already_off = true;
    left_already_on = false;
  for (int i = 255; i >= 0; i=i-1) {
    analogWrite(left_pin, i);
    delay(3);
  }
  // make sure it is off in the end
  analogWrite(left_pin, 0);
    }
}

void right_off() {
    if (!right_already_off) {
    right_already_off = true;
    right_already_on = false;
  for (int i = 255; i >= 0; i=i-1) {
    analogWrite(right_pin, i);
    delay(3);
  }
  // make sure it is off in the end
  analogWrite(right_pin, 0);
    }
}


// the setup function runs once when you press reset or power the board
void setup() {
    // initialize digital pins as an output.
    pinMode(right_pin, OUTPUT);
    pinMode(left_pin, OUTPUT);
    digitalWrite(right_pin, LOW);
    digitalWrite(left_pin, LOW);
    // enable serial port output
    //Serial.begin(115200);

    if(enableLCD){
        // LCD display will be used for debugging
        lcd.init(); // Инициализация LCD
        lcd.backlight();
    }

    while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz))              // init can bus : baudrate = 500k
    {
/*        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
*/
        if(enableLCD){ lcd.setCursor(0, 0); lcd.print("CAN init FAILED"); }
    }
/*    Serial.println("CAN shield init ok!"); */
    if (enableLCD){ lcd.setCursor(0, 0); lcd.print("CAN init OK"); }

    /* Now I'll try to init CAN message masks and filters
     *  CAN message ID is 11 bits long, full 11-bit mask is 0x7FF.
     *  Note: CAN ID 0x076 is 00001110110
     *  For now it is the only message I am using.
     */

    CAN.init_Mask(0, false, 0x76);
    CAN.init_Mask(1, false, 0x76);

    CAN.init_Filt(0, false, 0x76);
    CAN.init_Filt(1, false, 0x76);
    CAN.init_Filt(2, false, 0x76);
    CAN.init_Filt(3, false, 0x76);
    CAN.init_Filt(4, false, 0x76);
    CAN.init_Filt(5, false, 0x76);

}

void loop() {
    static byte sixth_byte;
    // where to assemble Steering Wheel angle from 14-bit message
    static short swAngleMessage;
    static short SW_ANGLE;
    static short SW_ANGLE_PREVIOUS;
    short ANGLE_DIFF;
    static short ANGLE_DIFF_ABS;
    static bool SW_ROTATION;
    static bool WHEELS_DIR;

    // try to receive CAN messages and show them in Serial port
    unsigned char len = 0;
    unsigned char buf[8];

    
    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        unsigned long canId = CAN.getCanId();
/*
        Serial.print("len "); Serial.print(len);
        Serial.print(" ID 0x");
        Serial.print(canId, HEX);
        Serial.print(": ");
*/
        //analogWrite(left_pin, 0); // dark if another message received
        if (canId == 0x076){ // filter CAN message flow for the message I am interested in
            //analogWrite(left_pin, 255); // light up if message with CAN ID 0x76 received

            // getting the direction where wheels are pointing at, left=0 right=1
            WHEELS_DIR = buf[0] & 0x40; // grab bit 1 from byte 0

            // getting rotation direction of the steering wheel, left=0 right=1
            SW_ROTATION = buf[2] & 0x80; // grab bit 0 from byte 2

            // getting the steering wheel angle
            /* I must cut off first two bits of 6th (7th if counting from 1) byte of message with CAN ID 0x076
                in order to get 14-bit value of steering angle from bytes 6 and 7.
                That is why there is this mask 0x3f, it is 00111111 in binary,
                ones means "copy this bit", zeroes mean "ignore" */
            sixth_byte = buf[6] & 0x3F;
            swAngleMessage = word(sixth_byte,buf[7]);
            SW_ANGLE = swAngleMessage * 0.04395;

            /* Try to at least see previous angle value
             *  probably will use for smoothing out angle */
            ANGLE_DIFF = SW_ANGLE - SW_ANGLE_PREVIOUS;
            ANGLE_DIFF_ABS = abs(ANGLE_DIFF);

        }   // end filtering for 0x076 message

        if(enableLCD){ 
            // print angle
            lcd.setCursor(0, 0);
            lcd.print(String(SW_ANGLE));
            lcd.print("° P:");
            lcd.print(String(SW_ANGLE_PREVIOUS));
            lcd.print("° D:");
            lcd.print(String(ANGLE_DIFF_ABS));
            lcd.print("                ");

            // print SW rotation and wheels direction
            lcd.setCursor(0, 1);
            lcd.print("ROT: ");
            if (SW_ROTATION == true){
                lcd.print("R");
            } else if (SW_ROTATION == false) {
                lcd.print("L");
            }
            lcd.print(" | WH: ");
            if (WHEELS_DIR == true){
                lcd.print("R");
            } else if (WHEELS_DIR == false) {
                lcd.print("L");
            }            
            lcd.print("                ");
        }

        // Let's light up the lamps!
        if (int(SW_ANGLE) > 40){
            if (WHEELS_DIR == true){ // wheels are turned right
                digitalWrite(right_pin, HIGH);
                digitalWrite(left_pin, LOW);
            } else if (WHEELS_DIR == false){ // wheels are turned left
                digitalWrite(right_pin, LOW);
                digitalWrite(left_pin, HIGH);
            }
        } else if (int(SW_ANGLE) < 40){
        // TODO: delay turn off previous side like WV does; probably use SW_ROTATION?
        // Check if L or R lamp is still on when the steering wheel has already
        // turned through zero position and SW_ROTATION is opposite to the previous enabled lamp
            digitalWrite(right_pin, LOW);
            digitalWrite(left_pin, LOW);
        }

/*
        for(int i = 0; i<len; i++)    // print the data
        {
            Serial.print(buf[i], BIN);
            Serial.print("\t");
            //if(enableLCD){ lcd.print(buf[i], HEX); lcd.print("|"); }
        }
        Serial.println();
*/
    } // end CAN receive
    
    SW_ANGLE_PREVIOUS = SW_ANGLE; // save previous SW_ANGLE value
}
