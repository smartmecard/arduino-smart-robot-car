//There id more information aboat this code in Page 26 of "Instruction manual-English.pdf"

// This code is for the bluetooth and infrared controlled ultrasonic arduino car. 
// By default, the buttons 2, 4, 6 en 8 of the remote will move the car in infrared mode. 
// Ultrasonic mode is enabled by the play/pause button.
// Track following is enabled by the EQ button.
// Button 5 will stop whatever the car is doing and enable button 2, 4, 6 en 8 again.
// Bluetooth is always enabled and available while in infrared mode. The password is 1234.
//
// Android car control app can be found here:
// https://play.google.com/store/apps/details?id=braulio.calle.bluetoothRCcontroller&hl=en

//Code based on code found at icstation.com and banggood.com (same sources)
//Modifications
//   Continuous movement possible with both bluetooth and ir control
//   Customization of servo angles
//   Moved up all constants that might need customization
//   Added simular code in functions, reducing code lines and complexity
//   Broken down long functions in smaller ones to reduce complexity
//   Removed unused signals
//   Reduced global variables by creating local variables

//********Include libraries*********************************************

/**
    FIX NOTE: 
    This source code can be downloaded from http://smartmecard.wordpress.com.
    This original source code is from https://forum.arduino.cc/index.php?topic=312570.msg2166317#msg2166317
    This source code was fixed by kino21c@gmail.com. If you have questions, leave your comments on  http://smartmecard.wordpress.com
    If you want to see demo animation, please access below
    1) Avoiding obstacles : https://youtu.be/lRprGMqcNWU
    2) Black line tracking : https://youtu.be/ExaqOe6Te0c
    3) How To Make Smart Robot Car With Arduino Uno R3 (IR RC + Bluetooth RC + Obstacle Avoidance + Line Tracking):
       https://youtu.be/G-Q8FEqmgZU
       
    1. Version
    1) V0.1
    Initial version
    Have a good day with your child. :)

    
    2) V0.2
    - Added configuration depending on arduino uno board connection
    - Fixed black line tracking
    - Added function description
    
    3) V0.3
    - Removed unused codes
    - Added configuration and library
    - Fixed infrared command and bluetooth command
   
    
    2. Library
    1) IR REMOTE CONTROL: ARDUINO LIBRARY
    https://github.com/z3t0/Arduino-IRremote    
    
    2) MsTimer2
    http://playground.arduino.cc/Main/MsTimer2
    MsTimer2 is a small and very easy to use library to interface Timer2 with humans. 
    It's called MsTimer2 because it "hardcodes" a resolution of 1 millisecond on timer2.
    
    3. Aruino Bluetooth RC Car Application
    1) app link: https://play.google.com/store/apps/details?id=braulio.calle.bluetoothRCcontroller&hl=ko
    2) arduino bluetooth example code
    https://sites.google.com/site/bluetoothrccar/home/3BluetoothModulesAndArduinoCode
    
    4. Reference
    1) IR Reciever
    IR Reciever Test Code
    https://arduino-info.wikispaces.com/file/view/IR_Remote_Kit_Blink.pde/330174204/IR_Remote_Kit_Blink.pde
    https://arduino-info.wikispaces.com/file/view/IR_Remote_Kit_Numeric.pde/330172406/IR_Remote_Kit_Numeric.pde

    IR REMOTE CONTROL: ARDUINO LIBRARY
    https://github.com/z3t0/Arduino-IRremote
    	
    IR Infrared Remote Control Kit 2
    http://yourduino.com/sunshop2/index.php?l=product_detail&p=153

    2) GY-80 Multi Sensor Board - 3 Axis Gyro -3 Axis Accelerometer - 3 Axis Magnetometer - Barometer - Thermometer
    https://github.com/cedtat/GY-80-sensor-samples

    3) MPU6050 Six-Axis (Gyro + Accelerometer) MEMS MotionTracking占쎄퐪 Devices
    http://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/
    http://playground.arduino.cc/Main/MPU-6050
    
    4) Software Serial Example
    https://www.arduino.cc/en/Tutorial/SoftwareSerialExample
    
    5) if "Low memory available, stability problems may occur" is happening during compiling, refer to below
    http://www.arduino.cc/en/Tutorial/Memory
**/

//************************************************
//* Arduino Uno : Car hardware and test configuration 
//************************************************


#define CONFIG_DEBUG        // if "Low memory available, stability problems may occur." is happening, don't declare CONFIG_DEBUG
//#define CONFIG_BLUETOOTH_DEBUG //if "Low memory available, stability problems may occur." is happening, don't declare CONFIG_BLUETOOTH_DEBUG
#define CONFIG_IR_REMOTE_DEBUG //if "Low memory available, stability problems may occur." is happening, don't declare CONFIG_IR_REMOTE_DEBUG

//#define CONFIG_DIRECTION_CNTRL_001
//#define CONFIG_ULTRASONIC_001 //second HC-SR04 Ultrasonic Sensor module

#define CONFIG_BLUETOOTH_CMD  //bluetooth control command
#define CONFIG_INFRARED_CMD   //infrared remote control command
//#define CONFIG_OBSTACLE_AVOIDANCE_SENSOR //KY-032 obstacle avoidance sensor
//#define CONFIG_OBSTACLE_AVOIDANCE_TEST //KY-032 obstacle avoidance sensor test
//#define CONFIG_OBSTACLE_AVOIDANCE_SENSOR_TEST
#define CONFIG_BT_SOFTWARE_SERIAL  //bluetooth software serial


//#define CONFIG_BLACK_LINE_TRACKING_TEST //TRCT5000 Line Track Sensor , TRCT5000 Infrarared Reflective IR Photoelectronic Switch Barrier Line Track sensor
//#define CONFIG_SERVO_TEST     //SG90 servo motor test
//#define CONFIG_INFRARED_TEST        //KY-022 Infrarared IR Sensor Receiver
//#define CONFIG_BLUETOOTH_CMD_TEST //HC-06 Bluetooth serial path-through module command test with android car app

//#define CONFIG_SOUND_SENSOR_TEST //Sensitive Sound Microphone Seneor Detection Module for Arduino AVR PIC
//#define CONFIG_SOUND_SENSOR
#ifdef CONFIG_SOUND_SENSOR
#define CONFIG_SOUND_SENSOR_ANALOG
#endif

//************************************************
//Library 
//************************************************
#include <IRremote.h>       //https://github.com/z3t0/Arduino-IRremote  
#include <Servo.h>          
#include <SoftwareSerial.h> 
#if defined(CONFIG_TIMER2) || defined(CONFIG_OBSTACLE_AVOIDANCE_TEST)
#include <MsTimer2.h>       //http://playground.arduino.cc/Main/MsTimer2
#endif


//************************************************
//*Arduino Uno :  Pin defintion configuration 
//************************************************
#define ENA_PIN   6
#define EN1_PIN   7
#define EN2_PIN   3

#define EN3_PIN   4
#define EN4_PIN   2
#define ENB_PIN   5

#define IR_RECIEVER_PIN A5 // ir recieer pin number
#define SERVO_PIN       9  // servo motor pin number


#define ULTRASONIC_TRIG_PIN_000  12 //
#define ULTRASONIC_ECHO_PIN_000  13

#ifdef CONFIG_ULTRASONIC_001
#define ULTRASONIC_TRIG_PIN_001  12
#define ULTRASONIC_ECHO_PIN_001  13
#endif

#ifdef CONFIG_BT_SOFTWARE_SERIAL
/*
    Receives from the hardware serial, sends to software serial.
    Receives from software serial, sends to hardware serial.

    The circuit:
    * RX is digital pin 10 (connect to TX of other device)
    * TX is digital pin 11 (connect to RX of other device)

    Note:
    Not all pins on the Mega and Mega 2560 support change interrupts,
    so only the following can be used for RX:
    10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

    Not all pins on the Leonardo support change interrupts,
    so only the following can be used for RX:
    8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

*/
#define BT_SERIAL_RX    10 //Rx pin
#define BT_SERIAL_TX    11 //Tx pin
#endif

//sensor configuration for Black line tracking 
#define TRACKING_SENSOR_LEFT_PIN     A0 //tracking sensor left pin  
#define TRACKING_SENSOR_MIDDLE_PIN   A2 //tracking sensor middle pin
#define TRACKING_SENSOR_RIGHT_PIN    A1 //tracking sensor right pin


#if defined(CONFIG_SOUND_SENSOR) || defined(CONFIG_SOUND_SENSOR_TEST)
#define SOUND_SENSOR_PIN            A4 //Sound Sensor pin
#endif

#ifdef CONFIG_SOUND_SENSOR_ANALOG
#define SOUND_READ(pinNum)      analogRead (pinNum)
#else
#define SOUND_READ(pinNum)      digitalRead(pinNum)
#endif

#define OBSTACLE_AVOIDANCE_LEFT_SENSOR_PIN      A3    //obstacle avoidance left sensor pin
#define OBSTACLE_AVOIDANCE_RIGHT_SENSOR_PIN     A4    //obstacle avoidance right sensor pin


#define DELAY_UNIT  10 

//Pin assignments and global variables per function. Customize if needed
//*******Pin assignments Motor board and IR receiver********************
const int MotorRight1   = EN1_PIN; //right1 side motor wheel pin
const int MotorRight2   = EN2_PIN; //right2 side motor wheel pin
const int MotorLeft1    = EN3_PIN; //left1 side motor wheel pin
const int MotorLeft2    = EN4_PIN; //left2 side motor wheel pin
const int MotorRightPWM = ENA_PIN; //right side enable pin
const int MotorLeftPWM  = ENB_PIN; //left side enable pin
const int irReceiverPin = IR_RECIEVER_PIN; //ir reciever pin
const int servoPin      = SERVO_PIN; //servo motor pin
int iSpeed              = 255; //speed, range 0 to 255

//******Track following pin assignments and signals**********************
const int SensorLeft    = TRACKING_SENSOR_LEFT_PIN; 
const int SensorMiddle  = TRACKING_SENSOR_MIDDLE_PIN;
const int SensorRight   = TRACKING_SENSOR_RIGHT_PIN;
IRrecv irrecv(irReceiverPin);  // IRrecv signal
decode_results infrared;       // decode result
//*******Ultrasonic pin assignments and signals**************************

Servo ultrasonicservo000; // define ultrasonicservo000

unsigned int degreesForward = 85;  //nr degrees to look forward 
const int degreesLeft       = 25;  //nr degrees to look left  (85 - 60 )
const int degreesRight      = 145; //nr degrees to look right (85 + 60 )

//*****Bluetooth signals**************************************************
char val; //stores received character. Needs to be global to perform continuous movement

//
#ifdef CONFIG_BT_SOFTWARE_SERIAL
SoftwareSerial btSerial(BT_SERIAL_RX, BT_SERIAL_TX);// RX, TX 
#endif

/*IR Infrared Remote Control Kit 2*/
typedef struct {
	String strKeyName;
	unsigned long ulKeyValue;	
	String strCmd;	
}IRvalueData;

const IRvalueData irData[] =
{
    /*Key      KeyValue    Cmd */
	{ "0",     0x00FF6897, ""                    },
	{ "1",     0x00FF30CF, "",                   },
	{ "2",     0x00FF18E7, "MOVE FORWARD"          },
	{ "3",     0x00FF7A85, "",                   },
	{ "4",     0x00FF10EF, "MOVE LEFT"             },
	{ "5",     0x00FF38C7, "STOP"                },
	{ "6",     0x00FF5AA5, "MOVE RIGHT"            },
	{ "7",     0x00FF42BD, ""                    },
	{ "8",     0x00FF4AB5, "MOVE BACKWARD"         },
	{ "9",     0x00FF52AD, ""                    },
	{ "100+",  0x00FF9867, ""                    },
	{ "200+",  0x00FFB04F, ""                    },
	{ "-",     0x00FFE01F, "SPEED DOWN"          },
	{ "+",     0x00FFA857, "SPEED UP"            },
	{ "EQ",    0x00FF906F, "BLACK LINE TRACKING" },
	{ "<<",    0x00FF22DD, ""                    },
	{ ">>",    0x00FF02FD, ""                    },
	{ ">|",    0x00FFC23D, "AUTOMATIC"           },
	{ "CH-",   0x00FFA25D, ""                    },
	{ "CH",    0x00FF629D, ""                    },
	{ "CH+",   0x00FFE21D, ""                    },
	{ "HOLD",  0xFFFFFFFF, "BUTTON HOLD DOWN",   }
};

typedef struct {
    int CurrenMove;
    int ForwardDistance;
    int RightDistance;
    int LeftDistance;
    int RightAvoidSensor;
    int LeftAvoidSensor;
    
}stCarInfoData;

#define CAR_INFO_NUM    5
stCarInfoData stCarInfo[CAR_INFO_NUM];
int CarInfoIndex;


// see getDistance()
typedef enum {
    ULTRASONIC_000,
    ULTRASONIC_001
}ULTRASONIC_INDEX;

//car movement status
typedef enum {
    MOVE_INITIAL =0,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_BACKWARD_LEFT,
    MOVE_BACKWARD_RIGHT,
    MOVE_LEFT,
    MOVE_RIGHT,
    MOVE_ROTATE,
    MOVE_LEFT_ROTATE,
    MOVE_RIGHT_ROTATE,
    MOVE_STOP,
    MOVE_SPEED_UP,
    MOVE_SPEED_DOWN,
    MOVE_END
}CAR_MOVE;


#define TIMER_PERIOD    100 //100 ms
#if defined(CONFIG_TIMER2) || defined(CONFIG_OBSTACLE_AVOIDANCE_TEST)
#define HEAD_ROTATE_TIME 10000 //10,000 ms -> 10ms 
#else
#define HEAD_ROTATE_TIME (20000) //experiment value
#endif
static unsigned long ul_timer_count=0;

void isr_timer()
{
    ul_timer_count++;    
}
unsigned long get_time_count()
{
#if defined(CONFIG_TIMER2) || defined(CONFIG_OBSTACLE_AVOIDANCE_TEST)
    return ul_timer_count;
#else
    return ul_timer_count++;
#endif    
}
unsigned long get_elapse_time_count(unsigned long ul_old_time_count, unsigned long ul_new_time_count)
{
    unsigned long ul_elapse_time_count = 0;
    if(ul_new_time_count >= ul_old_time_count)
    {
        ul_elapse_time_count = ul_new_time_count - ul_old_time_count;
    }
    else
    {
        ul_elapse_time_count = (0xffffffff - ul_old_time_count) + ul_new_time_count;
    }
    
    return ul_elapse_time_count;
}
unsigned long get_elapse_time(unsigned long ul_old_time_count, unsigned long ul_new_time_count)
{
    //it returns ms
    return get_elapse_time_count(ul_old_time_count,ul_new_time_count)*TIMER_PERIOD; 
}



#ifdef CONFIG_SERVO_TEST
/**
 * This function is servo motor test function.
 * Servo motor is run from 0 degree to 180 degree.
 * @param       none
 * @return      none
 * @see         none
 */
void servo_test(void)
{
    int pos = 0;
    
    for (pos = 0; pos <= 180; pos += 1) 
    { 
        // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        ultrasonicservo000.write(pos);   // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
    
    for (pos = 180; pos >= 0; pos -= 1) 
    { 
        // goes from 180 degrees to 0 degrees
        ultrasonicservo000.write(pos);   // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
}
#endif


#ifdef CONFIG_INFRARED_TEST
/**
 * This function dumps IR data to be recieved through UART.
 *
 * @param       decode_results *results
 * @return      none
 * @see         none
 */
void ir_dump(decode_results *results) 
{
	int count = results->rawlen;
	if (results->decode_type == UNKNOWN) 
	{
		Serial.print("Unknown encoding: ");
	}
	else if (results->decode_type == NEC) 
	{
		Serial.print("Decoded NEC: ");
	}
	else if (results->decode_type == SONY) 
	{
		Serial.print("Decoded SONY: ");
	}
	else if (results->decode_type == RC5) 
	{
		Serial.print("Decoded RC5: ");
	}
	else if (results->decode_type == RC6) 
	{
		Serial.print("Decoded RC6: ");
	}
	else if (results->decode_type == PANASONIC) 
	{
		Serial.print("Decoded PANASONIC - Address: ");
		Serial.print(" Value: ");
	}
	else if (results->decode_type == LG) 
	{
		Serial.print("Decoded LG: ");
	}
	else if (results->decode_type == JVC)
	{
		Serial.print("Decoded JVC: ");
	}
	Serial.print(results->value, HEX);
	Serial.print(" (");
	Serial.print(results->bits, DEC);
	Serial.println(" bits)");
	Serial.print("Raw (");
	Serial.print(count, DEC);
	Serial.print("): ");

	for (int i = 0; i < count; i++) 
	{
		if ((i % 2)) 
		{
			Serial.print(results->rawbuf[i] * USECPERTICK, DEC);
		}
		else 
		{
			Serial.print((int)results->rawbuf[i] * USECPERTICK, DEC);
		}
		Serial.print(" ");
	}
	Serial.println("");
}

/**
 * This function is IR usage example function.
 *
 * @param       none
 * @return      none
 * @see         none
 */
void ir_test(void)
{
    int a;
    a = irrecv.decode(&infrared);
    Serial.print(a, DEC);
    
    //this is true if a message has been received
    if (irrecv.decode(&infrared) == true)
    {
        if (infrared.bits > 0)
        {
        	ir_dump(&infrared);
        }
        irrecv.resume(); // watch out for another message
    }
    else
    {
        delay(1000); //wait 1000ms
    }

}
#endif


/**
 * This function decodes IR key value into command
 *
 * @param       unsigned long ulKeyValue
 * @return      command string
 * @see         none
 */
String decode_IRcmd(unsigned long ulKeyValue)
{
    int i = 0;
    int table_size;
    table_size = sizeof(irData)/sizeof(IRvalueData);
	for (i = 0; i < table_size; i++)
	{
		if (irData[i].ulKeyValue == ulKeyValue)
		{
		    #ifdef CONFIG_IR_REMOTE_DEBUG
		    Serial.print("\n");
		    Serial.println(ulKeyValue, HEX);
		    Serial.print("->"+ irData[i].strKeyName+ ": CMD :" + irData[i].strCmd);
		    #endif
			return irData[i].strCmd;
		}
    }

	//Serial.println("\nUNKNOWN CMD");
	return String("UNKNOWN CMD");
}

/**
 * This function decodes IR key value into key string
 *
 * @param       unsigned long ulKeyValue
 * @return      command string
 * @see         none
 */

String decode_IRkeyname(unsigned long ulKeyValue)
{
    int i = 0;
    int table_size;
    table_size = sizeof(irData)/sizeof(IRvalueData);
	for (i = 0; i < table_size; i++)
	{
		if (irData[i].ulKeyValue == ulKeyValue)
		{
		    #ifdef CONFIG_IR_REMOTE_DEBUG
			Serial.print("\n");
		    Serial.println(ulKeyValue, HEX);
		    Serial.print("->"+ irData[i].strKeyName+ ": CMD :" + irData[i].strCmd);
		    #endif
			return irData[i].strKeyName;
		}
	}
	//Serial.println("\nUNKNOWN KEY VALUE");
	return String("UNKNOWN KEY VALUE");
}


/**
 * This function controls arduino wheels of car.
 * If this function is run, car goes forward and then wait for d*10 ms.
 *
 * @param       int d - this input unit is 10 ms. e.g. 1 means 10ms.
 * @return      none
 * @see         none
 */


void advance(int d) 
{ 
#ifdef CONFIG_DIRECTION_CNTRL_001
    //go straight
    digitalWrite(MotorRight1, HIGH);
    digitalWrite(MotorRight2, LOW);
    analogWrite(MotorRightPWM, iSpeed);    
    
    digitalWrite(MotorLeft1, HIGH);
    digitalWrite(MotorLeft2, LOW);
    analogWrite(MotorLeftPWM, iSpeed);
    delay(d * DELAY_UNIT);
#else
    //go straight
    digitalWrite(MotorRight1, LOW);
    digitalWrite(MotorRight2, HIGH);
    analogWrite(MotorRightPWM, iSpeed);    
    
    digitalWrite(MotorLeft1, LOW);
    digitalWrite(MotorLeft2, HIGH);
    analogWrite(MotorLeftPWM, iSpeed);  
    delay(d * DELAY_UNIT);

#endif
}

/**
 * This function controls arduino wheels of car.
 * If this function is run, car turns on the right and then wait for d*10 ms.
 * Left wheels go forward. But right wheels does not run.
 * @param       int d - this input unit is 10 ms. e.g. 1 means 10ms.
 * @return      none
 * @see         none
 */

void right(int d) 
{ 
    //turn right (single wheel)
    digitalWrite(MotorLeft1, LOW);
    digitalWrite(MotorLeft2, HIGH);
    analogWrite(MotorRightPWM, iSpeed);    
    
    digitalWrite(MotorRight1, LOW);
    digitalWrite(MotorRight2, LOW);
    analogWrite(MotorLeftPWM, iSpeed);
    delay(d * DELAY_UNIT);
}
/**
 * This function controls arduino wheels of car.
 * If this function is run, car turns on the left and then wait for d*10 ms.
 * Right wheels go forward. But left wheels does not run.
 * @param       int d - this input unit is 10 ms. e.g. 1 means 10ms.
 * @return      none
 * @see         none
 */

void left(int d) 
{
    //turn left(single wheel)
    digitalWrite(MotorRight1, LOW);
    digitalWrite(MotorRight2, HIGH);
    analogWrite(MotorRightPWM, iSpeed);    

    digitalWrite(MotorLeft1, LOW);
    digitalWrite(MotorLeft2, LOW);
    analogWrite(MotorLeftPWM, iSpeed);
    delay(d * DELAY_UNIT);
}
/**
 * If this function is run, car turns on the right and then wait for d*10 ms.
 * Left wheels go forward and right wheels go backward.
 * @param       int d - this input unit is 10 ms. e.g. 1 means 10ms.
 * @return      none
 * @see         none
 */

void turnR(int d) 
{ 
    //turn right (two wheels)
    digitalWrite(MotorRight1, HIGH);
    digitalWrite(MotorRight2, LOW);
    analogWrite(MotorRightPWM, iSpeed);    
    
    digitalWrite(MotorLeft1, LOW);
    digitalWrite(MotorLeft2, HIGH);
    analogWrite(MotorLeftPWM, iSpeed);  
    delay(d * DELAY_UNIT);
}
/**
 * If this function is run, car turns on the left and then wait for d*10 ms.
 * Right wheels go forward and left wheels go backward.
 * @param       int d - this input unit is 10 ms. e.g. 1 means 10ms.
 * @return      none
 * @see         none
 */

void turnL(int d) 
{
    //turn left (two wheels)
    digitalWrite(MotorRight1, LOW);
    digitalWrite(MotorRight2, HIGH);
    analogWrite(MotorRightPWM, iSpeed);    
    
    digitalWrite(MotorLeft1, HIGH);
    digitalWrite(MotorLeft2, LOW);
    analogWrite(MotorLeftPWM, iSpeed);
    delay(d * DELAY_UNIT);
}
/**
 * If this function is run, car stops and then wait for d*10 ms.
 * Both right and left wheels stops.
 * @param       int d - this input unit is 10 ms. e.g. 1 means 10ms.
 * @return      none
 * @see         none
 */

void stopp(int d) 
{ 
    //stop
    digitalWrite(MotorRight1, LOW);
    digitalWrite(MotorRight2, LOW);
    analogWrite(MotorRightPWM, iSpeed);    

    digitalWrite(MotorLeft1, LOW);
    digitalWrite(MotorLeft2, LOW);
    analogWrite(MotorLeftPWM, iSpeed);  
    delay(d * DELAY_UNIT);
}
/**
 * If this function is run, car goes backward and then wait for d*10 ms.
 * Both right and left goes backward.
 * @param       int d - this input unit is 10 ms. e.g. 1 means 10ms.
 * @return      none
 * @see         none
 */


void back(int d) 
{ 
#ifdef CONFIG_DIRECTION_CNTRL_001
    //go back
    digitalWrite(MotorRight1, LOW);
    digitalWrite(MotorRight2, HIGH);
    analogWrite(MotorRightPWM, iSpeed);    
    
    digitalWrite(MotorLeft1, LOW);
    digitalWrite(MotorLeft2, HIGH);
    analogWrite(MotorLeftPWM, iSpeed);  
    delay(d * DELAY_UNIT);
#else
   //go back
    digitalWrite(MotorRight1, HIGH);
    digitalWrite(MotorRight2, LOW);
    analogWrite(MotorRightPWM, iSpeed);
    
    digitalWrite(MotorLeft1, HIGH);
    digitalWrite(MotorLeft2, LOW);
    analogWrite(MotorLeftPWM, iSpeed);
    delay(d * DELAY_UNIT);
#endif
}


/**
 * This function converts degree servo motor into micro second.
 * It can be used with servo motor API writeMicroseconds()
 * @param       int degree - inuput value range is 0 to 180. 
 * @return      none
 * @see         ServoWriteMicroseconds.html
 */

int getServoUsec(int degree)
{
    return (degree * 1500) / 90;
}
//************Ultrasonic distance calculator*************************************
/**
 * This function calulates object distance to be detected by servo motor.
 *
 * @param       int sel_dev - input value is ULTRASONIC_000,ULTRASONIC_001
 * @param       int degrees - 0 to 180 degree
 * @param       char dir - direction character. e.g. 'F', 'B' , 'R', 'L'
 * @return      It returns cm.
 * @see         none
 */

int getDistance(int sel_dev, int degrees, char dir) 
{
    float distance;
    int triggerPin;
    int echoPin;
    int delay_time = 250;
    int pre_degrees = 0;
    switch(sel_dev)
    {
#ifdef CONFIG_ULTRASONIC_001    
        case ULTRASONIC_001: //fixed ultra sonic
            triggerPin = ULTRASONIC_TRIG_PIN_001;
            echoPin    = ULTRASONIC_ECHO_PIN_001;
            break;
#endif  
        // 0 or others
        case ULTRASONIC_000: //ultra sonic to be able to be moved by servo motor
        default:
            //read previous servo motor position
            pre_degrees = ultrasonicservo000.read();
      
            // if servo motor was moved, wait for the servo motor to become stable
            if(pre_degrees != degrees)
            {
                // move motor position
                ultrasonicservo000.write(degrees);            
                delay(delay_time); 
            }
            
            triggerPin = ULTRASONIC_TRIG_PIN_000;
            echoPin    = ULTRASONIC_ECHO_PIN_000;
            break;
    }
    // ultrasonic echo low level in 2us
    digitalWrite(triggerPin, LOW); 
    delayMicroseconds(2);
    
    // ultrasonic echo high level in 10us, at least 10us    
    digitalWrite(triggerPin, HIGH); 
    delayMicroseconds(10);
    
    // ultgrasonic echo low level    
    digitalWrite(triggerPin, LOW); 
    
    // read time
    distance = pulseIn(echoPin, HIGH); 
    /*
        speed of sound is 340 m/sec or 29 us / cm 
        distance = distance / 29 /2 = distance / 5.8/ 10
                 = distance / 58
    */
    distance = distance / 58; // turn time to distance
#ifdef CONFIG_DEBUG    
    Serial.print(dir); // 
    Serial.print(int(sel_dev));
    if(sel_dev == ULTRASONIC_000)
    {
        Serial.print((int)ultrasonicservo000.read());
    }
    
    Serial.print(" distance: "); // 
    Serial.print(int(distance)); //  output distance (cm)
    Serial.print("\n");
#endif    
    return (int)distance;
}


//*************Ultrasonic direction decision making******************************
//measurements three angles (front, left, right
/**
 * This function decides movement direction from sensor data
 *
 * @param       none
 * @return      It returns CAR_MOVE enum value.
 * @see         none
 */
static unsigned long ul_old_timer_count = 0; 
int getDirectionFromdetection() 
{
    int Fspeedd = 0; // front distance
    int Rspeedd = 0; // right distance
    int Lspeedd = 0; // left distance
    int Ravoidsensor = 0; //Right avoidance sensor
    int Lavoidsensor = 0; //Left avoidance sensor
    
    int directionn =MOVE_INITIAL;

#ifdef CONFIG_ULTRASONIC_001    
    int Fspeedd001 = 0;
#endif    
    int FspeeddTmp = 0;
    int Index = 0;
    int PreIndex = 0;
    int CurIndex = 0;
    int BackwardCnt = 0;


    unsigned long ul_new_timer_count = 0;
    unsigned long ul_elapse_time = 0;
    
    PreIndex     = CarInfoIndex;
    CurIndex     = CarInfoIndex = (++CarInfoIndex) % CAR_INFO_NUM; 

   
    //timer
    ul_new_timer_count = get_time_count();
    ul_elapse_time     = get_elapse_time(ul_old_timer_count,ul_new_timer_count);
    //measure 40 degrees and 80 degrees more than 10000 ms to solve corner problem
    //Serial.print(ul_elapse_time); // 
    if(ul_elapse_time > HEAD_ROTATE_TIME)
    {
        stopp(10);
        Fspeedd    = getDistance(ULTRASONIC_000,40, 'F');
        FspeeddTmp = getDistance(ULTRASONIC_000,80, 'F');  
        Fspeedd    = min(Fspeedd,FspeeddTmp);
        //update old time
        ul_old_timer_count = get_time_count();
    }
    else
    {
        //get front distance
        Fspeedd    = getDistance(ULTRASONIC_000,degreesForward, 'F');
        FspeeddTmp = getDistance(ULTRASONIC_000,degreesForward, 'F');
        Fspeedd    = min(Fspeedd,FspeeddTmp);
#ifdef CONFIG_ULTRASONIC_001 
        /*get distance from fixed ultra sonic of 90 degree*/
        Fspeedd001 = getDistance(ULTRASONIC_001,90, 'F');
        Fspeedd    = min(Fspeedd,Fspeedd001);
#endif
    }

#ifdef CONFIG_OBSTACLE_AVOIDANCE_SENSOR    
    Lavoidsensor = analogRead(OBSTACLE_AVOIDANCE_LEFT_SENSOR_PIN);
    Ravoidsensor = analogRead(OBSTACLE_AVOIDANCE_RIGHT_SENSOR_PIN);    

    if(Lavoidsensor && Ravoidsensor)
    {
        stopp(10);
        directionn = MOVE_BACKWARD;
        goto end;
    }
    else if(Lavoidsensor && !Ravoidsensor)
    {
        stopp(10);
        directionn = MOVE_BACKWARD_RIGHT; //back and right
        goto end;
    }
    else if(!Lavoidsensor && Ravoidsensor)
    {
        stopp(10);
        directionn = MOVE_BACKWARD_LEFT; //back and left
        goto end;    
    }
        
#endif    

    // if distance is less than 10 cm
    if (Fspeedd <= 10) 
    {
        stopp(10);
        directionn = MOVE_BACKWARD;
        
    }
    
    // if distance less than 20 cm, measure left and right distance
    if (((Fspeedd <= 20) && (Fspeedd > 10)) || (stCarInfo[PreIndex].CurrenMove == MOVE_STOP)) 
    {
        stopp(10); 
        
        Lspeedd   = getDistance(ULTRASONIC_000,degreesLeft, 'L'); // detection distance on left side
        FspeeddTmp= getDistance(ULTRASONIC_000,degreesLeft, 'L');
        Lspeedd    = min(Lspeedd,FspeeddTmp);
        
        Rspeedd   = getDistance(ULTRASONIC_000,degreesRight, 'R'); // detection distance on right side
        FspeeddTmp= getDistance(ULTRASONIC_000,degreesRight, 'R');
        Rspeedd    = min(Rspeedd,FspeeddTmp);

        if(Lspeedd < 15 && Rspeedd < 15 ) 
        {
            directionn = MOVE_BACKWARD; //go back
        }
        else
        {
            // if left distance greater than right
            if (Lspeedd > Rspeedd) 
            {
                directionn = MOVE_LEFT; // go left
            }
            else 
            {
                //if left distance less than right
                directionn = MOVE_RIGHT; //go right
            }
        }
    }
    
end:  
    /*
        compare previous information and then decide direction
    */
    if(Rspeedd == 0)
    {
        Rspeedd = Fspeedd;
    }
    if(Lspeedd == 0)
    {
        Lspeedd = Fspeedd;  
    }

    BackwardCnt = 0;
    /*check all previous infomation */
    Index = PreIndex;
    while(Index !=  CurIndex)
    {
        if(stCarInfo[Index].CurrenMove != MOVE_INITIAL)
        {
            if(stCarInfo[Index].CurrenMove == MOVE_BACKWARD ||
               stCarInfo[Index].CurrenMove == MOVE_BACKWARD_LEFT ||
               stCarInfo[Index].CurrenMove == MOVE_BACKWARD_RIGHT 
            )
            {
                BackwardCnt++;
            }
            
            if(BackwardCnt >= 4 )
            {
                stCarInfo[Index].CurrenMove = MOVE_INITIAL;
                directionn = MOVE_STOP;
                break;
            }
        }
        Index--;
        if(Index < 0)
        {
            Index = CAR_INFO_NUM-1;
        }
    }
    
    /*if not decided, go forward*/
    if(directionn == MOVE_INITIAL)
    {
        directionn = MOVE_FORWARD;
    }
    
    /*update info*/  
    stCarInfo[CurIndex].CurrenMove      = directionn;    
    stCarInfo[CurIndex].ForwardDistance = Fspeedd;
    stCarInfo[CurIndex].RightDistance   = Rspeedd;
    stCarInfo[CurIndex].LeftDistance    = Lspeedd;
    stCarInfo[CurIndex].LeftAvoidSensor = Lavoidsensor;
    stCarInfo[CurIndex].RightAvoidSensor = Ravoidsensor;
    


    return directionn;
}
/**
 * This function runs car with using ultra sonic sensor, obstacle sensor and other sensor automatically.
 *
 * @param       none
 * @return      none
 * @see         none
 */
unsigned long go_forward_position_compensate_cnt = 0;
void autoRun() 
{
    bool stopPressed;
    int directionn = 0; 
    
    memset(&stCarInfo,0,sizeof(stCarInfoData)*CAR_INFO_NUM);
    CarInfoIndex = CAR_INFO_NUM - 1;
    while (1) 
    {
        directionn  = getDirectionFromdetection(); 
        stopPressed = stopCommandPressed();
        if (stopPressed) 
        {
            #ifdef CONFIG_DEBUG         
            Serial.print("\n-> Stop command ");
            #endif            
            break;
        }
     
        /*check direction */
        if (directionn == MOVE_FORWARD) 
        { 
            #ifdef CONFIG_DEBUG            
            Serial.print(" ->Move Forward \n"); //
            #endif  
            
            //to compensate position
            if(go_forward_position_compensate_cnt++ % 2)
            {
                turnL(0);
            }
            else
            {
                turnR(0);
            }   
            //go straight
            advance(0);

           
        }
        else if (directionn == MOVE_BACKWARD) 
        { 
            #ifdef CONFIG_DEBUG            
            Serial.print(" ->Move Back \n"); //
            #endif  
             //go back left :  180 degree will be roated by left
            back(8);
            turnL(40);
            stopp(0);

          
        }
        else if (directionn == MOVE_BACKWARD_LEFT) 
        { 
            #ifdef CONFIG_DEBUG            
            Serial.print(" ->Move Back Left \n"); //
            #endif  
             //go back left :  180 degree will be roated by left
            back(8);
            turnL(40);
            stopp(0);

          
        } 
        else if (directionn == MOVE_BACKWARD_RIGHT) 
        { 
            #ifdef CONFIG_DEBUG            
            Serial.print(" ->Move Back Right \n"); //
            #endif  
            
            back(8);
            turnR(40);
            stopp(0);
        }         
        else if (directionn == MOVE_RIGHT)
        { 
            #ifdef CONFIG_DEBUG            
            Serial.print(" ->Move Right \n"); //
            #endif  
            //turn right
            back(1);
            turnR(50);
            stopp(0);
           
        }
        else if (directionn == MOVE_LEFT) 
        { 
            #ifdef CONFIG_DEBUG            
            Serial.print(" ->Move Left \n");
            #endif   
            //turn left
            back(1);
            turnL(50);
            stopp(0);
        }
        else if (directionn == MOVE_STOP) 
        {
            #ifdef CONFIG_DEBUG            
            Serial.print(" ->Stop \n");
            #endif   
            stopp(50);        
        }
    }
    
    //it  will make head look in front
    ultrasonicservo000.write(degreesForward);    
}

//*************************Bluetooth functionality***********************
//Bluetooth commands
/**
 * This function controls car through bluetooth command.
 *
 * @param       none
 * @return      none
 * @see         none
 */

void bluetoothCommand() 
{
    
#ifdef CONFIG_BT_SOFTWARE_SERIAL
    if (btSerial.available()) 
    { 
        char szTmp[12];
        //check if bluetooth command available
        val = btSerial.read();
        btSerial.write(val);

        sprintf(szTmp,"\n->[%c]",val);
        Serial.println(szTmp); 

      
    }
    /*
      Commands/characters sent to the car of Arduino Bluetooth RC Car App.
      Forward -> F
      Back    -> B
      Left    -> L
      Rigtht  -> R
      Stop    -> S
      Forward Left -> G
      Forward Right -> I
      Back Left -> H
      Back Right -> J
      Front Lights On -> W  ->  modified as auto run
      Front Lights Off -> w ->  modified as Stop
      Horn On  -> V         ->  modified as black line tracking
      Horn Off -> v         ->  modified as Stop
    */
    /*val global variable was used to keep value without*/        
    if (val == 'F') 
    { 
        // Forward
        #ifdef CONFIG_BLUETOOTH_DEBUG
	    Serial.print("\n>Go Forward");
	    #endif
        advance(10);
    }
    else if (val == 'S' || val == 'v' || val == 'w') 
    {
        // Stop Forward
        #ifdef CONFIG_BLUETOOTH_DEBUG
	    Serial.print("\n>Stop");
	    #endif
        stopp(10) ;
        val = btSerial.read(); //read value again, otherwise can't continue with infrared
    }
    else if (val == 'B') 
    { 
        // Backwards
        #ifdef CONFIG_BLUETOOTH_DEBUG
	    Serial.print("\n>Go Backward");
	    #endif
        back(10);
    }
    else if (val == 'R') 
    { 
        // Right
        #ifdef CONFIG_BLUETOOTH_DEBUG
	    Serial.print("\n>Right");
	    #endif
        turnR(10);
    }
    else if (val == 'L') 
    { 
        // Left
        #ifdef CONFIG_BLUETOOTH_DEBUG
	    Serial.print("\n>Left");
	    #endif
        turnL(10);
    }
    else if (val == 'G') 
    { 
        #ifdef CONFIG_BLUETOOTH_DEBUG
        Serial.print("\n>Forward Left");
        #endif
        advance(10);
        turnL(10);
    }
    else if (val == 'I') 
    { 
        #ifdef CONFIG_BLUETOOTH_DEBUG
        Serial.print("\n>Forward Right");
        #endif
        advance(10);
        turnR(10);        
    }
    else if (val == 'H') 
    { 
        #ifdef CONFIG_BLUETOOTH_DEBUG
        Serial.print("\n>Back Left");  
        #endif
        back(10); 
        turnL(10); 
    }
    else if (val == 'J') 
    { 
        #ifdef CONFIG_BLUETOOTH_DEBUG
        Serial.print("\n>Back Right"); 
        #endif
        back(10);
        turnR(10);          
    }    
    else if (int(val) >= 49 && int(val) <= 57) 
    { 
        char szTmp[64];
        //set speed
        iSpeed = (int(val)-48)*26;
        sprintf(szTmp,"val %d:Speed set to: %d",val,iSpeed);
        Serial.println(szTmp); 
    }
    else if (val == 'q') 
    { 
        char szTmp[64];
        //set speed
        iSpeed = 255;
        #ifdef CONFIG_BLUETOOTH_DEBUG

        sprintf(szTmp,"Speed set to: %d",iSpeed);
        Serial.println(szTmp);  
        #endif
    }
    else if (val == 'W') 
    {
        #ifdef CONFIG_BLUETOOTH_DEBUG
        Serial.print("\n>auto run");
        #endif
        autoRun();
    }
    else if (val == 'V') 
    {
        #ifdef CONFIG_BLUETOOTH_DEBUG
        Serial.print("\n>black line tracking ");
        #endif
        followBlackLine();   
    }  


#else /*CONFIG_BT_SOFTWARE_SERIAL*/

    if (Serial.available()) 
    { 
        //check if bluetooth command available
        val = Serial.read();
        Serial.write(val);
  
    }
    
    /*val global variable was used to keep value without*/    
    if (val == 'F') 
    { 
        // Forward
        advance(10);
    }
    else if (val == 'S' || val == 'v' || val == 'w') 
    {
        // Stop Forward
        stopp(10) ;
        val = Serial.read(); //read value again, otherwise can't continu with infrared
    }
    else if (val == 'B') 
    { 
        // Backwards
        back(10);
    }
    else if (val == 'R') 
    { 
        // Right
        turnR(10);
    }
    else if (val == 'L') 
    { 
        // Left
        turnL(10);
    }
    else if (val == 'G') 
    { 
        advance(10);
        turnL(10);
    }
    else if (val == 'I') 
    { 
        advance(10);
        turnR(10);        
    }
    else if (val == 'H') 
    { 
        back(10); 
        turnL(10); 
    }
    else if (val == 'J') 
    { 
        back(10);
        turnR(10);          
    }    
    else if (int(val) >= 49 && int(val) <= 57) 
    { 
        //set speed
        iSpeed = (int(val)-48)*26;
    }
    else if (val == 'q') 
    { 
        //set speed
        iSpeed = 255;
    }
    else if (val == 'W') 
    {
        autoRun();
    }
    else if (val == 'V') 
    {
        followBlackLine();   
    } 


#endif    
}
/**
 * This function checks stop key of IR remote controller is pressed or not.
 * 
 * @param       none
 * @return      if stop key is pressed, it returns true. Otherwise, it returns false.
 * @see         none
 */

//Check if stop command on remote is pressed (button 5)
bool stopCommandPressed()
{
    bool stopPressed = false;
    
#ifdef CONFIG_INFRARED_CMD 
    char val=0;

    #ifdef CONFIG_BT_SOFTWARE_SERIAL
    if (btSerial.available()) 
    { 
        val = btSerial.read();
        btSerial.write(val);
        switch(val)
        {
            case 'S':
            case 'w':
            case 'v':
                stopp(10);
                stopPressed = true;   
                goto end;
                break;
            default:
                break;
        }
      
    }
    #else
    if (Serial.available()) 
    { 
        val = Serial.read();
        Serial.write(val);
        switch(val)
        {
            case 'S':
            case 'w':
            case 'v':
                stopp(10);
                stopPressed = true;   
                goto end;
                break;
            default:
                break;
        }
    }
    #endif
#endif

#ifdef CONFIG_INFRARED_CMD
    if (irrecv.decode(&infrared) == true) 
    {
        String strCmd;
        //Serial.println(infrared.value, HEX);
        //if (infrared.bits > 0) 
        {
            strCmd = decode_IRcmd(infrared.value);
            if (strCmd == "STOP") 
            {
                stopp(10);
                stopPressed = true;
            }
        }
        infrared.value = 0;
        irrecv.resume(); //watch out for another message
    }
    infrared.value = 0;
#endif
end:
    return stopPressed;
}
/**
 * This function decides black line or not.
 *
 * @param       int d - LOW or HIGH
 * @return      none
 * @see         none
 */

int isBlackLine(int d)
{
    return  (d == HIGH) ? true : false;   
}
/**
 * This function tracks black line if the black line is drawn on the floor.
 *
 * @param       none
 * @return      none
 * @see         none
 */

void followBlackLine() 
{
    bool stopPressed;
    int SL;  //sensor left
    int SM;  //sensor middle
    int SR;  //sensor right
    char szTmp[64];
    while (1) 
    {
        SL = digitalRead(SensorLeft);
        SM = digitalRead(SensorMiddle);
        SR = digitalRead(SensorRight);

        sprintf(szTmp,"\n[%d,%d,%d]",SL,SM,SR);
        Serial.println(szTmp);
       
        // white : LOW(0), black : HIGH(1)
        
        //middle sensor in black area
        if (SM == HIGH) 
        {
            if (SL == LOW && SR == HIGH) //middle black, left  white, right black
            {   
                #ifdef CONFIG_DEBUG
                Serial.print("->Move Right\n");
                #endif
                                
                right(0);
            }
            else if (SL == HIGH && SR == LOW)  //middle: black, left black, right white
            {   
                #ifdef CONFIG_DEBUG
                Serial.print("->Move Left\n");
                #endif
                                
                left(0);
            }
            else  //middle: black,  left and right both in white, go straight
            { 
                #ifdef CONFIG_DEBUG
                Serial.print("->Move Forward\n");
                #endif
                advance(0);
                
            }
        }
        // middle sensor in white area
        else 
        {
            if (SL == LOW && SR == HIGH) //middle white, left white ,right black
            { 
                #ifdef CONFIG_DEBUG
                Serial.print("->Move Right\n");
                #endif
                //stop to compensate inertia problem
                right(0);
            }
            else if (SL == HIGH && SR == LOW) //middle white, left black, right white
            {
                #ifdef CONFIG_DEBUG
                Serial.print("->Move Left\n");
                #endif
                left(0);
            }
            else if (SL == HIGH && SR == HIGH)
            {
                #ifdef CONFIG_DEBUG
                Serial.print("->Move Forward\n");
                #endif
                advance(0);
                
            }
            else //middle white, left and right both in white, stop
            {  
                #ifdef CONFIG_DEBUG
                Serial.print("-> Stop\n");
                #endif
                //stop and back to compensate sensor position
                stopp(1);
                back(2);
                
            }
        }
        
        stopPressed = stopCommandPressed();
        if (stopPressed) 
        {
            break;
        }
    }
}
/**
 * This function is arduino main loop function.
 * infraredCommand() function is controlled by infraread remote controller
 * @param       none
 * @return      none
 */
void infraredCommand()
{
    int speed;

    if (irrecv.decode(&infrared) == true)  
    {   
        String strCmd;
        strCmd = decode_IRcmd(infrared.value);
        Serial.print("\nCMD: " + strCmd);
        if (strCmd == "MOVE FORWARD") 
        {
            advance(10); //go straigt
        }
        else if (strCmd =="MOVE BACKWARD") 
        {
            back(10); //go back
        }
        else if (strCmd == "MOVE RIGHT") 
        {
            turnR(10); // go right
        }
        else if (strCmd == "MOVE LEFT") 
        {
            turnL(10); // go left
        }
        else if (strCmd == "STOP") 
        {
            //stop
            stopp(10);
        }
        else if (strCmd == "SPEED DOWN")
        {   
            speed = iSpeed;
            speed -= 20;
            //70 means 3.6V : 255 is 5V
            iSpeed = max(speed,70); 
        }
        else if (strCmd == "SPEED UP")
        {
            speed = iSpeed;
            speed += 20;
            //255 means 5V
            iSpeed = min(speed,255);
        }
        else if (strCmd == "BLACK LINE TRACKING") 
        {
            followBlackLine();
        }
        else if (strCmd == "AUTOMATIC" ) 
        {
            //automatically by utrasonic sensor, avoidance sensor, g
            autoRun();

        }
        //********************wait a little before continuing**************************
        irrecv.resume();
        //delay(300);
    }
    else 
    {
        //Serial.print(">NO signal > Stop ");
        stopp(0);
    }
}


/**
 * This function is arduino setup function.
 * This setup function initializes hardware port.
 * loop() function is called after setup() function is called.
 * @param       none
 * @return      none
 * @see         void loop()
 */
void setup() 
{
    //start receiving serial
    Serial.begin(9600);
#ifdef CONFIG_BT_SOFTWARE_SERIAL
    btSerial.begin(9600);
#endif
    //motor connections
    pinMode(MotorRight1,   OUTPUT); //
    pinMode(MotorRight2,   OUTPUT); //
    pinMode(MotorLeft1,    OUTPUT); //
    pinMode(MotorLeft2,    OUTPUT); //
    pinMode(MotorRightPWM, OUTPUT); //enable for right side motor
    pinMode(MotorLeftPWM,  OUTPUT); //enable for right side motor
    
#if defined(CONFIG_INFRARED_CMD) || defined(CONFIG_INFRARED_TEST)    
    //IR reciever
    pinMode(irReceiverPin, INPUT);
    irrecv.enableIRIn();      // start infrared decode
#endif

    //black track following
    pinMode(SensorLeft,   INPUT);
    pinMode(SensorMiddle, INPUT);
    pinMode(SensorRight,  INPUT);

    //Ultra sonic servo motor : it will make head look forward
    ultrasonicservo000.write(degreesForward); 
    ultrasonicservo000.attach(servoPin);

    //Ultra sonic
    pinMode(ULTRASONIC_ECHO_PIN_000, INPUT);
    pinMode(ULTRASONIC_TRIG_PIN_000, OUTPUT);
#ifdef CONFIG_ULTRASONIC_001
    pinMode(ULTRASONIC_ECHO_PIN_001, INPUT);
    pinMode(ULTRASONIC_TRIG_PIN_001, OUTPUT);
#endif


#if defined(CONFIG_SOUND_SENSOR) || defined(CONFIG_SOUND_SENSOR_TEST)
    //sound sensor
    pinMode(SOUND_SENSOR_PIN,INPUT);
#endif    

#ifdef CONFIG_OBSTACLE_AVOIDANCE_SENSOR  
    //infrarared avoidance sensor
    pinMode(OBSTACLE_AVOIDANCE_LEFT_SENSOR_PIN,INPUT);
    pinMode(OBSTACLE_AVOIDANCE_RIGHT_SENSOR_PIN,INPUT);
#endif 

#if defined(CONFIG_TIMER2) || defined(CONFIG_OBSTACLE_AVOIDANCE_TEST)
    //IRremote.h library is using Timer 2. So if you does not use IR, declare  CONFIG_TIMER2
    // isr_timer() will be called every TIMER_PERIOD ms
    MsTimer2::set(TIMER_PERIOD, isr_timer); 
    MsTimer2::start();
#endif    
}


//**************************************MAIN LOOP***************************************
/**
 * This function is arduino main loop function.
 * loop() function is called after setup() function is called.
 * @param       none
 * @return      none
 * @see         void setup()
 */
void loop() 
{
    /*test*/
#if defined(CONFIG_OBSTACLE_AVOIDANCE_TEST)
    autoRun();
#elif defined(CONFIG_BLACK_LINE_TRACKING_TEST)   
    followBlackLine();
#elif defined(CONFIG_SERVO_TEST)
    servo_test();
#elif defined(CONFIG_INFRARED_TEST)
    ir_test();
#elif defined(CONFIG_BLUETOOTH_CMD_TEST)  
    bluetoothCommand();
#elif defined(CONFIG_SOUND_SENSOR_TEST)  
    Serial.print("\n Sound: ");
    Serial.print(SOUND_READ(SOUND_SENSOR_PIN));  
    delay(100);
#elif defined(CONFIG_OBSTACLE_AVOIDANCE_SENSOR_TEST)   
    Serial.print("\n Avoidance Left: ");
    Serial.print(analogRead(OBSTACLE_AVOIDANCE_LEFT_SENSOR_PIN));  
    Serial.print("\n Avoidance Right: ");
    Serial.print(analogRead(OBSTACLE_AVOIDANCE_RIGHT_SENSOR_PIN));      
    delay(100);
#else

    /*main loop*/
    #ifdef CONFIG_BLUETOOTH_CMD
    //bluetooth commands : this command can be controlled by smart phone or serial debug port
    bluetoothCommand();
    #endif  

    #ifdef CONFIG_INFRARED_CMD  
    //infrared commands  : this command can be controlled by infrared remote controller
    infraredCommand();
    #endif

#endif    
}


