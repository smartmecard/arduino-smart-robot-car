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
    
    1. V0.1
    Initial version
    Have a good day with your child. :)

    
    2. V0.2
    1) Added configuration depending on arduino uno board connection
    2) Fixed black line tracking
    3) Added function description
    
   
**/

#include <IRremote.h>
#include <Servo.h>


//************************************************
//* Arduino Uno : Car hardware and test configuration 
//************************************************

//#define CONFIG_LED_DISPLAY  // It is not used by me
//#define CONFIG_DIRECTION_CNTRL_001
#define CONFIG_DEBUG        // UART serial debugging.. if debugging is finished, undefine it to improve performance
//#define CONFIG_ULTRASONIC_001 //HC-SR04 Ultrasonic Sensor module

//#define CONFIG_2WHEEL_DRIVE
//#define CONFIG_BLUETOOTH_CMD
#define CONFIG_OBSTACLE_AVOIDANCE_SENSOR //KY-032 obstacle avoidance sensor
//#define CONFIG_OBSTACLE_AVOIDANCE_TEST //KY-032 obstacle avoidance sensor test
//#define CONFIG_OBSTACLE_AVOIDANCE_SENSOR_TEST


//#define CONFIG_BLACK_LINE_TRACKING_TEST //TRCT5000 Line Track Sensor , TRCT5000 Infrarared Reflective IR Photoelectronic Switch Barrier Line Track sensor
//#define CONFIG_SERVO_TEST     //SG90 servo motor test
//#define CONFIG_IR_TEST        //KY-022 Infrarared IR Sensor Receiver
//#define CONFIG_BLUETOOTH_CMD_TEST //HC-06 Bluetooth serial path-through module command test with android car app

//#define CONFIG_SOUND_SENSOR_TEST //Sensitive Sound Microphone Seneor Detection Module for Arduino AVR PIC
//#define CONFIG_SOUND_SENSOR
#ifdef CONFIG_SOUND_SENSOR
#define CONFIG_SOUND_SENSOR_ANALOG
#endif




//************************************************
//*Arduino Uno :  Pin defintion configuration 
//************************************************
#define ENA_PIN   6 //
#define EN1_PIN   7
#define EN2_PIN   3

#define EN3_PIN   4
#define EN4_PIN   2
#define ENB_PIN   5

#define IR_RECIEVER_PIN A3 // ir recieer pin number
#define SERVO_PIN       9  // servo motor pin number


#define ULTRASONIC_TRIG_PIN_000  12 //
#define ULTRASONIC_ECHO_PIN_000  13

#ifdef CONFIG_ULTRASONIC_001
#define ULTRASONIC_TRIG_PIN_001  10
#define ULTRASONIC_ECHO_PIN_001  11
#endif


#define TRACKING_SENSOR_LEFT_PIN     A0 //tracking sensor left pin  
#define TRACKING_SENSOR_MIDDLE_PIN   A2 //tracking sensor middle pin
#define TRACKING_SENSOR_RIGHT_PIN    A1 //tracking sensor right pin
#ifdef CONFIG_LED_DISPLAY
#define LED_PIN 0
#endif


#define SOUND_SENSOR_PIN            A4 //Sound Sensor pin
#ifdef CONFIG_SOUND_SENSOR_ANALOG
#define SOUND_READ(pinNum)      analogRead (pinNum)
#else
#define SOUND_READ(pinNum)      digitalRead(pinNum)
#endif

#define OBSTACLE_AVOIDANCE_SENSOR_PIN A5    //obstacle avoidance sensor pin

//Pin assignments and global variables per function. Customize if needed
//*******Pin assignments Motor board and IR receiver********************
const int MotorRight1   = EN1_PIN; //front right side motor wheel pin
const int MotorRight2   = EN2_PIN; //rear right side motor wheel pin
const int MotorLeft1    = EN3_PIN; //front left side motor wheel pin
const int MotorLeft2    = EN4_PIN; //rear left side motor wheel pin
const int MotorRightPWM = ENA_PIN; //right side enable pin
const int MotorLeftPWM  = ENB_PIN; //left side enable pin
const int irReceiverPin = IR_RECIEVER_PIN; //ir reciever pin
const int servoPin      = SERVO_PIN; //servo motor pin
int iSpeed              = 255; //speed, range 0 to 255
#ifdef CONFIG_LED_DISPLAY
const int LedPin        = LED_PIN; 
#endif
//******Infrared key bindings********************************************
const long IRfront      = 0x00FF18E7; //go straight: button 2
const long IRback       = 0x00FF4AB5; //go back    : button 8
const long IRturnright  = 0x00FF5AA5; //turn right : button 6
const long IRturnleft   = 0x00FF10EF; //turn left  : button 4
const long IRstop       = 0x00FF38C7; //stop       : button 5
const long IRcny70      = 0x00FF906F; //CNY70 automatic mode: button EQ
const long IRAutorun    = 0x00FFC23D; //Ultrasonic mode : button play/pause
//******Track following pin assignments and signals**********************
const int SensorLeft    = TRACKING_SENSOR_LEFT_PIN; 
const int SensorMiddle  = TRACKING_SENSOR_MIDDLE_PIN;
const int SensorRight   = TRACKING_SENSOR_RIGHT_PIN;
IRrecv irrecv(irReceiverPin);  // IRrecv signal
decode_results infrared;       // decode result
//*******Ultrasonic pin assignments and signals**************************

Servo ultrasonicservo000; // define ultrasonicservo000
int degreesCount = 0;
unsigned int degreesForward = 90; //120; //nr degrees to look forward
const int degreesLeft       = 30;  //nr degrees to look left
const int degreesRight      = 150; //nr degrees to look right
//int predegreesServo000      = degreesForward;
const int MinDistance       = 10; //cm

const int Fgo = 8; // go straight
const int Rgo = 6; // turn right
const int Lgo = 4; // turn left
const int Bgo = 2; // go back
const int BBgo = 1; // go back back
//*****Bluetooth signals**************************************************
char val; //stores received character. Needs to be global to perform continuous movement


// see getDistance()
#define ULTRASONIC_000  0
#define ULTRASONIC_001  1

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
    //start receiving serial infor
    Serial.begin(9600);
    //motor connections
    pinMode(MotorRight1,   OUTPUT); //
    pinMode(MotorRight2,   OUTPUT); //
    pinMode(MotorLeft1,    OUTPUT); //
    pinMode(MotorLeft2,    OUTPUT); //
    pinMode(MotorRightPWM, OUTPUT); //enable for right side motor
    pinMode(MotorLeftPWM,  OUTPUT); //enable for right side motor
    
    //pinMode(7,INPUT);

    //IR reciever
    pinMode(irReceiverPin, INPUT);
    irrecv.enableIRIn();      // start infrared decode


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


#ifdef CONFIG_SOUND_SENSOR
    //sound sensor
    pinMode(SOUND_SENSOR_PIN,INPUT);
#endif    

#ifdef CONFIG_OBSTACLE_AVOIDANCE_SENSOR  
    //infrarared avoidance sensor
    pinMode(OBSTACLE_AVOIDANCE_SENSOR_PIN,INPUT);
#endif    

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


#ifdef CONFIG_IR_TEST
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
    digitalWrite(MotorLeft1, HIGH);
    digitalWrite(MotorLeft2, LOW);
    analogWrite(MotorRightPWM, iSpeed);
    analogWrite(MotorLeftPWM, iSpeed);
    delay(d * 10);
#else
    //go straight
    digitalWrite(MotorRight1, LOW);
    digitalWrite(MotorRight2, HIGH);
    digitalWrite(MotorLeft1, LOW);
    digitalWrite(MotorLeft2, HIGH);
    analogWrite(MotorRightPWM, iSpeed);
    analogWrite(MotorLeftPWM, iSpeed);  
    delay(d * 10);

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
    digitalWrite(MotorRight1, LOW);
    digitalWrite(MotorRight2, LOW);
    analogWrite(MotorRightPWM, iSpeed);
    analogWrite(MotorLeftPWM, iSpeed);
    delay(d * 10);
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
    digitalWrite(MotorLeft1, LOW);
    digitalWrite(MotorLeft2, LOW);
    analogWrite(MotorRightPWM, iSpeed);
    analogWrite(MotorLeftPWM, iSpeed);
    delay(d * 10);
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
    digitalWrite(MotorLeft1, LOW);
    digitalWrite(MotorLeft2, HIGH);
    analogWrite(MotorRightPWM, iSpeed);
    analogWrite(MotorLeftPWM, iSpeed);  
    delay(d * 10);
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
    digitalWrite(MotorLeft1, HIGH);
    digitalWrite(MotorLeft2, LOW);
    analogWrite(MotorRightPWM, iSpeed);
    analogWrite(MotorLeftPWM, iSpeed);
    delay(d * 10);
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
    digitalWrite(MotorLeft1, LOW);
    digitalWrite(MotorLeft2, LOW);
    analogWrite(MotorRightPWM, iSpeed);
    analogWrite(MotorLeftPWM, iSpeed);  
    delay(d * 10);
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
    digitalWrite(MotorLeft1, LOW);
    digitalWrite(MotorLeft2, HIGH);
    analogWrite(MotorRightPWM, iSpeed);
    analogWrite(MotorLeftPWM, iSpeed);  
    delay(d * 10);
#else
   //go back
    digitalWrite(MotorRight1, HIGH);
    digitalWrite(MotorRight2, LOW);
    digitalWrite(MotorLeft1, HIGH);
    digitalWrite(MotorLeft2, LOW);
    analogWrite(MotorRightPWM, iSpeed);
    analogWrite(MotorLeftPWM, iSpeed);
    delay(d * 10);
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
            
            // move motor position
            ultrasonicservo000.write(degrees);
            
            // if servo motor was moved, wait for the servo motor to become stable
            if(pre_degrees != degrees)
            {
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
 * @return      It returns Fgo = 8, Rgo = 6, Lgo = 4, Bgo = 2, BBgo = 1
 * @see         none
 */

int getDirectionFromdetection() 
{
    int Fspeedd = 0; // front distance
    int Rspeedd = 0; // right distance
    int Lspeedd = 0; // left distance
    
    int directionn =0;
#ifdef CONFIG_ULTRASONIC_001    
    int Fspeedd001 = 0;
#endif    
    int FspeeddTmp = 0;

    //get front distance
    Fspeedd    = getDistance(ULTRASONIC_000,degreesForward, 'F');
    FspeeddTmp = getDistance(ULTRASONIC_000,degreesForward, 'F');
    Fspeedd    = min(Fspeedd,FspeeddTmp);
#ifdef CONFIG_ULTRASONIC_001 
    /*get distance from fixed ultra sonic of 90 degree*/
    Fspeedd001 = getDistance(ULTRASONIC_001,90, 'F');
    Fspeedd    = min(Fspeedd,Fspeedd001);
#endif
    
#ifdef CONFIG_OBSTACLE_AVOIDANCE_SENSOR    
    FspeeddTmp = analogRead(OBSTACLE_AVOIDANCE_SENSOR_PIN);
    // 1023  ~ 1000 value means there is no obstacle
    // less than 10 means there is obstacle
    if(FspeeddTmp < 10)
    {
        Fspeedd = 9;
    }
#endif

    // if distance is less than 10 cm
    if (Fspeedd < 10) 
    {
        stopp(1); //  clear output
        directionn = Bgo; //go back
        
    }
    // if distance less than 20 cm
    else if (Fspeedd < 20) 
    {
        stopp(1); 
        
        Lspeedd   = getDistance(ULTRASONIC_000,degreesLeft, 'L'); // detection distance on left side
        FspeeddTmp= getDistance(ULTRASONIC_000,degreesLeft, 'L');
        Lspeedd    = min(Lspeedd,FspeeddTmp);
        
        Rspeedd   = getDistance(ULTRASONIC_000,degreesRight, 'R'); // detection distance on right side
        FspeeddTmp= getDistance(ULTRASONIC_000,degreesRight, 'R');
        Rspeedd    = min(Rspeedd,FspeeddTmp);

        if(Lspeedd < 15 && Rspeedd < 15 ) 
        {
            directionn = Bgo; //go back
        }
        else
        {
            // if left distance greater than right
            if (Lspeedd > Rspeedd) 
            {
                directionn = Lgo; // go left
            }
            else 
            {
                //if left distance less than right
                directionn = Rgo; //go right
            }
        }
    }
    else 
    {
        directionn = Fgo; //go straight
    }

    /*change forward direction to prevent corner problem*/
    degreesCount++;
    if(degreesCount <= 20)
    {
        degreesForward = 120;
        
    }
    else 
    {
        if(degreesCount >= 40)
        {
            degreesCount = 0;
        }
        degreesForward = 80;
    } 

 
    return directionn;
}
/**
 * This function runs car with using ultra sonic sensor automatically.
 *
 * @param       none
 * @return      none
 * @see         none
 */

void autoRunUsingUltraSonic() 
{
    bool stopPressed;
    int directionn = 0; // front=8, back=2, left=4, right=6
    while (IRAutorun) 
    {
        directionn  = getDirectionFromdetection(); 
       
#ifdef CONFIG_OBSTACLE_AVOIDANCE_TEST
        ; //NULL
#else
        stopPressed = stopCommandPressed();
        if (stopPressed) 
        {
            #ifdef CONFIG_DEBUG         
            Serial.print("\n-> Stop command ");
            #endif            
            break;
        }
#endif        
        /*check direction */
        if (directionn == Fgo) 
        { 
            #ifdef CONFIG_DEBUG            
            Serial.print(" ->Advance \n"); //
            #endif  
            //go straight
            advance(5);
           
        }
        else if (directionn == BBgo)
        {
            #ifdef CONFIG_DEBUG            
            Serial.print(" ->Back Back \n"); //
            #endif
            
            back(12);
            stopp(1);
        }
        else if (directionn == Bgo) 
        { 
            #ifdef CONFIG_DEBUG            
            Serial.print(" ->Back Left \n"); //
            #endif   
            #if 0
            //go back left :  180 degree will be roated by left
            back(8);
            turnL(3);
            #else
            back(8);
            turnL(1);
            #endif
          
        }
        else if (directionn == Rgo)
        { 
            #ifdef CONFIG_DEBUG            
            Serial.print(" ->Right \n"); //
            #endif  
            //turn right
            back(1);
            turnR(50);
           
        }
        else if (directionn == Lgo) 
        { 
            #ifdef CONFIG_DEBUG            
            Serial.print(" ->Left \n");
            #endif   
            //turn left
            back(1);
            turnL(50);
        }
    }
    infrared.value = 0;
   
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
    if (Serial.available()) 
    { 
        //check if bluetooth command available
        val = Serial.read();
        Serial.write(val);
    }

    if (val == 'F') 
    { 
        // Forward
        advance(10);
    }
    else if (val == 'S') 
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
        turnL(10);
    }
    else if (val == 'L') 
    { 
        // Left
        turnR(10);
    }
    else if (val == 's') 
    { 
        // Stop, not used though
        stopp(10);
    }
    else if (int(val) >= 49 && int(val) <= 57) 
    { 
        //set speed
        iSpeed = (int(val)-48)*26;
        Serial.println("Speed set to: " + iSpeed); 
    }
    else if (val == 'q') 
    { 
        //set speed
        iSpeed = 255;
#ifdef CONFIG_LED_DISPLAY    
        digitalWrite(LedPin,HIGH);   
#endif    
        Serial.println("Speed set to: " + iSpeed);  
    }
    else if (val == 'W') 
    {

#ifdef CONFIG_LED_DISPLAY 
        digitalWrite(LedPin,HIGH);
#endif    
    }
    else if (val == 'w') 
    {
#ifdef CONFIG_LED_DISPLAY   
        digitalWrite(LedPin,LOW);
#endif    
    }
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
    if (irrecv.decode(&infrared) == true) 
    {
        irrecv.resume(); //watch out for another message
        Serial.println(infrared.value, HEX);
        //if (infrared.bits > 0) 
        {
            if (infrared.value == IRstop) 
            {
                stopp(10);
                stopPressed = true;
            }
        }
        infrared.value = 0;
        //irrecv.resume(); //watch out for another message
    }
    infrared.value = 0;
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
    while (IRcny70) 
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
                Serial.print("->Go Right\n");
                #endif
                //stop to compensate inertia problem
                stopp(1);
                right(0);
            }
            else if (SL == HIGH && SR == LOW)  //middle: black, left black, right white
            {   
                #ifdef CONFIG_DEBUG
                Serial.print("->Go Left\n");
                #endif
                //stop to compensate inertia problem
                stopp(1);
                left(0);
            }
            else  //middle: black,  left and right both in white, go straight
            { 
                #ifdef CONFIG_DEBUG
                Serial.print("->Go Forward\n");
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
                Serial.print("->Go Right\n");
                #endif
                //stop to compensate inertia problem
                stopp(1);
                right(0);
            }
            else if (SL == HIGH && SR == LOW) //middle white, left black, right white
            {
                #ifdef CONFIG_DEBUG
                Serial.print("->Go Left\n");
                #endif
                //stop to compensate inertia problem
                stopp(1);
                left(0);
            }
            else if (SL == HIGH && SR == HIGH)
            {
                #ifdef CONFIG_DEBUG
                Serial.print("->Go Forward\n");
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
#if defined(CONFIG_OBSTACLE_AVOIDANCE_TEST)
    autoRunUsingUltraSonic();
    ultrasonicservo000.write(degreesForward);       // will make head look in front
#elif defined(CONFIG_BLACK_LINE_TRACKING_TEST)   
    followBlackLine();
#elif defined(CONFIG_SERVO_TEST)
    servo_test();
#elif defined(CONFIG_IR_TEST)
    ir_test();
#elif defined(CONFIG_BLUETOOTH_CMD_TEST)  
    bluetoothCommand();
#elif defined(CONFIG_SOUND_SENSOR_TEST)  
    Serial.print("\n Sound: ");
    Serial.print(SOUND_READ(SOUND_SENSOR_PIN));  
    delay(100);
#elif defined(CONFIG_OBSTACLE_AVOIDANCE_SENSOR_TEST)   
    Serial.print("\n Avoidance : ");
    //Serial.print(digitalRead(OBSTACLE_AVOIDANCE_SENSOR_PIN));  
    Serial.print(analogRead(OBSTACLE_AVOIDANCE_SENSOR_PIN));  
    delay(100);

#else

    /*main loop*/
#ifdef CONFIG_BLUETOOTH_CMD
  //bluetooth commands
  bluetoothCommand();
#endif  
  //************************************normal remote control mode ********
  // decoding success 'receive infrared signal
start:  
    if (irrecv.decode(&infrared) == true)  
    {         
        if (infrared.value == IRfront) 
        {
            Serial.print(">Go Strait ");
            advance(0); //go straigt
        }
        else if (infrared.value ==  IRback) 
        {
            Serial.print(">Go Back ");
            back(0); //go back
        }
        else if (infrared.value == IRturnright) 
        {
            Serial.print(">Go Rigth ");
            turnR(0); // go right
        }
        else if (infrared.value == IRturnleft) 
        {
            Serial.print(">Go Left ");
            turnL(0); // go left
        }
        else if (infrared.value == IRstop) 
        {
            //stop
            Serial.print(">Stop ");
            stopp(0);
        }
        //********************follow black line********cny70 automatic mode********
        else if (infrared.value == IRcny70) 
        {
            Serial.print(">black line ");
            followBlackLine();
        }
        //***********************ultrasonic automatic mode***************************
        else if (infrared.value == IRAutorun ) 
        {
            autoRunUsingUltraSonic();
            ultrasonicservo000.write(degreesForward);       // will make head look in front
        }
        //********************wait a little before continuing**************************
        irrecv.resume();
        delay(300);
    }
    else 
    {
        //Serial.print(">NO signal > Stop ");
        stopp(0);
    }
#endif    
}








