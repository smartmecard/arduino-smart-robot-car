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

/*
    Note: 
    This original source code is from https://forum.arduino.cc/index.php?topic=312570.msg2166317#msg2166317
    This code was fixed by kino21c@gmail.com.
    This code can be downloaded from http://smartmecard.wordpress.com.
    Have a good day with your child. :)
   
*/
#include <IRremote.h>
#include <Servo.h>

//#define CONFIG_LED_DISPLAY
//#define CONFIG_DIRECTION_CNTRL_001
//#define CONFIG_DEBUG
//************************************************
//*Arduino Uno :  Pin defintion configuration 
//************************************************
#define ENA_PIN   6
#define EN1_PIN   7
#define EN2_PIN   3

#define EN3_PIN   4
#define EN4_PIN   2
#define ENB_PIN   5

#define IR_RECIEVER_PIN A3 //A0
#define SERVO_PIN   9 // servo motor pin number

#define ULTRASONIC_ECHO_PIN  13
#define ULTRASONIC_TRIG_PIN  12

#define TRACKING_SENSOR_LEFT_PIN     A2 //2    
#define TRACKING_SENSOR_MIDDLE_PIN   A1 //8 
#define TRACKING_SENSOR_RIGHT_PIN    A0 //11 
#ifdef CONFIG_LED_DISPLAY
#define LED_PIN 0
#endif
//Pin assignments and global variables per function. Customize if needed
//*******Pin assignments Motor board and IR receiver********************
const int MotorRight1   = EN1_PIN;
const int MotorRight2   = EN2_PIN; 
const int MotorLeft1    = EN3_PIN; 
const int MotorLeft2    = EN4_PIN;
const int MotorRightPWM = ENA_PIN;
const int MotorLeftPWM  = ENB_PIN;
const int irReceiverPin = IR_RECIEVER_PIN;
const int servoPin = SERVO_PIN; 
int iSpeed = 255; //speed, range 0 to 255
#ifdef CONFIG_LED_DISPLAY
const int LedPin=LED_PIN; 
#endif
//******Infrared key bindings********************************************
const long IRfront      = 0x00FF18E7;     //go straight: button 2
const long IRback       = 0x00FF4AB5;       //go back    : button 8
const long IRturnright  = 0x00FF5AA5;  //turn right : button 6
const long IRturnleft   = 0x00FF10EF;   //turn left  : button 4
const long IRstop       = 0x00FF38C7;       //stop       : button 5
const long IRcny70      = 0x00FF906F;      //CNY70 automatic mode: button EQ
const long IRAutorun    = 0x00FFC23D; //0x00FFC23D;  //Ultrasonic mode : button play/pause
//******Track following pin assignments and signals**********************
const int SensorLeft    = TRACKING_SENSOR_LEFT_PIN  ; 
const int SensorMiddle  = TRACKING_SENSOR_MIDDLE_PIN ;
const int SensorRight   = TRACKING_SENSOR_RIGHT_PIN;
IRrecv irrecv(irReceiverPin);  // IRrecv signal
decode_results infrared;       // decode result
//*******Ultrasonic pin assignments and signals**************************
const int echoPin       = ULTRASONIC_ECHO_PIN; // ultrasonic receive=echo pin
const int triggerPin    = ULTRASONIC_TRIG_PIN; // ultrasonic send=trigger pin
Servo myservo; // define myservo
const int degreesForward    = 130; //nr degrees to look forward
const int degreesLeft       = 60; //nr degrees to look left
const int degreesRight      = 180; //nr degrees to look right
const int delay_time        = 250; // servo motor delay
const int MinDistance  = 12; //cm

const int Fgo = 8; // go straight
const int Rgo = 6; // turn right
const int Lgo = 4; // turn left
const int Bgo = 2; // go back
//*****Bluetooth signals**************************************************
char val; //stores received character. Needs to be global to perform continuous movement

//*********General SETUP: activate pins***********************************
void setup() {
  //start receiving serial infor
  Serial.begin(9600);
  //motor connections
  pinMode(MotorRight1, OUTPUT);  //
  pinMode(MotorRight2, OUTPUT);  //
  pinMode(MotorLeft1,  OUTPUT);  //
  pinMode(MotorLeft2,  OUTPUT);  //
  pinMode(MotorRightPWM, OUTPUT); //enable for right side motor
  pinMode(MotorLeftPWM, OUTPUT); //enable for right side motor
  
  pinMode(irReceiverPin, INPUT);
  irrecv.enableIRIn();      // start infrared decode
  
  myservo.write(degreesForward);       // will make head look in front

  //black track following
  pinMode(SensorLeft, INPUT);
  pinMode(SensorMiddle, INPUT);
  pinMode(SensorRight, INPUT);

  //Ultra sonic
  //digitalWrite(2,HIGH); //what is this pin for?
  pinMode(echoPin, INPUT);
  pinMode(triggerPin, OUTPUT);
  myservo.attach(servoPin);
}

//**************Movement functions******************************
#ifdef CONFIG_DIRECTION_CNTRL_001
void advance(int d) 
{ //go straight
  digitalWrite(MotorRight1, HIGH);
  digitalWrite(MotorRight2, LOW);
  digitalWrite(MotorLeft1, HIGH);
  digitalWrite(MotorLeft2, LOW);
  analogWrite(MotorRightPWM, iSpeed);
  analogWrite(MotorLeftPWM, iSpeed);
  delay(d * 10);
}
#else
void advance(int d) 
{ //go straight
    digitalWrite(MotorRight1, LOW);
    digitalWrite(MotorRight2, HIGH);
    digitalWrite(MotorLeft1, LOW);
    digitalWrite(MotorLeft2, HIGH);
    analogWrite(MotorRightPWM, iSpeed);
    analogWrite(MotorLeftPWM, iSpeed);  
    delay(d * 10);
}
#endif
void right(int d) 
{ //turn right (single wheel)
  digitalWrite(MotorLeft1, LOW);
  digitalWrite(MotorLeft2, HIGH);
  digitalWrite(MotorRight1, LOW);
  digitalWrite(MotorRight2, LOW);
  analogWrite(MotorRightPWM, iSpeed);
  analogWrite(MotorLeftPWM, iSpeed);
  delay(d * 10);
}
void left(int d) 
{//turn left(single wheel)
  digitalWrite(MotorRight1, LOW);
  digitalWrite(MotorRight2, HIGH);
  digitalWrite(MotorLeft1, LOW);
  digitalWrite(MotorLeft2, LOW);
  analogWrite(MotorRightPWM, iSpeed);
  analogWrite(MotorLeftPWM, iSpeed);
  delay(d * 10);
}
void turnR(int d) 
{//turn right (two wheels)
  digitalWrite(MotorRight1, HIGH);
  digitalWrite(MotorRight2, LOW);
  digitalWrite(MotorLeft1, LOW);
  digitalWrite(MotorLeft2, HIGH);
  analogWrite(MotorRightPWM, iSpeed);
  analogWrite(MotorLeftPWM, iSpeed);  
  delay(d * 10);
}
void turnL(int d) 
{//turn left (two wheels)
  digitalWrite(MotorRight1, LOW);
  digitalWrite(MotorRight2, HIGH);
  digitalWrite(MotorLeft1, HIGH);
  digitalWrite(MotorLeft2, LOW);
  analogWrite(MotorRightPWM, iSpeed);
  analogWrite(MotorLeftPWM, iSpeed);
  delay(d * 10);
}
void stopp(int d) 
{ //stop
  digitalWrite(MotorRight1, LOW);
  digitalWrite(MotorRight2, LOW);
  digitalWrite(MotorLeft1, LOW);
  digitalWrite(MotorLeft2, LOW);
  analogWrite(MotorRightPWM, iSpeed);
  analogWrite(MotorLeftPWM, iSpeed);  
  delay(d * 10);
}
#ifdef CONFIG_DIRECTION_CNTRL_001
void back(int d) 
{ 
  //go back
  digitalWrite(MotorRight1, LOW);
  digitalWrite(MotorRight2, HIGH);
  digitalWrite(MotorLeft1, LOW);
  digitalWrite(MotorLeft2, HIGH);
  analogWrite(MotorRightPWM, iSpeed);
  analogWrite(MotorLeftPWM, iSpeed);  
  delay(d * 10);
}
#else
void back(int d) 
{ 
   //go back
    digitalWrite(MotorRight1, HIGH);
    digitalWrite(MotorRight2, LOW);
    digitalWrite(MotorLeft1, HIGH);
    digitalWrite(MotorLeft2, LOW);
    analogWrite(MotorRightPWM, iSpeed);
    analogWrite(MotorLeftPWM, iSpeed);
    delay(d * 10);

}

#endif

//************Ultrasonic distance calculator*************************************
//detect distance for given angles and print char + direction
//it returns cm.
float getDistance(int degrees, char dir) 
{
    float distance;

    myservo.write(degrees);
    digitalWrite(triggerPin, LOW); // ultrasonic echo low level in 2us
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH); // ultrasonic echo high level in 10us, at least 10us
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW); // ultgrasonic echo low level
    distance = pulseIn(echoPin, HIGH); // read time
    /*
        speed of sound is 340 m/sec or 29 us / cm
        distance = distance / 29 /2 = distance / 5.8/ 10
                 = distance / 58
    */
    distance = distance / 58; // turn time to distance
#ifdef CONFIG_DEBUG    
    Serial.print(dir); // 
    Serial.print(" distance: "); // 
    Serial.print(int(distance)); //  output distance (mm)
    Serial.print("\n");
#endif    
    return distance;
}
//*************Ultrasonic direction decision making******************************
//measurements three angles (front, left, right
int getDirectionFromdetection() 
{
    int Fspeedd = 0; // front distance
    int Rspeedd = 0; // right distance
    int Lspeedd = 0; // left distance
    int delay_time = 250; //
    int directionn =0;

    //get front distance
    Fspeedd = getDistance(degreesForward, 'F');

    // if distance is less than 12 cm
    if (Fspeedd < 12) 
    {
        stopp(1); //  clear output
        directionn = Bgo;
    }
    // if distance less than 25 cm
    else if (Fspeedd < 25) 
    {
        stopp(1); 
        
        Lspeedd = getDistance(degreesLeft, 'L'); // detection distance on left side
        delay(delay_time); // waiting for the servo motor to become stable

        Rspeedd = getDistance(degreesRight, 'R'); // detection distance on right side
        delay(delay_time); // waiting for servo motor to be stable

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

        if (Lspeedd < 15 && Rspeedd < 15) 
        {
            //if distance less 10cm both right and left
            directionn = Bgo; //go back
        }
    }
    else 
    {
        directionn = Fgo; //go straight
    }
    return directionn;
}

void autoRunUsingUltraSonic() 
{
    bool stopPressed;
    int directionn = 0; // front=8, back=2, left=4, right=6
    while (IRAutorun) 
    {
        myservo.write(80); // make the servo motor reset
        //myservo.write(90); // make the servo motor reset
        directionn  = getDirectionFromdetection();
        stopPressed = stopCommandPressed();
        if (stopPressed) 
        {
#ifdef CONFIG_DEBUG         
            Serial.print("\r\n-> Stop command ");
#endif            
            break;
        }
        else if (directionn == Fgo) 
        { //go straight
            //infrared.value = 0;
            advance(5); //
#ifdef CONFIG_DEBUG            
            Serial.print(" ->Advance "); //
#endif            
        }
        else if (directionn == Bgo) 
        { //go back
            //infrared.value = 0;
            back(8); //
            turnL(3); //
#ifdef CONFIG_DEBUG            
            Serial.print(" ->Reverse "); //
#endif            
        }
        else if (directionn == Rgo)
        { //turn right
            //infrared.value = 0;
            back(1);
            turnR(60); //
#ifdef CONFIG_DEBUG            
            Serial.print(" ->Right "); //
#endif            
        }
        else if (directionn == Lgo) 
        { //turn left
            //infrared.value = 0;
            back(1);
            turnL(60);
#ifdef CONFIG_DEBUG            
            Serial.print(" ->Left ");
#endif            
        }
    }
    infrared.value = 0;
}

//*************************Bluetooth functionality***********************
//Bluetooth commands
void bluetoothCommand() 
{
  if (Serial.available()) { //check if bluetooth command available
    val = Serial.read();
    Serial.write(val);
  }
  if (val == 'F') { // Forward
    advance(10);
  }
  else if (val == 'S') { // Stop Forward
    stopp(10) ;
    val = Serial.read(); //read value again, otherwise can't continu with infrared
  }
  else if (val == 'B') { // Backwards
    back(10);
  }
  else if (val == 'R') { // Right
    turnL(10);
  }
  else if (val == 'L') { // Left
    turnR(10);
  }
  else if (val == 's') { // Stop, not used though
    stopp(10 ) ;
  }
  else if (int(val) >= 49 && int(val) <= 57) 
  { //set speed
    iSpeed = (int(val)-48)*26;
    Serial.println("Speed set to: " + iSpeed); 
  }
  else if (val == 'q') { //set speed
    iSpeed = 255;
#ifdef CONFIG_LED_DISPLAY    
    digitalWrite(LedPin,HIGH);   
#endif    
    Serial.println("Speed set to: " + iSpeed);  
  }
  else if (val == 'W') {

#ifdef CONFIG_LED_DISPLAY 
    digitalWrite(LedPin,HIGH);
#endif    
  }
  else if (val == 'w') {
#ifdef CONFIG_LED_DISPLAY   
    digitalWrite(LedPin,LOW);
#endif    
  }
}

//Check if stop command on remote is pressed (button 5)
bool stopCommandPressed()
{
    bool stopPressed = false;
    if (irrecv.decode(&infrared)) 
    {
        //irrecv.resume(); //watch out for another message
        Serial.println(infrared.value, HEX);
        if (infrared.bits > 0) 
        {
            if (infrared.value == IRstop) 
            {
                stopp(10);
                stopPressed = true;
            }
        }
        infrared.value = 0;
        irrecv.resume(); //watch out for another message
    }
    return stopPressed;
}

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
        
        sprintf(szTmp,"\r\n[%d,%d,%d]",SL,SM,SR);
        Serial.println(szTmp);
        //middle sensor in black area
        if (SM == HIGH) 
        {
            
            if (SL == LOW && SR == HIGH) 
            {   
                //left sensor in black area, right sensor in white, turn left
                left(50);
            }
            else if (SR == LOW && SL == HIGH) 
            {   
                //left white, right black, run right
                right(50);
            }
            else 
            { // left and right both in white, go straight
                advance(50);
            }
        }
        // middle sensor in white area
        else 
        {
            if (SL == LOW && SR == HIGH) 
            { 
                // left black ,right white, turn left
                left(50);
            }
            else if (SR == LOW && SL == HIGH) 
            {
                right(50);
            }
            else 
            {  
                //left and right both in white, stop
                stopp(50);
            }
        }
        
        stopPressed = stopCommandPressed();
        if (stopPressed) 
        {
            break;
        }
    }
    infrared.value = 0;  
}

//**************************************MAIN LOOP***************************************
void loop() 
{
  //bluetooth commands
  //bluetoothCommand();
  //************************************normal remote control mode ********
  // decoding success 'receive infrared signal
  
    if (irrecv.decode(&infrared))  
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
        {//stop
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
            myservo.write(degreesForward);       // will make head look in front
        }
        //********************wait a little before continuing**************************
        irrecv.resume();
        delay(300);
    }
    else 
    {
#if 1
        stopp(0);
#else
        autoRunUsingUltraSonic();
        myservo.write(degreesForward);       // will make head look in front
#endif
    }
}








