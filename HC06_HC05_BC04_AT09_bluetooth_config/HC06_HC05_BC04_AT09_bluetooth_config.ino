
/************************************************************************
 *   Copyright (c) by Sunghyun Ryu
 *
 *   Copyright is licensed under a
 *   Creative Commons Attribution-ShareAlike 4.0 International License.
 *
 *   You should have received a copy of the license along with this
 *   work. If not, see <http://creativecommons.org/licenses/by-sa/4.0/>
 *   @version : V1.0
 *   @author  : Sunghyun Ryu (smartmecard)
 *   @email   : kino21c@gmail.com
 *************************************************************************/                 
/************************************************************************
    HC-06/AT-09/HC-05/BC04 bluetooth configuration with aduino board.
    Change name, password and baud using serial port

    BT ---- Arduino
    TX ----> RX
    RX <---- TX
               
    This source code can be downloaded from 
    https://github.com/smartmecard/arduino-smart-robot-car
    
    1.Instruction
    1) connect arduino board pin 10 RX to bluetooth TX
    2) connect arduino board pin 11 TX to bluetooth RX
    3) connect arduino board GND to bluetooth GND
    4) connect arduino board 5V to bluetooth VCC
    5) compile this code in arduino IDE
    6) upload it with arduino IDE
    7) CTRL+Shift+M in arduino IDE
    8) Set Both NL & CR and 115200 baud
    9) serial debug message is as follows.

    Arduino Board Baud Rate : 115200
    BT(Bluetooth) Baud Rate : 9600
    -------------------------------------------------------
    0 : HC-06 BT Command Test Mode - without CR and LF
    1 : AT-09/ HC-05 / BC04-B BT Command Test Mode - with CR and LF
    2 : BT Baud Rate Change Mode
    3 : HC-06 : Read configuration
    4 : HC-06 : Write configuration
    5 : AT-09/ HC-05 / BC04-B : Read configuration
    Q or q : Escaped
    
    If you want to change communication baud rate between arduino and BT, use 2
    
    2. HC-06
    HC-05 default baud rate is 9600
    1) Enter 0
    2) Enter AT and it will return OK normally
    3) AT+BAUD8 for baud rate 115200
    4) AT+NAMEHC06_OTTO  for setting the name
    5) AT+PIN1234   for pairing password
    6) Enter q
    
    3. HC-05
    HC-05 default baud rate is 38400
    1) Enter 2
    2) Enter 38400
    3) Enter 1
    4) Enter AT and it will return OK normally
    5) AT+NAME=HC05_OTTO for setting the name
    6) AT+PSWD=1234 for pairing password
    7) AT+UART=57600,1,0  for baud rate 57600
    8) AT+POLAR=1,0  for enabling STATE pin to be used as reset for programming arduino over BT
    9) Enter q
    
    4. BC04
    BC04 default baud rate is 9600
    1) Enter 1
    2) Enter AT and it will return OK normally
    3) AT+NAMEBC04_OTTO "setting the name"
    5) AT+PIN1234 "pairing password"
    6) AT+BAUD7 for baud rate 57600
    7) Enter q
    
    
    
    5. Reference 
    1) https://www.hackster.io/ottoplus/otto-diy-33406c?team=30287
    2) http://42bots.com/tutorials/hc-06-bluetooth-module-datasheet-and-configuration-with-arduino
    3) https://medium.com/@yostane/using-the-at-09-ble-module-with-the-arduino-3bc7d5cb0ac2
    4) http://www.martyncurrey.com/bluetooth-modules/
   
*************************************************************************/

#include <SoftwareSerial.h> 



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
#define CONFIG_BT_SERIAL_RX    10 //Arduio Rx pin  <--> Bluetooth module TX
#define CONFIG_BT_SERIAL_TX    11 //Arduio Tx pin  <--> Bluetooth module RX





#define CONFIG_BT_SOFTWARE_BAUD_RATE    9600 //BT baud rate

//#define CONFIG_BT_AND_DEBUG_SERIAL_IS_SAME

#define CONFIG_BOARD_DEBUG_BAUD_RATE    115200 //arduino serial debug baud rate


//char ssid[]       = "HC06_4WD";   // Name for Bluetooth.
//char ssid[]       = "HC06_2WD";   // Name for Bluetooth.
//char ssid[]       = "HC06_Tank";   // Name for Bluetooth.
//char ssid[]       = "HC06_STank";   // Name for Bluetooth.
//char ssid[]       = "AT09_Test";   // Name for Bluetooth.
//char ssid[]       = "HC06_OTTO";   // Name for Bluetooth.
//char ssid[]       = "AT09_OTTO";   // Name for Bluetooth.
//char ssid[]       = "BC04_OTTO";   // Name for Bluetooth.
char ssid[]       = "HC06_Test";   // Name for Bluetooth.


//char baudios[]    = "4";           // 1=>1200 baudios, 2=>2400, 3=>4800, 4=>9600 (default), 5=>19200, 6=>38400, 7=>57600, 8=>115200
char baudios[]    = "8";           // 1=>1200 baudios, 2=>2400, 3=>4800, 4=>9600 (default), 5=>19200, 6=>38400, 7=>57600, 8=>115200
char password[]   = "1234";        // Password for pairing


/////////////////////////////////////////////////////////////////////////////////
#define CMD_STRING_MAX  (64)
SoftwareSerial btSerial(CONFIG_BT_SERIAL_RX, CONFIG_BT_SERIAL_TX);// RX, TX 
//SoftwareSerial BTSERIAL(CONFIG_BT_SERIAL_RX, CONFIG_BT_SERIAL_TX);// RX, TX 

long bt_baud_rate    = CONFIG_BT_SOFTWARE_BAUD_RATE;

#ifdef CONFIG_BT_AND_DEBUG_SERIAL_IS_SAME
#define BTSERIAL Serial
#else
#define BTSERIAL btSerial // SoftwareSerial btSerial(CONFIG_BT_SERIAL_RX, CONFIG_BT_SERIAL_TX);
#endif

typedef struct {
	long lBaudRate;	
    String strValue;
	String strBaudRateReplayMessage;

}HC06Table;

HC06Table HC06BaudRate[] = 
{
    {1200,  "1", "OK1200"  },
    {2400,  "2", "OK2400"  },
    {4800,  "3", "OK4800"  },    
    {9600,  "4", "OK9600"  },        
    {19200, "5", "OK19200" },            
    {38400, "6", "OK38400" },
    {57600, "7", "OK57600" },    
    {115200,"8", "OK115200"}
};

String Message;


/**
 * This function sends command to bluetooth module
 *
 * @param       char * cmd1 - command 
 * @param       char * cmd2 - sub data command
 * @param       int mode - 0 without \r\n,  1 with \r\n
 * @return      It returns response string
 * @see         none
 */

char reply[100];
char *sendBTCommand(const char * cmd1, const char * cmd2, int mode) 
{
    int i = 0;
    //debugging message
    Serial.print("Command send :");
    Serial.print(cmd1);
    Serial.print(cmd2);
    Serial.print("\r\n");


    /*send command
        AT
        AT+NAMExxxx
        AT+BAUDx
        AT+PINxxxx
    */
    if(mode == 0) 
    {   
        BTSERIAL.print(cmd1);
        if(cmd2 != NULL)
        {
            BTSERIAL.print(cmd2);
        }

    }
    else 
    {
        BTSERIAL.print(cmd1);
        if(cmd2 != NULL)
        {
            BTSERIAL.print(cmd2);
        }
        //AT-09/ HC-05 needs CR and LF(0xOD and 0x0A).
        BTSERIAL.print("\r\n");
    }
    
    
    //wait some time : The interval of command is about 1 second in case of HC-06
    delay(1000);

    Serial.print("Reply ");
    //read string is OK
    while (BTSERIAL.available()) 
    {
        reply[i++] = BTSERIAL.read();
        //Serial.print(reply[i],HEX);
        if(reply[i-1] == '\r' || reply[i-1] == '\n')
        {
            i--;
        }
    }

    Message = String(i);
    Message += ": ";
    Serial.print(Message);
    
    //end the string
    reply[i] = '\0';
    //debugging message
    Serial.println(reply);
    delay(50);
    
    return reply;
}

/**
 * This function write configuration to HC-06 bluetooth module
 *
 * @param       none
 * @return      none
 * @see         none
 */

void writeBTConfig(void)
{
    String strReply="";
    int table_size = 0;
    int i = 0;
    char cmdString[CMD_STRING_MAX];
    table_size = sizeof(HC06BaudRate)/sizeof(HC06Table);
    
    // Now configuration start
    strReply = (String)sendBTCommand("AT",NULL,0);
    if(strReply == "OK")
    {
        ;//skip
    }
    else
    {
        Serial.println("ERROR: BT Baud rate is different from HC-06");
        showBTBaudRate();   
        return;
    
    }
    // Change Name of BT: AT+NAMExxx e.g. AT+NAMEHC06
    Serial.println("AT+NAMExxx : Enter name string(Max 20)");
    while(1)
    {
        cmdString[0] = '\0';
        if(getCmdString(cmdString,CMD_STRING_MAX) > 1)
        {
            sendBTCommand("AT+NAME",cmdString,0); 
            break;
        }
        
        if(cmdString[0] == 'q' || cmdString[0] == 'Q')
        {
            break;
        }
    }
    
    // Change Password : AT+PINxxxx  e.g. AT+PIN1234  
    Serial.println("AT+PINxxxx : Enter pin string(Digit 4)");
    while(1)
    {
        cmdString[0] = '\0';
        if(getCmdString(cmdString,CMD_STRING_MAX) > 1)
        {
            sendBTCommand("AT+PIN",cmdString,0); 
            break;
        }

        if(cmdString[0] == 'q' || cmdString[0] == 'Q')
        {
            break;
        }

    }

    // Change Baud : AT+BAUDx  e.g. AT+BAUD4
    //1=>1200 baudios, 2=>2400, 3=>4800, 4=>9600 (default), 5=>19200, 6=>38400, 7=>57600, 8=>115200
set_retry_baudrate:    
    Serial.println("AT+BAUDx : Enter 1~8 (4=>9600,7=>57600,8=>115200)");
    //Serial.println("1=>1200, 2=>2400, 3=>4800,4=>9600(default),5=>19200,6=>38400,7=>57600,8=>115200");
    while(1)
    {
        cmdString[0] = '\0';
        if(getCmdString(cmdString,CMD_STRING_MAX) > 0)
        {
            //set baud rate command
            for(i = 0; i < table_size ; i++)
            {
                if((String)cmdString == HC06BaudRate[i].strValue)
                {
                    strReply = (String)sendBTCommand("AT+BAUD",cmdString,0); 
                    break;
                }
            } 
            if(i == table_size)
            {
                Serial.println("ERROR: invalid value!!");
                goto set_retry_baudrate;

            }

            for(i = 0; i < table_size ; i++)
            {
                if(strReply == HC06BaudRate[i].strBaudRateReplayMessage)
                {
                    bt_baud_rate = HC06BaudRate[i].lBaudRate; 
                    BTSERIAL.begin(bt_baud_rate);    
                    break;
                }
            } 
            break;
        }

        if(cmdString[0] == 'q' || cmdString[0] == 'Q')
        {
            break;
        }

    }
    
    
    if(i == table_size)
    {
        Serial.println("ERROR: Check BT Baud Rate to write!!!");
    }
    showBTBaudRate();
}

/**
 * This function get string from arduino uart
 *
 * @param       char *pStr
 * @param       int  len
 * @return      it returns string length
 * @see         none
 */
int getCmdString(char *pStr, int len)
{
    int i = 0;
    if(pStr == NULL || len < 2)
    {
        return i;
    }
    
    while(Serial.available())
    {
    
        pStr[i++] = Serial.read();
        //CR , LF
        if(pStr[i-1] == '\r' || pStr[i-1] == '\n')
        {
            i--;
        }
        //buffer is full
        if(i >= (len-1))
        {
            i = (len-1);
            break;
        }
        //wait for 100ms to assure reading data
        delay(100);                    
    }
    
    if(i >= 1)
    {
        pStr[i] = '\0';       
    }

    return i;
}

void showCmdHelp()
{
    Message   = "-------------------------------------------------------";
    Serial.println(Message);

    Message   = "0 : HC-06 BT Command Test Mode - without CR and LF";
    Serial.println(Message);

    Message   = "1 : AT-09/ HC-05 / BC04-B BT Command Test Mode - with CR and LF";
    Serial.println(Message);

    Message   = "2 : BT Baud Rate Change Mode";
    Serial.println(Message);

    Message   = "3 : HC-06 : Read configuration";
    Serial.println(Message);

    Message   = "4 : HC-06 : Write configuration";
    Serial.println(Message);

    Message   = "5 : AT-09/ HC-05 / BC04-B : Read configuration";
    Serial.println(Message);

    Message   = "Q or q : Escaped";
    Serial.println(Message);    
}

void showBTBaudRate()
{
    Message = "Current BT Baud Rate : ";
    Message += String(bt_baud_rate);
    Serial.println(Message);
}

//--------------------------------------------------------
// setup
//--------------------------------------------------------
void setup()
{
    //to solve low memory problem by string memory
    Message.reserve(CMD_STRING_MAX);

    //debug serial
    Serial.begin(CONFIG_BOARD_DEBUG_BAUD_RATE);
    
    Message = "Arduino Board Baud Rate : ";    
    Message += String(CONFIG_BOARD_DEBUG_BAUD_RATE);
    Serial.println(Message);

    //bt serial
    BTSERIAL.begin(bt_baud_rate);
    
    Message = "BT(Bluetooth) Baud Rate : ";
    Message += String(bt_baud_rate);
    Serial.println(Message);

    //show help
    showCmdHelp();
}

//--------------------------------------------------------
// main loop
//--------------------------------------------------------
void loop()
{
    char c=0;
    char cmdString[CMD_STRING_MAX];    
    int i = 0;
    String strCmd;
    long iTmp = 0;


    cmdString[0] = '\0';
    i = getCmdString(cmdString,CMD_STRING_MAX);
    if(i == 0)
    {
        return;
    }

    //check command and run command
    if(cmdString[0] == '0' &&  i == 1)
    {
        Message   = "-------------------------------------------------------";
        Serial.println(Message);

        Message   = "0 : HC-06 BT Command Test Mode - without CR and LF";
        Serial.println(Message);
        showBTBaudRate();
        while(1)
        {
            i = 0;
            cmdString[i] = '\0';

            i = getCmdString(cmdString,CMD_STRING_MAX);
            
            if(cmdString[0] == 'Q' || cmdString[0] == 'q')
            {
                Message   = "Q or q : Escaped";
                Serial.println(Message);  
                break;
            }
            
            if(i >= 1)
            {
                sendBTCommand(cmdString,NULL,0);
            }
        }
        Message   = "-------------------------------------------------------";
        Serial.println(Message);

    }
    else if(cmdString[0] == '1' &&  i == 1)
    {
        Message   = "-------------------------------------------------------";
        Serial.println(Message);
        
        Message   = "1 : AT-09/ HC-05 / BC04-B BT Command Test Mode - with CR and LF";
        Serial.println(Message);
        showBTBaudRate();

        while(1)
        {
            i = 0;
            cmdString[i] = '\0';

            i = getCmdString(cmdString,CMD_STRING_MAX);
            
            if(cmdString[0] == 'Q' || cmdString[0] == 'q')
            {
                
                Message   = "Q or q : Escaped";
                Serial.println(Message);  
                break;
            }
            
            if(i >= 1)
            {
                sendBTCommand(cmdString,NULL,1);
            }
        } 
        Message   = "-------------------------------------------------------";
        Serial.println(Message);

    }
    else if(cmdString[0] == '2' &&  i == 1)
    {
        Message   = "-------------------------------------------------------";
        Serial.println(Message);

        Message   = "2 : BT Baud Rate Change Mode";
        Serial.println(Message);
        showBTBaudRate();
        Message   = "Enter BT Baud Rate...(9600,57600, 115200...)";
        Serial.println(Message);
        while(1)
        {
            i = 0;
            cmdString[i] = '\0';

            i = getCmdString(cmdString,CMD_STRING_MAX);
            if(cmdString[0] == 'Q' || cmdString[0] == 'q')
            {
                
                Message   = "Q or q : Escaped";
                Serial.println(Message);  
                break;
            }
            
            if(i >= 1)
            {
                strCmd = cmdString;
                iTmp = strCmd.toInt();
                Serial.println("Changed BT Baud Rate as "+strCmd);

                bt_baud_rate = iTmp;

                //set bt serial
                BTSERIAL.begin(bt_baud_rate);
                break;

            }

        }    
        Message   = "-------------------------------------------------------";
        Serial.println(Message);

    }
    else if(cmdString[0] == '3' &&  i == 1)
    {
        Message   = "-------------------------------------------------------";
        Serial.println(Message);
        
        Message   = "3 : HC-06 : Read configuration";
        Serial.println(Message);
        showBTBaudRate();
        
        sendBTCommand("AT",NULL,0);
        sendBTCommand("AT+VERSION",NULL,0);
        sendBTCommand("AT+NAME",NULL,0); 
        Message   = "-------------------------------------------------------";
        Serial.println(Message);

    }
    else if(cmdString[0] == '4' &&  i == 1)
    {
        Message   = "-------------------------------------------------------";
        Serial.println(Message);
        
        Message   = "4 : HC-06 : Write configuration";
        Serial.println(Message);
        showBTBaudRate();

        writeBTConfig();
        Message   = "-------------------------------------------------------";
        Serial.println(Message);

    }
    else if(cmdString[0] == '5' &&  i == 1)
    {
        Message   = "-------------------------------------------------------";
        Serial.println(Message);

        Message   = "5 : AT-09/ HC-05 / BC04-B : Read configuration";
        Serial.println(Message);
        showBTBaudRate();
        
        sendBTCommand("AT",NULL,1);
        sendBTCommand("AT+VERSION",NULL,1);
        sendBTCommand("AT+BAUD",NULL,1); 
        sendBTCommand("AT+ROLE",NULL,1);         
        sendBTCommand("AT+UUID",NULL,1);         
        sendBTCommand("AT+CHAR",NULL,1);         
        sendBTCommand("AT+NAME",NULL,1);   
        Message   = "-------------------------------------------------------";
        Serial.println(Message);

    }
    else
    {
        showCmdHelp();
    }
    
}


