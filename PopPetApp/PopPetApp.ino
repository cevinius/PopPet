/*
  Serial Robot Interface for PopPet
  
  This program receives commands via serial, parses it, and uses these to control a robot.
  The commands will include:
      - Telling the robot move one of its motors
      - Tell the robot to take an ultrasonic sensor reading and report back on it
  There are also responses that the robot will transmit back:
      - To confirm receipt of commands
      - To report back on sensor readings

  These are the commands we'll parse and interpret. In general commands:
      - Start with '<'
      - Have a 2-character string to indicate the command
      - Have up to 3 numeric parameters
      - Parameters are separated by ','
      - Ends with '>'
      
  I.e. <CMD>, <CMD,#>, <CMD,#,#>, <CMD,#,#,#>
  
  NOTE: Characters are capitals, and there are NO spaces!
  
  
  Status Commands
  ===============
  
  Ready check - The board just echoes back the <OK>. This is to check the board is booted and ready.
    <OK>
    
    
  Movement Commands
  =================

  Drive Wheels - Set speed/direction of both motors
    <DW,#1,#2>, Where #1 is -255 to 255 for the left motor. -ve backwards, 0 stop, +ve forward
                Where #2 is -255 to 255 for the right motor. -ve backwards, 0 stop, +ve forward
                
    The response string will be the command echoed back. If the values were clamped, the response
    string will have the clamped values.
                
  Left Wheel - Set speed/direction of the left motor. (Right motor's speed/dir is not modified.)
    <LW,#1>, Where #1 is -255 to 255 for the left motor. -ve backwards, 0 stop, +ve forward
    
    The response string will be the command echoed back. If the values were clamped, the response
    string will have the clamped values.
                
  Right Wheel - Set speed/direction of the right motor. (Left motor's speed/dir is not modified.)
    <RW,#1>, Where #1 is -255 to 255 for the right motor. -ve backwards, 0 stop, +ve forward
    
    The response string will be the command echoed back. If the values were clamped, the response
    string will have the clamped values.
    
                
  Sensor Commands
  ===============
  
  Ultrasonic Sensor Activate/Deactivate - Use this to turn the ultrasonic sensor on/off
          If a sensor is on, it will take readings regularly and filter the results. That way,
          when a reading is requested using <US>, a filtered value will always be ready and the
          call will return right away. The response string is the command echoed back.
    <UA,#1>, Where #1 is 1 for on, and 0 for off
  
  Ultrasonic Sensor - Request a reading from the Ultrasonic sensor in CM. Rounded to nearest whole CM.
                      If the sensor has been turned on, it will return the latest filtered reading.
                      If the sensor has not been turned on, it will retrieve a raw value immediately and return that.
                      (Raw values may be noisy and contain spurious readings... Filtered is safer to use!)
    <US>
    
    The response string here will contain the sensor reading! The format will be:
        <US,#1>, where #1 is the sensor reading in rounded whole CM.


  Explore Mode
  ============
  
  Turn explore mode on/off. While explore mode is on, all commands except "XA" and "US" are ignored until
  explore mode is turned off. (i.e. <XA,0> is accepted after explore mode is on. <US> to get readings.)
  
    <XA,#1>, where #1 is 1 for on, and 0 for off.
  
  While epxlore mode is on, the following response codes are sent so that the controller
  can provide feedback and react.
    <XO>, Detected an obstacle
    <XC>, Changing direction at whim
    
 */

//-------------------------------------------------------------------------------
// Arduino Setup Constants and Pins

// Speed of serial communication with Bluetooth Board.
// Needs to match what your board is set to!
#define SERIAL_SPEED 9600

// H-Bridge Pins
#define LEFT_MOTOR_DIR_PIN  7
#define LEFT_MOTOR_PWM_PIN  9
#define RIGHT_MOTOR_DIR_PIN 8
#define RIGHT_MOTOR_PWM_PIN 10

// Pins for front Ultrasonic Sensor
#define TRIGGER_PIN  2
#define ECHO_PIN     3

// The directions in which the motors are wired... If a wheel is 
// going backwards, change this from 1 to -1.
// NOTE: When building as per PopPet instructions, LEFT needs to be -1.
#define LEFT_MOTOR_WIRE_DIR   -1
#define RIGHT_MOTOR_WIRE_DIR  1


//---------------------------------------------------------------------------------
// Constants <UA> command for turning ultrasonic sensor on/off.
// Used in parser commands and responses.

#define US_START 1
#define US_STOP  0


//-------------------------------------------------------------------------------
// Parser Variables

// Constant delimiter tag chars
const char START_CHAR = '<';
const char END_CHAR   = '>';
const char SEP_CHAR   = ',';

// Constants and a variable for the parser state.
const int PARSER_WAITING = 0; // Waiting for '<' to start parsing.
const int PARSER_COMMAND = 1; // Reading the command string.
const int PARSER_PARAM1  = 2; // Reading param 1.
const int PARSER_PARAM2  = 3; // Reading param 2.
const int PARSER_PARAM3  = 4; // Reading param 3.
const int PARSER_EXECUTE = 5; // Finished parsing a command, so execute it.

// Current parser state.
int currParserState = PARSER_WAITING; 

// String for storing the command. 2 chars for the command and 1 char for '\0'.
// We store the command here as we're parsing.
char currCmd[3] = "--";

// For tracking which letter we are in the command.
int currCmdIndex;

// Max command length.
const int CMD_LENGTH = 2;


// Current param values. Store them here after we parse them.
int currParam1Val;
int currParam2Val;
int currParam3Val;

// Variable for tracking which digit we're parsing in a param.
// We use this to convert the single digits back into a decimal value.
int currParamIndex;

// Whether the current param is negative.
boolean currParamNegative;

// Max parameter length. Stop parsing if it exceeds this.
const int MAX_PARAM_LENGTH = 6;


//---------------------------------------------------------------------------------
// Setup the ultrasonic sensors

// Milliseconds between reading each ultrasonic sensor
unsigned long MILLI_BETWEEN_READINGS = 50;

// After starting it waits this long before first reading.
unsigned long MILLI_BEFORE_FIRST_READING = 50; 

// Milliseconds at which to reading that sensor again.
unsigned long milliToPerformNextUSReading;

// Whether we are taking readings on the sensor (i.e. whether it's activated.)
boolean usSensorActivated = false;

// Current filtered ultrasonic reading.
float currFilteredUSValue = 0;


//-------------------------------------------------------------------------------
// Explore mode.

// Whether explore mode is currently on
boolean exploreModeActivated = false;

// Milliseconds at which to change explore mode state.
unsigned long milliToChangeExploreModeState;

// Curr speed for driving until an obstacle
int currExploreDriveSpeed;

// Curr state of the explore state machine.
int currExploreState;

// Constant states for the explore mode.
#define XM_BACKUP       1
#define XM_BACKUP_2     2
#define XM_CHOOSE_DIR   3
#define XM_CHOOSE_DIR_2 4
#define XM_FIND_GAP     5
#define XM_DRIVE_UNTIL_OBSTACLE 6
#define XM_WAIT_NEW_DIR         7

// Distance we use to determine when we find an obstacle.
#define EXPLORE_OBSTACLE_DISTANCE 25

//-------------------------------------------------------------------------------


void setup() 
{
    //---------------------------
    // Arduino Setup
    //---------------------------
    
    // Setup the main serial port.
    Serial.begin(SERIAL_SPEED);
  
    // Setup the Motor Controller pins
    pinMode( LEFT_MOTOR_DIR_PIN, OUTPUT);
    pinMode( LEFT_MOTOR_PWM_PIN, OUTPUT);
    pinMode( RIGHT_MOTOR_DIR_PIN, OUTPUT);
    pinMode( RIGHT_MOTOR_PWM_PIN, OUTPUT);
    
    // Set up ultrasonic sensor pins 
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT); 
 
    //---------------------------
    // Parser Setup
    //---------------------------
    
    // Wait for first command.
    currParserState = PARSER_WAITING;
    
    // Print this response to say we've booted and are ready.
    Serial.println("<OK>");
    
    //---------------------------
    // Ultrasonic Sensor Setup
    //---------------------------
    
    // Save the time at the last update.
    unsigned long currMillis = millis();
    
    // Work out time before reading each sensor.
    milliToPerformNextUSReading = currMillis + MILLI_BEFORE_FIRST_READING;
}


void loop() 
{
    unsigned long currMillis  = millis();

    
    //---------------------------------------------------------
    // LOOP
    //
    // Regular looped code to update sensors and other values.
    //---------------------------------------------------------
    
    // Update the ultrasonic sensor readings, if it's that sensors' turn to update and if
    // the sensor is activated.
    if (currMillis > milliToPerformNextUSReading)
    {
        // Update the time until we read the sensor again
        milliToPerformNextUSReading += MILLI_BETWEEN_READINGS;
        
        // Get a front sensor reading.
        if (usSensorActivated == true)
        {
            int newPingCM = Ping_CM();
            
            currFilteredUSValue = (0.9 * currFilteredUSValue) + (0.1 * newPingCM);
        }
    }
    

    //---------------------------------------------------------
    // PARSER CODE
    //
    // If there is data, parse it and process it.
    //---------------------------------------------------------
  
    // Read from pin serial port and write it out on USB port.
    if (Serial.available() > 0)
    {
        char c = Serial.read();
    
        // If we're in WAITING state, look for the START_CHAR.
        if (currParserState == PARSER_WAITING)
        {
            // If it's the START_CHAR, move out of this state...
            if (c == START_CHAR)
            {
                // Start parsing the command.
                currParserState = PARSER_COMMAND;
        
                // Reset thing ready for parsing
                currCmdIndex = 0;
                currCmd[0] = '-';
                currCmd[1] = '-';
                currParam1Val = 0;
                currParam2Val = 0;
                currParam3Val = 0;
            }
      
            // Otherwise, stay in this state.
        }
    
        // In the state to look for the command.
        else if (currParserState == PARSER_COMMAND)
        {
            // Else if it's a separator, parse parameter 1. But make sure it's not
            // empty, or else it's a parse error.
            if (c == SEP_CHAR)
            {
                if (currCmdIndex == CMD_LENGTH)
                {
                    currParserState = PARSER_PARAM1;
                    currParamIndex = 0;
                    currParamNegative = false;
                }
                else
                {
                    currParserState = PARSER_WAITING;
                }
            }
      
            // Else if it's the end char, there are no parameters, so we're ready to
            // process. But make sure it's not empty. Otherwise, it's a parse error.
            else if (c == END_CHAR)
            {
                if (currCmdIndex == CMD_LENGTH)
                {
                    currParserState = PARSER_EXECUTE;
                }
                else
                {
                    currParserState = PARSER_WAITING;
                }
            }
      
            // If we've got too many letters here, we have a parse error,
            // so abandon and go back to PARSER_WAITING
            else if ( (currCmdIndex >= CMD_LENGTH) || (c < 'A') || (c > 'Z') )
            {
                currParserState = PARSER_WAITING;
            }
      
            // Store the current character.
            else
            {
                currCmd[currCmdIndex] = c;
                currCmdIndex++;
            }
        }
    
        // In the state to parse param 1.
        else if (currParserState == PARSER_PARAM1)
        {
            // Else if it's a separator, parse parameter 1.
            if (c == SEP_CHAR)
            {
                if (currParamNegative)
                {
                    currParam1Val = -1 * currParam1Val;
                }

                currParserState = PARSER_PARAM2;
                currParamIndex = 0;
                currParamNegative = false;
            }
      
            // Else if it's the end char, there are no parameters, so we're ready to
            // process.
            else if (c == END_CHAR)
            {
                if (currParamNegative)
                {
                    currParam1Val = -1 * currParam1Val;
                }

                currParserState = PARSER_EXECUTE;
            }
      
            // Check for negative at the start.
            else if ( (currParamIndex == 0) && (c == '-') )
            {
                currParamNegative = true;
                currParamIndex++;
            }
            
            // If it's too long, or the character is not a digit, then it's
            // a parse error, so abandon and go back to PARSER_WAITING.
            else if ( (currParamIndex >= MAX_PARAM_LENGTH) || (c < '0') || (c > '9') )
            {
                currParserState = PARSER_WAITING;
            }

            // It's a valid character, so process it.
            else
            {
                // Shift existing value across and add new digit at the bottom.
                int currDigitVal = c - '0';
                currParam1Val = (currParam1Val * 10) + currDigitVal;
                currParamIndex++;
            }

        }
    
        // In the state to parse param 2.
        else if (currParserState == PARSER_PARAM2)
        {
            // Else if it's a separator, parse parameter 2.
            if (c == SEP_CHAR)
            {
                if (currParamNegative)
                {
                    currParam2Val = -1 * currParam2Val;
                }

                currParserState = PARSER_PARAM3;
                currParamIndex = 0;
                currParamNegative = false;
            }
      
            // Else if it's the end char, there are no parameters, so we're ready to
            // process.
            else if (c == END_CHAR)
            {
                if (currParamNegative)
                {
                    currParam2Val = -1 * currParam2Val;
                }

                currParserState = PARSER_EXECUTE;
            }
      
            // Check for negative at the start.
            else if ( (currParamIndex == 0) && (c == '-') )
            {
                currParamNegative = true;
                currParamIndex++;
            }
            
            // If it's too long, or the character is not a digit, then it's
            // a parse error, so abandon and go back to PARSER_WAITING.
            else if ( (currParamIndex >= MAX_PARAM_LENGTH) || (c < '0') || (c > '9') )
            {
                currParserState = PARSER_WAITING;
            }

            // It's a valid character, so process it.
            else
            {
                // Shift existing value across and add new digit at the bottom.
                int currDigitVal = c - '0';
                currParam2Val = (currParam2Val * 10) + currDigitVal;
                currParamIndex++;
            }

        }
    
        // In the state to parse param 3.
        else if (currParserState == PARSER_PARAM3)
        {
            // If it's the end char, there are no parameters, so we're ready to
            // process.
            if (c == END_CHAR)
            {
                if (currParamNegative)
                {
                    currParam3Val = -1 * currParam3Val;
                }
                currParserState = PARSER_EXECUTE;
            }
      
            // Check for negative at the start.
            else if ( (currParamIndex == 0) && (c == '-') )
            {
                currParamNegative = true;
                currParamIndex++;
            }
            
            // If it's too long, or the character is not a digit, then it's
            // a parse error, so abandon and go back to PARSER_WAITING.
            else if ( (currParamIndex >= MAX_PARAM_LENGTH) || (c < '0') || (c > '9') )
            {
                currParserState = PARSER_WAITING;
            }

            // It's a valid character, so process it.
            else
            {
                // Shift existing value across and add new digit at the bottom.
                int currDigitVal = c - '0';
                currParam3Val = (currParam3Val * 10) + currDigitVal;
                currParamIndex++;
            }

        }
    
        
        //---------------------------------------------------------
        // PARSER CODE HANDLER (Still part of Parser, but section that
        // processes completed commands)
        //
        // If the most recently read char completes a command,
        // then process the command, and clear the state to
        // go back to looking for a new command.
        //
        // The parsed items are stored in:
        //    currCmd, currParam1Val, currParam2Val, currParam3Val
        //
        // As we add more to the Attacknid, we can add handling for
        //  more commands here.
        //---------------------------------------------------------
    
        if (currParserState == PARSER_EXECUTE)
        {
            // Turn on/off explore mode.
            if ((currCmd[0] == 'X') && (currCmd[1] == 'A'))
            {
                if (currParam1Val == 1)
                {
                    currExploreState = XM_CHOOSE_DIR;
                    exploreModeActivated = true;
                    usSensorActivated = true;
                    DriveWheels(0, 0);
                    
                    Serial.println("<XA,1>");
                }
                else
                {
                    currExploreState = XM_CHOOSE_DIR;
                    exploreModeActivated = false;
                    usSensorActivated = false;
                    DriveWheels(0, 0);

                    Serial.println("<XA,0>");
                }
            }

            // Ultrasonic Sensor - If the sensor has been activated, this returns a filtered value.
            //                     If the sensor has not been turned on, it gets and returns a single reading.
            //                     Value is rounded to nearest whole CM.
            //   <US>
            // Response is:
            //   Ultrasonic Sensor Response:
            //   <US,#1>, Where #1 is the ultrasonic range sensor reading in cm (rounded to nearest whole cm).
            else if ((currCmd[0] == 'U') && (currCmd[1] == 'S'))
            {
                int sensorValue = -1;
                if (usSensorActivated)
                {
                    sensorValue = round(currFilteredUSValue);
                }
                else
                {
                   sensorValue = Ping_CM();
                }
                  
                Serial.print("<US,");
                Serial.print(sensorValue);
                Serial.println(">");
            }
            
            // Ready/OK Check: <OK>
            else if ((currCmd[0] == 'O') && (currCmd[1] == 'K'))
            {
                Serial.println("<OK>");
            }
                
            // We only accept other commands if explore mode is off.
            else if (exploreModeActivated == false)
            {
                // Wheel: <DW,#1,#2>, Where #1 is -255 to 255 for the left motor. -ve backwards, 0 stop, +ve forward
                //                    Where #2 is -255 to 255 for the right motor. -ve backwards, 0 stop, +ve forward
                if ((currCmd[0] == 'D') && (currCmd[1] == 'W'))
                {
                    int leftMotorValue = constrain(currParam1Val, -255, 255);
                    int rightMotorValue = constrain(currParam2Val, -255, 255);

                    DriveWheels(leftMotorValue, rightMotorValue);

                    Serial.print("<DW,");
                    Serial.print(leftMotorValue);
                    Serial.print(",");
                    Serial.print(rightMotorValue);
                    Serial.println(">");
                }
            
                // Left Wheel: <LW,#1>, Where #1 is -255 to 255 for the left motor. -ve backwards, 0 stop, +ve forward
                else if ((currCmd[0] == 'L') && (currCmd[1] == 'W'))
                {
                    int leftMotorValue = constrain(currParam1Val, -255, 255);
                
                    LeftWheel(leftMotorValue);

                    Serial.print("<LW,");
                    Serial.print(leftMotorValue);
                    Serial.println(">");
                }
            
                // Left Wheel: <RW,#1>, Where #1 is -255 to 255 for the right motor. -ve backwards, 0 stop, +ve forward
                else if ((currCmd[0] == 'R') && (currCmd[1] == 'W'))
                {
                    int rightMotorValue = constrain(currParam1Val, -255, 255);
                
                    RightWheel(rightMotorValue);

                    Serial.print("<RW,");
                    Serial.print(rightMotorValue);
                    Serial.println(">");
                }
            
                // Ultrasonic Sensor Activate/Deactivate - Use this to turn the sensor on/off
                // If a sensor is on, it will take readings regularly and filter the results. That way,
                // when a reading is requested using <US>, a filtered value will always be ready and the
                // call will return right away.
                //   <UA,#1>, Where #1 is US_START, US_STOP
                else if ((currCmd[0] == 'U') && (currCmd[1] == 'A'))
                {
                    if (currParam1Val == US_START)
                    {
                        usSensorActivated = true;
                      
                        Serial.print("<UA,");
                        Serial.print(US_START);
                        Serial.println(">");
                    }
                    else if (currParam1Val == US_STOP)
                    {
                        usSensorActivated = false;
                      
                        Serial.print("<UA,");
                        Serial.print(US_STOP);
                        Serial.println(">");
                    }
                }
            }
            
            //--------------------------------------------------
            // Clear the state and wait for the next command!
            // This must be done!
            //--------------------------------------------------
            currParserState = PARSER_WAITING;
        }
    }
        
    //---------------------------------------------------------
    // If explore mode is on, then check the sensor values
    // and take control of the motors.
    //---------------------------------------------------------
    if (exploreModeActivated == true)
    {
        if (currExploreState == XM_CHOOSE_DIR)
        {
            int turnSpeed = 120;
                
            int randomDirection = random(2); // Will be 0 or 1.
            if (randomDirection == 0)
            {
                DriveWheels(turnSpeed, -turnSpeed);
            }
            else
            {
                DriveWheels(-turnSpeed, turnSpeed);
            }
                
            milliToChangeExploreModeState = currMillis + 300;
            currExploreState = XM_CHOOSE_DIR_2;
        }
        else if (currExploreState == XM_CHOOSE_DIR_2)
        {
            if (currMillis >= milliToChangeExploreModeState)
            {
                milliToChangeExploreModeState = currMillis + random(2000,3000);
                currExploreState = XM_FIND_GAP;
            }
        }
        else if (currExploreState == XM_FIND_GAP)
        {
            // If we find a gap.
            if (round(currFilteredUSValue) > EXPLORE_OBSTACLE_DISTANCE)
            {
                DriveWheels(0,0);
                    
                milliToChangeExploreModeState = currMillis + random(3000,5000);
                currExploreDriveSpeed = random(160,190);
                currExploreState = XM_DRIVE_UNTIL_OBSTACLE;
            }
                
            // If we time out when trying to find a gap...
            else if (currMillis >= milliToChangeExploreModeState)
            {
                DriveWheels(0,0);
                currExploreState = XM_CHOOSE_DIR;
            }
        }
        else if (currExploreState == XM_DRIVE_UNTIL_OBSTACLE)
        {
            // If we hit an obstacle while driving. Stop and change directions.
            if (round(currFilteredUSValue) <= EXPLORE_OBSTACLE_DISTANCE)
            {
                DriveWheels(0,0);
                currExploreState = XM_BACKUP;
                Serial.println("<XO>");
            }
                
            // Otherwise, check if it's time to change directions.
            else if (currMillis >= milliToChangeExploreModeState)
            {
                DriveWheels(0,0);
                currExploreState = XM_CHOOSE_DIR;
                Serial.println("<XC>");
            }
                
            // Keep going!
            else
            {
                DriveWheels(currExploreDriveSpeed,currExploreDriveSpeed);
            }
        }
        else if (currExploreState == XM_BACKUP)
        {
            int backupSpeed = 120;
            int backupTime  = 1200;
            
            // Back up for a bit.
            DriveWheels(-backupSpeed, -backupSpeed);
            milliToChangeExploreModeState = currMillis + backupTime;
            currExploreState = XM_BACKUP_2;
        }
        else if (currExploreState == XM_BACKUP_2)
        {
            // Stop and find a new direction.
            if (currMillis >= milliToChangeExploreModeState)
            {
                DriveWheels(0,0);
                currExploreState = XM_CHOOSE_DIR;
            }
        }
    }
}


//--------------------------------------------------------------------------
// Attacknid Functionality
// These methods are called by the parser code in update().
//--------------------------------------------------------------------------

// Makes the robot roll.
// Values are between -255 to 255, with 0 being stop.
void DriveWheels(int leftMotor, int rightMotor)
{
    // Handle the left motor.
    LeftWheel(leftMotor);

    // Handle the right motor.
    RightWheel(rightMotor);
}

// Set the speed/direction of the left wheel.
// Values are between -255 to 255, with 0 being stop.
void LeftWheel(int leftMotor)
{
    //-----------------------------------
    // Handle the left motor.
    //-----------------------------------
    leftMotor = leftMotor * LEFT_MOTOR_WIRE_DIR;
    
    // Forward
    if (leftMotor > 0)
    {
        digitalWrite( LEFT_MOTOR_DIR_PIN, HIGH);
        analogWrite(  LEFT_MOTOR_PWM_PIN, leftMotor);
    }
    
    // Backward
    else if (leftMotor < 0)
    {
        digitalWrite( LEFT_MOTOR_DIR_PIN, LOW);
        analogWrite(  LEFT_MOTOR_PWM_PIN, -leftMotor);
    }

    // Stop
    else
    {
        //digitalWrite( LEFT_MOTOR_DIR_PIN, HIGH);
        analogWrite(  LEFT_MOTOR_PWM_PIN, 0);
    }
    
}

// Set the speed/direction of the right wheel.
// Values are between -255 to 255, with 0 being stop.
void RightWheel(int rightMotor)
{
    //-----------------------------------
    // Handle the right motor.
    //-----------------------------------
    rightMotor = rightMotor * RIGHT_MOTOR_WIRE_DIR;
    
    // Forward
    if (rightMotor > 0)
    {
        digitalWrite( RIGHT_MOTOR_DIR_PIN, HIGH);
        analogWrite(  RIGHT_MOTOR_PWM_PIN, rightMotor);
    }
    
    // Backward
    else if (rightMotor < 0)
    {
        digitalWrite( RIGHT_MOTOR_DIR_PIN, LOW);
        analogWrite(  RIGHT_MOTOR_PWM_PIN, -rightMotor);
    }

    // Stop
    else
    {
        //digitalWrite( RIGHT_MOTOR_DIR_PIN, HIGH);
        analogWrite(  RIGHT_MOTOR_PWM_PIN, 0);
    }
    
}


//----------------------------------------------------------
// send a ping from ultrasonic sensor HC-SR04 and return 
// distance in cm. Rounded to nearest whole CM.
int Ping_CM()
{
  // send a 10us+ pulse
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(20);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(20);
  
  //  read duration of echo 
  int duration = pulseIn(ECHO_PIN, HIGH);
 
  // dist = duration * speed of sound * 1/2
  // dist in cm = duration in us * 1 x 10^{-6} * 340.26 * 100 * 1/2
  // =  0.017*duration
  int dist = round(0.017 * duration);
  
  return dist;
}
 

