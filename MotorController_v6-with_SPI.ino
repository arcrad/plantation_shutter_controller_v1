#define OUTPUT_MIN -200
#define OUTPUT_MAX 200
#define KP 0.25 //0.2 //0.25
#define KI 0.0001 //0.01 //0.05
#define KD 0.00001 //0.005 //0.01

#define MOTOR_PWM_VALUE 255
#define MOTOR_PWM_MAX_VALUE 255
#define MOTOR_PWM_MIN_VALUE 180

#include <Encoder.h>
#include <AutoPID.h>
#include <SPI.h>

//SPI related
volatile boolean gotNewSpiByte;
volatile uint8_t lastSpiByteRecieved;
volatile uint8_t spiByteToReturn = 0;

//const byte LED = 13;
const byte encoderPin = 3;
const byte pwmPin = 9;
const byte phasePin = 8; //motor direction 

//uint8_t pwmValue = 200;
bool motorIsOn = false;
bool motorDirection = false; //false = forward, true = backward
long adjustedPWM = 0;
bool motorIsStalled = false;
bool doCalibrationRoutine = false;
bool completedCalibrationRoutine = false;
bool startedLastMoveCommand = false;
bool startMotorSeek = false;

/*
  0 = start, find forward stop
  1 = find backward stop
  2 = calculate middle stop
  3 = go to middle stop
  4 = go to forward stop
  5 = go to backward stop
*/
uint8_t homingState = 0;
uint8_t globalState = 0;


unsigned long lastMillisPrint = 0;
unsigned long lastMillisMotorStart = 0;
unsigned long lastMillisStallCheck = -1;
unsigned long lastMillisMotorStop = -1;
bool completedDeltaCheck = true;

long encoderCount = 0;
long lastEncoderCount = 0;
long encoderCountAtStop = 0;
long encoderCountForwardStop = 0;
long encoderCountBackwardStop = 0;
//long encoderCountMiddleStop = -100;

//uint16_t encoderOverrunOnStopAvg = 0;
int serialByteRead = -1;
//1440 counts = 1 rotation (12 counts per revolution * 120:1 motor gear ratio)
float nextTargetPercent = 0.5;
float prevTargetPercent = 0.0;
int nextAndPrevTargetPercentDelta = 5;
long nextTargetPulses = 0;
const long maxPulsesPerRev = 1440;
long deltaPulses = 0;
//uint8_t currentPositionPercent = 0;


long deltaPulsesForwardMagnitude[] = {0,0,0,0,0,0,0,0,0,0};
long deltaPulsesBackwardMagnitude[] = {0,0,0,0,0,0,0,0,0,0};

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder motorEncoder(2, 3);

double pidOutput; 
double encoderCountDouble;
double nextTargetPulsesDouble;

AutoPID myPID(&encoderCountDouble, &nextTargetPulsesDouble, &pidOutput, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

const byte numChars = 32;
char receivedChars[numChars];
bool newData = false;

/*
void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        } else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}
*/

/*
void showNewData() {
    if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
        newData = false;
    }
}
*/

double dbl_map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// SPI interrupt routine
ISR(SPI_STC_vect) {
  if(SPDR == 0) {
    //dummy byte/NOOP
  } else {
    lastSpiByteRecieved = SPDR;
    
    if(lastSpiByteRecieved == 1) {
        //Got "do calibration routine" command.
       spiByteToReturn = 7;
    } else if(lastSpiByteRecieved > 1 && lastSpiByteRecieved < 13) {
        //Got "seek position" request
        spiByteToReturn = 8;
    } else if(lastSpiByteRecieved == 13) {
        //Got request for current position
        spiByteToReturn = static_cast<int>(abs( (encoderCount-encoderCountBackwardStop) / ((encoderCountBackwardStop-encoderCountForwardStop)*1.0) )*100.0)+11;
    } else if(lastSpiByteRecieved == 14) {
        //Got request for last error code
        /////spiByteToReturn = current error code
    } else if(lastSpiByteRecieved == 15) {
        //Got request for current state
        /////spiByteToReturn = current state
    } else if(lastSpiByteRecieved == 16) {
        //Got request for current seek target
        /////spiByteToReturn = current seek target 
    } else if(lastSpiByteRecieved == 17) {
        //Got request for "is calibrated" status
        /////spiByteToReturn = "is calibrated" status
    } else {
        //Got unknown command
        spiByteToReturn = 2;
    }

    SPDR = spiByteToReturn;
    gotNewSpiByte = true;
  }
}

/***************************/

void setup() {
  pinMode(phasePin, OUTPUT);
  Serial.begin(250000);
  myPID.setTimeStep(5);

  //configure SPI
  //have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  // get ready for an interrupt 
  gotNewSpiByte = false;
  //now turn on interrupts
  SPI.attachInterrupt();
}

void loop()
{
  encoderCount = motorEncoder.read();
  

  

  //calculate adjustedPWM
  /*
  if(pidOutput >= -1 && pidOutput <= 1) {
    adjustedPWM = 0;
  } else {
    adjustedPWM = dbl_map(pidOutput, OUTPUT_MIN, OUTPUT_MAX, MOTOR_PWM_MIN_VALUE, MOTOR_PWM_MAX_VALUE);
  }
  */
  if(pidOutput >= -1.5 && pidOutput <= 1.5) {
    adjustedPWM = 0;
  } else if(pidOutput > 0 && pidOutput < MOTOR_PWM_MIN_VALUE){
    adjustedPWM = MOTOR_PWM_MIN_VALUE;
  } else if(pidOutput < 0 && pidOutput > -MOTOR_PWM_MIN_VALUE) {
    adjustedPWM = -MOTOR_PWM_MIN_VALUE;
  } else {
    adjustedPWM = pidOutput;
  }
  
  //currentPositionPercent = static_cast<int>(abs( (encoderCount-encoderCountBackwardStop) / ((encoderCountBackwardStop-encoderCountForwardStop)*1.0) )*100.0);
  //currentPositionPercent = ( (encoderCount-encoderCountBackwardStop) / (encoderCountBackwardStop-encoderCountForwardStop) );

  digitalWrite(phasePin, motorDirection);

  //myPID.run();

  encoderCountDouble = (double) encoderCount;
  nextTargetPulsesDouble = (double) nextTargetPulses;

  //if(Serial.available()) {
    /////recvWithEndMarker();
    //showNewData();

    //if (newData == true) {
    if(gotNewSpiByte && lastSpiByteRecieved > 0) {
        Serial.print("GOT DATA.");
        Serial.println(receivedChars);

        if(lastSpiByteRecieved == 1) {
            Serial.println("Got do calibration routine command.");
            globalState = 1;
            doCalibrationRoutine = true;
            completedCalibrationRoutine = false;
        } else if(lastSpiByteRecieved == 12) {
            Serial.println("Got position request 100%.");
            globalState = 3;
            prevTargetPercent = nextTargetPercent;
            nextTargetPercent = 1.0;
            startedLastMoveCommand = false;
        } else if(lastSpiByteRecieved == 2) {
            Serial.println("Got position request 0%.");
            globalState = 3;
            prevTargetPercent = nextTargetPercent;
            nextTargetPercent = 0.0;
            startedLastMoveCommand = false;
        } else if(lastSpiByteRecieved == 3) {
            Serial.println("Got position request 10%.");
            globalState = 3;
            prevTargetPercent = nextTargetPercent;
            nextTargetPercent = 0.1;
            startedLastMoveCommand = false;
        } else if(lastSpiByteRecieved == 4) {
            Serial.println("Got position request 20%.");
            globalState = 3;
            prevTargetPercent = nextTargetPercent;
            nextTargetPercent = 0.2;
            startedLastMoveCommand = false;
        } else if(lastSpiByteRecieved == 5) {
            Serial.println("Got position request 30%.");
            globalState = 3;
            prevTargetPercent = nextTargetPercent;
            nextTargetPercent = 0.3;
            startedLastMoveCommand = false;
        } else if(lastSpiByteRecieved == 6) {
            Serial.println("Got position request 40%.");
            globalState = 3;
            prevTargetPercent = nextTargetPercent;
            nextTargetPercent = 0.4;
            startedLastMoveCommand = false;
        } else if(lastSpiByteRecieved == 7) {
            Serial.println("Got position request 50%.");
            globalState = 3;
            prevTargetPercent = nextTargetPercent;
            nextTargetPercent = 0.5;
            startedLastMoveCommand = false;
        } else if(lastSpiByteRecieved == 8) {
            Serial.println("Got position request 60%.");
            globalState = 3;
            prevTargetPercent = nextTargetPercent;
            nextTargetPercent = 0.6;
            startedLastMoveCommand = false;
        } else if(lastSpiByteRecieved == 9) {
            Serial.println("Got position request 70%.");
            globalState = 3;
            prevTargetPercent = nextTargetPercent;
            nextTargetPercent = 0.7;
            startedLastMoveCommand = false;
        } else if(lastSpiByteRecieved == 10) {
            Serial.println("Got position request 80%.");
            globalState = 3;
            prevTargetPercent = nextTargetPercent;
            nextTargetPercent = 0.8;
            startedLastMoveCommand = false;
        } else if(lastSpiByteRecieved == 11) {
            Serial.println("Got position request 90%.");
            globalState = 3;
            prevTargetPercent = nextTargetPercent;
            nextTargetPercent = 0.9;
            startedLastMoveCommand = false;
        } else {
            Serial.println("Unknown command.");
        }
        
        gotNewSpiByte = false;
    }

    nextAndPrevTargetPercentDelta = abs((nextTargetPercent*10) - (prevTargetPercent*10));
    /*
    //serialByteRead = Serial.read();
    String teststr = Serial.readString();  //read until timeout
    noInterrupts();
    teststr.trim();
    // say what you got:
    Serial.print("I received: ");
    Serial.println(teststr);
    interrupts();
    */
  //}

  //logging for serial plotter
  if(millis() >= lastMillisPrint + 2000) {
    lastMillisPrint = millis();
    Serial.print("Encoder_Count:");
    Serial.print(encoderCount);
    Serial.print(",Next_Target_Pulses:");
    Serial.print(nextTargetPulses);
    Serial.print(" FS: ");
    Serial.print(encoderCountForwardStop);
    Serial.print(" BS: ");
    Serial.print(encoderCountBackwardStop);
    //Serial.print(",Position_%:");
    //Serial.print(currentPositionPercent);
    /*Serial.print(",PID_Output:");
    Serial.print(pidOutput);
    Serial.print(",Adjusted_PID_Output:");
    Serial.print(adjustedPWM);
    Serial.print(",Motor_Direction:");
    Serial.print(motorDirection);*/

    Serial.println();
    
  }
  /*
  //regular logging
  if(millis() >= lastMillisPrint + 500) {
    lastMillisPrint = millis();
    Serial.print(millis());
    Serial.print(" > CNT: ");
    Serial.print(encoderCount);
    Serial.print(" CNT@STOP: ");
    Serial.print(encoderCountAtStop);
    //Serial.print(" deltaPulses = ");
    //Serial.print(deltaPulses);
    Serial.print(" - NTPs: ");
    Serial.print(nextTargetPulses);
    Serial.print(" - N%/P%: ");
    Serial.print(nextTargetPercent);
    Serial.print("/");
    Serial.print(prevTargetPercent);
    Serial.print(" ^%: ");
    Serial.print(nextAndPrevTargetPercentDelta);
    Serial.print(" - LEC: ");
    Serial.print(lastEncoderCount);
    Serial.print(" H_ST: ");
    Serial.print(homingState);
    Serial.print(" DIR: ");
    Serial.print(motorDirection);
    Serial.print(" FS: ");
    Serial.print(encoderCountForwardStop);
    //Serial.print(" MiddleStop = ");
    //Serial.print(encoderCountMiddleStop);
    Serial.print(" BS: ");
    Serial.print(encoderCountBackwardStop);
    /*
    Serial.print(" ^FWD = {");
    for(size_t c = 0; c < (sizeof(deltaPulsesForwardMagnitude)/sizeof(deltaPulsesForwardMagnitude[0])); c++) {
      Serial.print(deltaPulsesForwardMagnitude[c]);
      if( c < (sizeof(deltaPulsesForwardMagnitude)/sizeof(deltaPulsesForwardMagnitude[0]))-1 ) {
        Serial.print(",");
      }
    }
    Serial.print("}");
    Serial.print(" ^BWD = {");
    for(size_t c = 0; c < (sizeof(deltaPulsesBackwardMagnitude)/sizeof(deltaPulsesBackwardMagnitude[0]));c++) {
      Serial.print(deltaPulsesBackwardMagnitude[c]);
      if( c < (sizeof(deltaPulsesBackwardMagnitude)/sizeof(deltaPulsesBackwardMagnitude[0]))-1 ) {
        Serial.print(",");
      }
    }
    Serial.print("}");
    */
    /*///////////////
    Serial.print(" - PID_OUT: ");
    Serial.print(pidOutput);
    Serial.print(" ADJ_PWM: ");
    Serial.print(adjustedPWM);
    Serial.print(" - SMS: ");
    Serial.print(startMotorSeek);
    Serial.print(" LMMS: ");
    Serial.print(lastMillisMotorStart);
    Serial.print(" MIO: ");
    Serial.print(motorIsOn);

    Serial.println();
    
  }
  *////////////////
  encoderCount = motorEncoder.read();
  
  //stall check
  if(motorIsOn && millis() >= lastMillisMotorStart + 250 && millis() > lastMillisStallCheck + 50) {
    lastMillisStallCheck = millis();
    if( 
        (!motorDirection && lastEncoderCount + 5 >= encoderCount) 
        || (motorDirection && lastEncoderCount - 5 <= encoderCount)
      ) {
      Serial.println("MOTOR IS STALLED. Stopping Motor.");
      Serial.print("     -- encoderCount = ");
      Serial.print(encoderCount);
      Serial.print(" lastEncoderCount = ");
      Serial.println(lastEncoderCount);
      motorIsOn = false;
      motorIsStalled = true;
      analogWrite(pwmPin, 0);
    }
    lastEncoderCount = encoderCount;
  }

  encoderCount = motorEncoder.read();

  if(globalState > 0 && globalState < 3) {
    if(doCalibrationRoutine) {
      //Serial.println("Start calibration routine...");
      switch(globalState) {
        case 1:
          //find forward stop (stall)
          //Serial.println("Finding forward stop...");
          motorDirection = false;
          startMotorSeek = true;
          if(motorIsStalled) {
            Serial.println("Found forward stop.");
            encoderCountForwardStop = encoderCount;
            encoderCountAtStop = encoderCount;
            motorIsStalled = false;
            globalState = 2;
          }
          break;
        case 2:
          //find backward stop (stall)
          motorDirection = true;
          startMotorSeek = true;
          //digitalWrite(phasePin, motorDirection);
          if(motorIsStalled) {
            Serial.println("Found backward stop.");
            encoderCountBackwardStop = encoderCount;
            encoderCountAtStop = encoderCount;
            motorIsStalled = false;
            //homingState = 2;
            completedCalibrationRoutine = true;
            doCalibrationRoutine = false;
          }
          break;
      }
    }

    //start motor (with timeout)
    if(startMotorSeek && !motorIsOn && millis() >= lastMillisMotorStart + 1000) {
      Serial.println("startMotorSeek routine start...");
      /////deltaPulses = encoderCount - encoderCountAtStop;
      //analogWrite(pwmPin, pwmValue);
      analogWrite(pwmPin, MOTOR_PWM_VALUE);
      Serial.print("encoderCount (start) =");
      Serial.println(encoderCount);
      //encoderCount = 0;
      motorIsOn = true;
      lastMillisMotorStart = millis();
      startMotorSeek = false;
    } 
  

    //stop motor once target is reached
    if(
        completedCalibrationRoutine && 
        motorIsOn 
        && ( 
          (motorDirection && encoderCount <= nextTargetPulses + abs(deltaPulsesBackwardMagnitude[nextAndPrevTargetPercentDelta-1]))
          || (!motorDirection && encoderCount >= nextTargetPulses - abs(deltaPulsesForwardMagnitude[nextAndPrevTargetPercentDelta-1]))
        )
    ) {
      motorIsOn = false;
      analogWrite(pwmPin, 0);
      lastMillisMotorStop = millis();
      Serial.print("encoderCount (stop) =");
      Serial.println(encoderCount);
      //encoderCount = 0;
      encoderCountAtStop = encoderCount;
      Serial.print("encoderCount (zero?) =");
      Serial.println(encoderCount);
      if(completedCalibrationRoutine) {
        completedDeltaCheck = false;
      } else {
        completedDeltaCheck = true;
      }
      lastEncoderCount = encoderCount;
    }

  }

  encoderCount = motorEncoder.read();

  if(globalState == 3) {
    if(completedCalibrationRoutine) {
        if(adjustedPWM < 0) {
          motorDirection = true; //backward
        } else {
          motorDirection = false; //forward
        }
        nextTargetPulses = encoderCountBackwardStop + ((encoderCountForwardStop - encoderCountBackwardStop) * nextTargetPercent);
        myPID.run();
        analogWrite(pwmPin, abs(adjustedPWM));
    }
  }

  encoderCount = motorEncoder.read();
}
