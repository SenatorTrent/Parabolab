
#include "Arduino_BMI270_BMM150.h"
#include <Servo.h>

//Dev timesavers
const bool DO_INIT = false;
const bool DO_FAN_TEST = true;

//Logging settings
//TEMP: Remember to enable logging for real flights
const bool DO_LOGGING = true;
//writable buffer for log writing
String logmsg = "";

//Flight ID, written to log TEMP (Remember to change)
String flightID = "ET6";


//Gyro settings
float lowpass = 0.6;
float apogeeTilt = 88; //Angle to consider apogee. X and Z axis both.
float angx, angy, angz;
float gyrox, gyroy, gyroz;

//Acceleration settings
float ax, ay, az;

//LED settings
const String RED = "red";
const String BLUE = "blue";
const String GREEN = "green";
const String OFF = "off";

//Rocket States
bool launched = false;
bool apogee = false;
int stage = 0;
long boostTimer = 0;

//Servo settings
Servo ServoF1;  // create servo object T1
Servo ServoF2;  // create servo object T2

//165 for full power, from testing
int fanFullPower = 165;
//Power to use for fan tests
int fanTestPower = 40;

//Boost
//Boost phase duration in seconds
//Here it's 15 seconds to keep the fans at max for the whole flight
int boostTime = 15; 

void setup() {
  



  ServoF1.attach(10);  // attaches the servo on pin 10 to the servo object for T1
  ServoF2.attach(9);  // attaches the servo on pin 9 to the servo object for T2
  //For ESC, servos must be immediatly be sent 0 signal
  ServoF1.write(long(0));
  ServoF2.write(long(0));

  //LED setup
  pinMode(6, OUTPUT);//led1 red
  pinMode(7, OUTPUT);//led1 green
  pinMode(8, OUTPUT);//led1 blue
  pinMode(5, OUTPUT);//led2 red
  pinMode(4, OUTPUT);//led2 green
  pinMode(3, OUTPUT);//led2 blue

  setLEDs(OFF);
  //Green to show that boot was succesful
  setLEDs(GREEN);

  //Debugging serial setup
  Serial.begin(9600);
  Serial.println("Connected board serial");
  
  //Data Logging serial setup
  Serial1.begin(9600); //open uart for openlog
  //Serial1.println("XYZ order for sensor data. By category: milliseconds - gyro angle - acceleration - logmessages ");
  Serial1.println("flight ID: " + flightID);
  Serial1.println("XYZ order for angle and acceleration");
  Serial1.println("BootTime (ms) \t\t Angle(deg) \t\t\t Acceleration(m/s^2) \t\t\t Log messages");
  
  //IMU setup
  IMU.begin();
  delay(3000);
  Serial.println("Initialized IMU");


  //Motors must be intialized with 0
  //(Legacy code. Theoretically not a problem because of ESC initilization, but kept just in case)
  ServoF1.write(long(0));
  ServoF2.write(long(0));

}

void loop() {

  //APPEND events to logmsg. start each with a comma for separation.
  logmsg = "";

  IMU.readAcceleration(ax, ay, az);
  ay = -ay; //correct y-axis sign

  //IMPORTANT: Must always update tilt
  updateTilt();

  /* BEGIN Stage Switch */
  switch (stage){
    case 0:
      // STAGE 0:Fan tests

      //Fan test
      if (DO_FAN_TEST){

        logmsg+= ",starting fantest";

        //Waits 7 seconds before starting fan test
        delay(7000);

        //Warning lights
        setLEDs(RED);
        delay(500);
        setLEDs(OFF);
        delay(500);

        setLEDs(RED);
        delay(500);
        setLEDs(OFF);
        delay(500);

        setLEDs(RED);
        delay(500);
        setLEDs(OFF);
        delay(500);

        setLEDs(BLUE);

        //T1
        Serial.println("Testing T1");
        ServoF1.write(fanTestPower);
        delay(2000);
        ServoF1.write(0);

        delay(5000);
        
        //T2
        Serial.println("Testing T2");
        ServoF2.write(fanTestPower);
        delay(2000);
        ServoF2.write(0);
        
        setLEDs(OFF);

      }

      //Setup stage done
      stage = 1;
      Serial.println("Stage 1 entered");
      
      break;
    case 1:
      //STAGE 1: Detect launch -> Turbine boost assist
      
      //Green to show primed for launch
      setLEDs(GREEN);

      //Launch detect -> boost phase
      // !launched to prevent multiple launch detections
      if((ay > 2) && !launched){

        Serial.println(ay);
        setLEDs(RED);

        launched = true;
        Serial.println("Launch detected! Yeehaw cowpokes");
        logmsg += ",Launch detected! Yeehaw cowpokes";
        
        //Starts 4 second boost
        ServoF1.write(fanFullPower);
        ServoF2.write(fanFullPower);

        //4 second timer start
        if (boostTimer == 0){
          boostTimer = millis();
        }
        
      }
      
      //4 second timer check
      //Not using delay() so we can continue updating tilt
      if((millis() - boostTimer >= (boostTime * 1000)) && launched){
          
          //If doing a jerk test send the rocket to stage 3 instead (TEMP)
          stage = 2;

          logmsg += "Boost over";
          setLEDs(RED);
          setLEDs(OFF);
          setLEDs(BLUE);
          setLEDs(OFF);
        }
      break;
    case 2:
      //STAGE 2: microgravity throttling and apogee detection
      Serial.println(ay);
      //Throttling
      if(ay > 0.1){
        ServoF1.write(long(0));
        ServoF2.write(long(0));
        setLEDs(GREEN);
        logmsg += "Boost 0";
      }
      if(ay < -0.1){
        ServoF1.write(long(fanFullPower));
        ServoF2.write(long(fanFullPower));
        setLEDs(RED);
        logmsg += "Boost full " + String(fanFullPower);
      }

      //Apogee detection
      //Currently mostly disabled, not needed with ZWT ESCs
      
      if((abs(angz)>apogeeTilt) || abs(angx)>apogeeTilt){
        /*
        apogee = true;
        stage = 3;
        ServoF1.write(long(0));
        ServoF2.write(long(0));
        */
        setLEDs(BLUE);
        logmsg += ",apogee detected";
      }
      
      break;
    case 3:
      Serial.println("Stage 3");
      // STAGE 3: post apogee and recovery
      
      break;


    }
  /* ENDStage Switch */


  //Writes log to datacards evey loop
  if(DO_LOGGING){
    updateLog(logmsg);
  }
  
}

//Tilt updating function
//MUST BE DONE CONSTANTLY, OR ANGLES WILL NOT BE ACCURATE
void updateTilt(){
  IMU.readGyroscope(gyrox, gyroy, gyroz);
    //Angles are updated as integral of angular speed from gyro
    //divisor 94 from testing

    //X gryo
    if(abs(gyrox)<lowpass){
      gyrox=0;
    }
    angx = angx +gyrox/94;

    //Y gyro 
    //not currently used, but here for data. (spin of rocket)
    if(abs(gyroy)<lowpass){
      gyroy=0;
    }
    angy = angy +gyroy/94;
    
    //Z gryo
    if(abs(gyroz)<lowpass){
      gyroz=0;
    }
    angz = angz +gyroz/94;
}

void setLEDs(String color) {
  digitalWrite(6, LOW); //led1 red
  digitalWrite(7, LOW); //led1 green
  digitalWrite(8, LOW); //led1 blue
  digitalWrite(5, LOW); //led2 red
  digitalWrite(4, LOW); //led2 green
  digitalWrite(3, LOW); //led2 blue

  if (color == RED) {
    digitalWrite(6, HIGH);
    digitalWrite(5, HIGH);
  } else if (color == GREEN) {
    digitalWrite(7, HIGH);
    digitalWrite(4, HIGH);
  } else if (color == BLUE) {
    digitalWrite(8, HIGH);
    digitalWrite(3, HIGH);
  }
}

//Simple log updater
//Pass empty string for msg if nothing extra to log
void updateLog(String msg){
  unsigned long ms = millis();
  Serial1.print(ms);

  //Angles
  Serial1.print("\t\t\t ");

  Serial1.print(angx);
  Serial1.print(",");
  Serial1.print(angy);
  Serial1.print(",");
  Serial1.print(angz);
  //Accelerations
  Serial1.print("\t\t\t ");

  Serial1.print(ax);
  Serial1.print(",");
  Serial1.print(ay);
  Serial1.print(",");
  Serial1.print(az);
  //Adds msg to line
  if(msg != ""){
    //Serial1.print(",");
    Serial1.print("\t\t\t ");
    Serial1.print(msg);
  }
  Serial1.print("\n");
  

}
