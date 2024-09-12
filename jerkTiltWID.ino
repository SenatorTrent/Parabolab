//#include <Arduino_LSM9DS1.h> library doesn't seem to work
#include "Arduino_BMI270_BMM150.h"
#include <Servo.h>



//Dev timesavers
const bool DO_INIT = true;
const bool DO_FAN_TEST = true;

//Logging settings
//tmp
const bool DO_LOGGING = true;
//writable buffer for log writing
String logmsg = "";

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

//ESC setup
//In seconds:
int escDelay = 30;

void setup() {
  


  pinMode(6, OUTPUT);//led1 red
  pinMode(7, OUTPUT);//led1 green
  pinMode(8, OUTPUT);//led1 blue
  pinMode(5, OUTPUT);//led2 red
  pinMode(4, OUTPUT);//led2 green
  pinMode(3, OUTPUT);//led2 blue

  setLEDs(OFF);

  ServoF1.attach(10);  // attaches the servo on pin 10 to the servo object for T1
  ServoF2.attach(9);  // attaches the servo on pin 9 to the servo object for T2
  //Servo 

  //Debugging serial
  Serial.begin(9600);
  //while (!Serial);
  Serial.println("Connected board serial");
  
  //Data Logging serial
  Serial1.begin(9600); //open uart for openlog
  //Serial1.println("TEST NOT REAL LAUNCH");
  Serial1.println("XYZ order for sensor data. data: milliseconds - gyro - acceleration - logmessages ");
  Serial1.println("logmessages are jank, might messup csv");
  
  //HOTFIX: code will start regardless of IMU
  /*
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  }
  */
  IMU.begin();
  delay(3000);
  Serial.println("Initialized IMU");

  //Motors must be intialized with 0.
  ServoF1.write(long(0));
  ServoF2.write(long(0));

  delay(escDelay * 1000);

  ServoF1.write(long(0));
  ServoF2.write(long(0));

  Serial.println("Calibrated ESC");
}

void loop() {

  //Append events to logmsg, start with a comma for seperation. Janky, but temporary
  logmsg = "";

  //if(true) fix for library function breaking. tmp
  if (true){

    IMU.readAcceleration(ax, ay, az);
    ay = -ay; //correct y-axis sign

    //IMPORTANT: Must always update tilt
    updateTilt();

    /* BEGIN Stage Switch */
    switch (stage){
      case 0:
        // STAGE 0: Amnesty period, fan tests
        
        //Amnesty period
        if(DO_INIT) {
          setLEDs(BLUE);
          Serial.println("Waiting for 5s buffer sequence");

          delay(5000);
          Serial.println("5s amnesty buffer finished");
        } else {
          Serial.println("Skipping amnesty buffer - change DO_INIT to enable");
        }

        //Fan test
        if (DO_FAN_TEST){

          logmsg+= ",starting fantest";
          //ServoF1.write(long(60));
          //delay(2000);
          //ServoF1.write(long(0));

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

          //T1
          Serial.println("Testing T1");
          ServoF1.write(long(30));
          delay(2000);
          ServoF1.write(long(0));

          delay(5000);
          
          //T2
          Serial.println("Testing T2");
          ServoF2.write(long(30));
          delay(2000);
          ServoF2.write(long(0));
          
        }

        setLEDs(OFF);
        stage = 1;
        
        break;
      case 1:
        //STAGE 1: Detect launch -> Turbine boost assist
        setLEDs(OFF);
        Serial.println("Stage 1");

        //Launch detect
        if(ay > 2){
          Serial.println(ay);
          setLEDs(RED);
          //tmp: disabled turbines to turn off 4 second boost
          //ServoF1.write(long(180));
          //ServoF2.write(long(180));
          stage = 2;
          launched = true;
          Serial.println("Launch detected! Yeehaw cowpokes");
          logmsg += ",Launch detected! Yeehaw cowpokes";
          //4 second timer start
          if (boostTimer == 0){
            boostTimer = millis();
          }
          
        }
        //4 second timer check
        //Not using delay() so we can continue updating tilt
        //tmp: removed /*millis() - boostTimer >= 4000) && */ from if to disable 4 sec boost

        if(/*(millis() - boostTimer >= 4000) && */launched){
            stage = 2;
            logmsg += ",Boost over";
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
        }
        if(ay < -0.1){
          ServoF1.write(long(180));
          ServoF2.write(long(180));
          setLEDs(RED);
        }

        //Apogee detection
        /*
        if((abs(angz)>apogeeTilt) || abs(angx)>apogeeTilt){
          apogee = true;
          stage = 3;
          ServoF1.write(long(0));
          ServoF2.write(long(0));
          setLEDs(BLUE);
          logmsg += ",apogee detected";
        }
        */
        break;
      case 3:
      Serial.println("Stage 3");
        // STAGE 3: post apogee and recovery
        
        break;


      }
    /* ENDStage Switch */

  } else {
    Serial.println("Acceleration not avaliable");
  }

  //Writes log to datacards evey loop
  if(DO_LOGGING){
    //tmp
    //logmsg += ",TESTING NOT A REAL FLIGHT";

    updateLog(logmsg);
  }
  
}

//Tilt updating function
//MUST BE DONE CONSTANTLY, OR ANGLES WILL NOT BE ACCURATE
void updateTilt(){
  IMU.readGyroscope(gyrox, gyroy, gyroz);

    //X gryo
    if(abs(gyrox)<lowpass){
      gyrox=0;
    }
    angx = angx +gyrox/94;

    //y angle not currently used, but here for future use
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

//Simple log updater, will be replaced with more advanced solution
//Pass empty string for msg if nothing extra to log
void updateLog(String msg){
  unsigned long ms = millis();
  Serial1.print(ms);
  Serial1.print(",");
  //Angles
  Serial1.print(angx);
  Serial1.print(",");
  Serial1.print(angy);
  Serial1.print(",");
  Serial1.print(angz);
  Serial1.print(",");
  //Accelerations
  Serial1.print(ax);
  Serial1.print(",");
  Serial1.print(ay);
  Serial1.print(",");
  Serial1.print(az);
  //Adds msg to line
  if(msg != ""){
    //Serial1.print(",");
    Serial1.print(msg);
  }
  //Terminating character
  Serial1.println("~");

}
