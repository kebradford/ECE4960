
/////////Motor and IMU Stuff////////////////////////////////////////////////////////////
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "math.h"
#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"

#define SERIAL_PORT Serial
#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL   1     // The value of the last bit of the I2C address. 

#include <PID_v1.h> //PID library

////////////////////////////////////////////////////////////////////////////////////////////

ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
SCMD myMotorDriver; //This creates the main object of one motor driver and connected slaves.

double dt, lastRead, rotationalSpeed; 
double pitch_g, roll_g, yaw_g; 

///////PID Stuff////////////////////////////////////////////////
double Setpoint, Input, Output; // for PID control
double Kp=1.5, Ki=.1, Kd=0; //CHANGE THESE CONSTANTS FOR PID
double last_yaw = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
////////////////////////////////////////////////////////////////

void setup() {

 SERIAL_PORT.begin(115200);
 while(!SERIAL_PORT){};

 WIRE_PORT.begin();
 WIRE_PORT.setClock(400000);


/////set up IMU////////
 bool initialized = false;
 while( !initialized ){

 myICM.begin( WIRE_PORT, AD0_VAL );

    if( myICM.status != ICM_20948_Stat_Ok ){
      //SERIAL_PORT.println( "Trying again..." );
      delay(500);
    }
    else initialized = true;
  } 

/////set up motor////////
  myMotorDriver.settings.commInterface = I2C_MODE;
  myMotorDriver.settings.I2CAddress = 0x5D; //config pattern "0101" on board for address 0x5A

  //*****initialize the driver get wait for idle*****//
  while ( myMotorDriver.begin() != 0xA9 ) //Wait until a valid ID word is returned
  {
    delay(500);
  }

  //  Check to make sure the driver is done looking for slaves before beginning
  while ( myMotorDriver.ready() == false );

  //*****Set application settings and enable driver*****//
  while ( myMotorDriver.busy() );
  myMotorDriver.enable();


  myPID.SetMode(AUTOMATIC); ///start PID 
}

void loop() {


/////////////motor and IMU writing///////////////////////////////////////////////////////////////////////////////////////////
  if( myICM.dataReady() ){
    
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'

    dt = (millis()-lastRead)/1000;
    lastRead = millis();

    yaw_g = yaw_g+myICM.gyrZ()*dt;


    Input    = myICM.gyrZ();
    Setpoint = 50;

    double motorVal;
    myPID.Compute(); //compute Output for motors
    if(Output>180) motorVal = 180;
    else motorVal = Output;
    myMotorDriver.setDrive( 1, 1, motorVal); 
    myMotorDriver.setDrive( 0, 1, motorVal);


    Serial.print("MotorValue:");
    Serial.print(motorVal);
    Serial.print(" ");
    Serial.print("GyroData:");
    Serial.println(Input);


    delay(1);
  }else{
    //Serial.println("Waiting for data");
    delay(500);
  }

}
