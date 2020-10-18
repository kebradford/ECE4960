
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

////////////////////////////////////////////////////////////////////////////////////////////

ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
SCMD myMotorDriver; //This creates the main object of one motor driver and connected slaves.

double pitch_a = 0, roll_a = 0;
double alpha, beta;
double pitch_alpha = 0, roll_alpha = 0; 
double old_pitch = 0, old_roll = 0;
double dt, lastRead; 
double pitch_g, roll_g, yaw_g; 
double pitch_combo, roll_combo;
double yaw_mag;

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
}

void loop() {


/////////////motor and IMU writing///////////////////////////////////////////////////////////////////////////////////////////
  if( myICM.dataReady() ){
    
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
//    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    //printScaledAGMT( myICM.agmt);   // This function takes into account the sclae settings from when the measurement was made to calculate the values with units

    pitch_a = atan2(myICM.accX(), myICM.accZ())*180/M_PI;
    roll_a  = atan2(myICM.accY(), myICM.accZ())*180/M_PI;

    alpha = .14;
    pitch_alpha = (alpha*pitch_a)+((1-alpha)*old_pitch);
    old_pitch = pitch_alpha;
    roll_alpha = (alpha*roll_a)+((1-alpha)*old_roll);
    old_roll = roll_alpha;
   
    dt = (millis()-lastRead)/1000;
    lastRead = millis();

    pitch_g = pitch_g - myICM.gyrY()*dt; 
    roll_g = roll_g + myICM.gyrX()*dt; 
    yaw_g = yaw_g+myICM.gyrZ()*dt;

    beta = .05; 

    pitch_combo = (pitch_combo-myICM.gyrY()*dt)*(1-beta)+pitch_alpha*beta;
    roll_combo =  (roll_combo-myICM.gyrX()*dt)*(1-beta)+roll_alpha*beta;

    Serial.println(yaw_g);
    myMotorDriver.setDrive( 1, 1, 180); 
    myMotorDriver.setDrive( 0, 1, 180);



    delay(10);
  }else{
    //Serial.println("Waiting for data");
    delay(500);
  }

}
