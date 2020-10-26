
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


///TOF Things
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include "SparkFun_VL53L1X.h" //TOF

SFEVL53L1X distanceSensor;
ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
SCMD myMotorDriver; //This creates the main object of one motor driver and connected slaves.

////////////////////////////////////////////////////////////////////////////////////////////

// maximum length of reply / data message
#define MAXREPLY 100
#define TODO_VAL 0
// buffer to reply to client
uint8_t val[MAXREPLY];
uint8_t *val_data = &val[2]; // start of optional data area
uint8_t *val_len = &val[1];  // store length of optional data

/********************************************************************************************************************
                 INCLUDES
 *******************************************************************************************************************/
#include "BLE_example.h"
#include "commands.h"
#include "related_funcs.h"

/********************************************************************************************************************
                GLOBAL VARIABLES
 *******************************************************************************************************************/

String s_Rcvd = "100"; //KHE declare extern in BLE_example_funcs.cpp to accept messages, if 100 don't bother checking case statements
uint16_t l_Rcvd = 0;
uint8_t *m_Rcvd = NULL;
String s_AdvName = "MyRobot"; //KHE 2 0 TOTAL CHARACHTERS ONLY!!  any more will be dropped

cmd_t empty_cmd = {NOT_A_COMMAND, 1, {0}};
cmd_t *cmd = &empty_cmd;
cmd_t *res_cmd = &empty_cmd;
bt_debug_msg_t *bt_debug_head = NULL;
bt_debug_msg_t *bt_debug_tail = NULL;
//SCMD motorDriver;
present_t presentSensors = {
    .motorDriver = 0,
    .ToF_sensor = 0,
    .prox_sensor = 0,
    .IMU = 0};
int bytestream_active = 0;


//IMU, motor stuff
double dt, lastRead, rotationalSpeed; 
double pitch_g, roll_g, yaw_g; 
int distance;

///////PID Stuff////////////////////////////////////////////////
double Setpoint, Input, Output; // for PID control
double Kp=3, Ki=.5, Kd=0; //CHANGE THESE CONSTANTS FOR PID
double last_yaw = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
////////////////////////////////////////////////////////////////

void setup()
{
    Serial.begin(115200);
    delay(1000);

     WIRE_PORT.begin();
     WIRE_PORT.setClock(400000);

    //Serial.printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    analogWriteResolution(16); //Set AnalogWrite resolution to 16 bit = 0 - 65535 (but make max 64k or trouble)

    set_Adv_Name(); //BLE_example_funcs.cpp near end of file, fn declared extern in BLE_example.h

    HciDrvRadioBoot(0);

    exactle_stack_init();

    HciVsA3_SetRfPowerLevelEx(TX_POWER_LEVEL_PLUS_3P0_dBm); //= 15 decimal = max power WORKS..default = 0

    AmdtpStart();

    pinMode(LED_BUILTIN, OUTPUT);

    //Set a starting point...for motors, servo, and LED_BUILTIN
    //delay(1000);

    // Bluetooth would start after blinking
    for (int i = 0; i < 20; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
    }

    //setupwdt();

    pinMode(blinkPin, OUTPUT);
    digitalWrite(blinkPin, LOW);

    // Configure the watchdog.
    //setupTimerA(myTimer, 31); // timerNum, period - //moved to BLE_example_funcs.cpp scheduler_timer_init
    setupWdt();
    am_hal_wdt_init(&g_sWatchdogConfig);
    //NVIC_EnableIRQ(CTIMER_IRQn); // Enable CTIMER interrupt in nested vector interrupt controller.
    NVIC_EnableIRQ(WDT_IRQn); // Enable WDT interrupt in nested vector interrupt controller.

    uint8_t a = 0;
    m_Rcvd = &a;
    //Serial.printf("Size of command: %d", sizeof(cmd_t));
    am_hal_interrupt_master_enable();
    //interrupts(); // Enable interrupt operation. Equivalent to am_hal_rtc_int_enable().
    //am_hal_wdt_start();
    //am_hal_wdt_int_enable(); - freezes boot


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
    
    
     ///set up TOF
     if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
      {
        //Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
        while (1)
          ;
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

} /*** END setup FCN ***/

void loop()
{
    //Serial.println("Loop...."); //KHE Loops constantly....no delays


    /////////////motor and IMU writing///////////////////////////////////////////////////////////////////////////////////////////
    if( myICM.dataReady() ){
  
      //Read yaw data
      myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
      dt = (millis()-lastRead)/1000;
      lastRead = millis();
      yaw_g = yaw_g+myICM.gyrZ()*dt;
      Serial.print("Yaw:");
      Serial.println(yaw_g);
  
      //get distance
      distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
      distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
      distanceSensor.clearInterrupt();
      distanceSensor.stopRanging();
      Serial.print("Distance(mm): ");
      Serial.println(distance);
  
      double motorVal;
      //myPID.Compute(); //compute Output for motors
      motorVal = 130;
     // else motorVal = Output;
      myMotorDriver.setDrive( 1, 1, motorVal); 
      myMotorDriver.setDrive( 0, 1, motorVal);
  
  
      delay(1);
    }else{
      //Serial.println("Waiting for data");
      delay(500);
    }

    if (l_Rcvd > 1) //Check if we have a new message from amdtps_main.c through BLE_example_funcs.cpp
    {

        cmd = (cmd_t *)m_Rcvd;

        switch (cmd->command_type)
        {
        case SET_MOTORS:

            Serial.println("Placeholder: Set Motors");

            break;
        case GET_MOTORS:

            Serial.println("Placeholder: Set Motors");
            //amdtpsSendData((uint8_t *)res_cmd, *val_len);
            break;
        case SER_RX:
            Serial.println("Got a serial message");
            pushMessage((char *)&cmd->data, cmd->length);
            break;
        case REQ_FLOAT:
            Serial.println("Going to send a float");
            //TODO: Put a float (perhaps pi) into a command response and send it.
            amdtpsSendData((uint8_t *)res_cmd, TODO_VAL);
            break;
        case PING:
            Serial.println("Ping Pong");
            cmd->command_type = PONG;
            amdtpsSendData(m_Rcvd, l_Rcvd);
            break;
        case START_BYTESTREAM_TX:
            bytestream_active = (int)cmd->data[0];
            bytestream_active = true;
            //Serial.printf("Start bytestream with active %d \n", bytestream_active);
            //((uint32_t *)res_cmd->data)[0] = 0;
            break;
        case STOP_BYTESTREAM_TX:
            bytestream_active = 0;
            break;
        default:
            Serial.printf("Unsupported Command 0x%x \n", cmd->command_type);
            break;
        }

        l_Rcvd = 0;
        am_hal_wdt_restart();
        free(m_Rcvd);
    } //End if s_Rcvd != 100
    else if ((s_Rcvd[0] == '6' && s_Rcvd[1] == '7'))
    {
        s_Rcvd[0] = 0;
        digitalWrite(LED_BUILTIN, HIGH);
        //Serial.printf("Connected, length was %d", l_Rcvd);
    }
    else if ((s_Rcvd[0] == '6' && s_Rcvd[1] == '8'))
    {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("disconnected");
        //Decimal value of D for Disconnect
        //Serial.println("got disconnect from case in ino file - set_Stop");
        digitalWrite(LED_BUILTIN, LOW);
        //amdtps_conn_close();
        DmDevReset();
    }

    if (availableMessage())
    {
        Serial.println("Bluetooth Message:");
        Serial.println(pullMessage());
        printOverBluetooth("Message Received.");
    }

   if (bytestream_active)
    {

        res_cmd->command_type = BYTESTREAM_TX;
        res_cmd->length = 64;
       
        uint32_t numToSendShort = yaw_g;
        uint64_t numToSendLong  = distance;

        memcpy(res_cmd->data, &numToSendShort, 4);
        memcpy(res_cmd->data+4, &numToSendLong, 8);
 
        //Serial.printf("Stream %d \n", bytestream_active);
        amdtpsSendData((uint8_t *)res_cmd, 14);

    }


    trigger_timers();


    delay(10);
} //END LOOP
