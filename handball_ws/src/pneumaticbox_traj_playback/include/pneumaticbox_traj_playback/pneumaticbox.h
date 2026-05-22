/* 
Copyright 2013-2016 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.  
*/


/*
  API to the pneumaticbox server

  The server listens on a Unix Domain socket named '/var/run/pneumaticbox/control'
  The socket has to be opened in Datagram mode (DGRAM flag).
  The client can then send individual messages (defined as structures here). 
  These messages are used to configure and activate controllers and to 
  communicate changes of client signals.


  ===============Model================
  The server has a notion of signals and controllers. 
  Signals provide state, while controllers read and write signals at each control loop.
  The client can tell the server which signals to patch to which controller.
  The server itself does not store or load any configuration, it is up to the client to provide persistence.

  *SIGNALS*
  There are four flavours of signals:

  sensor signals: provide sensor data gathered directly by the pneumaticbox server
  actuator signals: the valves are actuated according to these signals
  special signals: some fixed, predefined signals for readability.
  client signals: signals that can be changed by the client (e.g. as source of desired values)

  The latter can be changed by sending a MsgSignalEvent message, which is the main means of communication with the server.
  Changes should be scheduled ahead of time using the message's time field. 

  *CONTROLLERS*
  Individual Controllers are referenced by a 32 bit unssigned integer number. 
  Controllers can have 3 states: UNCONFIGURED, INACTIVE, or ACTIVE.
  Sending a MSGTYPE_CONTROLLER message, state changes to INACTIVE.
  Sending a MSGTYPE_CONTROLLER_ACTIVATE, state changes to ACTIVE
  Sending a MSGTYPE_CONTROLLER_INACTIVATE, state changes to INACTIVE

  When configuring a controller, the client tells the server which signals to patch outputs and inputs to.
  Internal controller states are maintained and updated only in ACTIVE and INACTIVE state.
  UNCONFIGURED effectively means that the controller doesn't exist.

  =============Intended Use===================

  Configuration and network communication (e.g. to ROS) is handled by the client.
  The server mostly just listens and executes the given commands


  simple use with a "quiet" server (sending only):

  * open unix socket, send an ADVERTISE_VERSION followed by a RESET message

  * send configuration messages to setup the controllers used

  * send activation messages to activate the controllers

  * start issuing changes on the client signals


  with server replies:

  * bind to a unix socket for receiving replies from the server

  * open unix socket, send a reset message, and register your listening socket

  * send configuration messages to setup all controllers potentially used

  * send an activation message to activate the controller, patching them into the system
  
  * subscribe to signals you are interested in, server will send you periodic updates

  * start issuing changes on the client signals and handle incoming messages (e.g. MsgData8 and MsgAdvertiseVersion)


  ===========Safety Measures=================

  To provide some safety for operation, client signals have to be written periodically. 
  If a signal has not been changed for ALIVE_TIMEOUT, and is used by a controller, 
  the server will consider the signal to be "stale" and enters a nonrecoverable 
  safe state (opens all deflation valves).

  The safe state can only be exited by sending a reset message.

*/
#ifndef AIRSERVER_API_H_
#define AIRSERVER_API_H_

#ifdef _WIN32  //Basically for the RLAB client
  #include "stdint_windows.h"
  #include <time.h>
  #include "rostime.h"
#else
#include <stdint.h>
  #include <rostime.h>
#endif

#define NON_BEAGLE 0

#define EXIT_RESET 2
#define EXIT_NOT_EXEC 3
#define MAX_MSG_SIZE 128//Length of MsgBuf
#define PWM_CYCLE 100
namespace AirserverAPI {

  /*
    Distinguishes incompatible versions of the API

    Incompatibilities:
    version 3: swapped inflation and deflation valves - check your program whether i uses the defines (then it's ok) or computes id's directly (then you have to swap them manually)\

    version 4: Changed Subscription Messages ID and structure
    version 5: removed subscriber_id from subscription message
    version 6: Add PressureLimiter, remove limit on controller id's, increase number of available client signals
    version 7: refactor configuration messages, replace old monolithic message with controller-specific ones
    version 8: add possibility to monitor the EmergencySoftStop state on the client side via subscribing to a signal
               sends the client an MsgAdvertiseVersion upon connect (makes it incompatible with 7)
    version 9: adds mass controller (backward compatible with API8 clients)               
  */
const uint32_t PNEUMATICBOX_API_VERSION=9;

//returns true if the version used by the client is compatible with the server's version
inline bool IsClientCompatible(uint32_t clientApiVersion) 
{
 if (clientApiVersion >  9)  {return false;} //Don't not assume forward-compatibility with future client versions 
 if (clientApiVersion == 9)  {return true;} //compatible
 if (clientApiVersion == 8)  {return true;} //compatible
 if (clientApiVersion <= 7)  {return false;} 
 return false;
}


  const ROStime_t ALIVE_TIMEOUT = {0,250*1000000}; //250ms timeout

  /*
  signals within the airserver are referenced using a 32 bit unsigned integer:
  */
  typedef uint32_t AirServerSignal;
  typedef float AirServerSignalValue;
  
  
  /*
    The server internally uses signals to route data, some of them are fixed to real world input and output, 
    and some of them can be changed by the client
  
    signals are referenced by a 32 bit ID, available as #defines here.

  */

  const AirServerSignal BLOCK_SIGNALS_SENSOR=0x100;
  const AirServerSignal BLOCK_SIGNALS_VALVE=0x200;
  const AirServerSignal BLOCK_SIGNALS_SPECIAL=0x300;
  const AirServerSignal BLOCK_SIGNALS_CLIENT=0x400;      //use this offset when assigning client signals
  const AirServerSignal BLOCK_SIGNALS_CLIENT_MAX=0x7FF;  //last possible value for client signals
  const AirServerSignal SIGNALS_HIGHEST_ID=0x7FF;
  

  //pressure signal, values in kPa
  const AirServerSignal SIGNAL_PRESSURE_0 = (BLOCK_SIGNALS_SENSOR+0);
  const AirServerSignal SIGNAL_PRESSURE_1 = (BLOCK_SIGNALS_SENSOR+1);
  const AirServerSignal SIGNAL_PRESSURE_2 = (BLOCK_SIGNALS_SENSOR+2);
  const AirServerSignal SIGNAL_PRESSURE_3 = (BLOCK_SIGNALS_SENSOR+3);
  const AirServerSignal SIGNAL_PRESSURE_4 = (BLOCK_SIGNALS_SENSOR+4);
  const AirServerSignal SIGNAL_PRESSURE_5 = (BLOCK_SIGNALS_SENSOR+5);
  const AirServerSignal SIGNAL_PRESSURE_6 = (BLOCK_SIGNALS_SENSOR+6);

  //touch sensor signals:
  const int I2C_AD7147_MAXCOUNT = 4;
  const AirServerSignal BLOCK_SIGNALS_TOUCHSENSOR_FINGER1     = BLOCK_SIGNALS_SENSOR+0x10;
  const AirServerSignal BLOCK_SIGNALS_TOUCHSENSOR_FINGER1_MAX = BLOCK_SIGNALS_SENSOR+0x17;
  const AirServerSignal BLOCK_SIGNALS_TOUCHSENSOR_FINGER2     = BLOCK_SIGNALS_SENSOR+0x18;
  const AirServerSignal BLOCK_SIGNALS_TOUCHSENSOR_FINGER2_MAX = BLOCK_SIGNALS_SENSOR+0x1F;
  const AirServerSignal BLOCK_SIGNALS_TOUCHSENSOR_FINGER3     = BLOCK_SIGNALS_SENSOR+0x20;
  const AirServerSignal BLOCK_SIGNALS_TOUCHSENSOR_FINGER3_MAX = BLOCK_SIGNALS_SENSOR+0x27;
  const AirServerSignal BLOCK_SIGNALS_TOUCHSENSOR_FINGER4     = BLOCK_SIGNALS_SENSOR+0x28;
  const AirServerSignal BLOCK_SIGNALS_TOUCHSENSOR_FINGER4_MAX = BLOCK_SIGNALS_SENSOR+0x2F;

  /*valve activation signals
    value > 0.5: valve is switched on
    value < 0.5: valve is switched off
  */
  const AirServerSignal SIGNAL_VALVE_0 = (BLOCK_SIGNALS_VALVE+0);
  const AirServerSignal SIGNAL_VALVE_1 = (BLOCK_SIGNALS_VALVE+1); 
  const AirServerSignal SIGNAL_VALVE_2 = (BLOCK_SIGNALS_VALVE+2); 
  const AirServerSignal SIGNAL_VALVE_3 = (BLOCK_SIGNALS_VALVE+3); 
  const AirServerSignal SIGNAL_VALVE_4 = (BLOCK_SIGNALS_VALVE+4); 
  const AirServerSignal SIGNAL_VALVE_5 = (BLOCK_SIGNALS_VALVE+5); 
  const AirServerSignal SIGNAL_VALVE_6 = (BLOCK_SIGNALS_VALVE+6); 
  const AirServerSignal SIGNAL_VALVE_7 = (BLOCK_SIGNALS_VALVE+7); 
  const AirServerSignal SIGNAL_VALVE_8 = (BLOCK_SIGNALS_VALVE+8); 
  const AirServerSignal SIGNAL_VALVE_9 = (BLOCK_SIGNALS_VALVE+9); 
  const AirServerSignal SIGNAL_VALVE_10 = (BLOCK_SIGNALS_VALVE+10); 
  const AirServerSignal SIGNAL_VALVE_11 = (BLOCK_SIGNALS_VALVE+11); 
  const AirServerSignal SIGNAL_VALVE_12 = (BLOCK_SIGNALS_VALVE+12);
  const AirServerSignal SIGNAL_VALVE_13 = (BLOCK_SIGNALS_VALVE+13); 
  const AirServerSignal SIGNAL_VALVE_14 = (BLOCK_SIGNALS_VALVE+14); 
  const AirServerSignal SIGNAL_VALVE_15 = (BLOCK_SIGNALS_VALVE+15); 

  const AirServerSignal SIGNAL_COMPRESSOR = (BLOCK_SIGNALS_VALVE+0x81); //extra compressor switch, digital output. 

  //some special signals:
  const AirServerSignal SIGNAL_ZERO   = (BLOCK_SIGNALS_SPECIAL+0);     //always supplies 0.0
  const AirServerSignal SIGNAL_ONE    = (BLOCK_SIGNALS_SPECIAL+1);     //always supplies 1.0
  const AirServerSignal SIGNAL_NEGONE = (BLOCK_SIGNALS_SPECIAL+2);     //always supplies -1.0
  const AirServerSignal SIGNAL_NONE   = (BLOCK_SIGNALS_SPECIAL+3);     //you can give this as /dev/null

  const AirServerSignal SIGNAL_PWM_DURATION_0 = (BLOCK_SIGNALS_SPECIAL+4); //304
  const AirServerSignal SIGNAL_PWM_DURATION_1 = (BLOCK_SIGNALS_SPECIAL+5); //305
  const AirServerSignal SIGNAL_PWM_DURATION_2 = (BLOCK_SIGNALS_SPECIAL+6); //306
  const AirServerSignal SIGNAL_PWM_DURATION_3 = (BLOCK_SIGNALS_SPECIAL+7); //307
  const AirServerSignal SIGNAL_PWM_DURATION_4 = (BLOCK_SIGNALS_SPECIAL+8); //308
  const AirServerSignal SIGNAL_PWM_DURATION_5 = (BLOCK_SIGNALS_SPECIAL+9); //309
  const AirServerSignal SIGNAL_PWM_DURATION_6 = (BLOCK_SIGNALS_SPECIAL+10); //30A
  const AirServerSignal SIGNAL_PWM_DURATION_7 = (BLOCK_SIGNALS_SPECIAL+11); //30B
  const AirServerSignal SIGNAL_PRESSURE_START_0 = (BLOCK_SIGNALS_SPECIAL+12); //30C
  const AirServerSignal SIGNAL_PRESSURE_START_1 = (BLOCK_SIGNALS_SPECIAL+13); //30D
  const AirServerSignal SIGNAL_PRESSURE_START_2 = (BLOCK_SIGNALS_SPECIAL+14); //30E 
  const AirServerSignal SIGNAL_PRESSURE_START_3 = (BLOCK_SIGNALS_SPECIAL+15); //30F
  const AirServerSignal SIGNAL_PRESSURE_START_4 = (BLOCK_SIGNALS_SPECIAL+16); //310
  const AirServerSignal SIGNAL_PRESSURE_START_5 = (BLOCK_SIGNALS_SPECIAL+17); //311
  const AirServerSignal SIGNAL_PRESSURE_START_6 = (BLOCK_SIGNALS_SPECIAL+18); //312
  const AirServerSignal SIGNAL_PRESSURE_START_7 = (BLOCK_SIGNALS_SPECIAL+19); //313
  
  const AirServerSignal SIGNAL_EMERGENCY_STOP  = (BLOCK_SIGNALS_SPECIAL+0xFE);     //0x333: signals whether the server is in emergency soft stop mode (>0.5) or not (<0.5). Setting it does NOT trigger an emergency stop!
  
  /*
    Most times, a pair of valves and a pressure sensors are connected to 
    form a channel where we can inflate, deflate, keep air, and measure the channel pressure

    The mapping here is a suggested default, and should be reflected in the actual pneumatics plumbing if used.
  */

  const AirServerSignal SIGNAL_CHANNEL0_INFLATE  = SIGNAL_VALVE_0; 
  const AirServerSignal SIGNAL_CHANNEL0_DEFLATE  = SIGNAL_VALVE_1;
  const AirServerSignal SIGNAL_CHANNEL0_PRESSURE = SIGNAL_PRESSURE_0;

  const AirServerSignal SIGNAL_CHANNEL1_INFLATE  = SIGNAL_VALVE_2; 
  const AirServerSignal SIGNAL_CHANNEL1_DEFLATE  = SIGNAL_VALVE_3; 
  const AirServerSignal SIGNAL_CHANNEL1_PRESSURE = SIGNAL_PRESSURE_1;

  const AirServerSignal SIGNAL_CHANNEL2_INFLATE  = SIGNAL_VALVE_4; 
  const AirServerSignal SIGNAL_CHANNEL2_DEFLATE  = SIGNAL_VALVE_5; 
  const AirServerSignal SIGNAL_CHANNEL2_PRESSURE = SIGNAL_PRESSURE_2;

  const AirServerSignal SIGNAL_CHANNEL3_INFLATE  = SIGNAL_VALVE_6;
  const AirServerSignal SIGNAL_CHANNEL3_DEFLATE  = SIGNAL_VALVE_7;
  const AirServerSignal SIGNAL_CHANNEL3_PRESSURE = SIGNAL_PRESSURE_3; 

  const AirServerSignal SIGNAL_CHANNEL4_INFLATE  = SIGNAL_VALVE_8;
  const AirServerSignal SIGNAL_CHANNEL4_DEFLATE  = SIGNAL_VALVE_9;
  const AirServerSignal SIGNAL_CHANNEL4_PRESSURE = SIGNAL_PRESSURE_4; 

  const AirServerSignal SIGNAL_CHANNEL5_INFLATE  = SIGNAL_VALVE_10;
  const AirServerSignal SIGNAL_CHANNEL5_DEFLATE  = SIGNAL_VALVE_11;
  const AirServerSignal SIGNAL_CHANNEL5_PRESSURE = SIGNAL_PRESSURE_5;

  const AirServerSignal SIGNAL_CHANNEL6_INFLATE  = SIGNAL_VALVE_12;
  const AirServerSignal SIGNAL_CHANNEL6_DEFLATE  = SIGNAL_VALVE_13;
  //no pressure sensor

  const AirServerSignal SIGNAL_CHANNEL7_INFLATE  = SIGNAL_VALVE_14;
  const AirServerSignal SIGNAL_CHANNEL7_DEFLATE  = SIGNAL_VALVE_15;
  //no pressure sensor

  const AirServerSignal SIGNAL_SUPPLY_PRESSURE = SIGNAL_PRESSURE_6;

  //Default configuration for the channels in a convenient array:
  const AirServerSignal SIGNALS_CHANNELS[8][3] 
  #ifndef _WIN32
      __attribute__((unused))
  #endif
   = {
           {SIGNAL_CHANNEL0_INFLATE, SIGNAL_CHANNEL0_DEFLATE, SIGNAL_CHANNEL0_PRESSURE}
          ,{SIGNAL_CHANNEL1_INFLATE, SIGNAL_CHANNEL1_DEFLATE, SIGNAL_CHANNEL1_PRESSURE}
          ,{SIGNAL_CHANNEL2_INFLATE, SIGNAL_CHANNEL2_DEFLATE, SIGNAL_CHANNEL2_PRESSURE}
          ,{SIGNAL_CHANNEL3_INFLATE, SIGNAL_CHANNEL3_DEFLATE, SIGNAL_CHANNEL3_PRESSURE}
          ,{SIGNAL_CHANNEL4_INFLATE, SIGNAL_CHANNEL4_DEFLATE, SIGNAL_CHANNEL4_PRESSURE}
          ,{SIGNAL_CHANNEL5_INFLATE, SIGNAL_CHANNEL5_DEFLATE, SIGNAL_CHANNEL5_PRESSURE}
          ,{SIGNAL_CHANNEL6_INFLATE, SIGNAL_CHANNEL6_DEFLATE, SIGNAL_NONE}
          ,{SIGNAL_CHANNEL7_INFLATE, SIGNAL_CHANNEL7_DEFLATE, SIGNAL_NONE}
  };

  /*********************The Header definition every message must have:******************************/

  /*
    message IDs
  */
  typedef uint32_t eMessageType; //defines all message types that exist on the interface
  
  const eMessageType  MSGTYPE_ADVERTISE_VERSION = 0xFFFF;    //used by client to advertise the api version it is going to use

  const eMessageType  MSGTYPE_EVENT = 1;                     //schedule value changes of signals

  const eMessageType  MSGTYPE_RESET = 2;                     //reset the complete server configuration

  const eMessageType  MSGTYPE_CONTROLLER = 0x100;            //(re)configure the controllers used (old style common configuration message)
  const eMessageType  MSGTYPE_CONTROLLER_ACTIVATE = 0x101;   //enable a deactivated controller
  const eMessageType  MSGTYPE_CONTROLLER_INACTIVATE = 0x102; //disable an activated controller, but configuration is kept/maintained
  const eMessageType  MSGTYPE_CONTROLLER_DELETE = 0x103;     //deactivates and destroys controller
  const eMessageType  MSGTYPE_CONFIG = 0x104;                //(re)configure a controller or input/output block

  const eMessageType  MSGTYPE_SYNC = 0x200;                  //sync all configuration changes / barrier to report back when all earlier messages were processed
  const eMessageType  MSGTYPE_SYNC_REPLY = 0x201;             //reply message of the sync/barrier request    

  const eMessageType  MSGTYPE_SUBSCRIBE = 0x301;             //subscribe to the given signal (periodic updates)
  const eMessageType  MSGTYPE_UNSUBSCRIBE = 0x302;           //unsubscribe from the given signal
  const eMessageType  MSGTYPE_DATA8 = 0x303;                 //subscription message containing (up to) 8 signals
  const eMessageType  MSGTYPE_BROADCAST8 = 0x304;            //reply message that contains up to 8 signals

  
  /*new-style individual configuration messages:*/
  const eMessageType MSGTYPE_CONFIG_CONTROLLER_THRESHOLD = 0x2000 + 1;  //configure a threshold controller
  const eMessageType MSGTYPE_CONFIG_CONTROLLER_BANGBANG = 0x2000 + 2; //configure a bang-bang controller
  const eMessageType MSGTYPE_CONFIG_CONTROLLER_MASS = 0x2000 + 3; //configure a mass/massflow controller
  const eMessageType MSGTYPE_CONFIG_CONTROLLER_WATCHDOG_PRESSURE = 0x2000 + 4; //configure a watchdog for pressure
  const eMessageType MSGTYPE_CONFIG_CONTROLLER_PRESSURE_LIMITER = 0x2000 + 5; //configure a pressure limiter
  const eMessageType MSGTYPE_CONFIG_I2CSENSOR_AD7147 = 0x3000 + 1;  //provides sensor readings from AD7147 via I2C 

  /*
    common header used for all messages
  */
  typedef struct {
    eMessageType type;
    ROStime_t s_time;  //timestamp or time at which a message should take effect
  } MsgHeader;

  //Header for individualized controller configuration messages
  //The plan is to phase out the common configuration message above and have special messages for each controller
  // uses the ID MSGTYPE_CONFIG
  typedef struct {
        MsgHeader header;
        uint32_t id;//controller id
  } HeaderConfigurationMsgs; 



  /**** client messages ****/


  /*
  
  This message advertises the API version the client is going to use
  
  This message is intended to catch incompatibilities, 
  the server will complain loudly (and drop the connection) if an unsupported version is used  
  
  The client should send it before issuing any other message
  */
  
  typedef struct {
    MsgHeader header;
    uint32_t version;  
  } MsgAdvertiseVersion;




/*
  Configures a Threshold Controller
  
  This controller provides control of two binary valves in a 5/3 configration via a ternary state.
  The desired state is given as float value. 
  
  in_desired: Desired state of the valves
                -0.5 < in_desired < 0.5: disconnect channel
                in_desired > 0.5: inflate channel
                in_desired < -0.5: deflate channel
  Usually, clients send the values [-1, 0, 1].
  
  out_inflate: signal that controls the inflation valve
  out_deflate: signal that controls the deflation valve
  (have a look at SIGNALS_CHANNELS for the default hardware setup)
  
  in_desired: Signal 
*/
  typedef struct {
    HeaderConfigurationMsgs header_config;    
    
      AirServerSignal in_desired; 
    AirServerSignal out_inflate;    
    AirServerSignal out_deflate;
  } MsgConfigurationControllerThreshold; 



/*
    This controller provides a simple Bang-Bang control using three trigger levels, and controls two antagonistic valves
    
    in_current: signal to compare the levels with (usually a pressure sensor signal)
    out_inflate: signal that controls an inflation valve
    out_deflate: signal that controls an deflation valve

    level_inflate:  value below which the inflation valve is switched on.
    level_off:      value above which the inflation valve is switched off and below which the deflation valve is switched off. (target value)
    level_deflate:  value above which the deflation valve is switched on.

    Usually: level_inflate < level_off < level_deflate

*/
  typedef struct {
    HeaderConfigurationMsgs header_config;    
    
      AirServerSignal in_current; 
      AirServerSignal out_inflate; 
      AirServerSignal out_deflate; 
    AirServerSignalValue level_inflate;    
    AirServerSignalValue level_off;
    AirServerSignalValue level_deflate;    
  } MsgConfigurationControllerBangBang; 


/*
  Configures a Mass Controller
  
  
  in_desired_massflow: set the desired mass flow, if set to SIGNAL_NONE use the desired mass signal instead
  in_desired_mass:     set the desired mass. Is ignored if in_desired_massflow is set to SIGNAL_NONE
  
  out_inflate: signal to the valve responsible for inflating
  out_deflate: signal to the valve responsible for deflating
  
  pressure_supply: pressure at the inlet of the inflating valve (usually the supply pressure)
  pressure_out:    pressure at the outlet of the inflating valve, inlet of the deflating valve (usually the channel pressure)

  out_massobserver: current estimate of the mass contained in the channel by the mass observer (for monitoring/calibration, set to SIGNAL_NONE if not used)
  out_massobserver_friction: current estimate of the channel mass, friction term (for monitoring/calibration, set to SIGNAL_NONE if not used)
  out_massobserver_injector: current estimate of the channel mass, injector term (for monitoring/calibration, set to SIGNAL_NONE if not used)
  
  c_*: Coefficients of the mass observer terms; used to calibrate the mass observer for the given channel hardware. 

  c_inflation_threshold, c_deflation_threshold: mass error to tolerate before controller switches a valve (deadband / hysteresis of the bang-bang control).
  
*/
  typedef struct {
    HeaderConfigurationMsgs header_config;    
        
    AirServerSignal in_desired_massflow;   
    AirServerSignal in_desired_mass;       
    
    AirServerSignal out_inflate;    
    AirServerSignal out_deflate;
    AirServerSignal pressure_supply;
    AirServerSignal pressure_out;

    AirServerSignal out_massobserver;
    AirServerSignal out_massobserver_term_friction;
    AirServerSignal out_massobserver_term_injector;
 
    AirServerSignalValue c_inflate;       
    AirServerSignalValue c_inflate_friction;
    AirServerSignalValue c_inflate_injector; 
    AirServerSignalValue c_inflate_Psupply; 
    AirServerSignalValue c_inflate_Pout;  

    AirServerSignalValue c_deflate;       
    AirServerSignalValue c_deflate_friction;
    AirServerSignalValue c_deflate_injector; 
    AirServerSignalValue c_deflate_Psupply; 
    AirServerSignalValue c_deflate_Pout;  

    AirServerSignalValue c_inflation_threshold;
    AirServerSignalValue c_deflation_threshold;

  } MsgConfigurationControllerMass; 


  typedef struct {
    HeaderConfigurationMsgs header_config;
    
    AirServerSignalValue limit_value;
    AirServerSignal signal_to_watch;
    float filtertime_signal_to_watch;
    AirServerSignal limit_signal;
    AirServerSignal inflate_valve;        
  } MsgConfigurationControllerPressureLimiter; 

/*
    CONFIG_CONTROLLER_WATCHDOG_PRESSURE: watchdog doing emergency shutdown upon reaching a certain pressure.
    The controller knows two limits, one is a fixed constant, the other is a signal    
    limit_value: absolute maximum pressure in kPa (fixed value)
    limit_signal: maximum pressure to allow in kPa (set to SIGNAL_NONE to disable)
    signal_to_watch: signal of the current pressure sensor
    filtertime_signal_to_watch: low pass filter constant (in seconds)
*/
  typedef struct {
    HeaderConfigurationMsgs header_config;
    
    AirServerSignalValue limit_value;
    AirServerSignal signal_to_watch;
    float filtertime_signal_to_watch;
    AirServerSignal limit_signal;
  } MsgConfigurationWatchdogPressure; 
  
  
  typedef struct {
    HeaderConfigurationMsgs header_config;
    
    uint32_t busnumber;
    uint32_t address;    
    uint32_t chip_index;
  } MsgConfigurationI2CSensorAD7147; 



  /*
    message to explicitly activate or deactivate a preconfigured controller
  */
  typedef struct {
    MsgHeader header;
    uint32_t id;//controller  id
  } MsgController;


  /*
    reset the server to its default state:

    reset all controllers
    reset all client signals to 0.0
    send future replies to the provided unix socket name
  */
  typedef struct {
    MsgHeader header;
  } MsgReset;  



  /*message to request a reply for client side blocking.

    when getting the stampreply, all configurations with lesser or equal times have been executed*/
  typedef struct {
    MsgHeader header;
    int32_t subscriber_id; //used in the server to distinguish subscribers. Set to 0
    uint32_t hash;    //arbitrary value to link request to reply message
  } MsgSync;  



  typedef struct {
    MsgHeader header;
    AirServerSignal signal_id; //which input signal it affects. Must obey BLOCK_SIGNALS_CLIENT <= signal_id <= BLOCK_SIGNALS_CLIENT_MAX
    AirServerSignalValue value;       //the value to set at the scheduled time
  } MsgSignalEvent;  //message to schedule value changes of client signals


  /**** server reply messages ****/

  /*reply*/
  typedef struct {
    MsgHeader header;
    int32_t subscriber_id;  //used in the server to distinguish subscribers. Set to 0    
    int32_t hash;
  } MsgSyncReply; //reply to a sync request


  typedef struct {
    MsgHeader header;
    AirServerSignal signal_id;     
    ROStime_t period;  
  } MsgSubscribe;

  typedef struct {
    MsgHeader header;
    AirServerSignal signal_id; 
  } MsgUnSubscribe;

  typedef struct {
    MsgHeader header;
    struct {
        AirServerSignal signal_id;  
        AirServerSignalValue value;
    } signals[8];
  } MsgData8;

  typedef struct {
    MsgHeader header;
    struct {
        AirServerSignal signal;
        AirServerSignalValue value;
    } signals[8];
  } MsgBroadcast8;



  /*
    convenience function to look up the right buffer size
  */
  inline const int getMessageTypeLength(eMessageType type)
  {
    if (type == MSGTYPE_EVENT)                 {return sizeof(MsgSignalEvent);}
    if (type == MSGTYPE_CONTROLLER_ACTIVATE)   {return sizeof(MsgController);}
    if (type == MSGTYPE_CONTROLLER_INACTIVATE) {return sizeof(MsgController);}
    if (type == MSGTYPE_CONTROLLER_DELETE)     {return sizeof(MsgController);}
    if (type == MSGTYPE_SYNC)                  {return sizeof(MsgSync);}
    if (type == MSGTYPE_SYNC_REPLY)            {return sizeof(MsgSyncReply);}
    if (type == MSGTYPE_RESET)                 {return sizeof(MsgReset);}
    if (type == MSGTYPE_ADVERTISE_VERSION)     {return sizeof(MsgAdvertiseVersion);}
    if (type == MSGTYPE_SUBSCRIBE)             {return sizeof(MsgSubscribe);}
    if (type == MSGTYPE_UNSUBSCRIBE)           {return sizeof(MsgUnSubscribe);}
    if (type == MSGTYPE_DATA8)                 {return sizeof(MsgData8);}
    if (type == MSGTYPE_BROADCAST8)            {return sizeof(MsgBroadcast8);}

    if (type == MSGTYPE_CONFIG_CONTROLLER_THRESHOLD)            {return sizeof(MsgConfigurationControllerThreshold);}
    if (type == MSGTYPE_CONFIG_CONTROLLER_BANGBANG)             {return sizeof(MsgConfigurationControllerBangBang);}
    if (type == MSGTYPE_CONFIG_CONTROLLER_MASS)                 {return sizeof(MsgConfigurationControllerMass);}
    if (type == MSGTYPE_CONFIG_CONTROLLER_WATCHDOG_PRESSURE)    {return sizeof(MsgConfigurationWatchdogPressure);}
    if (type == MSGTYPE_CONFIG_CONTROLLER_PRESSURE_LIMITER)     {return sizeof(MsgConfigurationControllerPressureLimiter);}
    if (type == MSGTYPE_CONFIG_I2CSENSOR_AD7147)                {return sizeof(MsgConfigurationI2CSensorAD7147);}

     
    return 0; //unknown message type will return 0
  }

  #define MAX_LENGTH (sizeof(MsgConfigurationControllerMass))
  
  const unsigned int MAX_MESSAGE_LENGTH = ((4+30)*sizeof(uint32_t));  //maximum: message with 30 fields 

} //namespace pneumaticbox
#endif /* PNEUMATICBOX_API_H_ */
