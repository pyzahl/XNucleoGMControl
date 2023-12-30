// X-Nucleo-Driver 3 Motor Configuration
// X-Nucleo Drivers via SPI on Adruino/Gunino Uno ONLY without modifcations
// x-compile on Duo for simulation only

// LX200 protcol
// http://www.meade.com/support/TelescopeProtocol_2010-10.pdf
//

//#define USE_UNO

#ifdef USE_UNO
#else
#define USE_DUO
#define  ENABLE_HELP
#define  QUICK_CMDS
#define  DEBUG_EXTRA
#define  OTHER_COMANDS
#endif

#define SERIAL_ECHO
#define DISPLAY_ECHO


//#define HD_RUN_SIMULTANEOUS_ENABLE //-- potential for power glitch troubles, disable

#define ENABLE_I2C_KEYPAD
#define TRACK_AFTER_STARTUP
//#define MOTOR_FWD_TEST_AFTER_STARTUP
//#define  VERBOSE_ECHO
#define  VERBOSE_STARTUP
#define USE_TIMER

#include <SPI.h>
#include <powerSTEP01ArduinoLibrary.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>

#include <Wire.h> // I2C
//  http://www.instructables.com/id/Adding-an-MCP23017-IO-Extender-to-Arduino-or-ESP82/ 

#ifdef USE_UNO
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#else // DUE
#include <HardwareSerial.h>
#include <DueTimer.h>
#endif

#include <SD.h>

File myFile;


#include <AstroMini_Library.h>

#define VERSION_FWDATE1    "Feb 02 2019#\n"
#define VERSION_FWDATE2    "Mar 03 2019#\n"
#define VERSION_NUMBER     "00.1#\n"
#define VERSION_PRODUCT    "GM-K150-Py-XNucleo#\n"
#define VERSION_CONTROLBOX "GM-K150-Py-I2C#\n"
#define VERSION_INFO       "Arduino X-Nucleo K150"
#define VERSION_INFO_LONG  "Arduino X-Nucleo IHM03A1 GM TVAT#\n"

#define SERIAL_PRG_BAUDRATE  19200
#define SERIAL1_BAUDRATE     19200
#define SERIAL2_BAUDRATE     19200
#define SERIAL3_BAUDRATE     9600 // BLUE BLE4.0 default

// MCP23x17 control registers
#define MCP_I2C_SELECT     0x24 // I2C Adresse: 0x20 + 4  (A2=HI)
// ==================================================================================================
#define MCP23S17_IODIRA    0x00 // IO7 IO6 IO5 IO4 IO3 IO2 IO1 IO0  1111 1111
#define MCP23S17_IODIRB    0x01 // IO7 IO6 IO5 IO4 IO3 IO2 IO1 IO0  1111 1111
#define MCP23S17_IPOLA     0x02 // IP7 IP6 IP5 IP4 IP3 IP2 IP1 IP0  0000 0000
#define MCP23S17_IPOLB     0x03 // IP7 IP6 IP5 IP4 IP3 IP2 IP1 IP0  0000 0000
#define MCP23S17_GPINTENA  0x04 // GPINT7 GPINT6 GPINT5 GPINT4 GPINT3 GPINT2 GPINT1 GPINT0  0000 0000
#define MCP23S17_GPINTENB  0x05 // GPINT7 GPINT6 GPINT5 GPINT4 GPINT3 GPINT2 GPINT1 GPINT0  0000 0000
#define MCP23S17_DEFVALA   0x06 // DEF7 DEF6 DEF5 DEF4 DEF3 DEF2 DEF1 DEF0  0000 0000
#define MCP23S17_DEFVALB   0x07 // DEF7 DEF6 DEF5 DEF4 DEF3 DEF2 DEF1 DEF0  0000 0000
#define MCP23S17_INTCONA   0x08 // IOC7 IOC6 IOC5 IOC4 IOC3 IOC2 IOC1 IOC0  0000 0000
#define MCP23S17_INTCONB   0x09 // IOC7 IOC6 IOC5 IOC4 IOC3 IOC2 IOC1 IOC0  0000 0000
#define MCP23S17_IOCONA    0x0A // BANK MIRROR SEQOP DISSLW HAEN ODR INTPOL —  0000 0000
#define MCP23S17_IOCONB    0x0B // BANK MIRROR SEQOP DISSLW HAEN ODR INTPOL —  0000 0000
#define MCP23S17_GPPUA     0x0C // PU7 PU6 PU5 PU4 PU3 PU2 PU1 PU0  0000 0000
#define MCP23S17_GPPUB     0x0D // PU7 PU6 PU5 PU4 PU3 PU2 PU1 PU0  0000 0000
#define MCP23S17_INTFA     0x0E // INT7 INT6 INT5 INT4 INT3 INT2 INT1 INTO  0000 0000
#define MCP23S17_INTFB     0x0F // INT7 INT6 INT5 INT4 INT3 INT2 INT1 INTO  0000 0000
#define MCP23S17_INTCAPA   0x10 // ICP7 ICP6 ICP5 ICP4 ICP3 ICP2 ICP1 ICP0  0000 0000
#define MCP23S17_INTCAPB   0x11 // ICP7 ICP6 ICP5 ICP4 ICP3 ICP2 ICP1 ICP0  0000 0000
#define MCP23S17_GPIOA     0x12 // GP7 GP6 GP5 GP4 GP3 GP2 GP1 GP0  0000 0000
#define MCP23S17_GPIOB     0x13 // GP7 GP6 GP5 GP4 GP3 GP2 GP1 GP0  0000 0000
#define MCP23S17_OLATA     0x14 // OL7 OL6 OL5 OL4 OL3 OL2 OL1 OL0  0000 0000
#define MCP23S17_OLATB     0x15 // OL7 OL6 OL5 OL4 OL3 OL2 OL1 OL0  0000 0000
// ===================================================================================================

// KeyPad

// MCP23S16 PORT B write
#define KX1 ((~4)&7)
#define KX2 ((~2)&7)
#define KX3 ((~1)&7)


// MCP23S16 PORT A read
#define KY1 8
#define KY2 4
#define KY3 2
#define KY4 1

#define ST4_HP 16
#define ST4_HM 32
#define ST4_DP 64
#define ST4_DM 128

byte key_pad_led_status = 0;
int key_pad_enable = 0;

int auto_track_HAbyRA = 0;
long track_error=0L;
long track_error_dt=0L;

byte update_display = 0;
int  i2c_error = 0;
byte clear_errors = 0;


/*
// ISR 0 on pin 2
// ISR 1 on pin 3
volatile int count;

#define encoderI 2
#define encoderQ 4 // only use one ISR

void handlerEncoder()
{
if (digitalRead(encoderI) == digitalRead(encoderQ))
count++;
else
count--;
}

*/

#define STEPPER_POS_RANGE   4194304L  // (1<<22)
#define STEPPER_POS_RANGE0 -2097151L  //((1L<<21)-1)
#define STEPPER_POS_RANGEH  1048576L  // USED FOR LOOP RANGE CHECK 

#define C_HH  (long)(288*3)          // Getriebeuebersetzung * Steps je Umdrehung x50 with 2nd worm drive, 60/20 x3 with belt
#define NS_U  (long)(C_HH*200*1.0)     // at full step
#define NS_US (long)(C_HH*200*128.0)   // at 1/128 step
#define STAGs 86164.091

#define PI2        (float)(6.28318530717958647692)
#define NS_US_RAD  (float)(PI2/(float)NS_US)
#define NS_US_DEG  (float)(360./(float)NS_US)
#define NS_US_HMS  (float)(24./(float)NS_US)
#define NS_US_DEGSEC (float)(360.*3600./(float)NS_US)
#define NS_US_HMSEC  (float)(24.*3600./(float)NS_US)
#define NS_PER_DEG (long)(NS_U / 360)

#define RAD_NS_US  (float)((float)NS_US/PI2)
#define RAD_HSEC   (float)(24.*3600./PI2)
#define RAD_SEC    (float)(360.*3600./PI2)
#define SEC_RAD    (float)(PI2/(360.*3600.))
#define HSEC_RAD   (float)(PI2/(24.*3600.))

#define SIDERAL_PERIOD (23.9344699*3600.0) // seconds
#define SIDERAL_RATIO  (23.9344699/24)
#define SIDERAL_H_HZ   ((float)NS_U / SIDERAL_PERIOD)  // = 256.7009 Hz (theoretically) vs measured via millis timer: 257.72770158, Q is which time base is better

#define GT_CONVERT60HZ  (60.0*SIDERAL_PERIOD/(float)NS_U)   // = 15402.054
#define ST_CONVERT60HZ  ((float)NS_U/(60.0*SIDERAL_PERIOD)) // = 0.0000649264049
#define TR_ADJUST01HZ   (ST_CONVERT60HZ*0.1)

#define SOLAR_PERIOD (24.0*3600.0) // seconds
#define SOLAR_H_HZ   ((float)NS_U / SOLAR_PERIOD)  // = 267.3967712676 Hz

#define LUNAR_PERIOD (24.0*3600.0+50.0*60.0) // seconds
#define LUNAR_H_HZ   ((float)NS_U / LUNAR_PERIOD)  // = 267.3967712676 Hz

#define SLEW_FINE_HZ        0  // fine 1
#define SLEW_FFINE_HZ       1  // fine 2
#define SLEW_FFFINE_HZ      2  // fine 3
#define SLEW_FAST_HZ        3  // fast 1
#define SLEW_FFAST_HZ       4  // fast 2 
#define SLEW_FFFAST_HZ      5  // fast 3 
#define SLEW_FFFFAST_HZ     6  // fast 4 

#define NUM_SLEW_RATES 7

#define MOTOR_H      0
#define MOTOR_D      1
#define MOTOR_F      2

int trk_power=42; // Tracking RunKVAL

int slew_rate_mode [2] = { 3,3 };

#define MICRO_STEP 1
float slew_rate_list[8] = {
  MICRO_STEP*20.0,   // SLEW_FINE_HZ
  MICRO_STEP*60.0,  // SLEW_FFINE_HZ
  MICRO_STEP*100.0,  // SLEW_FFFINE_HZ
  MICRO_STEP*200.0,  // SLEW_FAST_HZ
  MICRO_STEP*400.0, // SLEW_FFAST_HZ
  MICRO_STEP*800.0, // SLEW_FFFAST_HZ
  MICRO_STEP*1600.0, // SLEW_FFFFAST_HZ
  0.0
};


float st4correct_speed_list[8] = {
  SIDERAL_H_HZ*0.2, // SLEW_FINE_HZ
  SIDERAL_H_HZ*0.5, // SLEW_FFINE_HZ
  SIDERAL_H_HZ*1.0, // SLEW_FFFINE_HZ
  SIDERAL_H_HZ*2.0, // SLEW_FAST_HZ
  SIDERAL_H_HZ*5.0, // SLEW_FFAST_HZ
  SIDERAL_H_HZ*10.0,  // SLEW_FFFAST_HZ
  SIDERAL_H_HZ*100.0, // SLEW_FFFFAST_HZ
  0.0
};

#define ST4_CORRECT_SPEED st4correct_speed_list [slew_i] // ST4 AUTO GUIDE CORRECTION SPEED


#define SECS_PER_MIN    (60UL)
#define SECS_PER_HOUR   (3600UL)
#define SECS_PER_DEGREE (3600UL)
#define SECS_PER_DAY    (SECS_PER_HOUR * 24L)
#define SECS_PER_REV    (SECS_PER_DEGREE * 360L)

/* Macros for HMS, DMS */
#define numberOfSeconds(_dec_) (_dec_ % SECS_PER_MIN)
#define numberOfMinutes(_dec_) ((_dec_ % SECS_PER_HOUR) / SECS_PER_MIN)
#define numberOfHours(_dec_) (( _dec_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_dec_) ( _dec_ / SECS_PER_DAY)
#define numberOfDegrees(_dec_) (( _dec_% SECS_PER_REV) / SECS_PER_DEGREE)

#define H_FWD FWD
#define H_REV REV
#define D_FWD FWD
#define D_REV REV


enum LX200_K150_GSTAT
{
    GSTAT_UNSET                       = -999,
    GSTAT_TRACKING                    = 0,
    GSTAT_STOPPED                     = 1,
    GSTAT_PARKING                     = 2,
    GSTAT_UNPARKING                   = 3,
    GSTAT_SLEWING_TO_HOME             = 4,
    GSTAT_PARKED                      = 5,
    GSTAT_SLEWING_OR_STOPPING         = 6,
    GSTAT_NOT_TRACKING_AND_NOT_MOVING = 7,
    GSTAT_MOTORS_TOO_COLD             = 8,
    GSTAT_TRACKING_OUTSIDE_LIMITS     = 9,
    GSTAT_FOLLOWING_SATELLITE         = 10,
    GSTAT_NEED_USEROK                 = 11,
    GSTAT_UNKNOWN_STATUS              = 98,
    GSTAT_ERROR                       = 99
};

struct _StarTime {
  int hms[3];
  int hoffset;
  int ymd[3];
} startime;

struct _Coords {
  int lat[3];
  int lon[3];
} site;

float focus_rate = 128.0;
float slew_rate[2] = { MICRO_STEP*400.0, MICRO_STEP*400.0 };
float tracking_rate[2] = { SIDERAL_H_HZ, 0.0 };
float custom_tracking[2] = { SIDERAL_H_HZ, 0.0 }; // custom tracking speeds H, D
byte  tracking_dir[2] = { H_FWD, D_FWD };

long alt_dec_backlash = 0L;
long az_ra_backlash = 0L;

// scratch string buffer
char tmp[64];


#ifdef USE_TIMER
void controller_isr();
#endif

// UNO
// SerialUSB / Bluetooth, Serial
#define rxPin 6
#define txPin 7

// set up a new serial port

#ifdef USE_UNO
SoftwareSerial BlueSerial(rxPin, txPin); // RX pin, TX pin
#endif

// X-Nucleo Controllers
// Pin definitions for the X-NUCLEO-IHM03A1 connected to an Uno-compatible board
#define nCS_PIN 10
#define STCK_PIN 9
#define nSTBY_nRESET_PIN 8
#define nBUSY_PIN 5 // not used actually, SPI status register is used

// SSD1351 display (SPI) 1.5" OLED
// Screen dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128

#define SSD1351_DC_PIN   7
#define SSD1351_CS_PIN   52
#define SSD1351_RST_PIN  6

// SD Card
#define SD_CS_PIN  4

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define ORANGE          0xFBE0
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

// Option 1: use any pins but a little slower
//Adafruit_SSD1351 display = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, CSSD1351_S_PIN, SSD1351_DC_PIN, MSSD1351_OSI_PIN, SSSD1351_CLK_PIN, RST_PIN);  

// Option 2: must use the hardware SPI pins 
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be 
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
Adafruit_SSD1351 display = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, SSD1351_CS_PIN, SSD1351_DC_PIN, SSD1351_RST_PIN);


// powerSTEP library instance, parameters are distance from the end of a daisy-chain
// of stepperHs, !CS pin, !STBY/!Reset pin

powerSTEP stepperH(0, nCS_PIN, nSTBY_nRESET_PIN); // Hour Axis Motor
powerSTEP stepperD(1, nCS_PIN, nSTBY_nRESET_PIN); // Declination Axis Motor
powerSTEP stepperF(2, nCS_PIN, nSTBY_nRESET_PIN); // Focus Motor

// Mini Astro Library
AstroMini Astro;



// Manage Motor Position (+/- 2^21) internally only
// NS_US =(long)(C_HH*200*128.0)   // at 1/128 step
class Position {
  public:
  Position (powerSTEP *motor, long initial_pos=0L) {
      ip = 0;
      tip = 0;
      distance = 0;
      goto_pos = -1;
      last_reading = 0;
      m = motor;
  };
  void Update (){
      if (goto_pos >= 0)
        Goto(goto_pos);
      else
        Get();
      // loop detection
      if (last_reading-mp > STEPPER_POS_RANGEH)
        ++ip;
      if (last_reading-mp < -STEPPER_POS_RANGEH)
        --ip;
      last_reading = mp;
  };
  void Abort (){
      goto_pos = -1;
      distance = 0;
  };
  void Goto(long p){
      int tip;
      long tmp;
      long fin;
      goto_pos = p;
      Get ();
      tmp = p-pos;
      distance = tmp/NS_PER_DEG;
      tip = tmp/(STEPPER_POS_RANGE);
      if (m->busyCheck() || tmp == 0L)
        return;
        
      if (tip != 0){
        //Serial.println(" * GOTO: MOVE"+String(pos)+" -> "+String(p)+" ["+String(tip)+", Delta="+String(tmp)+"]");
        m->move(tip>0?FWD:REV, STEPPER_POS_RANGE);
        if (tip>0)
          --tip;
        else
          ++tip;
      } else {
        fin = p%STEPPER_POS_RANGE;
        //Serial.println(" * GOTO: FINAL"+String(pos)+" -> "+String(p)+" ["+String(tmp)+"]");
        m->goToDir(tmp>0?FWD:REV, fin - STEPPER_POS_RANGE0);
        goto_pos = -1; // completed
        distance = 0;
      }
  };
  int Distance(){
      return abs(distance);
  };
  int isSlewing(){ return goto_pos > 0 ? 1:0; };
  long Get() {
      mp = m->getPos();
      pos = STEPPER_POS_RANGE*ip + mp + STEPPER_POS_RANGE0;
      return pos;
  };
  float Get_DEG_Angle() {
    Get ();
    return (float)NS_US_DEG * pos;
  };
  float Get_DEGSEC_Angle() {
    Get ();
    return (float)NS_US_DEGSEC * pos;
  };
  float Get_HMSEC_Angle() {
    Get ();
    return (float)NS_US_HMSEC * pos;
  };
  float Get_RAD_Angle() {
    Get ();
    return (float)NS_US_RAD * pos;
  };
  float Get_HR_Angle() {
    Get ();
    return (float)NS_US_HMS * pos;
  };
  void Set(long p) { 
      pos = p;
      ip  = pos/STEPPER_POS_RANGE;
      mp  = (pos%STEPPER_POS_RANGE) - STEPPER_POS_RANGE0;
      m->setPos(mp);
      last_reading = mp;
  };
  int distance;
  int tip;
  int  ip;
  long pos;
  long mp;
  long goto_pos;
  long last_reading;
  powerSTEP *m;
};

Position axisH(&stepperH);
Position axisD(&stepperD);

class Settings {
public:
  Settings(float max=1000.0, float full=2000.0, float min=1.0, float ac=2000.0, float dc=2000.0, byte acck=80, byte deck=80, byte runk=80, byte holdk=12){
    max_speed = max;   // stepsPerSecond  *** The available range is from 15.25 to 15610 step/s with a resolution of 15.25 step/s.
    full_speed = full; // stepsPerSecond *** The FS_SPD threshold speed value over which the step mode is automatically switched to full-step two-phase on.
    min_speed = min;   // stepsPerSecond   *** The available range is from 0 to 976.3 step/s with a resolution of 0.238 step/s.
    /* The MIN_SPEED parameter contains the speed profile minimum speed. Its value is
       expressed in step/tick and to convert it in step/s the following formula can be used:

                    MIN_SPEED ⋅ 2^24
       [ step/s ] = ----------------
                             tick
       where MIN_SPEED is the integer number stored in the register and tick is the ramp 250 ns.
       The available range is from 0 to 976.3 step/s with a resolution of 0.238 step/s.
       When the LSPD_OPT bit is set high, low speed optimization feature is enabled (voltage
       mode driving only) and the MIN_SPEED value indicates the speed threshold below which
       the compensation works. In this case the minimum speed of the speed profile is set to zero.
       Any attempt to write the register when the motor is running causes the CMD_ERROR flag to
       rise.
       LSPD_OPT (bit 12) is set high = low speed opt. on -- NEED THIS for acurate tracking!!

       When the motor is driven at a very low speed using a small driving voltage, the resulting
       phase current can be distorted. As a consequence, the motor position is different ffrom the
       ideal one (see Figure 16).
       The device implements a low speed optimization in order to remove this effect.

       The optimization can be enabled setting high the LSPD_OPT bit in the MIN_SPEED register
       (Section 11.1.8 on page 55) and is active in a speed range from zero to MIN_SPEED. When
       low speed optimization is enabled, speed profile minimum speed is forced to zero.

       ==> LSPD_OPT configure at initial startup setup routine for H and D axis. and MIN SPEED is set to 10xSideral Tracking Speed. <==
    */
    acc = ac; // stepsPerSecond^2  *** The available range is from 14.55 to 59590 step/s 2 with a resolution of 14.55 step/s 2 .
    dec = dc; // stepsPerSecond^2  *** The available range is from 14.55to 59590 step/s2 with a resolution of 14.55 step/s2.
    acc_kval = acck; // power 0..255  *** The available range is from 0 to 0.996 x V_S x kval/256 with a resolution of 0.004 x V_S
    dec_kval =  deck; // power 0..255 ***
    run_kval =  runk; // power 0..255 ***
    hold_kval = holdk; // power 0..255 ***
  };

  float getfvalue(const char* cmd){
    int i;
    for (i = 0; cmd[i] != '#' && i < 50; ++i) tmp[i] = cmd[i];
    tmp[i] = 0;
    return atof(tmp);
  }
  
  long getlvalue(const char* cmd){
    int i;
    for (i = 0; cmd[i] != '#' && i < 50; ++i) tmp[i] = cmd[i];
    tmp[i] = 0;
    return atol(tmp);
  }
  
  int getivalue(const char* cmd){
    int i;
    for (i = 0; cmd[i] != '#' && i < 50; ++i) tmp[i] = cmd[i];
    tmp[i] = 0;
    return atoi(tmp);
  }

  void setup (const char* cmd){ // :SM[FST]..#
    //Serial.print("Setup: :SM{"); Serial.print(cmd); Serial.println("}#");
    switch (cmd[0]){
    case 'M': max_speed = getfvalue (&cmd[1]);  /* Serial.println(max_speed); */  break;  // :SM[FST]Mxxx#
    case 'F': full_speed = getfvalue (&cmd[1]); /* Serial.println(full_speed); */; break; // :SM[FST]Fxxx#
    case 'A': acc = getfvalue (&cmd[1]); /* Serial.print(acc); */ break; // :SM[FST]Axxx#
    case 'D': dec = getfvalue (&cmd[1]); /* Serial.print(dec); */ break; // :SM[FST]Dxxx#
    case 'K':
      switch (cmd[1]){
      case 'R': run_kval = getivalue (&cmd[2]); /* Serial.print(run_kval); */ break; // :SM[FST]KRxxx#
      case 'A': acc_kval = getivalue (&cmd[2]); /* Serial.print(acc_kval); */ break; // :SM[FST]KAxxx#
      case 'D': dec_kval = getivalue (&cmd[2]); /* Serial.print(dec_kval); */ break; // :SM[FST]KDxxx#
      case 'H': hold_kval = getivalue (&cmd[2]); /* Serial.print(hold_kval); */ break; // :SM[FST]KHxxx#
      default: break;
      }
    };
  };
  
  void query_setup (){
    Serial.print("Max  Speed="); Serial.println(max_speed);
    Serial.print("Full Speed="); Serial.println(full_speed);
    Serial.print("Acc       ="); Serial.println(acc);
    Serial.print("Dec       ="); Serial.println(dec);
    Serial.print("run_kval  ="); Serial.println(run_kval);
    Serial.print("acc_kval  ="); Serial.println(acc_kval);
    Serial.print("dec_kval  ="); Serial.println(dec_kval);
    Serial.print("hold_kval  ="); Serial.println(hold_kval);
  };

  void config_motor (powerSTEP &m, float stepsPerSec=-1.){
    // must stop before full reconfiguration!
    m.softStop();
    while (m.busyCheck());

    if (stepsPerSec > 0.)
      m.setMaxSpeed(stepsPerSec); // max speed in units of full steps/s = 1000
    else
      m.setMaxSpeed(max_speed); // max speed in units of full steps/s = 1000
    //m.setMinSpeed(max_speed); // min speed in units of full steps/s = 1000
    m.setFullSpeed(full_speed); // full steps/s threshold for disabling microstepping =2000
    m.setAcc(acc); // full steps/s^2 acceleration
    m.setDec(dec); // full steps/s^2 deceleration
    // setup power
    m.setRunKVAL(run_kval);
    m.setAccKVAL(acc_kval);
    m.setDecKVAL(dec_kval);
    m.setHoldKVAL(hold_kval);
  };

  void set_speed_motor (powerSTEP &m, float stepsPerSec=-1.){
    if (stepsPerSec > 0.)
      m.setMaxSpeed(stepsPerSec); // max speed in units of full steps/s = 1000
    else
      m.setMaxSpeed(max_speed); // max speed in units of full steps/s = 1000
  }
  
  float max_speed; // stepsPerSecond
  float full_speed; // stepsPerSecond
  float min_speed; // stepsPerSecond
  float acc; // stepsPerSecond^2
  float dec; // stepsPerSecond^2
  byte acc_kval; // power 0..255
  byte dec_kval; // power 0..255
  byte run_kval; // power 0..255
  byte hold_kval; // power 0..255
  /*
    void configStepMode(byte stepMode);
    void setMaxSpeed(float stepsPerSecond);
    void setMinSpeed(float stepsPerSecond);
    void setFullSpeed(float stepsPerSecond);
    void setAcc(float stepsPerSecondPerSecond);
    void setDec(float stepsPerSecondPerSecond);
    void setOCThreshold(byte threshold);
    void setPWMFreq(int divisor, int multiplier);
    void setSlewRate(int slewRate);
    void setOCShutdown(int OCShutdown);
    void setVoltageComp(int vsCompMode);
    void setSwitchMode(int switchMode);
    void setOscMode(int oscillatorMode);
    void setAccKVAL(byte kvalInput);
    void setDecKVAL(byte kvalInput);
    void setRunKVAL(byte kvalInput);
    void setHoldKVAL(byte kvalInput);
  */
};









void scan_i2c_bus(){
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  //delay(5000);           // wait 5 seconds for next scan
}


// Check Key Status of Spalte (Column x) via I2C MCP23S17 on 3x4 keypad
byte SpKey (byte x) {
  int i;
  byte ret;
  byte data[2] = { MCP23S17_GPIOB, x | (key_pad_led_status<<3) };
  if (i2c_error)
    return 0;
    
  // write via I2C
  Wire.beginTransmission(MCP_I2C_SELECT);
  Wire.write(data, 2); // GPIOB, x
  //Wire.write(MCP23S17_GPIOB); // GPIOB
  //Wire.write(x | (key_pad_led_status<<3)); // port B -- column selector bit 0,1,2 and LED status on 3,4,5,6,7
  if (ret=Wire.endTransmission()){
        display.setCursor(0, 120);
        display.setTextColor(ORANGE);  
        display.println("I2C WR1 Error "+String(ret));
        i2c_error = ret;
        return 0;
  }
  delayMicroseconds(200);// stabilize signal
  // read via I2C
  Wire.beginTransmission(MCP_I2C_SELECT);
  Wire.write(MCP23S17_GPIOA); // set MCP23017 memory pointer to GPIOA address
  if (ret=Wire.endTransmission()){
        display.setCursor(0, 120);
        display.setTextColor(ORANGE);  
        display.println("I2C WR2 Error "+String(ret));
        i2c_error = ret;
        return 0;
  }
  Wire.requestFrom(MCP_I2C_SELECT, 1); // 0x20 request one byte of data from MCP20317
  for(i=10; i-- && !Wire.available();)
    delayMicroseconds(5);// stabilize signal
  if (!Wire.available()){
        display.setCursor(0, 120);
        display.setTextColor(ORANGE);  
        display.println("I2C RD Timeout");
        i2c_error = -1;
        return 0;
  }
  return Wire.read(); // return the incoming byte
}

void Clear_I2C_Error(){
  if (i2c_error){
    i2c_error = 0;
    display.fillRect(0, 120, 128, 10, BLACK);
  }
}

#if 0
// Direct IO mode -- needs review
int SpKey (int x) {
  int i, val;
  for (i = 0; i < 3; ++i)
    digitalWrite(KX_PIN[i], x == i ? HIGH : LOW);
  delayMicroseconds(100);// stabilize signal
  for (val = i = 0; i < 4; ++i)
    val += digitalRead (KY_PIN[i]) ? 1 : 0 << i;
  return ( val );
}
#endif


//#define SP_KEYPAD_DEBUG

// Tastenstatusaenderung feststellen
int KeyChg(int dk[4][4]) {
  static byte Col[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}}; // [3] = ST4
  static byte PYval[8]  = {KY1, KY2, KY3, KY4, ST4_HP, ST4_HM, ST4_DP, ST4_DM };
  static byte st4 = 0;
  static int q = 0;
  static int sum_prev = 0;
  byte   x, y;
  int   sum = 0;
  Col[0][q] = SpKey (KX1) & (KY1|KY2|KY3|KY4);
  Col[1][q] = SpKey (KX2) & (KY1|KY2|KY3|KY4);
  Col[2][q] = (st4=SpKey (KX3)) & (KY1|KY2|KY3|KY4);
  Col[3][q] = st4 & (ST4_HP|ST4_HM|ST4_DP|ST4_DM);

  for (x = 0; x < 3; x++)
    for (y = 0; y < 4; y++)
      sum += dk[x][y] = (Col[x][q] & PYval[y]) - (Col[x][q ^ 1] & PYval[y]);

  for (y = 0; y < 4; y++)
      sum += dk[3][y] = (Col[3][q] & PYval[4+y]) - (Col[3][q ^ 1] & PYval[4+y]);

#ifdef SP_KEYPAD_DEBUG
  if (sum != sum_prev && sum){
    Serial.print("DKey: ");
    Serial.print(Col[0][q]);
    Serial.print(", ");
    Serial.print(Col[1][q]);
    Serial.print(", ");
    Serial.print(Col[2][q]);
    Serial.print(" => ");
    Serial.println(sum);
    sum_prev = sum;
  }
#endif

  if (Col[3][q]){
    display.fillRect(0, 120, 64, 8, BLACK);
    display.setCursor(0, 120);
    display.setTextColor(ORANGE);  
    display.println("ST4="+String(Col[3][q]));
  }
#if 0
  if (sum != sum_prev && sum){
    display.fillRect(60, 120, 8, 8, BLACK);
    for (x = 0; x < 3; x++)
      for (y = 0; y < 4; y++)
        display.drawPixel(60+x, 120+y, dk[x][y] ? GREEN : BLACK);
  }
#endif

  q ^= 1;
  return sum;
}

#ifdef SP_KEYPAD_DEBUG
#define KEY_PAD_PRESSED(X) Serial.println(F(X " pressed"))
#define KEY_PAD_RELEASED(X) Serial.println(F(X " released"))
#else
#define KEY_PAD_PRESSED(X)
#define KEY_PAD_RELEASED(X)
#endif


void KeyPadComand(int DKey[4][4]) {
  static int focus_mode = 0;
  static int slew_i = 0;
  static int tdh = 1, tdd = 1;
  static int st4_delay = 0;
  int resume_tracking=0;
#ifdef HD_RUN_SIMULTANEOUS_ENABLE
  switch (DKey[0][0]) {   // H-, D+
    case KY1: KEY_PAD_PRESSED ("00 H-D+"); // Key Pressed
      stepperH.setMaxSpeed(slew_rate[MOTOR_H]);
      stepperH.run(H_REV, slew_rate[MOTOR_H]);
      stepperD.setMaxSpeed(slew_rate[MOTOR_D]);
      stepperD.run(D_FWD, slew_rate[MOTOR_D]);
      break;
    case -KY1: KEY_PAD_RELEASED ("00"); // Key Relesed
      resume_tracking=3;
      break;
  }
  switch (DKey[2][0]) {   // H+, D+
    case  KY1: KEY_PAD_PRESSED ("20 H+D+");   // Key Pressed
      stepperH.setMaxSpeed(slew_rate[MOTOR_H]);
      stepperH.run(H_FWD, slew_rate[MOTOR_H]);
      stepperD.setMaxSpeed(slew_rate[MOTOR_D]);
      stepperD.run(D_FWD, slew_rate[MOTOR_D]);
      break;
    case -KY1: KEY_PAD_RELEASED ("20");  // Key Relesed
      resume_tracking=3;
      break;
  }
  switch (DKey[0][2]) {   // H-, D-
    case  KY3: KEY_PAD_PRESSED ("02 H-D-"); // Key Pressed
      stepperH.setMaxSpeed(slew_rate[MOTOR_H]);
      stepperH.run(H_REV, slew_rate[MOTOR_H]);
      stepperD.setMaxSpeed(slew_rate[MOTOR_D]);
      stepperD.run(D_REV, slew_rate[MOTOR_D]);
      break;
    case -KY3: KEY_PAD_RELEASED ("02");   // Key Relesed
      resume_tracking=3;
      break;
  }
  switch (DKey[2][2]) {   // H+, D-
    case  KY3: KEY_PAD_PRESSED ("22 H+D-"); // Key Pressed
      stepperH.setMaxSpeed(slew_rate[MOTOR_H]);
      stepperH.run(H_FWD, slew_rate[MOTOR_H]);
      stepperD.setMaxSpeed(slew_rate[MOTOR_D]);
      stepperD.run(D_REV, slew_rate[MOTOR_D]);
      break;
    case -KY3: KEY_PAD_RELEASED ("22"); // Key Relesed
      resume_tracking=3;
      break;
  }
#endif
  switch (DKey[1][0]) {   // D+
    case  KY1: KEY_PAD_PRESSED ("10 D+");   // Key Pressed
      if (focus_mode){
        stepperF.setMaxSpeed(focus_rate);
        stepperF.run(D_FWD, focus_rate);
      }
      else {
        stepperD.setMaxSpeed(slew_rate[MOTOR_D]);
        stepperD.run(D_FWD, slew_rate[MOTOR_D]);
      }
      break;
    case -KY1: KEY_PAD_RELEASED ("10");// Key Relesed
      resume_tracking=2;
      break;
  }
  switch (DKey[1][2]) {   // D-
    case  KY3: KEY_PAD_PRESSED ("12 D-");  // Key Down
      if (focus_mode){
        stepperF.setMaxSpeed(focus_rate);
        stepperF.run(D_REV, focus_rate);
      }
      else {
        stepperD.setMaxSpeed(slew_rate[MOTOR_D]);
        stepperD.run(D_REV, slew_rate[MOTOR_D]);
      }
      break;
    case -KY3: KEY_PAD_RELEASED ("12");   // Key Relesed
      resume_tracking=2;
      break;
  }
  switch (DKey[0][1]) {   // H-
    case  KY2: KEY_PAD_PRESSED ("01 H-"); // Key Down
      stepperH.setMaxSpeed(slew_rate[MOTOR_H]);
      stepperH.run(H_REV, slew_rate[MOTOR_H]);
      break;
    case -KY2: KEY_PAD_RELEASED ("01"); // Key Relesed
      resume_tracking=1;
      break;
  }
  switch (DKey[2][1]) {   // H+
    case  KY2: KEY_PAD_PRESSED ("21 H+");   // Key Down
      stepperH.setMaxSpeed(slew_rate[MOTOR_H]);
      stepperH.run(H_FWD, slew_rate[MOTOR_H]);
      break;
    case -KY2: KEY_PAD_RELEASED ("21"); // Key Relesed
      resume_tracking=1;
      break;
  }

  // ST4 commands

  switch (DKey[3][2]) {   // D+
    case  ST4_DP: KEY_PAD_PRESSED ("ST4 D+");   // Key Pressed
      st4_delay++;
      stepperD.setMaxSpeed(ST4_CORRECT_SPEED);
      stepperD.run(D_FWD, ST4_CORRECT_SPEED);
      display.fillRect(64, 120, 64, 8, BLACK);
      display.setCursor(65, 120);
      display.setTextColor(GREEN);  
      display.println("ST-4 D+");
      break;
    case -ST4_DP: KEY_PAD_RELEASED ("ST4 D+ clear");// Key Relesed
      st4_delay=0;
      display.fillRect(64, 120, 64, 8, BLACK);
      resume_tracking=2;
      break;
  }
  switch (DKey[3][3]) {   // D-
    case  ST4_DM: KEY_PAD_PRESSED ("ST4 D-");  // Key Down
      st4_delay++;
      stepperD.setMaxSpeed(ST4_CORRECT_SPEED);
      stepperD.run(D_REV, ST4_CORRECT_SPEED);
      display.fillRect(64, 120, 64, 8, BLACK);
      display.setCursor(65, 120);
      display.setTextColor(GREEN);  
      display.println("ST-4 D-");
      break;
    case -ST4_DM: KEY_PAD_RELEASED ("ST4 D- clear");   // Key Relesed
      st4_delay=0;
      display.fillRect(64, 120, 64, 8, BLACK);
      resume_tracking=2;
      break;
  }
  switch (DKey[3][1]) {   // H-
    case  ST4_HM: KEY_PAD_PRESSED ("ST4 H-"); // Key Down
      if (st4_delay==0 || st4_delay>100){
        stepperH.setMaxSpeed(ST4_CORRECT_SPEED);
        stepperH.run(H_REV, ST4_CORRECT_SPEED);
        display.setTextColor(GREEN);  
      } else {
        display.setTextColor(RED);  
      }
      display.fillRect(64, 120, 64, 8, BLACK);
      display.setCursor(65, 120);
      display.println("ST-4 H-");
      break;
    case -ST4_HM: KEY_PAD_RELEASED ("ST4 H- clear"); // Key Relesed
      display.fillRect(64, 120, 64, 8, BLACK);
      resume_tracking=1;
      break;
  }
  switch (DKey[3][0]) {   // H+
    case  ST4_HP: KEY_PAD_PRESSED ("ST4 H+");   // Key Down
      if (st4_delay==0 || st4_delay>100){
        stepperH.setMaxSpeed(ST4_CORRECT_SPEED);
        stepperH.run(H_FWD, ST4_CORRECT_SPEED);
        display.setTextColor(GREEN);  
      } else {
        display.setTextColor(RED);  
      }
      display.fillRect(64, 120, 64, 8, BLACK);
      display.setCursor(65, 120);
      display.setTextColor(GREEN);  
      display.println("ST-4 H+");
      break;
    case -ST4_HP: KEY_PAD_RELEASED ("ST4 H+ clear"); // Key Relesed
      display.fillRect(64, 120, 64, 8, BLACK);
      resume_tracking=1;
      break;
  }

// ===============  
  
  if (resume_tracking){
      if (resume_tracking & 1){
          stepperH.softStop();
          if (tdh){
              //stepperH.setMaxSpeed(tracking_rate[MOTOR_H]);
              stepperH.run(tdh>0? H_FWD:H_REV, tracking_rate[MOTOR_H]);
              bitSet   (key_pad_led_status, tdh>0? 0:1);
              bitClear (key_pad_led_status, tdh>0? 1:0);
          } else {
              bitClear (key_pad_led_status, 0);
              bitClear (key_pad_led_status, 1);
          }
      }
      if (resume_tracking & 2)
          stepperH.softStop();
          if (tdh){
              //stepperH.setMaxSpeed(tracking_rate[MOTOR_H]);
              stepperH.run(tdh>0? H_FWD:H_REV, tracking_rate[MOTOR_H]);
              bitSet   (key_pad_led_status, tdh>0? 0:1);
              bitClear (key_pad_led_status, tdh>0? 1:0);
          } else {
              bitClear (key_pad_led_status, 0);
              bitClear (key_pad_led_status, 1);
          }
          if (focus_mode)
            stepperF.softStop();
          else
            stepperD.softStop();
  }

  // skip though slew rates 
  if (DKey[1][1] == KY2) {
      stepperH.softStop();
      stepperD.softStop();
      KEY_PAD_PRESSED ("11 SLR++%");   
      slew_i = ++slew_i % NUM_SLEW_RATES;
      slew_rate[0] = slew_rate[1] = slew_rate_list [slew_i];
      if(slew_i > 2)
          bitSet (key_pad_led_status, 3);
      display.fillRect(0, 120, 64, 8, BLACK);
      display.setCursor(0, 120);
      display.setTextColor(ORANGE);  
      display.println("Slew: " + String(slew_i));
  }


  
  // Nachfuehrmodus toggeln
  if (DKey[0][3] == KY4) {
      stepperH.softStop();
      stepperD.softStop();
      KEY_PAD_PRESSED ("03 TSideral");   
      tdh++; if (tdh>1) tdh = -1; // [-1, 0, 1]
      if (tdh){
          slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = SLEW_FINE_HZ];
          tracking_rate[MOTOR_H] = SIDERAL_H_HZ;
          //stepperH.setMaxSpeed(tracking_rate[MOTOR_H]);
          stepperH.run(tdh>0? H_FWD: H_REV, tracking_rate[MOTOR_H]);
      }
      bitClear (key_pad_led_status, 2);
      display.fillRect(64, 120, 64, 8, BLACK);
      display.setCursor(65, 120);
      display.setTextColor(tdh > 0 ? GREEN : ORANGE);  
      display.println("THD: " + String(tdh) + " S");
  }
  if (DKey[2][3] == KY4) {
      stepperH.softStop();
      stepperD.softStop();
      KEY_PAD_PRESSED ("23 TLunar");   
      tdh++; if (tdh>1) tdh = -1; // [-1, 0, 1]
      if (tdh){
          slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = SLEW_FINE_HZ];
          tracking_rate[MOTOR_H] = LUNAR_H_HZ;
          //stepperH.setMaxSpeed(tracking_rate[MOTOR_H]);
          stepperH.run(tdh>0? H_FWD:H_REV, tracking_rate[MOTOR_H]);
      }
      bitSet (key_pad_led_status, 2);
      display.fillRect(64, 120, 64, 8, BLACK);
      display.setCursor(65, 120);
      display.setTextColor(tdh > 0 ? GREEN : ORANGE);  
      display.println("THD: " + String(tdh) + " L");
  }
  if (DKey[1][3] == KY4) {
      KEY_PAD_PRESSED ("13 slew fine/focus");   
      if (focus_mode)
        focus_mode = 0;
      else
        if (slew_i == 0)
          focus_mode = 1;
      slew_i = 0;
      slew_rate[0] = slew_rate[1] = slew_rate_list [slew_i];
      bitClear (key_pad_led_status, 3);
      display.fillRect(0, 120, 64, 8, BLACK);
      display.setCursor(0, 120);
      display.setTextColor(ORANGE);  
      if (focus_mode)
        display.println("Focus Mode");
      else  
        display.println("Slew: " + String(slew_i));
  }
}





#define MAX_CMD_LEN 256

//Print* printer = &Serial;

//Settings(float max=1000.0, float full=2000.0, float min=1.0, float ac=2000.0, float dc=2000.0, byte acck=80, byte deck=80, byte runk=80, byte holdk=12){
 
Settings set_slew_fast(2000.0, 6000.0, 1.0, 330.0, 800.0, 35, 35, 35, 12);
Settings set_slew_slow(1000.0, 6000.0, 1.0, 100.0, 800.0, 35, 35, 35, 12);
Settings set_tracking (1000.0, 6000.0, 1.0, 100.0, 100.0, 35, 35, 45, 12); // needs power for precise 1/128 slow stepping! Motor running pretty warm...
Settings set_focus    (1000.0, 6000.0, 1.0, 100.0, 800.0, 35, 35, 35, 12);



#ifdef SERIAL_ECHO
# ifdef DISPLAY_ECHO
#  define SEND(X) { if (hw_comm) hw_comm->print(X); if (hw_echo) hw_echo->print(X); if (sw_comm) sw_comm->print(X); if (sw_echo) sw_echo->print(X); if (usb_comm) usb_comm->print(X); if (usb_echo) usb_echo->print(X); display.fillRect(0, 80, 128, 20, BLACK); display.setCursor(0, 80); display.setTextColor(YELLOW); display.print(String(X).substring(0,40)); }
# else
#  define SEND(X) { if (hw_comm) hw_comm->print(X); if (hw_echo) hw_echo->print(X); if (sw_comm) sw_comm->print(X); if (sw_echo) sw_echo->print(X); if (usb_comm) usb_comm->print(X); if (usb_echo) usb_echo->print(X); }
# endif
#else
# ifdef DISPLAY_ECHO
#  define SEND(X) { if (hw_comm) hw_comm->print(X); if (sw_comm) sw_comm->print(X);  if (usb_comm) usb_comm->print(X); display.fillRect(0, 80, 128, 20, BLACK); display.setCursor(0, 80); display.setTextColor(YELLOW); display.print(String(X).substring(0,40)); }
# else
#  define SEND(X) { if (hw_comm) hw_comm->print(X); if (sw_comm) sw_comm->print(X); if (usb_comm) usb_comm->print(X); }
# endif
#endif



class Communication {
  public:
  Communication(const char *_id, HardwareSerial *hw_port, HardwareSerial *hw_echo_port = NULL) {
      id = _id;
      cmdi = 0;
      hw_comm = hw_port;
      hw_echo = hw_echo_port;
      sw_comm = NULL;
      sw_echo = NULL;
      usb_comm = NULL;
      usb_echo = NULL;
   
#ifdef VERBOSE_STARTUP
      send(id);
      send("#\n");
#endif
  };
#ifdef USE_UNO
  Communication(const char *_id, SoftwareSerial *sw_port, SoftwareSerial *sw_echo_port = NULL) {
      id = _id;
      cmdi = 0;
      hw_comm = NULL;
      hw_echo = NULL;
      sw_comm = sw_port;
      sw_echo = sw_echo_port;
      usb_comm = NULL;
      usb_echo = NULL;
   
#ifdef VERBOSE_STARTUP
      send(id);
      send("#\n");
#endif
  };
#endif
#if 1
  Communication(const char *_id, Serial_ *usb_port, Serial_ *usb_echo_port = NULL) {
      id = _id;
      cmdi = 0;
      hw_comm = NULL;
      hw_echo = NULL;
      sw_comm = NULL;
      sw_echo = NULL;
      usb_comm = usb_port;
      usb_echo = usb_echo_port;
   
#ifdef VERBOSE_STARTUP
      send(id);
      send("#\n");
#endif
    };
#endif
    void send(const __FlashStringHelper *msg) {
      SEND(msg);
    };

    void send(const char *msg) {
      SEND(msg);
    };

    void send(char *msg) {
      SEND(msg);
    };

    void send(String &msg) {
      SEND(msg);
    };

    void send(int i) {
      SEND(i);
    };

    void sendi02(int i) {
      if (i<10 && i>0)
        SEND("0");
      SEND(i);
    };

    void sendtrue(){ SEND("1#\n"); };
    void sendfalse(){ SEND("0#\n"); };
    
    void send(long l) {
      SEND(l);
    };

    void send(float f) {
      SEND(f);
    };

    void send(double df) {
      SEND(df);
    };

    void sendNewLine() {
      SEND("\n");
    };

    void get_sexi(char *str, int sexi[3]){
      int i;
      sexi[2] = atoi(&str[0]);
      for (i=0; i<40; ++i)
        if (str[i] == ':' || str[i] == '/' || str[i] == '-')
          break;
      ++i;
      sexi[1] = atoi(&str[i]);
    
      for (; i<40; ++i){
        if (str[i] == ':' || str[i] == '/' || str[i] == '-')
          break;
        if (str[i] == '#'){
          sexi[0]=0;
          return;
        }
      }
      ++i;
      sexi[0] = atoi(&str[i]);
    };

    long convert_HMSsec_to_position(long ss){
      return ss << 8; // convert to NS_U: 288*3*200 * 128 / HMS: 24/60/60 = 2 * 128 --> ss * 256
    };

    long convert_position_to_HMSsec(long pos){
      while (pos < 0) pos += NS_US;
      while (pos > NS_US) pos -= NS_US;
      return pos >> 8; // convert NS_U to sec: inv( 288*3*200 * 128 / HMS: 24/60/60 = 2 * 128 ) -> pos/256
    };

    long convert_DMSsec_to_position(long ss){
      return ss*17 + ss/15; // NS_U: 288*3*200 *128 / DMS: 360/60/60 = 17.06666 = 17 + 1/15 = ss*17 + ss/15
    };

    long convert_position_to_DMSsec(long pos){
      while (pos < 0) pos += NS_US;
      while (pos > NS_US) pos -= NS_US;
      return (pos*3 + pos*3/4) >> 6; // NS_U: 288*3*200 *128 / HMS: 360/60/60 = 17.06666 = 17 + 1/15 => pos/(17+1/15) = pos*(3 + 3/4) / 64
    };

    void get_pos_HHMMSS(char *str){
      int hms[3];
      long ss;
      get_sexi(str, hms);
      ss = hms[0] + hms[1]*60L + hms[2]*3600L;
      Astro.Set_memRA ((float)ss*HSEC_TO_RAD);
      //SetPos[0] = convert_HMSsec_to_position (ss);
    };

    void get_pos_DDMMSS(char *str){
      int dms[3];
      long ss;
      get_sexi(str, dms);
      if (dms[2] > 0)
        ss = dms[0] + dms[1]*60L + dms[2]*3600L;
      else
        ss = 1296000L - dms[0] - dms[1]*60L + dms[2]*3600L;
      Astro.Set_memDecl ((float)ss*SEC_TO_RAD);
      //SetPos[1] = convert_DMSsec_to_position(ss);
    };

  
    //d = integer(dec)
    //m = integer((dec - d) × 60)
    //s = (dec - d - m/60) × 3600
    void HHMMSS_String_from_sec(String &hms, long sec) {
      int hh, mm, ss;
      hh = numberOfHours (sec);
      mm = numberOfMinutes (sec);
      ss = numberOfSeconds (sec);
      if (hh < 10){
        hms += "0";
      }
      hms += String(hh)+":";
      if (mm < 10){
        hms += "0";
      }
      hms += String(mm)+":";
      if (ss < 10){
        hms += "0";
      }
      hms += String(ss);
    };
    void send_HHMMSS_from_sec(long sec) {
      String hms;
      HHMMSS_String_from_sec(hms, sec);
      send(hms+"#\n");
    };
    
    void send_HHMMSS(long pos) {
      // SERIAL_ECHO_PORT.print(pos); SERIAL_ECHO_PORT.print("\n");
      send_HHMMSS_from_sec (convert_position_to_HMSsec (pos));
    };

    void DDMMSS_String_from_sec(String &dms, long sec, String sep="*") {
      int dd, mm, ss;
      dd = numberOfDegrees (sec);
      mm = numberOfMinutes (sec);
      ss = numberOfSeconds (sec);
      if (dd < 10){
        dms += "00";
      } else if (dd < 100){
        dms += "0";
      }
      dms += String(dd)+sep;
      if (mm < 10){
        dms += "0";
      }
      dms += String(mm)+":";
      if (ss < 10){
        dms += "0";
      }
      dms += String(ss);
    };
    void send_DDMMSS_from_sec(long sec) {
      String dms;
      DDMMSS_String_from_sec(dms, sec);
      send(dms+"#\n");
    };
    void send_DDMMSS(long pos) {
      //  SERIAL_ECHO_PORT.print(pos); SERIAL_ECHO_PORT.print("\n");
      send_DDMMSS_from_sec (convert_position_to_DMSsec (pos));
    };

    void send_Ginfo (){
         // :Ginfo# 10micron -- This format may consist of more parts some day
         //     nbytes_read = sscanf(data, "%g,%g,%c,%g,%g,%g,%d,%d#", &Ginfo.RA_JNOW, &Ginfo.DEC_JNOW, &Ginfo.SideOfPier,
         //     &Ginfo.AZ, &Ginfo.ALT, &Ginfo.Jdate, &Ginfo.Gstat, &Ginfo.SlewStatus);
        int gs = GSTAT_STOPPED;
        if (stepperH.busyCheck() || stepperD.busyCheck())
          gs = GSTAT_TRACKING;
        if (axisH.isSlewing() || axisD.isSlewing())
          gs = GSTAT_SLEWING_OR_STOPPING;
        Astro.calc_JD_Time_GrHrAngle (millis());
        Astro.Set_Decl (axisD.Get_RAD_Angle());
        send(String(Astro.calc_RA_from_hrangle (axisH.Get_RAD_Angle())*RAD_TO_HMS, 8)+","+String(axisD.Get_DEG_Angle(), 8)+","); 
        send("E"); send(","); 
        send(String(Astro.calc_Azimuth ()*RAD_TO_DEG, 8)+","+String(Astro.calc_Elevation ()*RAD_TO_DEG, 8)+","); 
        send(String(Astro.JDWhole ())+String(Astro.JDFrac (), 8).substring(1)+","+String(gs)+","+String(axisH.Distance()+axisD.Distance())+"#\n"); // GrHA="+String(Astro.GrHA (), 8)+" HA="+String(Astro.HA (), 8)+" M-HA="+String(axisH.Get_HR_Angle (), 8)+"#\n");
        //send("0,0#\n");
        //send("0,0,"+String(Astro.GrHA (), 8)+";"+String(axisH.Get_RAD_Angle(), 8)+"#\n");
    };
    
    void update_coords_display (int restart=0){
        static byte start=1;
        static String s[10];
        const int c1=24;
        float hrad = axisH.Get_RAD_Angle();
        float drad = axisD.Get_RAD_Angle();
        long  hpos = axisH.Get();
        long  dpos = axisD.Get();
        int hbusy=stepperH.busyCheck();
        int dbusy=stepperD.busyCheck();
        
        Astro.calc_JD_Time_GrHrAngle (millis());
        Astro.Set_Decl (drad);

        if (start || restart){
          start=0;
          display.fillScreen(BLACK);
          display.setTextColor(RED);  
          display.setCursor(0,  0); display.println("RA:");
          display.setCursor(0, 10); display.println("DE:");
          display.setCursor(0, 25); display.println("Alt:");
          display.setCursor(0, 35); display.println("Az:");
          display.setCursor(0, 50); display.println("JD:");
        }

        // RA HA
        display.setTextSize(1);
        if (!start){
          display.setTextColor(BLACK);  
          display.setCursor(c1, 0); display.println(s[0]); s[0]="";
        }
        
        HHMMSS_String_from_sec(s[0], (long)(Astro.calc_RA_from_hrangle (hrad)*RAD_TO_HSEC));
        s[0]+=" ";
//      HHMMSS_String_from_sec(s[0], (long)(Astro.JDFrac()*24.*3600.+0.5));
        HHMMSS_String_from_sec(s[0], (long)(Astro.calc_HrAngle ()*RAD_TO_HSEC));
        display.setTextColor(RED);  
        display.setCursor(c1, 0); display.println(s[0]);

        // DE
        if (!start){
          display.setTextColor(BLACK); 
          display.setCursor(c1, 10); display.println(s[1]); s[1]="";
        }
        
        DDMMSS_String_from_sec(s[1], (long)(drad*RAD_TO_SEC),"\xf8");
        display.setTextColor(RED);  
        display.setCursor(c1, 10); display.println(s[1]);

        // Alt, Az
        if (!start){
          display.setTextColor(BLACK);
          display.setCursor(c1, 25); display.println(s[3]);
          display.setCursor(c1, 35); display.println(s[4]);
          s[3]=""; s[4]="";
        }
        DDMMSS_String_from_sec(s[3], (long)(Astro.calc_Azimuth ()*RAD_TO_SEC),"\xf8");
        DDMMSS_String_from_sec(s[4], (long)(Astro.calc_Elevation ()*RAD_TO_SEC),"\xf8");
        display.setTextColor(RED);  
        display.setCursor(c1, 25); display.println(s[3]);
        display.setCursor(c1, 35); display.println(s[4]);

        // JD
        if (!start){
          display.setTextColor(BLACK);
          display.setCursor(c1, 50); display.println(s[6]);
        }
        s[6] = String(Astro.JDWhole ())+String(Astro.JDFrac (), 8).substring(1);
        display.setTextColor(RED);  
        display.setCursor(c1, 50); display.println(s[6]);

        display.setCursor(0, 65);
        // Axis Motor Status
        if (axisH.isSlewing() || axisD.isSlewing())
          display.setTextColor(ORANGE);  
        else
          display.setTextColor(BLACK);  
        display.println("Slewing: "+String(axisH.Distance()+axisD.Distance()));
  
        display.setCursor(80, 65);
        if (hbusy)
          display.setTextColor(ORANGE); 
        else
          display.setTextColor(BLACK);  
        display.println("H!");

        display.setCursor(100, 65);
        if (dbusy)
          display.setTextColor(ORANGE);  
        else
          display.setTextColor(BLACK);  
        display.println("D!");
#if 1    
        // Pos
        if (!start){
          display.setTextColor(BLACK);
          display.setCursor(0, 100); display.println(s[8]);
        }
        s[8] = String(hpos) + "  " + String(dpos);
        display.setTextColor(GREEN);  
        display.setCursor(0, 100); display.println(s[8]);
#endif
    
    };

    void CheckAndProcess();
    void Process(char controlbyte);

  private:
    const char *id;
    HardwareSerial *hw_comm;
    HardwareSerial *hw_echo;
    Serial_ *usb_comm;
    Serial_ *usb_echo;
#ifdef USE_UNO
    SoftwareSerial *sw_comm;
    SoftwareSerial *sw_echo;
#else
    HardwareSerial *sw_comm;
    HardwareSerial *sw_echo;
#endif
    int  cmdi;
    char cmdc[MAX_CMD_LEN];
};





Communication *comm_master = NULL;
Communication *comm_blue = NULL;
Communication *comm_usb = NULL;
Communication *comm_0 = NULL;
Communication *comm_1 = NULL;


void Communication::CheckAndProcess() {
  char controlbyte;
  // send data only when you receive data:

  if (hw_comm)
    if (hw_comm->available() > 0) {
      // read the incoming byte:
      controlbyte = hw_comm->read();
      Process (controlbyte);
    }

  if (sw_comm)
    if (sw_comm->available() > 0) {
      // read the incoming byte:
      controlbyte = sw_comm->read();
      Process (controlbyte);
    }

  if (usb_comm)
    if (usb_comm->available() > 0) {
      // read the incoming byte:
      controlbyte = usb_comm->read();
      Process (controlbyte);
    }
}

void Communication::Process(char controlbyte) {
  int i, j, k;
  long num;
  static byte hdir=0, ddir=0;
  static float hspeed=1.0, dspeed=1.0;
  
#ifdef SERIAL_ECHO
    // Echo ----  :GR##:GR#...
    if (hw_echo)
      hw_echo->print (controlbyte);
    if (sw_echo)
      sw_echo->print (controlbyte);
#endif

    if (cmdi == 0) {

      switch (controlbyte) {
#ifdef  QUICK_CMDS
        case '!': send ("H-Hz="); send (hspeed); send (",\t D-Hz="); send (dspeed); send (",\t Pos: "); send_Ginfo ();  
                  // send (",\t PosMem: "); send (Astro.Get_memRA()); send(", ); send (Astro.Get_meDeclA());
                  //send (",\t PosMem: "); send (SetPos[0]); send(", "); send (SetPos[1]); 
                  //send(" RA"); send_HHMMSS (SetPos[0]); send(" DE"); send_DDMMSS(SetPos[1]);
                  send ("\n");                
                  cmdi=0; break;
        case 's': set_slew_slow.set_speed_motor (stepperH, hspeed); set_slew_slow.set_speed_motor (stepperD, dspeed); cmdi=0; break;
        case 'S': set_slew_fast.set_speed_motor (stepperH, hspeed); set_slew_fast.set_speed_motor (stepperD, dspeed); cmdi=0; break;
        case 'T': set_tracking.set_speed_motor (stepperH, hspeed); set_tracking.set_speed_motor (stepperD, dspeed); cmdi=0; break;
        case 'F': set_focus.set_speed_motor (stepperF, dspeed); cmdi=0; break;
        case 'h': stepperH.softStop(); stepperH.run(hdir, hspeed); cmdi=0; break;
        case 'd': stepperD.softStop(); stepperD.run(ddir, dspeed); cmdi=0; break;
        case 'H': hdir ^= 1; stepperH.softStop(); stepperH.run(hdir, hspeed); cmdi=0; break;
        case 'D': ddir ^= 1; stepperD.softStop(); stepperD.run(ddir, dspeed); cmdi=0; break;
        case '+': hspeed *= 2.0; send (hspeed); sendNewLine(); stepperH.run(hdir, hspeed); cmdi=0; break;
        case '-': hspeed /= 2.0; send (hspeed); sendNewLine(); stepperH.run(hdir, hspeed); cmdi=0; break;
        case '>': dspeed *= 2.0; send (dspeed); sendNewLine(); stepperD.run(ddir, dspeed); cmdi=0; break;
        case '<': dspeed /= 2.0; send (dspeed); sendNewLine(); stepperD.run(ddir, dspeed); cmdi=0; break;

        case 'P': if (trk_power < 100) trk_power += 4; stepperH.setRunKVAL(trk_power); stepperD.setRunKVAL(trk_power);  send ("Trk RunKVAL="); send (trk_power); sendNewLine(); cmdi=0; break;
        case 'p': if (trk_power > 10) trk_power -= 4; stepperH.setRunKVAL(trk_power); stepperD.setRunKVAL(trk_power);  send ("Trk RunKVAL="); send (trk_power); sendNewLine(); cmdi=0; break;

	  
        case '4': stepperH.softStop(); stepperH.run(H_REV, slew_rate[MOTOR_H]); cmdi=0; break; 
        case '6': stepperH.softStop(); stepperH.run(H_FWD, slew_rate[MOTOR_H]); cmdi=0; break; 
        case '2': stepperD.softStop(); stepperD.run(D_REV, slew_rate[MOTOR_D]); cmdi=0; break; 
        case '8': stepperD.softStop(); stepperD.run(D_FWD, slew_rate[MOTOR_D]); cmdi=0; break; 

        case '7': stepperH.softStop(); stepperH.run(H_REV, slew_rate[MOTOR_H]);
	          stepperD.softStop(); stepperD.run(D_FWD, slew_rate[MOTOR_D]); cmdi=0; break; 
        case '9': stepperH.softStop(); stepperH.run(H_FWD, slew_rate[MOTOR_H]);
	          stepperD.softStop(); stepperD.run(D_FWD, slew_rate[MOTOR_D]); cmdi=0; break; 
        case '1': stepperH.softStop(); stepperH.run(H_REV, slew_rate[MOTOR_H]);
	          stepperD.softStop(); stepperD.run(D_REV, slew_rate[MOTOR_D]); cmdi=0; break; 
        case '3': stepperH.softStop(); stepperH.run(H_FWD, slew_rate[MOTOR_H]);
	          stepperD.softStop(); stepperD.run(D_REV, slew_rate[MOTOR_D]); cmdi=0; break; 

        case '5': case '0': stepperH.softStop();  stepperD.softStop(); cmdi=0; break; 

        case '.': case 't':
                  set_tracking.set_speed_motor (stepperH, tracking_rate[MOTOR_H]);
                  //stepperH.setMaxSpeed(tracking_rate[MOTOR_H]);
                  stepperH.run(tracking_dir[MOTOR_H], tracking_rate[MOTOR_H]);
                  cmdi=0; break; 
                  
#endif
        case ':': cmdi = 1; break; // start command
        case '#': cmdi = 0; break; // end command
        default: break;
      }
    } else {
      switch (controlbyte) {
        case '#':
#ifdef DISPLAY_ECHO
          display.fillRect(0, 110, 128, 10, BLACK);
          display.setCursor(0, 110);
          display.setTextColor(BLUE);  
          display.print(&cmdc[1]);
#endif
          switch (cmdc[1]) {
            case 0x06: // <ACK> Query of alignemnt mode
              send("P#\n");
              break;
            case 0x04: // <EOT> Query Firmware Download Request
              send("no supported. ardu_tvat V00.01 PyZ#\n");
              break;
            case 'A': // Alignment Commands
              switch (cmdc[2]) {
                case 'a': // :Aa# auto alignment ret 1: complete, ret 0 fail or not AzEl
                  sendtrue();
                  break;
                case 'L': // :AL# set Land Alignment mode
                  break;
                case 'P': // :AP# set Polar alignemnt mode
                  break;
                case 'A': // :AA# set AltAz mode
                  break;
              }
              break;
            case 'B': // Active Backlash Compensation
              switch (cmdc[2]) {
                case 'a': // :BAdd# set alt/dec antibacklash
                  for (i = 0; cmdc[3 + i] != '#' && i < 64; ++i) tmp[i] = cmdc[3 + i];
                  tmp[i] = 0;
                  alt_dec_backlash = atol(tmp);
                  break;
                case 'Z': // :BZdd# set AZ/RA antibacklash
                  for (i = 0; cmdc[3 + i] != '#' && i < 64; ++i) tmp[i] = cmdc[3 + i];
                  tmp[i] = 0;
                  az_ra_backlash = atol(tmp);
                  break;
                  // ... accessory controls
              }
              break;
            case 'C': // Sync Control
              switch (cmdc[2]) {
                case 'M': // :CM#  Synchronizes the telescope's position with the currently selected database object's coordinates. Returns " Object#"
                  // stop any slewing, motions, sync to position last set
                  stepperH.softStop();
                  stepperD.softStop();
                  while(stepperH.busyCheck());
                  while(stepperD.busyCheck());

                  Astro.calc_JD_Time_GrHrAngle (millis());
                  Astro.Use_memRADecl ();
                  axisH.Set((long)(Astro.calc_HrAngle ()*RAD_NS_US));
                  axisD.Set((long)(Astro.Get_memDecl ()*RAD_NS_US)); // SetPos[1]);
                  
                  //send(String(Astro.GrHA (), 8)+","+String(Astro.calc_HrAngle (), 8)+","+String(axisH.Get_RAD_Angle(), 8)+" SYNC W OBJ#\n");
                  //send(" *HA: "+String(Astro.HA()*RAD_TO_DEG, 8)+", Lon:"+String(Astro.Longitude()*RAD_TO_DEG,8)+"#\n"); 
                  //send(" *AXIS: "+String(axisH.Get_DEG_Angle(), 8)+" memRA: "+String(Astro.Get_memRA()*RAD_TO_DEG,8)+"#\n"); 
                  //send(" *RA: "+String(Astro.calc_RA_from_hrangle (axisH.Get_RAD_Angle())*RAD_TO_DEG, 8)+"#\n"); 
                  send(" *SYNC W OBJ and TRK#\n"); // OK.
                  break; // @@@@@
              }
              break;
              
            case 'f': // Fan/Heater Commands
              send("NA#\n");
              break;

            case 'F': // Focuser Commands :F+#:, F-#, FB#
              switch (cmdc[2]) {
                case '+': // :F+# Start Focuser moving inward (toward objective)
                  stepperF.softStop();
                  while(stepperF.busyCheck());
                  stepperF.setMaxSpeed(focus_rate);
                  stepperF.run(FWD, focus_rate);
                  send(" *F+#\n");
                  break;
                case '-': // :F-# Start Focuser moving outward (away from objective)
                  stepperF.softStop();
                  while(stepperF.busyCheck());
                  stepperF.setMaxSpeed(focus_rate);
                  stepperF.run(REV, focus_rate);
                  send(" *F-#\n");
                  break;
                case 'N': // :FN Snnn# Start Focuser moving nnn steps
                  num = atoi(&cmdc[3]);
                  stepperF.softStop();
                  while(stepperF.busyCheck());
                  stepperF.setMaxSpeed(focus_rate);
                  stepperF.move(num>0?FWD:REV, abs(num));
                  send(" *FN "+String(num)+"#\n");
                  break;
                case 'R': // :FR fff# Set Focuser moving speed steps/sec
                  num = atoi(&cmdc[3]);
                  focus_rate = (float)num;
                  set_focus.set_speed_motor (stepperF, focus_rate);
                  send(" *FR "+String(num)+"/s#\n");
                  break;
                case 'C': // :FC ...# configure Focus Motor
                  //set_focus.set_speed_motor (stepperF, focus_rate);
                  break;
                case 'S': // :FS# Stop Focuser movments
                  stepperF.softStop();
                  send(" *FStop#\n");
                  break;
                case 'P': // :FP# Get Focus Position index
                  send(String(stepperF.getPos())+" * Focus Index#\n");
                  break;
                case '0': // :F0# Reset Focus Position index to 0
                  stepperF.setPos(0);
                  send(" *F0#\n");
                  break;
              }
              break;

            case 'G': // G – Get Telescope Information
              switch (cmdc[2]) {
                case 'N': // :GN# Get Telescope RA, Returns position integer counts [RA,DE]: [0xHHHHHHHH,0xHHHHHHHH]#1
                  // NOT SITE NAME HERE
                  send(String(axisH.Get(), HEX)+","+String(axisD.Get(),HEX)+"#\n");
                  break;

                case 'i': // :Ginfo# 10micron -- This format may consist of more parts some day
                  send_Ginfo ();
                  break; // OK  @@@@
      

                case 'R': // :GR...# Get ...
                  switch (cmdc[3]) {
                    case 'T': // :GRTMP# RefractionModelTemperature is %0+6.1f degrees
                      send ("5.0#\n");
                      break;
                    case 'P': // :GRPS# RefractionModelPressure is %06.1f hPa
                      send ("1011.0#\n");
                      break;
                    default:
                      Astro.calc_JD_Time_GrHrAngle (millis());
                      send_HHMMSS_from_sec ((long)(Astro.calc_RA_from_hrangle (axisH.Get_RAD_Angle())*RAD_TO_HSEC));
                      break;
                  }
                  break;
                  
                case 'D': // :GD# Get Telescope Declination. Returns: sDD*MM# or sDD*MM’SS#:GZ#
                  send_DDMMSS_from_sec ((long)axisD.Get_DEGSEC_Angle());
                  break;
                  
                case 'r': // :Gr# Get current/target object RA
                  send_HHMMSS_from_sec ((long)(Astro.Get_memRA ()*RAD_HSEC)); 
                  break;
                case 'd': // :Gd# Get current/target object DE
                  send_DDMMSS_from_sec ((long)(Astro.Get_memDecl ()*RAD_SEC));
                  break;
#ifdef DEBUG_EXTRA
                case '*': // :G*#
                  send(" *MemCoord: "+String(Astro.Get_memRA (), 8)+", "+String(Astro.Get_memDecl (), 8)+" rad\n"); // @@@@
                  send(" *AxisH: "+String(axisH.Get ())+", AxisD: "+String(axisD.Get ())+" pos\n"); // @@@@
                  send(" *HA: "+String(axisH.Get_RAD_Angle(), 8)+" rad, Decl: "+String(axisD.Get_DEG_Angle(), 8)+" deg\n"); 
                  break;
#endif
                case 'Z': // :GZ# Get telescope azimuth, Returns: DDD*MM#T or DDD*MM’SS#
                  Astro.calc_JD_Time_GrHrAngle (millis());
                  Astro.Set_Decl (axisD.Get_RAD_Angle());
                  Astro.calc_RA_from_hrangle (axisH.Get_RAD_Angle());
                  send_DDMMSS_from_sec ((long)(Astro.calc_Azimuth()*RAD_TO_SEC));
                  break;
                  
                case 'G': // :GG# Get UTC offset time
                  send(Astro.TZOff()); send("#\n");
                  break;
                case 'H': // :GH# DST
                  send("0#\n");
                  break;
                case 'L': // :GL# Local Time HH:MM:SS
                  Astro.calc_JD_Time_GrHrAngle (millis());
                  send_HHMMSS_from_sec((long)(Astro.JDFrac()*24.*3600.+0.5));
                  break;
                case 'C': // :GC#  Get current date. Returns: MM/DD/YY#
                  sendi02(Astro.amonth); send("/"); sendi02(Astro.aday); send("/"); sendi02(Astro.ayear-2000); send("#\n");
                  break;
                case 'm': // :Gm# Get distance to Meridian [Max Only]
                          // RA angle to the Meridian (LST-RA). If the scope is on a German mount, a negative
                          // sign indicates the scope has moved to the offside of the mount where interference is possible. On fork mounts,
                          // the value is always positive.
                  send("00:00:00#\n");
                  break;
                case 'g': // :Gg# Get Current Site Longitude, East Longitudes are expressed as negative
                  send_DDMMSS_from_sec ((long)(Astro.Longitude()*RAD_TO_SEC));
                  //send("287*04#\n"); // 40 57.3080 N
                  //send("287:04:19#\n"); // 40 57.3080 N
                  break;
                case 't': // :Gt# Get Site Latitude, North positive
                  send_DDMMSS_from_sec((long)(Astro.Latitude()*RAD_TO_SEC));
                  //send("40*57#\n"); // 72 55.6640 W
                  //send("40:57:19#\n"); // 72 55.6640 W
                  break;
                case 'S': // :GS#  Get the Sidereal Time -- HourAngle
                  Astro.calc_JD_Time_GrHrAngle (millis());
                  send_HHMMSS_from_sec ((long)(Astro.calc_HrAngle ()*RAD_TO_HSEC));
                  break;
#ifdef OTHER_COMANDS
                case 'f': // :Gf# Get Browse Faint Magnitude Limit
                  send("00.0#\n");
                  break;
                case 'l': // :Gf# Get Browse Large Magnitude Limit
                  send("00.0#\n");
                  break;
  
                case 'E': // :GE#  Get Selenographic Latitude (Moon Coords)
                  send("000:00#\n");
                  break;
                case 'e': // :Ge#  Get Selenographic Longitude (Moon Coords)
                  send("000:00#\n");
                  break;

                case 'F': // :GF#  Find Field Diameter
                  send("01#\n");
                  break;
                case 'K': // K150 pyz cmd :GK# Get telescope coordinates H::D# in raw
                  send(String(axisH.Get()) +","+String(axisD.Get())+"#\n");
                  break;
#endif

                case 'T': // :GT# --- actual stepping rate, NOT 60.0Hz motor / rev relative
                  send(String(tracking_rate[MOTOR_H] * GT_CONVERT60HZ, 4) + "#\n"); // tracking_rate[MOTOR_H] = ((float)NS_U / SIDERAL_PERIOD)  !!== 60.0 'Hz'
                  break;

                case '~': // :G~# --- actual stepping rate, NOT 60.0Hz motor / rev relative
                  send(String(tracking_rate[MOTOR_H], 4) + ",");
                  send(String(tracking_rate[MOTOR_D], 4) + "#\n");
                  break;

                case 'V': // Firmware
                  switch (cmdc[3]) {
                    case 'D': send(VERSION_FWDATE1); break;
                    case 'N': send(VERSION_NUMBER); break;
                    case 'P': send(VERSION_PRODUCT); break;
                    case 'I': send(VERSION_INFO_LONG); break;
                    case 'Z': send(VERSION_CONTROLBOX); break;
                    case 'T': send(VERSION_FWDATE2); break;
                  }
                  break;
                default:
                  break;
              }
              break;
            case 'h': // Home Position Commands
              send("NA#\n");
              break;
            case 'H': // Time Format Commands
              send("24#\n");
              break;
            case 'I': // Initialize Commands
              send("#\n");
              break;
            case 'L': // Obj Lib Commands
              send("NA#\n");
              break;
            case 'D': // Distance Bars :D# Requests a string of bars indicating the distance to the current target location.
              i = axisH.Distance() + axisD.Distance();
              while (i > 0){
                send("-");
              }
              send("#");
              break;
            case 'S': // S – Set Commands
              switch (cmdc[2]) {
                case 'g': // :SgDDD*MM# Set current site’s longitude to DDD*MM an ASCII position string
                  get_sexi(&cmdc[3], site.lon);
                  Astro.Set_LocationSexi(site.lon, site.lat);
                  sendtrue();
                  break;
                case 't': // :St +DDD:MM:SS# set Site Latitude
                  get_sexi(&cmdc[3], site.lat);
                  Astro.Set_LocationSexi(site.lon, site.lat);
                  sendtrue();
                  break;
                case 'L': // :SL HH:MM:SS# Set the local Time 
                  get_sexi(&cmdc[3], startime.hms);
                  Astro.Set_Time (startime.hms[2], startime.hms[1], startime.hms[0], millis(), startime.hoffset);
                  sendtrue();
                  break;
                case 'C': // :SC YYYY/MM/DD# Set the local Date
                  get_sexi(&cmdc[3], startime.ymd);
                  Astro.Set_Date (startime.ymd[2], startime.ymd[1], startime.ymd[0]);
                  sendtrue();
                  break;
                case 'G': // ::SGsHH.H# Set the number of hours added to local time to yield UTC -- do this first!
                  startime.hoffset = atoi(&cmdc[3]);
                  Astro.Set_Time (startime.hms[2], startime.hms[1], startime.hms[0], millis(), startime.hoffset);
                  sendtrue();
                  break;
                case 'r': // :Sr HH:MM:SS#   RA set pos (object, ...) 
                  get_pos_HHMMSS(&cmdc[3]);
                  sendtrue();
                  break;
                case 'd': // :Sd DD:MM:SS#      DE set pos (object, ...)
                  get_pos_DDMMSS(&cmdc[3]);
                  sendtrue();
                  break;
                case 'K': // K150 pyz cmd :GK# Set telescope coordinates H;D# in raw
                  for (i = 0; cmdc[3 + i] != ';' && i < 64; ++i) tmp[i] = cmdc[3 + i];
                  tmp[i] = 0;
                  axisH.Set(atol(tmp));
                  j = i + 1;
                  for (i = 0; cmdc[3 + i + j] != '#' && i < 64; ++i) tmp[i] = cmdc[3 + i + j];
                  tmp[i] = 0;
                  axisD.Set(atol(tmp));
                  break;
	              case 'M': // :SM..# Setup Motor Controller Modes
            		  switch (cmdc[3]) {
              		  case 'F': set_slew_fast.setup(&cmdc[4]); break; // :SMF...# setup motor slew_fast
              		  case 'S': set_slew_slow.setup(&cmdc[4]); break; // :SMS...# setup motor slew_slow
              		  case 'T': set_tracking.setup(&cmdc[4]); break; // :SMT...# setup motor tracking
                    case 'f': set_focus.setup(&cmdc[4]); break; // :SMf...# setup motor focus
             		  } break;
                   
                default: break;
              }
              break;
            case 'Q': // Q – Movement Commands
              switch (cmdc[2]) {
	              case '*': // :Q*# or
      	        case 0: // :Q# stop slew, return to tracking if in FINE mode
                  axisH.Abort ();
                  axisD.Abort ();
            		  set_tracking.set_speed_motor (stepperH, tracking_rate[MOTOR_H]);
            		  stepperH.run(tracking_dir[MOTOR_H], tracking_rate[MOTOR_H]);
		  /*
                  if (slew_rate_mode[MOTOR_H] == SLEW_FINE_HZ) {
                    stepperH.run(tracking_dir[MOTOR_H], tracking_rate[MOTOR_H]);
                  } else {
                    stepperH.softStop();
                  }
		  */
            		  set_tracking.set_speed_motor (stepperD, tracking_rate[MOTOR_D]);
            		  stepperD.softStop();
		  /*
                  if (slew_rate_mode[MOTOR_D] == SLEW_FINE_HZ) {
                    stepperD.run(tracking_dir[MOTOR_D], tracking_rate[MOTOR_D]);
                  } else {
                    stepperD.softStop();
                  }
		  */
                  break;
                case '0': // :Q0# stop all motions
                  axisH.Abort ();
                  axisD.Abort ();
                  stepperH.softStop();
                  stepperD.softStop();
            		  while(stepperH.busyCheck());
            		  while(stepperD.busyCheck());
            		  set_tracking.set_speed_motor (stepperH, SIDERAL_H_HZ);
            		  set_tracking.set_speed_motor (stepperD, slew_rate[MOTOR_D]);
                  break;
                case 'X': // :QX# Emergency Stop All
                  axisH.Abort ();
                  axisD.Abort ();
                  stepperH.hardStop(); stepperD.hardStop(); break;
                case 'V': // :QV#  coarse fast slew -- reconfiguration will stop motor before adjusting, necessary by driver!
                  slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = SLEW_FFAST_HZ];
                  slew_rate[MOTOR_D] = slew_rate_list [slew_rate_mode[MOTOR_D] = SLEW_FFAST_HZ];
            		  set_slew_fast.config_motor (stepperH, slew_rate[MOTOR_H]);
            		  set_slew_fast.config_motor (stepperD, slew_rate[MOTOR_D]);
                  break;
                case 'v': // :Qv#  fine fastest slew
                  slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = SLEW_FFFINE_HZ];
                  slew_rate[MOTOR_D] = slew_rate_list [slew_rate_mode[MOTOR_D] = SLEW_FFFINE_HZ];
            		  set_slew_fast.config_motor (stepperH, slew_rate[MOTOR_H]);
            		  set_slew_fast.config_motor (stepperD, slew_rate[MOTOR_D]);
                  break;
                case 'F': // :QF#  fine fast slew
                  slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = SLEW_FFINE_HZ];
                  slew_rate[MOTOR_D] = slew_rate_list [slew_rate_mode[MOTOR_D] = SLEW_FFINE_HZ];
            		  set_slew_slow.config_motor (stepperH, slew_rate[MOTOR_H]);
            		  set_slew_slow.config_motor (stepperD, slew_rate[MOTOR_D]);
                  break;
                case 'f': // :Qf#  fine slew
                  slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = SLEW_FINE_HZ];
                  slew_rate[MOTOR_D] = slew_rate_list [slew_rate_mode[MOTOR_D] = SLEW_FINE_HZ];
            		  set_slew_slow.config_motor (stepperH, slew_rate[MOTOR_H]);
            		  set_slew_slow.config_motor (stepperD, slew_rate[MOTOR_D]);
                  break;
                case 'w': // :Qw# slew west
                  //break;
                case 'R': // :QR#  H+ RA+++ slew
                  stepperH.setMaxSpeed(slew_rate[MOTOR_H]);
                  stepperH.run(H_FWD, slew_rate[MOTOR_H]);
                  break;
                case 'e': // :Qe# slew east
                  //break;
                case 'r': // :Qr#  H- RA--- slew
                  stepperH.setMaxSpeed(slew_rate[MOTOR_H]);
                  stepperH.run(H_REV, slew_rate[MOTOR_H]);
                  break;
                case 'n': // :Qn# slew north
                  //break;
                case 'D': // :QD#  D+  DE+++ slew
                  stepperD.setMaxSpeed(slew_rate[MOTOR_D]);
                  stepperD.run(D_FWD, slew_rate[MOTOR_D]);
                  break;
                case 's': // :Qs# slew south
                  //break;
                case 'd': // :Qd#   D-  DE--- slew
                  stepperD.setMaxSpeed(slew_rate[MOTOR_D]);
                  stepperD.run(D_REV, slew_rate[MOTOR_D]);
                  break;
                case 'M': // K150 pyz cmd :=QMnnnn;nnnn# request Hn, Dn rel movement in raw
                  for (i = 0; cmdc[3 + i] != ';' && i < 64; ++i) tmp[i] = cmdc[3 + i];
                  tmp[i] = 0;
                  axisH.Goto(atol(tmp));
                  j = i + 1;
                  for (i = 0; cmdc[3 + i + j] != '#' && i < 64; ++i) tmp[i] = cmdc[3 + i + j];
                  tmp[i] = 0;
                  axisD.Goto(atol(tmp));
                default:
                  break;
              }
              break;
            case 'R': // Slew Rate Commands
              switch (cmdc[2]) {
                case 'C': // :RC# Set Slew rate to Centering rate (2nd slowest)
                  slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = SLEW_FFFINE_HZ];
                  slew_rate[MOTOR_D] = slew_rate_list [slew_rate_mode[MOTOR_D] = SLEW_FFFINE_HZ];
                  break;
                case 'G': // :RG# Set Slew rate to Guiding Rate (slowest)
                  slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = SLEW_FINE_HZ];
                  slew_rate[MOTOR_D] = slew_rate_list [slew_rate_mode[MOTOR_D] = SLEW_FINE_HZ];
                  break;
                case 'M': // :RM# Set Slew rate to Find Rate (2nd Fastest)
                  slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = SLEW_FFINE_HZ];
                  slew_rate[MOTOR_D] = slew_rate_list [slew_rate_mode[MOTOR_D] = SLEW_FFINE_HZ];
                  break;
                case 'S': // :RS# Set Slew rate to max (fastest)
                  slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = SLEW_FAST_HZ];
                  slew_rate[MOTOR_D] = slew_rate_list [slew_rate_mode[MOTOR_D] = SLEW_FAST_HZ];
                  break;
                case '*': // :R[RS]*n,fff.ff# Read / Reconfigure slew rate list settings:  slew_rate_list[n] = f
                  switch (cmdc[3]) {
                    case 'R': // :R*Rn# Read
                      for (i = 0; cmdc[4 + i] != '#' && i < 64; ++i) tmp[i] = cmdc[4 + i];
                      tmp[i] = 0;
                      k = atoi(tmp);
                      if (k >= 0 && k < NUM_SLEW_RATES) {
                        send(String(slew_rate_list[k], 4) + "#\n");
                      } else {
                        send("ERROR: R*R BAD INDEX " + String(k) + "#\n");
                      }
                      break;
                    case 'S': // :R*Sn,fff.ff#  Set
                      for (i = 0; cmdc[4 + i] != ';' && i < 64; ++i) tmp[i] = cmdc[4 + i];
                      tmp[i] = 0;
                      k = atoi(tmp);
                      if (k >= 0 && k < NUM_SLEW_RATES) {
                        j = i + 1;
                        for (i = 0; cmdc[4 + i + j] != '#' && i < 64; ++i) tmp[i] = cmdc[4 + i + j];
                        tmp[i] = 0;
                        slew_rate_list[k] = atof(tmp);
                        send("OK. Set[" + String(k) + "]=" + String(slew_rate_list[k], 4) + "#\n");
                      } else {
                        send("ERROR: R*S BAD INDEX " + String(k) + "#\n");
                      }
                      break;
                    case 'C': // :R*Cn# Choose Slew Rate
                      tmp[0] = cmdc[4];
                      tmp[1] = 0;
                      k = atoi(tmp);
                      if (k >= 0 && k < NUM_SLEW_RATES) {
                        slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = k];
                        slew_rate[MOTOR_D] = slew_rate_list [slew_rate_mode[MOTOR_D] = k];
                        send("SLEW HZ=" + String(slew_rate_list[k], 4) + "#\n");
                      } else {
                        send("ERROR: R*C BAD INDEX " + String(k) + "#\n");
                      }
                      break;
                  }
                  break;
                case 'R': // :RRn,fff.ff# Reconfigure slew rate list settings:  slew_rate_list[n] = f
                  for (i = 0; cmdc[3 + i] != ';' && i < 64; ++i) tmp[i] = cmdc[3 + i];
                  tmp[i] = 0;
                  k = atoi(tmp);
                  if (k >= 0 && k < NUM_SLEW_RATES) {
                    j = i + 1;
                    for (i = 0; cmdc[3 + i + j] != '#' && i < 64; ++i) tmp[i] = cmdc[3 + i + j];
                    tmp[i] = 0;
                    slew_rate_list[k] = atof(tmp);
                  }
                  break;
              }
            case 'M': // Telescope Movement Commands (experimental)
              switch (cmdc[2]) {
                case 'g': // :Mg[nsew]DDDD#  Guide telescope in the commanded direction (nsew) for the number of milliseconds indicated by the unsigned number passed in the command. These commands support serial port driven guiding
                  switch (cmdc[3]) {
                    case 'n': break;
                  }
                  break;
                case 'S': // :MS#  Slew to Target object (cur pos), track
                  // slew to mem position set last

                  Astro.calc_JD_Time_GrHrAngle (millis());
                  Astro.Use_memRADecl ();
                  axisH.Goto((long)(Astro.calc_HrAngle ()*RAD_NS_US));
                  axisD.Goto((long)(Astro.Get_memDecl ()*RAD_NS_US));
                  sendfalse();
                  send (" *SLEWING TO OBJ...#\n");

                  // RECURSIVE CHECK/RA->HA UPDATE, FINAL ENABLE TRACKING
                  //slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = SLEW_FINE_HZ];
                  //set_tracking.config_motor (stepperH, slew_rate[MOTOR_H]);
                  //tracking_rate[MOTOR_H] = SIDERAL_H_HZ;
                  //stepperH.setMaxSpeed(tracking_rate[MOTOR_H]);
                  stepperH.run(tracking_dir[MOTOR_H], tracking_rate[MOTOR_H]);                

                  break;
                case 'T': // :MTbool# auto track HR angle error and correct, bool = 0,1
                  auto_track_HAbyRA = atoi (&cmdc[3]);
                  break;
              }
              break;
            case 'T': // T – Tracking Commands
              switch (cmdc[2]) {
                case '+': // :T+#
                  tracking_rate[MOTOR_H] += TR_ADJUST01HZ;
	                set_tracking.set_speed_motor (stepperH, slew_rate[MOTOR_H]);
                  //stepperH.setMaxSpeed(tracking_rate[MOTOR_H]);
                  stepperH.run(tracking_dir[MOTOR_H], tracking_rate[MOTOR_H]);
                  break;
                case '-': // :T+#
                  tracking_rate[MOTOR_H] -= TR_ADJUST01HZ;
		              set_tracking.set_speed_motor (stepperH, slew_rate[MOTOR_H]);
                  //stepperH.setMaxSpeed(tracking_rate[MOTOR_H]);
                  stepperH.run(tracking_dir[MOTOR_H], tracking_rate[MOTOR_H]);
                  break;
                case 'R': // :TR# report current tracking rates
                  send(String(tracking_rate[MOTOR_H], 4) + ";" + String(tracking_rate[MOTOR_D], 4) + "#\n");
                  break;
                case 'H': // :TH#  track hour ++ axis sideral
                  slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = SLEW_FINE_HZ];
	            	  set_tracking.config_motor (stepperH, slew_rate[MOTOR_H]);
                  tracking_rate[MOTOR_H] = SIDERAL_H_HZ;
                  //stepperH.setMaxSpeed(tracking_rate[MOTOR_H]);
                  stepperH.run(tracking_dir[MOTOR_H], tracking_rate[MOTOR_H]);
                  break;
                case 'D': // track decl ++ axis sideral
                  slew_rate[MOTOR_D] = slew_rate_list [slew_rate_mode[MOTOR_D] = SLEW_FINE_HZ];
	            	  set_tracking.config_motor (stepperD, slew_rate[MOTOR_D]);
                  tracking_rate[MOTOR_D] = SIDERAL_H_HZ;
                  //stepperD.setMaxSpeed(tracking_rate[MOTOR_D]);
                  stepperD.run(tracking_dir[MOTOR_D], tracking_rate[MOTOR_D]);
                  break;

                case 'C': // :TCffff.f,ffff.f# configure custom tracking rates H,D := fHz, fHz
                  switch (cmdc[3]) {
                    case 'R': // :TCR# report custom tracking rates
                      send(String(custom_tracking[MOTOR_H], 4) + ";" + String(custom_tracking[MOTOR_D], 4) + "#\n");
                      break;
                    default:
                      for (i = 0; cmdc[3 + i] != ';' && i < 64; ++i) tmp[i] = cmdc[3 + i];
                      tmp[i] = 0;
                      custom_tracking[MOTOR_H] = atof(tmp) * ST_CONVERT60HZ;
                      j = i + 1;
                      for (i = 0; cmdc[3 + i + j] != '#' && i < 64; ++i) tmp[i] = cmdc[3 + i + j];
                      tmp[i] = 0;
                      custom_tracking[MOTOR_D] = atof(tmp) * ST_CONVERT60HZ;
                      send(String(custom_tracking[MOTOR_H], 4) + ";" + String(custom_tracking[MOTOR_D], 4) + "#\n");
                      break;
                  }
                  break;
                case 'M': // :TM# Select custom tracking rate [ no-op in Autostar II]
                  slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = SLEW_FINE_HZ];
                  slew_rate[MOTOR_D] = slew_rate_list [slew_rate_mode[MOTOR_D] = SLEW_FINE_HZ];
                  //stepperH.setMaxSpeed (tracking_rate[MOTOR_H] = custom_tracking[MOTOR_H]);
                  //stepperD.setMaxSpeed (tracking_rate[MOTOR_D] = custom_tracking[MOTOR_D]);
            		  set_tracking.set_speed_motor (stepperH, slew_rate[MOTOR_H]);
             		  set_tracking.set_speed_motor (stepperD, slew_rate[MOTOR_D]);
                  stepperH.run(tracking_dir[MOTOR_H], tracking_rate[MOTOR_H]);
                  stepperD.run(tracking_dir[MOTOR_D], tracking_rate[MOTOR_D]);
                  break;
                case 'Q':  // :TQ# Selects sidereal tracking rate
                  slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = SLEW_FINE_HZ];
                  tracking_rate[MOTOR_H] = SIDERAL_H_HZ;
            		  set_tracking.set_speed_motor (stepperH, slew_rate[MOTOR_H]);
                  //stepperH.setMaxSpeed(tracking_rate[MOTOR_H]);
                  stepperH.run(tracking_dir[MOTOR_H], tracking_rate[MOTOR_H]);
                  break;
                case 'S':  // :TS# Selects solar tracking rate
                  slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = SLEW_FINE_HZ];
                  tracking_rate[MOTOR_H] = SOLAR_H_HZ;
		              set_tracking.set_speed_motor (stepperH, slew_rate[MOTOR_H]);
                  //stepperH.setMaxSpeed(tracking_rate[MOTOR_H]);
                  stepperH.run(tracking_dir[MOTOR_H], tracking_rate[MOTOR_H]);
                  break;
                case 'L':  // :TL# Selects lunar tracking rate
                  slew_rate[MOTOR_H] = slew_rate_list [slew_rate_mode[MOTOR_H] = SLEW_FINE_HZ];
                  tracking_rate[MOTOR_H] = LUNAR_H_HZ;
		              set_tracking.set_speed_motor (stepperH, slew_rate[MOTOR_H]);
                  //stepperH.setMaxSpeed(tracking_rate[MOTOR_H]);
                  stepperH.run(tracking_dir[MOTOR_H], tracking_rate[MOTOR_H]);
                  break;
                case '>':  // :T>[HD]# Select forward tracking on axis
                  switch (cmdc[3]) {
              		  case 'H': tracking_dir[MOTOR_H] = H_FWD;
              		    stepperH.run(tracking_dir[MOTOR_H], tracking_rate[MOTOR_H]);
                      bitSet   (key_pad_led_status, 0);
                      bitClear (key_pad_led_status, 1);
              		    break;
              		  case 'D': tracking_dir[MOTOR_D] = D_FWD;
              		    stepperD.run(tracking_dir[MOTOR_D], tracking_rate[MOTOR_D]);
              		    break;
                  }
                  break;
                case '<':  // :T<[HD]# Select reverse tracking on axis
                  switch (cmdc[3]) {
              		  case 'H': tracking_dir[MOTOR_H] = H_REV;
              		    stepperH.run(tracking_dir[MOTOR_H], tracking_rate[MOTOR_H]);
                      bitSet   (key_pad_led_status, 1);
                      bitClear (key_pad_led_status, 0);
              		    break;
              		  case 'D': tracking_dir[MOTOR_D] = D_REV;
              		    stepperD.run(tracking_dir[MOTOR_D], tracking_rate[MOTOR_D]);
              		    break;
                  }
                  break;
                case '0':  // :T0[HD]# disable tracking on axis, T00: Emergency Stop All Motions
                  switch (cmdc[3]) {
                    case 'H':
                      stepperH.softStop(); 
                      bitClear (key_pad_led_status, 0);
                      bitClear (key_pad_led_status, 1);
                      break;
                    case 'D': stepperD.softStop(); break;
                    case '0': 
                      stepperH.hardStop(); stepperD.hardStop(); break;
                  }
                  break;
                default:
                  break;
              }
              break;
              
            case 'U': 
              switch (cmdc[2]) {
                case '2': send("GM K150 X-NUCLEO#\n"); break;
                default:  send("PREC TOGGLE N/A: HIGH#\n"); break;
              }
              break;
            case 'V': send("PEC N/A#\n"); break;
            case 'W': send("SITE SELECT N/A#\n"); break;


          // various, dummy
          // #:modelcnt#  LOGF_INFO("%d Alignment Models", (int) ModelCountN[0].value);
          // #:getalst#   LOGF_INFO("%d Alignment Stars in active model", (int) AlignmentPointsN[0].value);

            case 'm': // :modelcnt# ...
              send("0#\n"); break;
              
            case 'g': // :getalst# ...
              switch (cmdc[2]) {
                case 'e': send("0#\n"); break;
                case '#': // GPS
                  send("GPS N/A#\n");
                  break;
                default:  send("E?#\n"); break;
              }
              break;


            case '?':
#ifdef ENABLE_HELP
              send(F("HELP\n"));
              send(F("Arduino + 3 Axis PowerStep01 * X-Nucleo-IHM03A1 Telescope Control K150\n"));
              send(F(" based on LX200 MEADE TELESCOPE SERIAL COMMAND PROTOCOL\n"));
              send(F(" with custom K150 extensions and 10-Micron high precision extensions (C) by P.Zahl\n\n"));
              send(F("Extensions:\n"));
              send(F("Get Telescope Information:\n"));
              send(F(" :Ginfo# Get RA,DA Axis Status\n"));
              send(F(" :GN# Get Telescope RA, Returns position integer counts [RA,DE]: [0xHHHHHHHH,0xHHHHHHHH]#\n"));
              send(F(" :Gn# get interger steps count perfull circle H,D\n"));
              send(F("Movement Commands:\n"));
              send(F(" :Q# stop slew, return to tracking if in FINE mode\n"));
              send(F(" :Q0# stop all motions\n"));
              send(F(" :QV# start fast slew forward\n"));
              send(F(" :Qv# start fast slew reverse\n"));
              send(F(" :QF# start fine slew forward\n"));
              send(F(" :Qf# start fine slew reverse\n"));
              send(F("Tracking:\n"));
              send(F(" :TCfff.f;fff.f# Configure custom tracking rate, stepper Hz \n"));
              send(F(" :T>[HD]# Select forward tracking on axis \n"));
              send(F(" :T<[HD]# Select reverse tracking on axis \n"));
              send(F(" :T0[HD]# disable tracking on axis \n"));
              send(F("KeyPad control:\n"));
              send(F(" :%K# enable Keypad \n"));
              send(F(" :%k# disable Keypad \n"));
              send(F(" :%0...9,A,B# query DKey[][] matrix \n"));
              send(F(" \n\n"));
              send(F("Quick Commands:\n !: Print Status, Pos\n s,S,T: set_slew and power slow, fast, tracking\n h,d run H, D axis\n H,D: toggle dir\n +-,<>: speed adjust H,D\n NumBlock 4,6: -/+H, 2,8: -/+D, 1,3,7,9 dual H+D motion, 5,0: STOP\n .,t: tracking on\n\n"));
        
              send(F(" stepper H status:"));
              Serial.println((int)stepperH.getStatus(), HEX); // print STATUS register
              send(F(" stepper D status:"));
              Serial.println((int)stepperD.getStatus(), HEX); // print STATUS register
              Serial.println("Motors Fast Setup:"); // print STATUS register
        set_slew_fast.query_setup ();
              Serial.println("Motors Slow Setup:"); // print STATUS register
        set_slew_slow.query_setup ();
              Serial.println("Motors Tracking Setup:"); // print STATUS register
        set_tracking.query_setup ();
              Serial.println("Motors Focus Setup:"); // print STATUS register
        set_focus.query_setup ();
              // print_pos();
              send("###\n");
#else
              send(F("NA#\n"));
#endif
              break;

            case '@': // :@# Test Astro Class
              send("*Astro Class Test*\n");
              Astro.Test (&Serial, millis());
              break;

            case '%': // :%N# KeyPad Controland Query DKey
              switch (cmdc[2]) {
                case 'K': key_pad_enable = 1; break; // :%K#  keyPad enable
                case 'k': key_pad_enable = 0; break; // :%k#  keyPad disable
#if 0
                case '7': send("K[00]="); send(DKey[0][0]); send("#\n"); break;
                case '8': send("K[10]="); send(DKey[1][0]); send("#\n"); break;
                case '9': send("K[20]="); send(DKey[2][0]); send("#\n"); break;

                case '4': send("K[01]="); send(DKey[0][1]); send("#\n"); break;
                case '5': send("K[11]="); send(DKey[1][1]); send("#\n"); break;
                case '6': send("K[21]="); send(DKey[2][1]); send("#\n"); break;

                case '1': send("K[02]="); send(DKey[0][2]); send("#\n"); break;
                case '2': send("K[12]="); send(DKey[1][2]); send("#\n"); break;
                case '3': send("K[22]="); send(DKey[2][2]); send("#\n"); break;

                case '0': send("K[03]="); send(DKey[0][3]); send("#\n"); break;
                case 'A': send("K[13]="); send(DKey[1][3]); send("#\n"); break;
                case 'B': send("K[23]="); send(DKey[2][3]); send("#\n"); break;
#endif
                case 's': scan_i2c_bus(); break;
                case 'C': update_coords_display (1); break; // clear display and refresh
                case 'm': {
                    long p1,p2;
                    send("H Tracking SIDERAL_H_HZ enabled.");
                    stepperH.run(H_FWD, SIDERAL_H_HZ);
                    while (stepperH.busyCheck());
                    delay(1000); p1=axisH.Get();
                    send("Time in ms: " + String(millis()) + ", H#: "+String(p1) + ", Rate: " + String(stepperH.getCurrentSpeed(),8) + ", measuring 30s...");
                    delay(30000); p2=axisH.Get();
                    send("Time in ms: " + String(millis()) + ", H#: "+String(p2) + ", Rate: " + String(stepperH.getCurrentSpeed(),8));
                    send(" --> dH#: " + String(p2-p1) + "/30s --> Sideral Step Rate in Hz: " + String((float)(p2-p1)*SIDERAL_RATIO/30.,8));
                  }
                  break;
              }

          }
          cmdi = 0; cmdc[2] = '#';
#ifdef SERIAL_ECHO
          if (hw_echo)
            hw_echo->print("\n");
          if (sw_echo)
            sw_echo->print("\n");
#endif
          break;
        default: // dispatch to command string
          if (cmdi < (MAX_CMD_LEN-1)){
            cmdc[cmdi++] = controlbyte;
            cmdc[cmdi]   = 0;
	  }else
            cmdi = 0;
          for (i = 0; i < 64; ++i) tmp[i] = 0;
          break;
      }
    }
}


void startup_message_string(String &msg, unsigned int color=0) {
  Serial.print(msg);
  if (color)
    display.setTextColor(color);
  display.print(msg);
}

void startup_message(const char *msg, unsigned int color=0){
  Serial.print(msg);
  if (color)
    display.setTextColor(color);
  display.print(msg);
}

void startup_message_ln(const char *msg, unsigned int color=0){
  Serial.println(msg);
  if (color)
    display.setTextColor(color);
  display.println(msg);
}

void startup_message_serial_only(const char *msg){
  Serial.println(msg);
}

#if 0
void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}
#endif

void setup() 
{
  int i;
  long p1,p2;

  // Start serial
#ifdef USE_UNO

  // Serial.begin(9600);
  // Serial.begin(115200);
  Serial.begin(19200);

  // define pin modes for tx, rx used by BlueTouth SoftwareSerial 
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

#else

  Serial.begin(SERIAL_PRG_BAUDRATE);
  Serial.println (F("THIS IS USB SERIAL PRG PP."));
  
  Serial1.begin(SERIAL1_BAUDRATE);
  Serial1.println (F("THIS IS SERIAL1."));
  
  Serial2.begin(SERIAL2_BAUDRATE);
  Serial2.println (F("THIS IS SERIAL2."));
  
  
  SerialUSB.begin(2000000);
  while(!SerialUSB && millis() < 1000);
  SerialUSB.println (F("THIS IS NATIVE SERIAL USB."));

  Serial3.begin(SERIAL3_BAUDRATE);
  Serial3.println (F("AT"));

  i=100;
  while (!Serial3.available() && --i)
    delay(100);

  if (Serial3.available())
    SerialUSB.println(Serial3.read());
  else
    SerialUSB.println("BLE not responding.");
  
  Serial3.println (F("THIS IS SERIAL3."));

  
#endif
  // Prepare pins
#ifdef USE_UNO
  pinMode(nSTBY_nRESET_PIN, OUTPUT);
  pinMode(nCS_PIN, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);

#else
  // X-Nucleo
  pinMode(nSTBY_nRESET_PIN, OUTPUT);
  pinMode(nCS_PIN, OUTPUT);
  
  // SSD1351 OLED display
  pinMode(SSD1351_DC_PIN, OUTPUT);
  pinMode(SSD1351_RST_PIN, OUTPUT);
  pinMode(SSD1351_CS_PIN, OUTPUT);

  // SD
  pinMode(SD_CS_PIN, OUTPUT);
#endif

  // Reset powerSTEP and set CS
  digitalWrite(nSTBY_nRESET_PIN, HIGH);
  delay(10);
  digitalWrite(nSTBY_nRESET_PIN, LOW);
  delay(10);
  digitalWrite(nSTBY_nRESET_PIN, HIGH);
  delay(10);
  digitalWrite(nCS_PIN, HIGH);

  // Reset Display CS Pin
  digitalWrite(SSD1351_CS_PIN, HIGH);

  // Reset SD CS Pin
  digitalWrite(SD_CS_PIN, HIGH);

  delay(10);

  // Init Display
  display.begin();
  display.fillRect(0, 0, 128, 128, BLACK);
  display.fillScreen(BLACK);
  display.setCursor(0, 5);
  display.setTextSize(1);
  display.setTextColor(YELLOW);
  startup_message_ln(VERSION_INFO, YELLOW);
  display.setTextColor(RED);  

#ifdef ENABLE_I2C_KEYPAD
  startup_message_serial_only("I2C initializing...");
  // Start I2C for MCP23S17 IO extender for KeyPad
  Wire.begin(); // wake up I2C bus
  // MCP23017 A read KeyY, enable pullup
  // MCP23017 B write KeyX

  // Wire.setClock(100000);
  startup_message("KEYPAD I2C... ", BLUE);
  Wire.beginTransmission(MCP_I2C_SELECT);
  if (!Wire.endTransmission()){
  
    Wire.beginTransmission(MCP_I2C_SELECT);
    Wire.write(MCP23S17_IODIRB); // IODIRB register
    Wire.write(0x00); // set all of port B to outputs
    Wire.endTransmission();
  
    Wire.beginTransmission(MCP_I2C_SELECT);
    Wire.write(MCP23S17_IPOLA); // IPOLA (enable 100k pull up) register
    Wire.write(0xFF); // set all of port A pullup
    Wire.endTransmission();
    
    Wire.beginTransmission(MCP_I2C_SELECT);
    Wire.write(MCP23S17_GPPUA); // GPPUA (enable 100k pull up) register
    Wire.write(0xFF); // set all of port A to inputs
    Wire.endTransmission();
    key_pad_enable = 1;
    startup_message_ln("OK", GREEN);
  } else {
    key_pad_enable = 0;
    startup_message_ln("not found", RED);
  }
#endif


  if (!SD.begin(SD_CS_PIN)) {
    startup_message_ln("SD init: failed", RED);
    startup_message_string("SD error: "+String(SD.errorCode())+"\n", RED);
    startup_message_string("SD data:  "+String(SD.errorData())+"\n", RED);
  } else {
    startup_message_ln("SD card found", GREEN);
    //File root = SD.open("/");
    //printDirectory(root, 0);
  }
  
  // Configure powerSTEP
  // Motor 1 (stepperH)
  startup_message("Motor H init...", BLUE);
  delay(10);
  // Reset powerSTEP and set CS
  digitalWrite(nSTBY_nRESET_PIN, HIGH);
  delay(10);
  digitalWrite(nSTBY_nRESET_PIN, LOW);
  delay(10);
  digitalWrite(nSTBY_nRESET_PIN, HIGH);
  delay(10);

  // Start SPI
  SPI.begin(nCS_PIN);
  SPI.setDataMode(SPI_MODE3);

  stepperH.SPIPortConnect(&SPI); // give library the SPI port (only the one on an Uno)
  
  stepperH.configSyncPin(BUSY_PIN, 0); // use SYNC/nBUSY pin as nBUSY, 
                                     // thus syncSteps (2nd paramater) does nothing
                                     
  stepperH.configStepMode(STEP_FS_128); // 1/128 microstepping, full steps = STEP_FS,
                                // options: 1, 1/2, 1/4, 1/8, 1/16, 1/32, 1/64, 1/128
                                
  stepperH.setLoSpdOpt(true); // enable LSPD_OPT bit in the MIN_SPEED register
  stepperH.setMinSpeed(10.*SIDERAL_H_HZ);

  stepperH.setMaxSpeed(500); // max speed in units of full steps/s = 1000
  stepperH.setFullSpeed(2000); // full steps/s threshold for disabling microstepping =2000
  stepperH.setAcc(200); // full steps/s^2 acceleration
  stepperH.setDec(200); // full steps/s^2 deceleration
  
  stepperH.setSlewRate(SR_520V_us); // faster may give more torque (but also EM noise),
                                  // options are: 114, 220, 400, 520, 790, 980(V/us)
                                  
  stepperH.setOCThreshold(8); // over-current threshold for the 2.8A NEMA23 motor
                            // used in testing. If your motor stops working for
                            // no apparent reason, it's probably this. Start low
                            // and increase until it doesn't trip, then maybe
                            // add one to avoid misfires. Can prevent catastrophic
                            // failures caused by shorts
  stepperH.setOCShutdown(OC_SD_ENABLE); // shutdown motor bridge on over-current event
                                      // to protect against permanant damage
  
  stepperH.setPWMFreq(PWM_DIV_1, PWM_MUL_0_75); // 16MHz*0.75/(512*1) = 23.4375kHz 
                            // power is supplied to stepper phases as a sin wave,  
                            // frequency is set by two PWM modulators,
                            // Fpwm = Fosc*m/(512*N), N and m are set by DIV and MUL,
                            // options: DIV: 1, 2, 3, 4, 5, 6, 7, 
                            // MUL: 0.625, 0.75, 0.875, 1, 1.25, 1.5, 1.75, 2
                            
  stepperH.setVoltageComp(VS_COMP_DISABLE); // no compensation for variation in Vs as
                                          // ADC voltage divider is not populated
                                          
  stepperH.setSwitchMode(SW_USER); // switch doesn't trigger stop, status can be read.
                                 // SW_HARD_STOP: TP1 causes hard stop on connection 
                                 // to GND, you get stuck on switch after homing
                                      
  stepperH.setOscMode(INT_16MHZ); // 16MHz internal oscillator as clock source

  // KVAL registers set the power to the motor by adjusting the PWM duty cycle,
  // use a value between 0-255 where 0 = no power, 255 = full power.
  // Start low and monitor the motor temperature until you find a safe balance
  // between power and temperature. Only use what you need
  stepperH.setRunKVAL(32);
  stepperH.setAccKVAL(80);
  stepperH.setDecKVAL(64);
  stepperH.setHoldKVAL(12);

  stepperH.setParam(ALARM_EN, 0x8F); // disable ADC UVLO (divider not populated),
                                   // disable stall detection (not configured),
                                   // disable switch (not using as hard stop)

  stepperH.getStatus(); // clears error flags
  startup_message_ln((const char*)F("OK"), GREEN);


  // Motor 2 (stepperD)
  startup_message((const char*)F("Motor D init..."), BLUE);
  stepperD.SPIPortConnect(&SPI); // give library the SPI port (only the one on an Uno)
  
  stepperD.configSyncPin(BUSY_PIN, 0); // use SYNC/nBUSY pin as nBUSY, 
                                     // thus syncSteps (2nd paramater) does nothing
                                     
  stepperD.configStepMode(STEP_FS_128); // 1/128 microstepping, full steps = STEP_FS,
                                // options: 1, 1/2, 1/4, 1/8, 1/16, 1/32, 1/64, 1/128
                                
  stepperD.setLoSpdOpt(true); // enable LSPD_OPT bit in the MIN_SPEED register
  stepperD.setMinSpeed(10.*SIDERAL_H_HZ);
  stepperD.setMaxSpeed(500); // max speed in units of full steps/s = 1000
  stepperD.setFullSpeed(2000); // full steps/s threshold for disabling microstepping =2000
  stepperD.setAcc(200); // full steps/s^2 acceleration
  stepperD.setDec(200); // full steps/s^2 deceleration
  
  stepperD.setSlewRate(SR_520V_us); // faster may give more torque (but also EM noise),
                                  // options are: 114, 220, 400, 520, 790, 980(V/us)
                                  
  stepperD.setOCThreshold(8); // over-current threshold for the 2.8A NEMA23 motor
                            // used in testing. If your motor stops working for
                            // no apparent reason, it's probably this. Start low
                            // and increase until it doesn't trip, then maybe
                            // add one to avoid misfires. Can prevent catastrophic
                            // failures caused by shorts
  stepperD.setOCShutdown(OC_SD_ENABLE); // shutdown motor bridge on over-current event
                                      // to protect against permanant damage
  
  stepperD.setPWMFreq(PWM_DIV_1, PWM_MUL_0_75); // 16MHz*0.75/(512*1) = 23.4375kHz 
                            // power is supplied to stepper phases as a sin wave,  
                            // frequency is set by two PWM modulators,
                            // Fpwm = Fosc*m/(512*N), N and m are set by DIV and MUL,
                            // options: DIV: 1, 2, 3, 4, 5, 6, 7, 
                            // MUL: 0.625, 0.75, 0.875, 1, 1.25, 1.5, 1.75, 2
                            
  stepperD.setVoltageComp(VS_COMP_DISABLE); // no compensation for variation in Vs as
                                          // ADC voltage divider is not populated
                                          
  stepperD.setSwitchMode(SW_USER); // switch doesn't trigger stop, status can be read.
                                 // SW_HARD_STOP: TP1 causes hard stop on connection 
                                 // to GND, you get stuck on switch after homing
                                      
  stepperD.setOscMode(INT_16MHZ); // 16MHz internal oscillator as clock source

  // KVAL registers set the power to the motor by adjusting the PWM duty cycle,
  // use a value between 0-255 where 0 = no power, 255 = full power.
  // Start low and monitor the motor temperature until you find a safe balance
  // between power and temperature. Only use what you need
  stepperD.setRunKVAL(32);
  stepperD.setAccKVAL(80);
  stepperD.setDecKVAL(64);
  stepperD.setHoldKVAL(12);

  stepperD.setParam(ALARM_EN, 0x8F); // disable ADC UVLO (divider not populated),
                                   // disable stall detection (not configured),
                                   // disable switch (not using as hard stop)

  stepperD.getStatus(); // clears error flags
  startup_message_ln((const char*)F("OK"), GREEN);


  // Motor 3 (stepperF)
  startup_message((const char*)F("Motor Focus init..."), BLUE);
  stepperF.SPIPortConnect(&SPI); // give library the SPI port (only the one on an Uno)
  
  stepperF.configSyncPin(BUSY_PIN, 0); // use SYNC/nBUSY pin as nBUSY, 
                                     // thus syncSteps (2nd paramater) does nothing
                                     
  stepperF.configStepMode(STEP_FS_128); // 1/128 microstepping, full steps = STEP_FS,
                                // options: 1, 1/2, 1/4, 1/8, 1/16, 1/32, 1/64, 1/128
                                
  stepperF.setLoSpdOpt(true); // enable LSPD_OPT bit in the MIN_SPEED register
  stepperF.setMinSpeed(10.*SIDERAL_H_HZ);
  stepperF.setMaxSpeed(500); // max speed in units of full steps/s = 1000
  stepperF.setFullSpeed(2000); // full steps/s threshold for disabling microstepping =2000
  stepperF.setAcc(200); // full steps/s^2 acceleration
  stepperF.setDec(200); // full steps/s^2 deceleration
  
  stepperF.setSlewRate(SR_520V_us); // faster may give more torque (but also EM noise),
                                  // options are: 114, 220, 400, 520, 790, 980(V/us)
                                  
  stepperF.setOCThreshold(8); // over-current threshold for the 2.8A NEMA23 motor
                            // used in testing. If your motor stops working for
                            // no apparent reason, it's probably this. Start low
                            // and increase until it doesn't trip, then maybe
                            // add one to avoid misfires. Can prevent catastrophic
                            // failures caused by shorts
  stepperF.setOCShutdown(OC_SD_ENABLE); // shutdown motor bridge on over-current event
                                      // to protect against permanant damage
  
  stepperF.setPWMFreq(PWM_DIV_1, PWM_MUL_0_75); // 16MHz*0.75/(512*1) = 23.4375kHz 
                            // power is supplied to stepper phases as a sin wave,  
                            // frequency is set by two PWM modulators,
                            // Fpwm = Fosc*m/(512*N), N and m are set by DIV and MUL,
                            // options: DIV: 1, 2, 3, 4, 5, 6, 7, 
                            // MUL: 0.625, 0.75, 0.875, 1, 1.25, 1.5, 1.75, 2
                            
  stepperF.setVoltageComp(VS_COMP_DISABLE); // no compensation for variation in Vs as
                                          // ADC voltage divider is not populated
                                          
  stepperF.setSwitchMode(SW_USER); // switch doesn't trigger stop, status can be read.
                                 // SW_HARD_STOP: TP1 causes hard stop on connection 
                                 // to GND, you get stuck on switch after homing
                                      
  stepperF.setOscMode(INT_16MHZ); // 16MHz internal oscillator as clock source

  // KVAL registers set the power to the motor by adjusting the PWM duty cycle,
  // use a value between 0-255 where 0 = no power, 255 = full power.
  // Start low and monitor the motor temperature until you find a safe balance
  // between power and temperature. Only use what you need
  stepperF.setRunKVAL(32);
  stepperF.setAccKVAL(80);
  stepperF.setDecKVAL(64);
  stepperF.setHoldKVAL(12);

  stepperF.setParam(ALARM_EN, 0x8F); // disable ADC UVLO (divider not populated),
                                   // disable stall detection (not configured),
                                   // disable switch (not using as hard stop)

  stepperF.getStatus(); // clears error flags
  startup_message_ln((const char*)F("OK"), GREEN);



 #if 0
  startup_message_serial_only("TESTING MOTOR D");
  static long mpl[6]={-2097151L, -1000L, 0L, 10000L, 100000L, 2097152L};
  long mp;
  for (int ii=0; ii<6; ++ii){
    mp = mpl[ii];
    //startup_message_serial_only("Begin" + String (mp) + " -> " + String (stepperD.getPos ()));
    stepperD.setPos (mp);
    //startup_message_serial_only("Set  " + String (mp) + " -> " + String (stepperD.getPos ()));
    mp += 5000L;
    stepperD.goTo (mp);
    while (stepperD.busyCheck())
      ;//startup_message_serial_only("B+Run" + String (mp) + " -> " + String (stepperD.getPos ()));
    mp -= 15000L;
    stepperD.goTo (mp);
    while (stepperD.busyCheck())
      ;//startup_message_serial_only("B-Run" + String (mp) + " -> " + String (stepperD.getPos ()));
  }
#endif


 startup_message_ln((const char*)F("X-Nucleo READY"), GREEN);

#ifdef USE_UNO
  comm_master  = new Communication ("COMM_MASTER->Serial",  &Serial);
  comm_blue    = new Communication ("COMM_BLUE->SoftSerial",  &BlueSerial);
#else
  comm_master  = new Communication ("COMM_MASTER_SERIAL",  &SerialUSB);
  comm_usb     = new Communication ("COMM_USBNATIVE", &Serial);
  comm_blue    = new Communication ("COMM_SERIAL3",  &Serial3);
#endif

  startup_message_ln((const char*)F("COMM READY"), GREEN);

  //  attachInterrupt(0, handlerEncoder, CHANGE);

  slew_rate[MOTOR_H] = slew_rate_list [SLEW_FFAST_HZ];
  slew_rate[MOTOR_D] = slew_rate_list [SLEW_FFAST_HZ];


#ifdef MOTOR_FWD_TEST_AFTER_STARTUP
  while (stepperH.busyCheck());
  while (stepperD.busyCheck());
  set_slew_fast.set_speed_motor (stepperH, slew_rate[MOTOR_H]);
  set_slew_fast.set_speed_motor (stepperD, slew_rate[MOTOR_D]);
  startup_message_serial_only("HA Worm Gear FWD one turn test.");
  while (stepperH.busyCheck());
  startup_message_serial_only("Time in ms: " + String(millis()) + ", HA#: "+String(axisH.Get()) + ", Rate: " + String(stepperH.getCurrentSpeed(),8));
  stepperH.move(H_FWD, 128*200*3); // one turn worm gear
  while (stepperH.busyCheck());
  delay(1000);
  startup_message_serial_only("Time in ms: " + String(millis()) + ", HA#: "+String(axisH.Get()) + ", Rate: " + String(stepperH.getCurrentSpeed(),8));

  startup_message_serial_only("DE Worm Gear FWD one turn test.");
  while (stepperD.busyCheck());
  startup_message_serial_only("Time in ms: " + String(millis()) + ", DE#: "+String(axisD.Get()) + ", Rate: " + String(stepperD.getCurrentSpeed(),8));
  stepperD.move(D_FWD, 128*200*3); // one turn worm gear
  while (stepperD.busyCheck());
  delay(1000);
  startup_message_serial_only("Time in ms: " + String(millis()) + ", DE#: "+String(axisD.Get()) + ", Rate: " + String(stepperD.getCurrentSpeed(),8));
#endif

  startup_message_ln((const char*)F("SETTING SLEW/TRACK"), GREEN);

  slew_rate[MOTOR_H] = slew_rate_list [SLEW_FAST_HZ];
  slew_rate[MOTOR_D] = slew_rate_list [SLEW_FAST_HZ];
  startup_message((const char*)F("setH"), GREEN);
  set_slew_slow.set_speed_motor (stepperH, slew_rate[MOTOR_H]);
  startup_message((const char*)F("D"), GREEN);
  set_slew_slow.set_speed_motor (stepperD, slew_rate[MOTOR_D]);
  startup_message((const char*)F("."), GREEN);

  startup_message_serial_only("H Tracking and D Slew, F focus setup completed.");
  startup_message((const char*)F("HB:"), GREEN);
  //while (stepperH.busyCheck());
  startup_message((const char*)F("DB:"), GREEN);
  //while (stepperD.busyCheck());
  set_tracking.set_speed_motor (stepperH, SIDERAL_H_HZ);
  set_focus.set_speed_motor (stepperF, focus_rate);

  startup_message_ln((const char*)F("TRACKING NOW"), GREEN);


#ifdef TRACK_AFTER_STARTUP
  Serial.println("H Tracking enabled.");
  stepperH.run(H_FWD, SIDERAL_H_HZ);
  //while (stepperH.busyCheck());
  delay(1000); p1=axisH.Get();
  Serial.println("Time in ms: " + String(millis()) + ", H#: "+String(p1) + ", Rate: " + String(stepperH.getCurrentSpeed(),8));
  delay(1000); p2=axisH.Get();
  Serial.println("Time in ms: " + String(millis()) + ", H#: "+String(p2) + ", Rate: " + String(stepperH.getCurrentSpeed(),8));
  Serial.println(" --> dH#: " + String(p2-p1) + "/s Sideral Rate: " + String((float)(p2-p1)*SIDERAL_RATIO,8));
#endif

 startup_message_ln((const char*)F("setup TMR3"), GREEN);

#ifdef USE_TIMER
  Timer3.attachInterrupt(controller_isr);
  Timer3.start(1000000L); // Calls every 1000ms
#endif

#ifdef VERBOSE_STARTUP
  //comm_hw_blue->send (F("Setup done. Sideral Tracking is ON. (send via communication server, enter ':?#' for help)\n"));
  comm_master->send ("LX200-10u K150 listening#\n");
#else
  comm_master->send ("#\n");
#endif
 startup_message_ln((const char*)F("setup complete"), GREEN);
}

#ifdef USE_TIMER
void controller_isr(){
  if (auto_track_HAbyRA){
      track_error = 1;
  }
  if (!update_display)
    update_display = 2;
  if (!clear_errors)
    clear_errors = 5;
} 
#endif

void loop()
{
  long hapos;
  static int DKey[4][4];
  if (key_pad_enable) {
    if (KeyChg(DKey)) {
      KeyPadComand(DKey);
    }
  }

  axisH.Update ();
  axisD.Update ();

  if (!--update_display)
    comm_master->update_coords_display ();

  if (!--clear_errors)
    Clear_I2C_Error ();

  if (track_error){
    Astro.calc_JD_Time_GrHrAngle (millis());
    Astro.Use_memRADecl ();
    hapos=(long)(Astro.calc_HrAngle ()*RAD_NS_US);
    track_error = axisH.Get() - hapos;
    // Serial.println(" *TKErr="+String(track_error) + "steps, " + String((float)track_error*NS_US_DEG,8) + "deg TRH=" +String(tracking_rate[MOTOR_H],8) + " SPD=" + String(stepperH.getCurrentSpeed(),8));
    Serial.println(String(millis()) + ", " + String((float)track_error*NS_US_DEG*3600,8)+", "+String(tracking_rate[MOTOR_H],8)+", "+String(hapos)+", "+String(axisH.Get()) );

    if (abs(track_error) > 50.){
      axisH.Goto(hapos);
      //tracking_rate[MOTOR_H] = SIDERAL_H_HZ;
    } else {
      tracking_rate[MOTOR_H] += (float)track_error*2e-6;
  
      if (tracking_rate[MOTOR_H] < SIDERAL_H_HZ/10.)
        tracking_rate[MOTOR_H] = SIDERAL_H_HZ/10.;
  
      if (tracking_rate[MOTOR_H] > 10*SIDERAL_H_HZ)
        tracking_rate[MOTOR_H] = 10*SIDERAL_H_HZ;
  
      //stepperH.setMaxSpeed(tracking_rate[MOTOR_H]);
      //set_tracking.config_motor (stepperH, slew_rate[MOTOR_H]);
      stepperH.run(tracking_dir[MOTOR_H], tracking_rate[MOTOR_H]);
    }
    track_error=0;
  }

  comm_master->CheckAndProcess();
  if (comm_usb) comm_usb->CheckAndProcess();
  if (comm_blue) comm_blue->CheckAndProcess();
  if (comm_0) comm_0->CheckAndProcess();
  if (comm_1) comm_1->CheckAndProcess();

  // Hmi.softSerial(BlueSerial);
}




















#if 0

// LX200 protcol
// http://www.meade.com/support/TelescopeProtocol_2010-10.pdf
// TEST COMANDS LX200 [10-micron]
#:U2##:GVP#
#:GVZ#
#:GVN#
#:GVD#
#:GVT#

:GR#:GT#
#:GRTMP#
#:GRPRS#

#:modelcnt#
#:getalst#

-- Get Latitude, Longitude (East Longitudes are expressed as negative), zone, JDFac*24h, date

:Gt#
:Gg#
:GG#
:GL#
:GC#

#:Ginfo#
#:GS#

-- set time date location --

:Sg-072:55#
:St+40:57:18#
:SG +05#
:SL 22:41:56#
:SC2019-03-06#

:Gg#
:Gt#
:GG#
:GL#
:GC#


#:Ginfo#
#:GS#

--SYNC TEST--

:Sr 18:37:34#
:Sd +30:48:04#

:Sr 18:00:00#
:Sd +30:00:00#
:Gr#
:Gd#
:CM#
:Ginfo#
:GR#
:GD#
:GN#

:Sr 18:00:00#
:Sd +38:00:00#
:Gr#
:Gd#
:CM#
:Ginfo#
:GR#
:GD#
:GN#
:TH#

--SLEW TEST--

:QV# :Qv# :QF# :Qf:

:QV#
:Sr 17:50:00#
:Sd +30:10:00#
:Gr#
:Gd#
:Ginfo#
:MS#:TH#
:Ginfo#
:Ginfo#
:Ginfo#
:Ginfo#

:G*#
:GK#
#:Ginfo#
#:GS#

:Sr 2:52:01#
:Sd 17:06:33#


:Sr 14:16:33#
:Sd +19:05:24#
:G*#
:Gr#
:Gd#
:CM#
:TH#

#:Ginfo#
#:GS#

-- Tracking -- +/- adjust by 0.1Hz, Select H Sideral rate, <> Select forward/reverse, TCR report, GT Get Tracking rate in 60Hz equiv
:T+#
:T-#
:TH#
:T>[HD]#
:T<[HD]#
:TCR#
:GT#
:TR#

--SLEW--

:Sr 14:16:33#
:Sd +19:05:24#
:MS#
:Sr 14:16:33#
:Sd +19:05:24#
:Qf:
:QQ#
:MS#
:Ginfo#
:GS#

-- auto control RA/HA, DE --

:MT0#

#:Ginfo#:MS#:Ginfo##:Ginfo##:Ginfo##:Ginfo#


:Qf#:MS#:TH#:MT1#:TR#

--GET telescope RA, Decl,  mem RA, mem Decl, Azimuth--
:GR#
:GD#
:Gr#
:Gd#

:GZ#

-- GET Time Zone, Local Time, Date
:GG#
:GL#
:GC#

-- GET distance to go, site longiture, latituide, sideral time (HRAngle) --
:Gm#
:Gg# 
:Gt#
:GS#

-- FOCUS --
:F0#
:F+#
:F-#
:F 128#
:FP#
:F0#


#endif
