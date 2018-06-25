/* Changes made by Pete Cranwell WA2EIN 
 Ported Keyer code by Steven Elliot, 2009
 Modified logic to work with Patriot Hardware.
 Changes default startup frequencies to 40CW (Qrp watering hole) and 20 Meters PSK.
 Included code from WD9GYM.
 Added Tune function.  Function switch > .5 sec.
 Added proportional Freq Tune function.
 If Function.Step selected, use original tuning steps.
 else use tuning steps based on encoder rate change.
 
 */

/* Changes made by W2ROW to allow for receive frequency CW offset by band
 40 meters requires a positive offset (currently set to 500 hertz)
 20 meters requires a negative offset (currently set to -500 hertz)
 */


/////// Code Compile Defines ////////////

#define WA2EIN_Keyer
//#define PADDLE_REVERSED     If your paddles are reversed



/*  Code for Production 3_2_15
 <Patriot_507_Alpha_Rev01, Basic Software to operate a 2 band SSB/CW QRP Transceiver.
 
 
 Prog for ad9834
 Serial timming setup for AD9834 DDS
 start > Fsync is high (1), Sclk taken high (1), Data is stable (0, or 1),
 Fsync is taken low (0), Sclk is taken low (0), then high (1), data changes
 Sclk starts again.
 Control Register D15, D14 = 00, D13(B28) = 1, D12(HLB) = X,
 Reset goes high to set the internal reg to 0 and sets the output to midscale.
 Reset is then taken low to enable output. 
 ***************************************************   
 This is real basic code to get things working. 
 *****************************************************************
 The pinout for the LCD is as follows: Also the LCD is setup up for 4 lines 20 charactors.
 * LCD RS pin to digital pin 26
 * LCD Enable pin to digital pin 27
 * LCD D4 pin to digital pin 28
 * LCD D5 pin to digital pin 29
 * LCD D6 pin to digital pin 30
 * LCD D7 pin to digital pin 31
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)    analogWrite(Side_Tone, 127);
 *****************************************************************
 SELECT button steps from in 
 BW ( <Wide, green>, <Medium, yellow>, <Narrow, red> ).
 STEP ( <100 hz, green, <1Khz, yellow>, 10Khz, red> ).
 BND ( < 40M >, < 20M >, < , > ) OTHER has yet to be defined
 
 Default Band_width will be wide ( Green led lite ).
 When pressing the function button one of three leds will lite. 
 as explained above the select button will choose which setting will be used. 
 The Orange led in the Ten-Tec logo will flash to each step the STEP is set 
 too when tuning.  As it will also turn on when at the BAND edges.
 The TT logo led will also flash to indicate ALC. Input levels should be kept low enough
 to only flash this led on Peaks.  
 Default frequency on power up will be the calling frequency of the 
 40 meter band. 
 I.F. Frequency used is 9.0 mhz.
 DDS Range is: 
 40 meters will use HI side injection.
 9(I.F.) + 7(40m) = 16mhz.  9(I.F.) + 7.30 = 16.3 mhz.
 20 meters will use LO side injection.
 14(20m) - 9(I.F.) = 5mhz.  14.350(20m) - 9(I.F.) = 5.35 mhz.
 
 The Headphone jack can supply a headphone or speaker. The header pins(2) 
 if shorted will drive a speaker.
 Unshorted inserts 100 ohm resistors in series with the headphone to limit 
 the level to the headphones.
 
 The RIT knob will be at 0 offset in the Top Dead Center position. And will 
 go about -500 hz to +500 hz when turned to either extreme. Total range 
 about +/- 500 hz. This may change!
 
 **************************************************************************************  
 
 Added an MCP23017 16-bit I/O Expander with Serial Interface to free up
 some I/O pins on the ChipKit Uno32 board.
 The parts of the 507 being controlled by this ic will be the Multi-purpose
 leds, the Select leds and the Wide/medium/Narrow control.
 5/1/2014 added a couple of routines to keep the filter wide on TX of SSB or CW
 Port B can be used by the user for external control.
 
 GPAO (21) Select Green led
 GPA1 (22) Select Yellow led
 GPA2 (23) Select Red led
 GPA3 (24) MP_A Green led
 GPA4 (25) MP_B Yellow led
 GPA5 (26) MP_C Red led
 GPA6 (27) Medium A8 BW_control
 GPA7 (28) Narrow A9 BW_control
 
 */

// Required for External EEPROM
#include <IOShieldEEPROM.h>

// Required for Timer based Checkpoint
#include <RTCC.h>



#ifdef WA2EIN_Keyer

unsigned long       ditTime;                    // No. milliseconds per dit
unsigned long       OldWPM;
unsigned char       keyerControl;
unsigned char       keyerState;


///////////////////////////////////////////////////////////////////////////////
//
//  State Machine Defines

enum KSTYPE
{
  IDLE, CHK_DIT, CHK_DAH, KEYED_PREP, KEYED, INTER_ELEMENT
};

#endif


// various defines
#define Keyer_Mode                          1
#define Straight_Key                        1
#define Keyer                               0
#define SDATA_BIT                           11          //
#define SCLK_BIT                            12          //
#define FSYNC_BIT                           13          //
#define RESET_BIT                           10          //
#define FREQ_REGISTER_BIT                   9           //
#define PHASE_REGISTER_BIT                  8
#define AD9834_FREQ0_REGISTER_SELECT_BIT    0x4000      //
#define AD9834_FREQ1_REGISTER_SELECT_BIT    0x8000      //
#define FREQ0_INIT_VALUE                    0x00000000  // 0x01320000

#define led                                 13
#define MUTE                                4
#define MIC_LINE_MUTE                       34

#define Side_Tone                           3           //

#define PTT_SSB                             22          // ptt input pulled high
#define SSB_CW                              42          // control line for /SSB_CW switches output
// high for cw , low for ssb

#ifdef PADDLE_REVERSED
#define TX_Dah                           33          //
#define TX_Dit                           32          //
#else
#define TX_Dah                           32
#define TX_Dit                           33
#endif

#define TX_OUT                              38          //
#define Band_End_Flash_led                  24          // // also this led will flash every 100/1khz/10khz is tuned
#define Band_Select                         41          // output for band select
#define Multi_Function_Button               5           //
#define Flash                               Band_End_Flash_led

#define Select_Button                       2           //

#define Wide_BW                             0           //
#define Medium_BW                           1           //
#define Narrow_BW                           2           //


#define Step_100_Hz                         0
#define Step_1000_hz                        1
#define Step_10000_hz                       2

#define  Other_1_user                       0           // 40 meters
#define  Other_2_user                       1           // 20 meters
#define  Other_3_user                       2           // anything you desire!

// WA2EIN
#define SoftwareSerialOutPin                37          // J5(8)
#define SoftwareSerialInPin                 24          // A9  J7(8) 

#ifdef WA2EIN_Keyer

#define     DIT_L      0x01     // Dit latch
#define     DAH_L      0x02     // Dah latch
#define     DIT_PROC   0x04     // Dit is being processed
#define     IAMBICB    0x10     // 0 for Iambic A, 1 for Iambic B

#endif

//#define NOLIM

/*
struct  MemTune 
{
  long Freq;
  long offset;
  int  mode;
  int  bsm;   
};



MemTune  Memory[10];

*/

// EEPROM Values


struct EEProm
{

  int     bsm;
  int     mode;
  long    recv_offset;
  int     Step_Select_Button1;
  int     Step_Multi_Function_Button1;
  int     frequency;
  int     Data_Valid;
};




EEProm  CheckpointData;
EEProm  LastCheckpointData;

EEProm  MemoryTune[50];

int DoCheckpoint = 0;


const int RitReadPin        = A0;  // pin that the sensor is attached to used for a rit routine later.
int RitReadValue            = 0;
int RitFreqOffset           = 0;
int old_RitFreqOffset       = 0;

const int SmeterReadPin     = A1;  // To give a realitive signal strength based on AGC voltage.
int SmeterReadValue         = 0;

const int BatteryReadPin    = A2;  // Reads 1/5 th or 0.20 of supply voltage.
int BatteryReadValue        = 0;
float BatteryVconvert       = 0.01707;    // calibration value


const int PowerOutReadPin   = A3;  // Reads RF out voltage at Antenna.
int PowerOutReadValue       = 0;

const int CodeReadPin       = A6;  // Can be used to decode CW.
int CodeReadValue           = 0;

const int CWSpeedReadPin    = A7;  // To adjust CW speed for user written keyer.
int CWSpeedReadValue        = 0;


const long offset_20m = -500;         //W2ROW CW offset for 20 meters (negative)
const long offset_40m = 500;          //W2ROW CW offset for 40 meters (positive)
long recv_offset = 0;                 //W2ROW Computed receiver offset based on mode and band


#include "Wire.h"


const char txt52[5] = " ";
const char txt57[6] = "FREQ:";
const char txt60[6] = "STEP:";
const char txt62[3] = "RX";
const char txt63[3] = "TX";
const char txt64[4] = "RIT";
const char txt65[5] = "Band";
const char txt66[4] = "20M";
const char txt67[4] = "40M";
const char txt69[4] = "   ";
const char txt70[5] = "    ";
const char txt71[6] = "     ";
const char txt72[10] = "        ";
const char txt85[2] = "W";
const char txt86[2] = "M";
const char txt87[2] = "N";
const char txt90[5] = "STEP";
const char txt110[4] = "BAT";
const char txt120[3] = "BW";
const char txt130[5] = "MODE";
const char txt132[3] = "CW";
const char txt135[4] = "SSB";
const char txt140[5] = "WIDE";
const char txt150[7] = "MEDIUM";
const char txt160[7] = "NARROW";
const char txt170[7] = "      ";

const char txt0[7]          = "  2.1";
const char txt2[8]          = "WA2EIN";

// bsm=0 is 40 meter, bsm=1 is 20 meter (used in CAT routine)
int Band_bsm0_Low              = 7000000;
int Band_bsm0_High             = 7300000;
int Band_bsm1_Low              = 14000000;    
int Band_bsm1_High             = 14350000;




String stringFREQ;
char   FreqTxt[10] = "";
String stringREF;
String string_Frequency_Step;
String stringRIT;
String stringVolts;
String stringBW;

// EEPROM testing
char buf[27];

int TX_key;
int PTT_SSB_Key;
int old_PTT_SSB_Key;
int Key_Mode = Straight_Key;
int Keyed = false;
int CWSpeedWPM;                // Speed controlled by A7 


int band_sel;                           // select band 40 or 20 meter
int band_set;
int mode                        = 1;            // mode 1 = cw ,  mode 0 = ssb
int bsm                         = 0;            // bsm = 0 is 40 meters ,  bsm = 1 is 20 meters


int Step_Select_Button          = 0;
int Step_Select_Button1         = 0;
int Step_Multi_Function_Button  = 0;
int Step_Multi_Function_Button1 = 0;

int Selected_BW                 = 0;    // current Band width
// 0= wide, 1 = medium, 2= narrow
int Selected_Step               = 0;    // Current Step
int Selected_Other              = 0;    // To be used for anything

int old_bsm                     = 0;    //  this helps 5/13/14
int old_mode;

int old_BatteryReadValue        = 0;

byte s = 0x00;                    // s = select
byte m = 0x00;                    // m = multi
byte b = 0x00;                    // b = bandwidth
byte t = 0x00;                    // s + m ored
byte old_b = 0x00;                // for the TX routine

// CAT  //

//static long      idletimer;               // will inhibit all polling to keep keying clean

// some text variables for the display and terminal functions
#define bw 3
String bwtext[bw] = { 
  "W", "M", "N"};
#define stp 3
String steptext[stp] = {
  "10 ", "100", "1K "};
#define mod 4
String modetext[mod] = {
  "LSB", "USB", "CW ", "CW "};

// define terminal / cat active at start
int terminal = 1;                              // terminal active at start

unsigned long  catStartTime    = 0;
unsigned long  catElapsedTime  = 0;
// stage buffer to avoid blocking on serial writes when using CAT
#define STQUESIZE 64
unsigned char stg_buf[STQUESIZE];
int stg_in = 0;
int stg_out = 0;




//-----------------------------------------------------
// Encoder Stuff
const int encoder0PinA          = 6; // reversed for 507
const int encoder0PinB          = 7; // reversed for 507

int val;
int encoder0Pos                 = 0;
int encoder0PinALast            = LOW;
int n                           = LOW;

//-----------------------------------------------------


//-----------------------------------------------------


// WA2EIN  Changed Default Startup freq to 40 Meter CW QRP
// HI side injection 40 meter
const long meter_40             =  16.030e6;    // CW QRP Frequency
// range 16 > 16.3 mhz

// WA2EIN Changed Default Startup freq to 20 Meter PSK
// side injection 20 meter
// range 5 > 5.35 mhz
const   long meter_20           =   5.070e6;


//const long Reference            = 50.0e6;   // for ad9834 this may be
const long Reference            =50000350;
// tweaked in software to
long frequency_TX;                                                // fine tune the Radio
long TX_frequency;
long RIT_frequency;
long RX_frequency;
long save_rec_frequency;
long Frequency_Step;
long old_Frequency_Step;
long frequency                  = 0;
long frequency_old              = 0;
long frequency_old_TX           = 0;
long frequency_tune             = 0;
long old_frequency_tune         = 0;
long frequency_default          = 0;
long fcalc;
long IF                         = 9.00e6;          //  I.F. Frequency
long TX_Frequency               = 0;
long ktimer;


// WA2EIN  OLED Code
char Clear[2] = { 
  12};
char DefaultFont[2] = { 
  3};
char NewFont[2] = { 
  2};
char LargeFont[2] = {
  20};
long EventTime;
long LastEventTime;
long DisplayCtr = 0;
int DisplayDone = false;
char LastMsg[20];


// WA2EIN  Progressive Tune Step
//
// Set StepLowMark and StepHighMark to values that give desired tuning rate switch point.
//  This will vary depending on your operating taste.
//
long LastEncoderTime;
long EncoderTimeDiff;
int  StepLowMark   = 200;
int  StepMedMark   = 550;       // Set for Lower tuning rate break point
//int  StepHighMark  = 600;       // Set for Upper tuning rate break point



//-----------------------------------------------------
// Debug Stuff

unsigned long   loopCount       = 0;
unsigned long   lastLoopCount   = 0;
unsigned long   loopsPerSecond  = 0;
unsigned int    printCount      = 0;

unsigned long   loopStartTime   = 0;
unsigned long   loopElapsedTime = 0;
float           loopSpeed       = 0;

unsigned long LastFreqWriteTime = 0;

void    serialDump();

//-----------------------------------------------------


////////// function prototypes  ////////////
void Default_frequency();
void AD9834_init();
void AD9834_reset();
void program_freq0(long freq);
void program_freq1(long freq1);
void UpdateFreq(long freq);

void RX_Rit();


void TX_routine();
void RX_routine();
void Encoder();
void AD9834_reset_low();
void AD9834_reset_high();

void Change_Band();
void Step_Flash();
void RIT_Read();

void Multi_Function();          //
void Step_Selection();          //
void Selection();               //
void Step_Multi_Function();     //

void splash_RX_freq();
void splash_RIT_Read();
void splash_RIT();
void Splash_MODE();
void Splash_Band();
void splash_TX_freq();
void Select_Multi_BW_Ored();
void Band_20_Limit();
void Band_40_Limit();
void Band_Set_40_20M();
void stop_led_on();
void stop_led_off();
void Step_Function();
void Step_Select();
void Splash_Step_Size();
void Splash_BW();
void Step_Delect();
void update_PaddleLatch();
int ReadCWSpeed(void);
void LoadWPM(int);
void Display_Freq(void);
void Display_RIT(void);
void CHK(void);
void Display_Msg(char *Msg);






//-----------------------------------------------------

void clock_data_to_ad9834(unsigned int data_word);

//-----------------------------------------------------

void setup()
{

  byte *ptr;
  EEProm *ptr1;




  // these pins are for the AD9834 control
  pinMode(SCLK_BIT,                 OUTPUT);   // clock
  pinMode(FSYNC_BIT,                OUTPUT);   // fsync
  pinMode(SDATA_BIT,                OUTPUT);   // data
  pinMode(RESET_BIT,                OUTPUT);   // reset
  pinMode(FREQ_REGISTER_BIT,        OUTPUT);   // freq register select

  //---------------  Encoder ----------------------------
  pinMode(encoder0PinA,             INPUT);    //
  pinMode(encoder0PinB,             INPUT);    //

  //-----------------------------------------------------
  pinMode(TX_Dit,                   INPUT);    // Dit Key line
  pinMode(TX_Dah,                   INPUT);    // Dah Key line
  //-----------------------------------------------------

  pinMode(CWSpeedReadPin,                     INPUT);    // Keyer Speed

#ifdef WA2EIN_Keyer


  keyerState = IDLE;
  keyerControl = IAMBICB;     // Or 0 for IAMBICA


#endif


  pinMode(TX_OUT,                   OUTPUT);   // control line for TX stuff
  pinMode(Band_End_Flash_led,       OUTPUT);   // line that flashes an led
  pinMode(PTT_SSB,                  INPUT);    // mic key has pull-up
  pinMode(SSB_CW,                   OUTPUT);   // control line for ssb cw switches

  pinMode(Multi_Function_Button,    INPUT);    // Choose from Band width, Step size, Other

  pinMode(Select_Button,            INPUT);    //  Selection from the above

  pinMode(Side_Tone,                OUTPUT);   // sidetone enable

  pinMode(Band_Select,              OUTPUT);

  pinMode(MUTE,                     OUTPUT);

  pinMode(MIC_LINE_MUTE,            OUTPUT);  // low on receive

  digitalWrite(Band_End_Flash_led,  LOW); //  not in 81324

  digitalWrite(MUTE,                LOW);


  BatteryReadValue = analogRead(BatteryReadPin);

  Default_Settings();
  // I2C stuff
  Wire.begin();                            // wake up I2C bus
  Wire.beginTransmission(0x20);
  Wire.send(0x00);                         // IODIRA register
  Wire.send(0x00);                         // set all of port A to outputs
  Wire.endTransmission();
  Wire.beginTransmission(0x20);
  Wire.send(0x01);                         // IODIRB register
  Wire.send(0x00);                         // set all of port B to outputs
  Wire.endTransmission();

  //-----------------------------------------------------
  // DDS
  AD9834_init();
  AD9834_reset();                           // low to high
  //-----------------------------------------------------

  digitalWrite(TX_OUT,              LOW);     // turn off TX
  digitalWrite(SSB_CW,              LOW);     // keeps tx in ssb mode until high

  //-----------------------------------------------------
  Frequency_Step = 100;  //  Can change this whatever step size one wants
  Selected_Step = Step_100_Hz;
  DDS_Setup();
  encoder0PinALast = digitalRead(encoder0PinA);
  //attachInterrupt(encoder0PinA, Encoder, CHANGE);
  //attachInterrupt(encoder0PinB, Encoder, CHANGE);
  attachCoreTimerService(TimerOverFlow); //See function at the bottom of the file.

  // serial baudrate for K3 emulation at highist speed.
  Serial.begin(38400);

  Serial1.begin(9600);


  Serial.println("Patriot Ready:");
  Serial1.println("Patriot Ready");



  // If Dah pressed or Key Jack Shorted, set Straight Key Mode.
#ifdef PADDLE_REVERSED
  if ( digitalRead(TX_Dah) == LOW )
#else
    if ( digitalRead(TX_Dit) == LOW )
#endif
    {
      Key_Mode = Straight_Key;
    }
    else
    {
      Key_Mode = Keyer;
    }

  mode = 1;         // WA2EIN   Set CW Offset



  // Initialize the RTCC module
  RTCC.begin();

  // Attach our routine to send the time through the serial port
  RTCC.attachInterrupt(&CHK);



  // Set the alarm to trigger every second
  RTCC.alarmMask(AL_SECOND);
  RTCC.chimeEnable();
  RTCC.alarmEnable();



  Restore();

}   //    end of setup


//===================================================================


void Default_Settings()
{
  m = 0x08;                //

  s = 0x01;                //

  bsm = 0;                 //  bsm = 0 is 40 meters bsm = 1 is 20 meters

  frequency_default = meter_40; // change this to meter_20 for 20 meter default
  recv_offset = offset_40m;    // Set CW offset
  mode = 1;                    // CW default 
  Default_frequency();

  b = 0x00; // Hardware control of I.F. filter shape wide setting
  Selected_BW = Wide_BW;

  digitalWrite(TX_OUT,               LOW);
  digitalWrite(Band_End_Flash_led,   LOW);
  digitalWrite(Side_Tone,            LOW);
  digitalWrite(FREQ_REGISTER_BIT,    LOW);
  digitalWrite(SSB_CW,               LOW);    // Keep in SSB Mode
  digitalWrite(Band_Select,          LOW);
  digitalWrite(MUTE,                 HIGH);
  digitalWrite(MIC_LINE_MUTE,        LOW);   //  receive mode

}



//-----------------------------------------------------------------
void DDS_Setup()
{
  digitalWrite(FSYNC_BIT,             HIGH); //
  digitalWrite(SCLK_BIT,              HIGH); //
}

//======================= Main Part =================================
void loop()
{
  long now;



  if ( Keyed == false )
  {
    if ( !un_stage() ) Poll_Cat();
    Encoder();
    RX_Rit();
    Multi_Function();

    if ( millis() >= DisplayCtr + 1000 )
    {
      if ( DisplayDone == false )
      {
        Display_Freq();
        DisplayDone = true;
      }

    }

  }


  TX_routine();


  if ( DoCheckpoint == 1 )
  {
    Checkpoint();
  }


}
//-----------------------------------------------------
void  RX_Rit()
{
  RIT_Read();
  frequency_tune  = frequency + RitFreqOffset;   // RitFreqOffset is from Rit_Read();
  UpdateFreq(frequency_tune);
  splash_RX_freq();     // this only needs to be updated when encoder changed.
}


void RIT_Read()
{
  static int RIT_Old_Value;
  int RitReadValueNew = 0;
  RitReadValueNew = analogRead(RitReadPin);

  RitReadValue = (RitReadValueNew + (12 * RitReadValue)) / 13;   //Lowpass filter possible display role if changed
  if ( RitReadValue < 500 ) RitFreqOffset = RitReadValue - 500;
  else if ( RitReadValue < 523 ) RitFreqOffset = 0;   //Deadband in middle of pot
  else RitFreqOffset = RitReadValue - 523;

  if ( RIT_Old_Value != RitFreqOffset )
  {
    Display_RIT;
    RIT_Old_Value = RitFreqOffset;
  }



  //splash_RIT();    //   comment out if display is not needed
}


void UpdateFreq(long freq)
{


  if ( LastFreqWriteTime != 0 )
  {
    if ( (millis() - LastFreqWriteTime) < 100 ) return;
  }
  LastFreqWriteTime = millis();
  if ( (freq == frequency_old) && (mode == old_mode) ) return;              //W2ROW
  program_freq0(freq + recv_offset);                //W2ROW

  frequency_old = freq;
  old_mode = mode;                           //W2ROW
  //Checkpoint();
}





//--------------------------------------------------------------
void UpdateFreq1(long frequency_TX)
{
  if ( LastFreqWriteTime != 0 )
  {
    if ( (millis() - LastFreqWriteTime) < 100 ) return;
  }
  LastFreqWriteTime = millis();
  if ( frequency_TX == frequency_old_TX ) return;
  program_freq1(frequency_TX);
  frequency_old_TX = frequency_TX;
}
//------------------------------------------------------------------
//##################################################################
//---------------------  TX Routine  -------------------------------
void TX_routine() 
{
  //------------------  SSB Portion  ----------------------------


  PTT_SSB_Key = digitalRead(PTT_SSB);              // check to see if PTT is pressed,
  if ( PTT_SSB_Key == LOW )                        // if pressed do the following
  {
    do
    {
      TurnXmtrOn();

      PTT_SSB_Key = digitalRead(PTT_SSB);        // check to see if PTT is pressed
    }
    while ( PTT_SSB_Key == LOW );
    delay(10);   // debounce
    mode = 0;
    recv_offset = 0;
    ForceFreqDisplay();
    TurnXmtrOff();

  }   // End of SSB TX routine
  else
  {


    //---------------  CW Portion  --------------------------------
    // Separate Keyer from Straight Key mode

#ifdef WA2EIN_Keyer
      if ( Key_Mode == Keyer )
    {

      LoadWPM(ReadCWSpeed());
      switch ( keyerState )
      {

      case IDLE:
        // Wait for direct or latched paddle press
        if ( (digitalRead(TX_Dit) == LOW) ||
          (digitalRead(TX_Dah) == LOW) ||
          (keyerControl & 0x03) )
        {
          update_PaddleLatch();
          keyerState = CHK_DIT;
        }
        break;

      case CHK_DIT:
        // See if the dit paddle was pressed
        if ( keyerControl & DIT_L )
        {
          keyerControl |= DIT_PROC;
          ktimer = ditTime;
          keyerState = KEYED_PREP;
        }
        else
        {
          keyerState = CHK_DAH;
        }
        break;

      case CHK_DAH:
        // See if dah paddle was pressed
        if ( keyerControl & DAH_L )
        {
          ktimer = ditTime * 3;
          keyerState = KEYED_PREP;
        }
        else
        {
          keyerState = IDLE;
        }
        break;

      case KEYED_PREP:
        // Assert key down, start timing, state shared for dit or dah
        TurnXmtrOn();
        ktimer += millis();    // set ktimer to interval end time
        keyerControl &= ~(DIT_L + DAH_L);   // clear both paddle latch bits
        keyerState = KEYED;    // next state
        break;

      case KEYED:
        TurnXmtrOn();
        // Wait for timer to expire
        if ( millis() > ktimer )    // are we at end of key down ?
        {
          TurnXmtrOff();
          ktimer = millis() + ditTime; // inter-element time
          keyerState = INTER_ELEMENT;  // next state
        }
        else if ( keyerControl & IAMBICB )
        {
          update_PaddleLatch();     // early paddle latch in Iambic B mode
        }
        break;

      case INTER_ELEMENT:
        // Insert time between dits/dahs
        update_PaddleLatch();  // latch paddle state
        if ( millis() > ktimer )    // are we at end of inter-space ?
        {

          if ( keyerControl & DIT_PROC )     // was it a dit or dah ?
          {
            keyerControl &= ~(DIT_L + DIT_PROC);   // clear two bits
            keyerState = CHK_DAH;   // dit done, check for dah
          }
          else
          {
            keyerControl &= ~(DAH_L);        // clear dah latch
            keyerState = IDLE;      // go idle
          }
        }
        break;
      }
    }
    else
    {
      // Straight Key Logic, within WA2EIN Keyer Logic
#ifdef PADDLE_REVERSED
      TX_key = digitalRead(TX_Dit);
#else
      TX_key = digitalRead(TX_Dah);
#endif

      if ( TX_key == LOW )
      {

        do
        {
          TurnXmtrOn();
#ifdef PADDLE_REVERSED
          TX_key = digitalRead(TX_Dit);
#else
          TX_key = digitalRead(TX_Dah);
#endif

        }
        while ( TX_key == LOW );

        TurnXmtrOff();
      }
    }
#else
    // Straight Key Logic outside WA2EIN Keyer Logic
#ifdef PADDLE_REVERSED
    TX_key = digitalRead(TX_Dit);
#else
    TX_key = digitalRead(TX_Dah);
#endif
    if ( TX_key == LOW )
    {


      do
      {
        TurnXmtrOn();
#ifdef PADDLE_REVERSED
        TX_key = digitalRead(TX_Dit);
#else
        TX_key = digitalRead(TX_Dah);
#endif

      }
      while ( TX_key == LOW );
      TurnXmtrOff();
    }
#endif
  }
}
// end  TX_routine()

//--------------------------------------------------------------------
void TurnXmtrOn()
{

  // Key Transmitter
  // Note this must be executed in a loop while transmitter is keyed.


  old_b = b;
  b = 0x00;                                 // b is now set to wide filter setting
  Select_Multi_BW_Ored();                   // b is sent to port expander ic



  Keyed = true;

  // Compensate for this particular Patriot.
  if ( bsm == 0 )
  {
    // Adjust 40 Meter Xmt 
    TX_Frequency = frequency ;
  }
  else
  {
    // Adjust 20 Meter Freq 
    TX_Frequency = frequency ;
  }

  frequency_tune  = TX_Frequency;              // RitFreqOffset is from Rit_Read();
  digitalWrite(FREQ_REGISTER_BIT,   HIGH);     //
  UpdateFreq1(frequency_tune);

  PTT_SSB_Key = digitalRead( PTT_SSB );  
  if ( PTT_SSB_Key == LOW )
  {
    DoCheckpoint = 1;
  }
  else
  {
    //W2ROW mode is CW
    mode = 1;
    if ( bsm == 0 )
    {                                            //W2ROW 40 m
      recv_offset = offset_40m;         //W2ROW
    }
    else
    {                                           //W2ROW 20 m
      recv_offset = offset_20m;        //W2ROW
    }
    ForceFreqDisplay();
  }






  //splash_TX_freq(); 


  if ( PTT_SSB_Key == LOW )
  {
    digitalWrite(SSB_CW, HIGH);            // this causes the ALC line to connect
    digitalWrite(MIC_LINE_MUTE, HIGH);      // turns Q35, Q16 off, unmutes mic/line

  }
  else
  {
    digitalWrite(SSB_CW, LOW);                   // Set CW Mode
    digitalWrite(Side_Tone, HIGH);
  }
  digitalWrite(TX_OUT, HIGH);


}

void TurnXmtrOff()
{
  // Turn Off Xmtr for SSB and CW Modes
  Keyed = false;
  b = old_b;                                  // original b is now restored
  Select_Multi_BW_Ored();
  digitalWrite(TX_OUT, LOW);                  // trun off TX cw key is now high
  digitalWrite(FREQ_REGISTER_BIT,   LOW);     // return to DDS register 0  not in other
  digitalWrite(Side_Tone, LOW);               // side-tone off
  digitalWrite(MIC_LINE_MUTE, LOW);           // turns Q36, Q16 on, mutes mic/line
}


int ReadCWSpeed()
{

  // CW Speed control varies pin voltage from 0 - 3.3 Volts.

  int CWSpeed;
  CWSpeed = analogRead(CWSpeedReadPin);           //read Analog Value attached to KeyerSpeedPot

  return((CWSpeed / 30) + 5);                     // Scale pot value from 5 - 40 WPM.
}




//------------------------------------------------------------------------

#ifdef WA2EIN_Keyer
void update_PaddleLatch()
{
  if ( digitalRead(TX_Dah) == LOW )
  {
    keyerControl |= DIT_L;
  }
  if ( digitalRead(TX_Dit) == LOW )
  {
    keyerControl |= DAH_L;
  }
}

///////////////////////////////////////////////////////////////////////////////
//
//    Calculate new time constants based on wpm value
//
///////////////////////////////////////////////////////////////////////////////

void LoadWPM(int wpm)
{
  CWSpeedWPM = wpm;
  if ( wpm != OldWPM )
  {
    OldWPM = wpm;
    Serial1.print(Clear);
    Serial1.print("WPM : ");
    Serial1.print(wpm);
    DisplayCtr = millis();
    DisplayDone = false;
  }
  ditTime = 1200 / wpm;

}

#endif

//--------------------------- Encoder Routine ---------------------

void Encoder()
{

  n = digitalRead(encoder0PinA);
  if ( encoder0PinALast != n )
  {

    SetStepSize();

    if ( (encoder0PinALast == LOW) && (n == HIGH) )
    {
      if ( digitalRead(encoder0PinB) == LOW ) //  Frequency_down
      {
        //encoder0Pos--;
        frequency = frequency - Frequency_Step;

        Step_Flash();
        if ( bsm == 1 )
        {
          Band_20_Limit();
        }
        else if ( bsm == 0 )
        {
          Band_40_Limit();
        }
      }
      else                                 //  Frequency_up
      {
        //encoder0Pos++;
        frequency = frequency + Frequency_Step;
        Step_Flash();
        if ( bsm == 1 )
        {
          Band_20_Limit();
        }
        else if ( bsm == 0 )
        {
          Band_40_Limit();
        }
      }
    }
    encoder0PinALast = n;
    //Checkpoint();
  }
}

//-------------------------------------------------------
//-------------------------------------------------------
void Change_Band()
{
  if ( bsm == 1 )                           //  select 40 or 20 meters 1 for 20 0 for 40
  {
    digitalWrite(Band_Select, HIGH);
    Band_Set_40_20M();
  }
  else
  {
    digitalWrite(Band_Select, LOW);
    Band_Set_40_20M();
    IF *= -1;                             //  HI side injection
  }
}

//------------------ Band Select ------------------------------------
void Band_Set_40_20M()
{
  if ( old_bsm != bsm )                       //  this helps 5/13/14
  {
    if ( bsm == 1 )                         //  select 40 or 20 meters 1 for 20 0 for 40
    {
      frequency_default = meter_20;
      //Splash_Band();

      if ( mode == 0 )               //W2ROW SSB
        recv_offset = 0;           //W2ROW
      else                                  //W2ROW CW
      recv_offset = offset_20m;      //W2ROW
    }
    else
    {
      frequency_default = meter_40;
      // Splash_Band();

      IF *= -1;  
      //  HI side injection
      if ( mode == 0 )              //W2ROW SSB
        recv_offset = 0;          //W2ROW
      else                                 //W2ROW CW
      recv_offset = offset_40m;     //W2ROW

    }
    Default_frequency();
  }
  old_bsm = bsm;                             //  this helps 5/13/14
}

//--------------------Default Frequency-----------------------------
void Default_frequency()
{
  frequency = frequency_default;
  UpdateFreq(frequency);
  splash_RX_freq();
}         //  end   Default_frequency

//-----------------------------------------------------
//-----------------------------------------------------
void  Band_40_Limit()
{

#ifdef NOLIM 
  return ;
#endif

  if ( frequency >= 16.3e6 )
  {
    frequency = 16.3e6;
    stop_led_on();
  }
  else if ( frequency <= 16.0e6 )
  {
    frequency = 16.0e6;
    stop_led_on();
  }
  else
  {
    stop_led_off();
  }
}
//-----------------------------------------------------
void  Band_20_Limit()
{


#ifdef NOLIM 
  return  ;
#endif

  if ( frequency >= 5.35e6 )
  {
    frequency = 5.35e6;
    stop_led_on();
  }
  else if ( frequency <= 5.0e6 )
  {
    frequency = 5.0e6;
    stop_led_on();
  }
  else
  {
    stop_led_off();
  }
}

//-----------------------------------------------------
void Step_Flash()
{
  stop_led_on();
  for ( int i = 0; i <= 25e3; i++ ) ;   // short delay
  stop_led_off();
}

//-----------------------------------------------------
void stop_led_on()     //  band edge and flash
{
  digitalWrite(Band_End_Flash_led, HIGH);
}

//-----------------------------------------------------
void stop_led_off()
{
  digitalWrite(Band_End_Flash_led, LOW);
}

//===================================================================

void Multi_Function() //  pushbutton for BW, Step, Other
{

  unsigned long time;
  unsigned long start_time;
  unsigned long long_time;


  long_time = millis();
  time = millis();


  // look into a skip rtoutine for this
  Step_Multi_Function_Button = digitalRead(Multi_Function_Button);
  if ( Step_Multi_Function_Button == HIGH )
  {
    while ( digitalRead(Multi_Function_Button) == HIGH )
    {

      // function button is pressed longer then 0.5 seconds
      if ( (millis() - long_time) > 500 )
      {
        // tune
        old_mode=mode;
        mode=1;   //cw
        TurnXmtrOn();
        // wait for button release
        while ( digitalRead(Multi_Function_Button) == HIGH )
        {
        }   
        TurnXmtrOff();
        mode=old_mode;
        return;        
      }






    }  // added for testing
    for ( int i=0; i <= 150e3; i++ ); // short delay

    Step_Multi_Function_Button1 = Step_Multi_Function_Button1++;
    if ( Step_Multi_Function_Button1 > 2 )
    {
      Step_Multi_Function_Button1 = 0; 
    }
  }
  Step_Function();
}  // end Multi





void Step_Function()
{
  switch ( Step_Multi_Function_Button1 )
  {
  case 0:
    m = 0x08;   // GPA3(24) Controls Function Green led
    Select_Multi_BW_Ored();
    Step_Select_Button1 = Selected_BW; //
    Step_Select(); //
    Selection();
    for ( int i = 0; i <= 255; i++ ) ; // short delay
    break;   //

  case 1:
    m = 0x10;   // GPA4(25) Controls Function Yellow led
    Select_Multi_BW_Ored();
    Step_Select_Button1 = Selected_Step; //
    Step_Select(); //
    Selection();
    for ( int i = 0; i <= 255; i++ ) ; // short delay
    break;   //

  case 2:
    m = 0x20;   // GPA5(26) Controls Function Red led
    Select_Multi_BW_Ored();
    Step_Select_Button1 = Selected_Other; //
    Step_Select(); //
    Selection();
    for ( int i = 0; i <= 255; i++ ) ; // short delay
    break;   //
  }
}     // end Step_Function()

//===================================================================
void  Selection()
{

  unsigned long time;
  unsigned long start_time;
  unsigned long long_time;
  int SelectMode;
  int NONE   = 0;
  int CANCEL = 4000;
  int CLEAR  = 3000;
  int DELETE = 2000;
  int STORE  = 1000;
  int TUNE   = 500;
  int STEP   = 1;


  long_time = millis();
  time = millis();
  SelectMode = NONE;


  /*
   
   Memory Tune is activated by long press of Select Button
   
   Select  > .5 sec = Memory Tune
   > 1 Sec  = Store in EEPROM
   > 2 Sec  = Delete from EEPROM
   > 3 Sec  = Erase EEPROM
   
   
   */

  memset(LastMsg,'0',sizeof(LastMsg)); 

  // Memory Tune Selection 
  Step_Select_Button = digitalRead(Select_Button);


  if ( Step_Select_Button == HIGH )
  {
    SelectMode = STEP;    // Default mode
    while ( digitalRead(Select_Button) == HIGH )
    {

      // function button is pressed longer then 0.5 seconds
      if ( (millis() - long_time) > 4000 )
      {
        // Clear All EEPROM       
        SelectMode = CANCEL;
        Display_Msg("Cancel");

      }


      // function button is pressed longer then 0.5 seconds
      if ( (millis() - long_time) > 3000  && (millis() - long_time) < 4000)
      {
        // Clear All EEPROM       
        SelectMode = CLEAR;
        Display_Msg("Clear ");

      }


      // function button is pressed longer then 0.5 seconds
      if ( (millis() - long_time) > 2000  && (millis() - long_time) < 3000)
      {
        // Delete from EEOROM  
        SelectMode = DELETE;
        Display_Msg("Delete");    

      }


      // function button is pressed longer then 0.5 seconds
      if ( (millis() - long_time) > 1000 && (millis() - long_time) < 2000)
      {
        // Store in EEPROM   
        SelectMode = STORE; 
        Display_Msg("Store ");

      }



      // function button is pressed longer then 0.5 seconds
      if ( (millis() - long_time) > 500 && (millis() - long_time) < 1000)
      {
        // Set Memory Tuine Mode  
        SelectMode = TUNE;  
        Display_Msg("Mem Tune");

      }
    }



    Step_Select_Button = LOW;
  }

  if (SelectMode == TUNE)     Display_Msg("Mem Tune");
  if (SelectMode == STORE)    Display_Msg("Store ");
  if (SelectMode == DELETE)   Display_Msg("Delete");
  if (SelectMode == CLEAR)    Display_Msg("Clear ");
  if (SelectMode == CANCEL)   Display_Msg("Cancel");




  if ( SelectMode == STEP )
  {


    //Step_Select_Button = digitalRead(Select_Button);
    //if ( Step_Select_Button == HIGH )
    {
      //while ( digitalRead(Select_Button) == HIGH )     // added for testing
      //{
      //}
      for ( int i = 0; i <= 150e3; i++ ) ;    // short delay

      Step_Select_Button1 = Step_Select_Button1++;
      if ( Step_Select_Button1 > 2 )
      {
        Step_Select_Button1 = 0;
      }
    }
    Step_Select();
  }
  else
  {
    SelectMode = NONE;
    Step_Select_Button = LOW;
    // Process Memory Tune Functions
  }
}  // end Selection()

//-----------------------------------------------------------------------
void Step_Select()
{
  switch ( Step_Select_Button1 )
  {
  case 0: //   Select_Green
    s = 0x01;  // GPA0(21) Controls Selection Green led
    if ( Step_Multi_Function_Button1 == 0 )
    {
      b = 0x00; // Hardware control of I.F. filter shape
      Selected_BW = Wide_BW;  // GPA7(28)LOW_GPA6(27)LOW wide
    }
    else if ( Step_Multi_Function_Button1 == 1 )
    {
      Frequency_Step = 100;   //  Can change this whatever step size one wants
      Selected_Step = Step_100_Hz;
    }
    else if ( Step_Multi_Function_Button1 == 2 )
    {
      bsm = 0;
      Change_Band();
      Encoder();
      Selected_Other = Other_1_user;
      // Other_1();
    }
    for ( int i = 0; i <= 255; i++ ) ; // short delay
    break;

  case 1: //   Select_Yellow
    s = 0x02; //  GPA1(22) Controls Selection Green led
    if ( Step_Multi_Function_Button1 == 0 )
    {
      b = 0x40; // Hardware control of I.F. filter shape
      Selected_BW = Medium_BW;  //  GPA7(28)LOW_GPA6(27)HIGH medium
    }
    else if ( Step_Multi_Function_Button1 == 1 )
    {
      Frequency_Step = 1e3;   //  Can change this whatever step size one wants
      Selected_Step = Step_1000_hz;
    }
    else if ( Step_Multi_Function_Button1 == 2 )
    {
      bsm = 1;
      Change_Band();
      Encoder();
      Selected_Other = Other_2_user;

      //   Other_2();
    }
    for ( int i = 0; i <= 255; i++ ) ; // short delay
    break;

  case 2: //   Select_Red
    s = 0x04; //  GPA2(23) Controls Selection Green led
    if ( Step_Multi_Function_Button1 == 0 )
    {
      b = 0x80; // Hardware control of I.F. filter shape
      Selected_BW = Narrow_BW;   //  GPA7(28)HIGH_GPA6(27)LOW narrow
    }
    else if ( Step_Multi_Function_Button1 == 1 )
    {
      Frequency_Step = 10e3; //  Can change this whatever step size one wants
      Selected_Step = Step_10000_hz;
    }
    else if ( Step_Multi_Function_Button1 == 2 )
    {
      Selected_Other = Other_3_user;

      //       Other_3();
    }
    for ( int i = 0; i <= 255; i++ ) ; // short delay
    break;
  }
  Select_Multi_BW_Ored();
  //Splash_Step_Size();
  //Splash_BW();
}  // end Step_Select()

//----------------------------------------------------------------------------------
void Select_Multi_BW_Ored()
{
  t = s | m | b;    // or'ed bits

  Wire.beginTransmission(0x20);
  Wire.send(0x12);    // GPIOA
  Wire.send(t);    // port A  result of s, m, b
  Wire.endTransmission();

}  // end  Select_Multi_BW_Ored()















//////////////////////////  CAT  //////////////////////////

//------------------------CAT Routine based on Elecraft K3 -------------------------------
//   some general routines for serial printing

int un_stage(){           // send a char on serial 
  char c;
  if ( stg_in == stg_out ) return 0;
  c = stg_buf[stg_out++];
  stg_out &= ( STQUESIZE - 1);
  Serial.write(c);
  return 1;
}
void stage( unsigned char c ){
  stg_buf[stg_in++] = c;
  stg_in &= ( STQUESIZE - 1 );
}
void stage_str( String st ){
  int i;
  char c;
  for ( i = 0; i < st.length(); ++i )
  {
    c= st.charAt( i );
    stage(c);
  }    
}
void stage_num( int val ){   // send number in ascii 
  char buf[12];
  char c;
  int i;
  itoa( val, buf, 10 );
  i= 0;
  while ( c = buf[i++] ) stage(c);  
}

void Poll_Cat() {
  static String command = "";
  String lcommand;
  char c;
  int rit;

  if ( Serial.available() == 0 ) return;

  while ( Serial.available() )
  {
    c = Serial.read();
    command += c;
    if ( c == ';' ) break;
  }

  if ( c != ';' )
  {
    terminal = 0; 
    return;
  }                                       // command not complete yet but need to switch of terminal

  lcommand = command.substring(0,2);

  if ( command.substring(2,3) == ";" || command.substring(2,4) == "$;" || command.substring(0,2) == "RV" )
  {                                                                                                          /* it is a get command */
    stage_str(lcommand);    // echo the command 
    if ( command.substring(2,3) == "$" ) stage('$');

    if ( lcommand == "IF" )
    {
      RX_frequency = frequency_tune - IF;
      stage_str("000");
      if ( RX_frequency < 10000000 ) stage('0');
      stage_num(RX_frequency);  
      stage_str("     ");
      rit= RitFreqOffset;
      if ( rit >= 0 ) stage_str("+0");
      else
      {
        stage_str("-0"); 
        rit = - rit;
      }
      if ( rit < 100 ) stage('0');
      if ( rit < 10 ) stage('0');                          // IF[f]*****+yyyyrx*00tmvspbd1*;
      stage_num(rit);
      stage_str("10 0003000001");                          // rit,xit,xmit,cw mode fixed filed 
    }
    else if ( lcommand == "FA" )
    {                                                        // VFO A
      stage_str("000"); 
      if ( frequency_tune -IF < 10000000 ) stage('0');
      stage_num(frequency_tune - IF);  
    }
    else if ( lcommand == "KS" ) stage_num(CWSpeedWPM);          // KEYER SPEED
    else if ( lcommand == "FW" ) stage_str("0000") , stage_num(Selected_BW+1);
    else if ( lcommand == "MD" )
    {
      if ( mode==0 && bsm==0 ) stage('1');
      if ( mode==0 && bsm==1 ) stage('2');
      if ( mode==1 ) stage ('3');
    }                                                                                                                                               //

    else if ( lcommand == "RV" && command.substring(2,3) == "F" )
    {                                                       // battery voltage in Front field 
      stage(command.charAt(2));
      double value = analogRead(BatteryReadPin)* BatteryVconvert;
      int left_part, right_part;
      char buffer[50];
      sprintf(buffer, "%lf", value);
      sscanf(buffer, "%d.%1d", &left_part, &right_part);
      stage(' ');
      stage_num(left_part);
      stage('.');
      stage_num(right_part);
      stage(' ');
    }
    else if ( lcommand == "RV" && command.substring(2,3) == "A" )
    {                                                       // Rebel Alliance Mod version in Aux: field
      stage(command.charAt(2));
      stage_str(txt0);
    }
    else if ( lcommand == "RV" && command.substring(2,3) == "D" )
    {                                                       // Rebel Alliance Mod in DSP: field   
      stage(command.charAt(2));
      stage_str(txt2);
    }
    else if ( lcommand == "RV" && command.substring(2,3) == "M" )
    {                                                       // Keyer Speed in MCU: field
      stage(command.charAt(2));
      stage_num(CWSpeedWPM);
    }
    else if ( lcommand == "RV" && command.substring(2,3) == "R" )
    {                                                       // Keyer Speed in MCU: field
      stage(command.charAt(2));
      stage_str(steptext[Selected_Step]);
    }
    else if ( lcommand == "SM" )
    {
      stage_str("00");
      SmeterReadValue = analogRead(SmeterReadPin);
      SmeterReadValue = map(SmeterReadValue, 0, 1023, 0, 21);
      if ( SmeterReadValue < 10 ) stage('0');
      stage_num(SmeterReadValue);
    }
    else
    {
      stage('0');  // send back nill command not know / used
    }
    stage(';');     // response terminator 
  }

  else
  {
  } 
  set_cat(lcommand,command);        // else it's a set command 

    command = "";  // clear for next command
}

void set_cat(String lcom, String com ){
  long value;
  int split =0 ;

  if ( lcom == "FA" )
  {                   // set vfo freq 
    value = com.substring(2,13).toInt(); 
    if ( (value > Band_bsm0_Low && value < Band_bsm0_High) || (value > Band_bsm1_Low && value < Band_bsm1_High) )
    {
      // valid frequnecy according band configuration?
      if ( (value > Band_bsm0_Low && value < Band_bsm0_High) && bsm == 1 )
      {
        // need to change band?
        bsm = 0;
        Change_Band();
      }
      if ( (value > Band_bsm1_Low && value < Band_bsm1_High) && bsm == 0 )
      {
        // need to change band?
        bsm = 1;
        Change_Band();
      }
      if ( lcom == "FA" && ( value > 1800000 && value < 30000000) ) frequency = value + IF;
    }
  }
  else if ( lcom == "FW" )
  {                                 // xtal filter select
    value = com.charAt(6) - '0';
    if ( value < 4 && value != 0 )
    {
      if ( value == 1 )
      {
        b = 0x00; // Hardware control of I.F. filter shape
        Selected_BW = Wide_BW;  // GPA7(28)LOW_GPA6(27)LOW wide
      }
      if ( value == 2 )
      {
        b = 0x40; // Hardware control of I.F. filter shape
        Selected_BW = Medium_BW;  //  GPA7(28)LOW_GPA6(27)HIGH medium
      }
      if ( value == 3 )
      {
        b = 0x80; // Hardware control of I.F. filter shape
        Selected_BW = Narrow_BW; //  GPA7(28)HIGH_GPA6(27)LOW narrow
      }
      Select_Multi_BW_Ored();            // original b is sent to port expander
    }
  }
  else if ( lcom == "MD" )
  {
    value = com.charAt(2) - '0';
    if ( value == 1 || value == 2 )
    {
      mode=0; 
      frequency_old=0; 
      UpdateFreq(frequency_tune); //Splash_MODE();
    }
    if ( value == 3 )
    {
      mode=1; 
      frequency_old=0; 
      UpdateFreq(frequency_tune); //Splash_MODE();
    }
  }
}


//-----------------------------------------------------------------------------
// ****************  Dont bother the code below  ******************************
// \/  \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
//-----------------------------------------------------------------------------
void program_freq0(long frequency)
{
  AD9834_reset_high();
  int flow, fhigh;
  fcalc = frequency * (268.435456e6 / Reference);    // 2^28 =
  flow = fcalc & 0x3fff;          //  49.99975mhz
  fhigh = (fcalc >> 14) & 0x3fff;
  digitalWrite(FSYNC_BIT, LOW); //
  clock_data_to_ad9834(flow | AD9834_FREQ0_REGISTER_SELECT_BIT);
  clock_data_to_ad9834(fhigh | AD9834_FREQ0_REGISTER_SELECT_BIT);
  digitalWrite(FSYNC_BIT, HIGH);
  AD9834_reset_low();
} // end   program_freq0

//------------------------------------------------------------------------------

void program_freq1(long frequency_TX)
{
  AD9834_reset_high();
  int flow, fhigh;
  fcalc = frequency * (268.435456e6 / Reference);    // 2^28 =
  flow = fcalc & 0x3fff;          //  use for 49.99975mhz
  fhigh = (fcalc >> 14) & 0x3fff;
  digitalWrite(FSYNC_BIT, LOW);
  clock_data_to_ad9834(flow | AD9834_FREQ1_REGISTER_SELECT_BIT);
  clock_data_to_ad9834(fhigh | AD9834_FREQ1_REGISTER_SELECT_BIT);
  digitalWrite(FSYNC_BIT, HIGH);
  AD9834_reset_low();
}

//------------------------------------------------------------------------------
void clock_data_to_ad9834(unsigned int data_word)
{
  char bcount;
  unsigned int iData;
  iData = data_word;
  digitalWrite(SCLK_BIT, HIGH); //portb.SCLK_BIT = 1;
  // make sure clock high - only chnage data when high
  for ( bcount = 0; bcount < 16; bcount++ )
  {
    if ( (iData & 0x8000) ) digitalWrite(SDATA_BIT, HIGH);     //portb.SDATA_BIT = 1;
    // test and set data bits
    else  digitalWrite(SDATA_BIT, LOW);
    digitalWrite(SCLK_BIT, LOW);
    digitalWrite(SCLK_BIT, HIGH);
    // set clock high - only change data when high
    iData = iData << 1;    // shift the word 1 bit to the left
  } // end for
}   // end  clock_data_to_ad9834

//-----------------------------------------------------------------------------
void AD9834_init()   // set up registers
{
  AD9834_reset_high();
  digitalWrite(FSYNC_BIT, LOW);
  clock_data_to_ad9834(0x2300); // Reset goes high to 0 the registers and enable the output to mid scale.
  clock_data_to_ad9834((FREQ0_INIT_VALUE & 0x3fff) | AD9834_FREQ0_REGISTER_SELECT_BIT);
  clock_data_to_ad9834(((FREQ0_INIT_VALUE >> 14) & 0x3fff) | AD9834_FREQ0_REGISTER_SELECT_BIT);
  clock_data_to_ad9834(0x2200);    // reset goes low to enable the output.
  AD9834_reset_low();
  digitalWrite(FSYNC_BIT, HIGH);
}  //  end   AD9834_init()

//----------------------------------------------------------------------------
void AD9834_reset()
{
  digitalWrite(RESET_BIT, HIGH); // hardware connection
  for ( int i = 0; i <= 2048; i++ ) ; // small delay

  digitalWrite(RESET_BIT, LOW);  // hardware connection
}  // end AD9834_reset()

//-----------------------------------------------------------------------------
void AD9834_reset_low()
{
  digitalWrite(RESET_BIT, LOW);
}  // end AD9834_reset_low()

//..............................................................................
void AD9834_reset_high()
{
  digitalWrite(RESET_BIT, HIGH);
}  // end  AD9834_reset_high()

//^^^^^^^^^^^^^^^^^^^^^^^^^  DON'T BOTHER CODE ABOVE  ^^^^^^^^^^^^^^^^^^^^^^^^^
//=============================================================================

//------------------------Display Stuff below-----------------------------------

// All the code for display below seems to work well. 6-18-14

//------------------------------------------------------------------------------
void splash_TX_freq()
{
  long TXD_frequency; // ADDED 6-18-14 OK

  if ( bsm == 1 )                // test for 20M
  {
    TXD_frequency = frequency_tune;
  }

  else if ( bsm == 0 )            // test for 40M
  {
    TXD_frequency = frequency_tune;
  }
  //---------------------------------------------------

  if ( TXD_frequency < 5.36e6 )
  {
    TXD_frequency = TXD_frequency + 9e6;
  }

  else if ( TXD_frequency > 15.95e6 )
  {
    TXD_frequency = TXD_frequency - 9e6;
  }
  //--------------------------------------------------

  //lcd.setCursor(3, 1);
  stringFREQ = String(TXD_frequency / 10, DEC);
  Display_Freq();

  //lcd.print(stringFREQ);

}

//------------------------------------------------------------------------------
void splash_RX_freq()
{



  long RXD_frequency;    // ADDED 6-18-14 OK

  if ( old_frequency_tune != frequency_tune )
  {
    if ( bsm == 1 )               // test for 20M
    {
      RXD_frequency = frequency_tune;
    }

    else if ( bsm == 0 )          // test for 40M
    {
      RXD_frequency = frequency_tune;
    }
    //-------------------------------------------


    // 20 Meters
    if ( RXD_frequency < 5.36e6 )
    {
      RXD_frequency = RXD_frequency  + 9e6;
    }


    // 40 Meters
    else if ( RXD_frequency > 15.95e6 )
    {
      RXD_frequency = RXD_frequency  - 9e6;
    }
    //--------------------------------------------
    stringFREQ = String(RXD_frequency, DEC);


    Display_Freq();

  }
  old_frequency_tune = frequency_tune;
}


void Display_Freq(){


  char work[8];
  int i;


  // WA2EIN Changes for Seetron OLED  GLO-216
  // Seetron Clear Screen, Reset Font to default, Set font to Large, Display Frequency
  Serial1.print(Clear);
  Serial1.print(DefaultFont);
  Serial1.print(NewFont);
  Serial1.print(NewFont);
  Serial1.print(NewFont);

  memset(work,'\0',sizeof(work));
  // convert String to char array .  Limit freq to 7 char.
  for ( i=0; i<6; i++ )
  {
    work[i] = stringFREQ.charAt(i);
  }



  //Serial1.print(stringFREQ);
  Serial1.print(work);
  Serial1.print(DefaultFont);

  if ( mode == 0 )
  {
    Serial1.print("S");
  }
  else
  {
    Serial1.print("C");
  }


}





void Display_RIT(){


  // WA2EIN Changes for Seetron OLED  GLO-216
  // Seetron Clear Screen, Reset Font to default, Set font to Large, Display RIT
  Serial1.print(Clear);
  Serial1.print(DefaultFont);
  Serial1.print(NewFont);
  Serial1.print(NewFont);
  Serial1.print(NewFont);
  Serial1.print(stringRIT);
  DisplayCtr = millis();
  DisplayDone = false;

}

void Display_Msg(char *Msg)
{

 

  if (strcmp(LastMsg,Msg) != 0)
  {
    Serial1.print(Clear);
    Serial1.print(DefaultFont);
    Serial1.print(NewFont);
    Serial1.print(NewFont);
    Serial1.print(NewFont);
    Serial1.print(Msg);
    DisplayCtr = millis();
    DisplayDone = false;
    strcpy(LastMsg,Msg);
  }






}


void ForceFreqDisplay(void)
{
  DisplayCtr = millis() - 1000;
  DisplayDone = false;
}




//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//stuff above is for testing using the Display Comment out if not needed
//-----------------------------------------------------------------------------
uint32_t TimerOverFlow(uint32_t currentTime)
{
  return(currentTime + CORE_TICK_RATE * (1));    //the Core Tick Rate is 1ms
}



void SetStepSize()
{
  long temp;


  // Bypass rate based step if Function.Step selected
  if ( Step_Multi_Function_Button1 == 1 ) return;

  // WA2EIN Progressive Tune

  EncoderTimeDiff = millis() - LastEncoderTime;


  if ( EncoderTimeDiff > 0 )
  {
    temp = 10000 / EncoderTimeDiff;


    if ( temp > StepMedMark-1 )
    {
      Frequency_Step = 10000;

    }

    if ( temp > StepLowMark-1 && temp <=StepMedMark )
    {
      Frequency_Step = 1000;

    }

    if ( temp < StepLowMark )
    {
      Frequency_Step = 100;
    }



    LastEncoderTime = millis();

  }



} // end 


void CHK()
{
  DoCheckpoint = 1;


}  


void Checkpoint()
{


  EEProm *p1;
  uint8_t *p2;
  int len;
  static int ctr;
  uint16_t Address = 1;




  DoCheckpoint = 0;


  CheckpointData.bsm = bsm;
  CheckpointData.mode = mode;
  CheckpointData.recv_offset = recv_offset;
  CheckpointData.Step_Select_Button1 = Step_Select_Button1;
  CheckpointData.Step_Multi_Function_Button1 = Step_Multi_Function_Button1;
  CheckpointData.frequency = frequency_tune;
  CheckpointData.Data_Valid = 9999;

  p1 = &CheckpointData;
  p2 = (uint8_t *) p1;
  len = sizeof(CheckpointData);

  if ( CheckpointData.frequency != LastCheckpointData.frequency )
  {

    IOShieldEEPROM.writeString(Address, p2,len);
    LastCheckpointData = CheckpointData;
    Step_Flash();
  }




}

void Restore(void)
{


  EEProm  *p1;
  uint8_t *p2;
  int len;

  p1 = &CheckpointData;
  len = sizeof(CheckpointData);
  p2 = (uint8_t *)p1;
  uint16_t Address = 1;



  IOShieldEEPROM.readString(Address, p2, len);

  if ( CheckpointData.Data_Valid == 9999 )
  {

    bsm =  CheckpointData.bsm;
    mode = CheckpointData.mode;
    recv_offset = CheckpointData.recv_offset;
    Step_Select_Button1 = CheckpointData.Step_Select_Button1;
    Step_Multi_Function_Button1 =CheckpointData.Step_Multi_Function_Button1;
    frequency = CheckpointData.frequency;
    frequency_tune = frequency;



    Step_Select();
    Step_Function();

    frequency_default = frequency; 
    Default_frequency();   
    Change_Band();
    UpdateFreq(frequency);



    RTCC.begin();


  }


}















