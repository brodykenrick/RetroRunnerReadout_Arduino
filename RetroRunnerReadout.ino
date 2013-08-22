/* Retro Runner Readout
http://retrorunnerreadout.blogspot.com
Copyright 2013 Brody Kenrick.

Interfacing of a cheap Nordic nRF24AP (ANT+) UART module to an Arduino and an 8 character 7-segment display module (using MAX7219).

An Arduino Pro Mini 3v3
This nRF24AP2 module : http://www.goodluckbuy.com/nrf24ap2-networking-module-zigbee-module-with-ant-transceiver-.html
This 7-segment display : http://dx.com/p/8-segment-led-display-board-module-for-arduino-147814
Batteries, a 5V regulator and a level converter (http://www.freetronics.com/products/logic-level-converter-module)


The connector on nRF24AP2 board is (looking from the front, pin 1 is marked []):
[]GND(=VSS) | VDD(=3.3 volts)
UART_TX   | UART_RX
!(SUSP)   | SLEEP
RTS       | !(RESET)

Wiring to the Arduino Pro Mini 3v3 can be seen in 'antplus' below.

The display board is labelled ant the connections are as follows:
MAX7219/7221 -> Arduino:Pin
  DIN       -> MOSI:11    (Arduino output)
  CLK       -> SCK:13     (Arduino output)
  LOAD/#CS  -> SS:10      (Arduino output)

Wiring to the Arduino Pro Mini 3v3 can be seen in 'mydisplay' below.
*/

#include <Arduino.h>

//#define NDEBUG
#define __ASSERT_USE_STDERR
#include <assert.h>

//#define USE_NARCOLEPTIC_DELAY //<! Use Narcoleptic to save battery with power down sleep (vs. busy delay())
//Can't use this and ANTPlus because of software serial
//TODO: Need to rework and use IDLE mode and HardwareSerial for the ANTPlus instead...
//NOTE: Narcoleptic.disableSerial() might impact above.....
#define USE_NARCOLEPTIC_DISABLE //<! Use Narcoleptic to save some battery with disabling certain items but not use narco delay....

//#define ANTPLUS_ON_HW_UART //!< H/w UART (i.e. Serial) instead of software serial


#if (defined(USE_NARCOLEPTIC_DELAY) || defined(USE_NARCOLEPTIC_DISABLE))
#include <Narcoleptic.h>
#endif

#if !defined(ANTPLUS_ON_HW_UART)
#include <SoftwareSerial.h>
#endif

#include <LedControl.h>
#include <ANTPlus.h>



//TODO: Later when we swap to hardware serial then there can be no console.....
#define USE_SERIAL_CONSOLE //!<Use the hardware serial as the console
#define CONSOLE_BAUD_RATE (115200)

#define LOOP_TEXT //<! A debugging mode that makes the strings displayed sequential (instead of random).


#define DISPLAY_DURATION_MS (1200)  //!< This is the display time for 8 characters. Scrolling takes longer (not quite linear increase).
#define DELAY_BETWEEN_DISPLAYS_MS (1200) //<! Duration to have screen shut down between displaying messages
#define DISPLAY_INTENSITY (12) //<! 0..15


 
#if defined(NDEBUG) || defined(ANTPLUS_ON_HW_UART)
#undef CONSOLE_BAUD_RATE
#undef USE_SERIAL_CONSOLE
#endif





//Logging macros
//********************************************************************

#define SERIAL_DEBUG
#define SERIAL_INFO
#define SERIAL_WARNING
#if !defined(USE_SERIAL_CONSOLE)
//Disable logging under these circumstances
#undef SERIAL_DEBUG
#undef SERIAL_INFO
#undef SERIAL_WARNING
#endif
//F() stores static strings that come into existence here in flash (makes things a bit more stable)
//TODO: Update with using a logging (like log4j) system - with level (tricky using F() though), time, file etc.
#ifdef SERIAL_DEBUG
#define SERIAL_DEBUG_PRINT(x)  	        (Serial.print(x))
#define SERIAL_DEBUG_PRINTLN(x)	        (Serial.println(x))
#define SERIAL_DEBUG_PRINT_F(x)  	(Serial.print(F(x)))
#define SERIAL_DEBUG_PRINTLN_F(x)	(Serial.println(F(x)))

#define SERIAL_DEBUG_PRINT2(x,y)  	(Serial.print(x,y))
#define SERIAL_DEBUG_PRINTLN2(x,y)	(Serial.println(x,y))

#else
#define SERIAL_DEBUG_PRINT(x)
#define SERIAL_DEBUG_PRINTLN(x)
#define SERIAL_DEBUG_PRINT_F(x)
#define SERIAL_DEBUG_PRINTLN_F(x)

#define SERIAL_DEBUG_PRINT2(x,y)
#define SERIAL_DEBUG_PRINTLN2(x,y)
#endif


#ifdef SERIAL_INFO
#define SERIAL_INFO_PRINT(x)  	        (Serial.print(x))
#define SERIAL_INFO_PRINTLN(x)	        (Serial.println(x))
#define SERIAL_INFO_PRINT_F(x)  	(Serial.print(F(x)))
#define SERIAL_INFO_PRINTLN_F(x)	(Serial.println(F(x)))
#else
#define SERIAL_INFO_PRINT(x)
#define SERIAL_INFO_PRINTLN(x)
#define SERIAL_INFO_PRINT_F(x)
#define SERIAL_INFO_PRINTLN_F(x)
#endif

#ifdef SERIAL_WARNING
#define SERIAL_WARN_PRINT(x)  	        (Serial.print(x))
#define SERIAL_WARN_PRINTLN(x)	        (Serial.println(x))
#define SERIAL_WARN_PRINT_F(x)  	(Serial.print(F(x)))
#define SERIAL_WARN_PRINTLN_F(x)	(Serial.println(F(x)))
#else
#define SERIAL_WARN_PRINT(x)
#define SERIAL_WARN_PRINTLN(x)
#define SERIAL_WARN_PRINT_F(x)
#define SERIAL_WARN_PRINTLN_F(x)
#endif
//********************************************************************



//Special strings that will be replaced when encountered.
#define DISTANCE_LEFT_REPLACE   ("LEFT_REP")
#define DISTANCE_DONE_REPLACE   ("DONE_REP")
#define DISTANCE_TODAY_REPLACE  ("TODAY_REP")
#define NAME_REPLACE            ("NAME_REP")
#define MOTIVATE_REPLACE        ("MOTV_REP")
#define BPM_REPLACE             ("BPM_REP")
#define STRIDE_COUNT_REPLACE    ("SC_REP")

#define MAX_CHARS_TO_DISPLAY (56)
#define MAX_CHARS_TO_DISPLAY_STR (MAX_CHARS_TO_DISPLAY+1) //'\0' terminated

#define ANTPLUS_BAUD_RATE (9600)

//TODO: Do not publish.....
#define ANT_SENSOR_NETWORK_KEY {0xb9, 0xa5, 0x21, 0xfb, 0xbd, 0x72, 0xc3, 0x45}
#define ANT_GPS_NETWORK_KEY    {0xa8, 0xa4, 0x23, 0xb9, 0xf5, 0x5e, 0x63, 0xc1}



// ****************************************************************************
// ******************************  GLOBALS  ***********************************
// ****************************************************************************

//Arduino Pro Mini pins to the nrf24AP2 modules pinouts
#if !defined(ANTPLUS_ON_HW_UART)
const int TX_PIN       = 8; //Using software serial for the UART
const int RX_PIN       = 9; //Ditto
#else
//TX=0
//RX=1
#endif

const int RTS_PIN      = 2;
const int RTS_PIN_INT  = 0;


#if !defined(ANTPLUS_ON_HW_UART)
SoftwareSerial ant_serial(TX_PIN, RX_PIN); // RXArd, TXArd -- Arduino is opposite to nRF24AP2 module
#endif

ANTPlus        antplus   = ANTPlus(RTS_PIN, 3/*SUSPEND*/, 4/*SLEEP*/, 5/*RESET*/ );
LedControl     mydisplay = LedControl(11/*DIN:MOSI*/, 13/*CLK:SCK*/, 10/*CS:SS*/, 1/*Device count*/);


//ANT Channels for various device types
ANT_Channel hrm_channel =
{
  0,//channel_number
  0,//network_number   -- public
  12,//timeout         -- 12 * 2.5 = 30 seconds
  DEVCE_TYPE_HRM,//device_type    -- bit 7 = 0 pairing request bits 6..0 = 120 for HRM
  57,//freq            -- Garmin radio frequency
  32280,//period       -- Lowest sampling allowed for HRM
  ANT_SENSOR_NETWORK_KEY, //ant_net_key
};

#if 0
ANT_Channel fr410_channel =
{
  1,//channel_number
  0,//network_number   -- public
  12,//timeout         -- 12 * 2.5 = 30 seconds
  DEVCE_TYPE_GPS,//device_type    -- 
  50,//freq            -- Garmin radio frequency
  8070,//period        -- 
  ANT_GPS_NETWORK_KEY, //ant_net_key
};

ANT_Channel cadence_channel =
{
  2,//channel_number
  0,//network_number   -- public
  12,//timeout         -- 12 * 2.5 = 30 seconds
  DEVCE_TYPE_CADENCE,//device_type    -- 
  50,//freq            -- Garmin radio frequency
  8085,//period        -- 
  ANT_SENSOR_NETWORK_KEY, //ant_net_key
};
#endif

//Garmin footpod
ANT_Channel sdm_channel =
{
  4,//channel_number
  0,//network_number   -- public
  12,//timeout         -- 12 * 2.5 = 30 seconds
  DEVCE_TYPE_SDM,//device_type    -- bit 7 = 0 pairing request bits 6..0 = 124 for HRM
  57,//freq            -- Garmin radio frequency
  16268,//period       -- Lowest sampling allowed for SDM
  ANT_SENSOR_NETWORK_KEY, //ant_net_key
};


int last_computed_heart_rate = -1;

static long int metres_today  = 42195;
static long int metres_left   = metres_today;

boolean gReceivedHRMData = false;
boolean gReceivedSDMData = false;

int gCumulativeDistance      = 0;
int gPreviousMessageDistance = -1;

int gCumulativeStrideCount      = 0;
int gPreviousMessageStrideCount = -1;

//TODO: Make this dynamic later on
//These match the ANT_Channel channel numbers above
byte channel_device[ANT_DEVICE_NUMBER_CHANNELS] = {
  DEVCE_TYPE_HRM,
  DEVCE_TYPE_GPS,
  0,
  0,
  DEVCE_TYPE_SDM,
  0,
  0,
  0
};

ANT_CHANNEL_ESTABLISH ret_val_ce_hrm = ANT_CHANNEL_ESTABLISH_PROGRESSING;
ANT_CHANNEL_ESTABLISH ret_val_ce_sdm = ANT_CHANNEL_ESTABLISH_PROGRESSING;

volatile int rts_ant_received = 0; //!< ANT RTS interrupt flag see isr_rts_ant()

//Used in setup and then in loop -- save stack on this big variable by using globals
static char    adjusted_text[MAX_CHARS_TO_DISPLAY_STR] = "";
static boolean temp_string_decimals[MAX_CHARS_TO_DISPLAY];

// ****************************************************************************
// *******************************  TEXT  *************************************
// ****************************************************************************


const char startup_text_00[] PROGMEM = "- _ - ~ - _ - ~ - _ - ~ -";
#if defined(LOOP_TEXT)
const char startup_text_looper[] PROGMEM = "Looper ";
#else  
const char startup_text_01[] PROGMEM = "Oh HI! ";
const char startup_text_02[] PROGMEM = "Mandy ";
const char startup_text_03[] PROGMEM = "Ready ";
const char startup_text_04[] PROGMEM = "Set  ";
#endif //defined(LOOP_TEXT)
const char startup_text_05[] PROGMEM = "Go!!! ";

//! Texts sent to the display only at startup [in setup()]
PROGMEM const char * const startup_texts[] =
{
  startup_text_00,
#if defined(LOOP_TEXT)
  startup_text_looper,
#else  
  startup_text_01,
  startup_text_02,
  startup_text_03,
  startup_text_04,
#endif //defined(LOOP_TEXT)
  startup_text_05,
};
#define STARTUP_TEXTS_COUNT ( sizeof(startup_texts)/sizeof(const char *) )

// ****************************************************************************

const char loop_text_00[] PROGMEM     = "CanToo ";
const char loop_text_00a[] PROGMEM    = "Cure Cancer";
const char loop_text_01[] PROGMEM     = DISTANCE_TODAY_REPLACE;
//const char loop_text_02[] PROGMEM     = "today ";
const char loop_text_04[] PROGMEM     = " Run  ";
const char loop_text_NAME[] PROGMEM   = NAME_REPLACE;
const char loop_text_MOTIV8[] PROGMEM = MOTIVATE_REPLACE;
const char loop_text_07[] PROGMEM     = "Do It ";
const char loop_text_DONE[] PROGMEM   = DISTANCE_DONE_REPLACE;
const char loop_text_LEFT[] PROGMEM   = DISTANCE_LEFT_REPLACE;
const char loop_text_BPM[] PROGMEM    = BPM_REPLACE;
const char loop_text_STRIDES[] PROGMEM= STRIDE_COUNT_REPLACE;

PROGMEM const char * const loop_texts[] =
{
#if defined(LOOP_TEXT)
  loop_text_NAME,
  loop_text_MOTIV8,
  loop_text_BPM,
  loop_text_STRIDES,
  loop_text_DONE,
  loop_text_LEFT,
#else
  loop_text_00,
  loop_text_00a,
  loop_text_01,
//  loop_text_02,
  loop_text_04,
  loop_text_NAME,
  loop_text_MOTIV8,
  loop_text_07,
  loop_text_NAME,
  loop_text_MOTIV8,
  loop_text_BPM,
  loop_text_STRIDES,
  loop_text_DONE,
  loop_text_LEFT,
#endif //!defined(LOOP_TEXT)
};
#define LOOP_TEXTS_COUNT ( sizeof(loop_texts)/sizeof(loop_texts[0]) )

// ****************************************************************************

const char names_text_00[] PROGMEM = "Mandy";
const char names_text_01[] PROGMEM = "Flip  ";
const char names_text_02[] PROGMEM = "Fran  ";
const char names_text_03[] PROGMEM = "Andy  ";
const char names_text_04[] PROGMEM = "Steve ";
const char names_text_05[] PROGMEM = "Micheal";
const char names_text_06[] PROGMEM = "Hayley ";
const char names_text_07[] PROGMEM = "Ange  ";
const char names_text_08[] PROGMEM = "EMMa ";
const char names_text_09[] PROGMEM = "Annie ";

PROGMEM const char * const names_texts[] =
{
  names_text_00,
  names_text_01,
  names_text_02,
  names_text_03,
  names_text_04,
  names_text_05,
  names_text_06,
  names_text_07,
  names_text_08,
  names_text_09,
};
#define NAMES_TEXTS_COUNT ( sizeof(names_texts)/sizeof(names_texts[0]) )

// ****************************************************************************

//                                       "0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890"
//                                       "0         1         2          3         4         5         6        7         8         9        10        11        12"
const char motivates_text_00[] PROGMEM = "Channel your Inner Kenyan";
const char motivates_text_01[] PROGMEM = "Run Like You Stole SoMething";
const char motivates_text_02[] PROGMEM = "Keep It Up";
const char motivates_text_03[] PROGMEM = "Chafe noW - Brag Later!";
const char motivates_text_04[] PROGMEM = "Run Hard";
const char motivates_text_05[] PROGMEM = "Core on!";
const char motivates_text_06[] PROGMEM = "You have to Want it!";
const char motivates_text_07[] PROGMEM = "Catch Me If you Can";
const char motivates_text_08[] PROGMEM = "Mind your step...";
const char motivates_text_09[] PROGMEM = "AlMost there!";
const char motivates_text_10[] PROGMEM = "Keep - Earning - Ice CreaM";
const char motivates_text_11[] PROGMEM = "Long - hard -- fast?";
const char motivates_text_12[] PROGMEM = "AlMost Beer tiMe";
const char motivates_text_13[] PROGMEM = "You have staMina!   Call Me";
//const char motivates_text_14[] PROGMEM = "I just Met you; this is Crazy -- Here's My bib nuMber...";
const char motivates_text_15[] PROGMEM = "Stop reading - keep running";
const char motivates_text_16[] PROGMEM = "Toenails are Overrated";
const char motivates_text_17[] PROGMEM = "Lookout - Behind you. RUN - RUN - RUN";
const char motivates_text_18[] PROGMEM = "Having fun yet?";
const char motivates_text_19[] PROGMEM = "You thought they said ruM too?";
const char motivates_text_20[] PROGMEM = "Your pace or Mine?";
const char motivates_text_21[] PROGMEM = "Relentless ForWard Motion";
//const char motivates_text_22[] PROGMEM = "Do not fear Moving sloWly forWard...  Fear standing still!";
const char motivates_text_23[] PROGMEM = "If found on ground -- please drag to finish line";
const char motivates_text_24[] PROGMEM = "I found My happy pace";
const char motivates_text_25[] PROGMEM = "I Wonder hoW this thing Works?";
const char motivates_text_26[] PROGMEM = "Pain noW... beer Later";
//const char motivates_text_27[] PROGMEM = "Do my eMitted photons push her faster?";
//const char motivates_text_28[] PROGMEM = "These Messages Not brought to you by the letter 'kay'";
const char motivates_text_29[] PROGMEM = "I hope you haven't seen _ALL_ these Messages....";
const char motivates_text_30[] PROGMEM = "Running is a Mental sport... We are all Insane!";
const char motivates_text_31[] PROGMEM = "Friends Don't Let Friends Run Marathons";
const char motivates_text_32[] PROGMEM = "Hey! I just passed you!";
//const char motivates_text_33[] PROGMEM = "I run like a girl - try to keep up";
//const char motivates_text_34[] PROGMEM = "Running Won't kill you... you'll pass out first.";
const char motivates_text_35[] PROGMEM = "In it for the long run!";
const char motivates_text_36[] PROGMEM = "Hurdle coMing up";
const char motivates_text_37[] PROGMEM = "Does this thing have space Invaders?";
//const char motivates_text_38[] PROGMEM = "Race entry and running shoes 200.00. Finishing one more marathon than my boyfriend....priceless.";
const char motivates_text_39[] PROGMEM = "Fast girls have good tiMes";
const char motivates_text_40[] PROGMEM = "I've got the runs";
const char motivates_text_41[] PROGMEM = "Good tiMes ahead";
const char motivates_text_42[] PROGMEM = "Running Makes you Hot";
const char motivates_text_43[] PROGMEM = "Ouch!!";
//                                       "0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890"
//                                       "0         1         2          3         4         5         6        7         8         9        10        11        12"

PROGMEM const char * const motivates_texts[] =
{
  motivates_text_00,
  motivates_text_01,
  motivates_text_02,
  motivates_text_03,
  motivates_text_04,
  motivates_text_05,
  motivates_text_06,
  motivates_text_07,
  motivates_text_08,
  motivates_text_09,
  motivates_text_10,
  motivates_text_11,
  motivates_text_12,
  motivates_text_13,
//  motivates_text_14,
  motivates_text_15,
  motivates_text_16,
  motivates_text_17,
  motivates_text_18,
  motivates_text_19,
  motivates_text_20,
  motivates_text_21,
//  motivates_text_22,
  motivates_text_23,
  motivates_text_24,
  motivates_text_25,
  motivates_text_26,
//  motivates_text_27,
//  motivates_text_28,
  motivates_text_29,
  motivates_text_30,
  motivates_text_31,
  motivates_text_32,
//  motivates_text_33,
//  motivates_text_34,
  motivates_text_35,
  motivates_text_36,
  motivates_text_37,
//  motivates_text_38,
  motivates_text_39,
  motivates_text_40,
  motivates_text_41,
  motivates_text_42,
  motivates_text_43,
//  motivates_text_44,
//  motivates_text_45,
//  motivates_text_46,
//  motivates_text_47,
//  motivates_text_48,
//  motivates_text_49,
};
#define MOTIVATES_TEXTS_COUNT ( sizeof(motivates_texts)/sizeof(motivates_texts[0]) )



// **************************************************************************************************
// *********************************  ISRs  *********************************************************
// **************************************************************************************************

//! Interrupt service routine to get RTS from ANT messages
void isr_rts_ant()
{
  rts_ant_received = 1;
}

// **************************************************************************************************
// **********************************  Helper *******************************************************
// **************************************************************************************************

//TODO: Look to see if there is a way to force Serial to become a subclass that is extended with padded functions (as Serial is created on startup somewhere...)
void serial_print_byte_padded_hex(byte value)
{
#if defined(USE_SERIAL_CONSOLE)
  if(value <= 0x0F)
  {
      Serial.print(0, HEX);
  }
  Serial.print(value, HEX);
#endif //defined(USE_SERIAL_CONSOLE)
}

void serial_print_int_padded_dec( int number, byte width, boolean final_carriage_return )
{
#if defined(USE_SERIAL_CONSOLE)
  int div_num = number;
  int div_cnt = 0;
  while( div_num /= 10 )
  {
    div_cnt++;
  }
  if(div_cnt < width)
  {
    div_cnt = width - div_cnt - 1;
  }
  else
  {
    div_cnt = 0;
  }
  while( div_cnt-- )
  {
    Serial.print("0");
  }
  if(final_carriage_return)
  {
    Serial.println(number);
  }
  else
  {
    Serial.print(number);
  }
#endif //defined(USE_SERIAL_CONSOLE)
}

// **************************************************************************************************

unsigned long my_millis_function()
{
#if defined(USE_NARCOLEPTIC_DELAY)
  return( millis() + Narcoleptic.millis() );
#else
  return millis();
#endif
}

//Function allowing the LED library to do a callback to our delay functions
void my_delay_function(unsigned long duration_ms)
{
  //TODO: Perhaps add in a loop_antplus here if duration is large-ish
#if defined(USE_NARCOLEPTIC_DELAY)
#if defined(USE_SERIAL_CONSOLE)
  Serial.flush(); //Need to let the serial buffer finish transmissions
#endif //defined(USE_SERIAL_CONSOLE)
  Narcoleptic.delay( duration_ms );
#else
  delay( duration_ms );
#endif

}

void get_text_from_pm_char_ptr_array(char dst_text[], int dst_text_size, /*PROGMEM*/ const char * const pm_char_ptr_array[], int pm_char_ptr_array_size, int counter)
{
  const char * const* str_in_pm = &pm_char_ptr_array[ counter % pm_char_ptr_array_size ];
  
  for(int i = 0; i<dst_text_size; i++)
  {
    dst_text[i] = '\0';
  }
  strncpy_P(dst_text, (char*)pgm_read_word( str_in_pm ), MAX_CHARS_TO_DISPLAY_STR);
  
//  SERIAL_DEBUG_PRINT_F( "Text = " );
//  SERIAL_DEBUG_PRINT_F( " @ 0x" );
//  SERIAL_DEBUG_PRINT2( (int)str_in_pm, HEX );
//  SERIAL_DEBUG_PRINT_F( " = " );
//  SERIAL_DEBUG_PRINTLN( dst_text );
}

// **************************************************************************************************
// **********************************  Display  *****************************************************
// **************************************************************************************************

//Will either return the incoming text if no replacement happens
//Or do the replacement and return the changed text in replace_text
const char * replace_special_strings(const char * const text, char * const replace_text)
{
  const char * ret_text = text;
  if(!strcmp(text, DISTANCE_LEFT_REPLACE))
  {
    if(metres_left > 0)
    {
      itoa(metres_left / 1000, replace_text, 10);
      strcat (replace_text,".");
      itoa(metres_left % 1000, replace_text + strlen(replace_text), 10);

      strcat (replace_text," to go");
    }
    else
    {
      strcpy (replace_text,"Finished!");
    }
    ret_text = replace_text;
  }
  else
  if(!strcmp(text, DISTANCE_DONE_REPLACE))  
  {
    itoa(gCumulativeDistance / 1000, replace_text, 10);
    strcat (replace_text,".");
    itoa(gCumulativeDistance % 1000, replace_text + strlen(replace_text), 10);
    
    strcat (replace_text," done");
    
    ret_text = replace_text;
  }
  else
  if(!strcmp(text, STRIDE_COUNT_REPLACE))  
  {
    itoa(gCumulativeStrideCount, replace_text, 10);
    strcat (replace_text," strides");
    ret_text = replace_text;
  }
  else
  if(!strcmp(text, DISTANCE_TODAY_REPLACE))  
  {
    itoa(metres_today / 1000, replace_text, 10);
    strcat (replace_text,".");
    itoa(metres_today % 1000, replace_text + strlen(replace_text), 10);
    strcat (replace_text," today");
    ret_text = replace_text;
  }
  else  
  if(!strcmp(text, NAME_REPLACE))  
  {
    static int loop_names = 0;
#if !defined(LOOP_TEXT)
    loop_names = random(NAMES_TEXTS_COUNT);
#endif //!defined(LOOP_TEXT)
    get_text_from_pm_char_ptr_array(replace_text, MAX_CHARS_TO_DISPLAY_STR,
                                    names_texts, NAMES_TEXTS_COUNT, loop_names++);
    ret_text = replace_text;
  }
  else
  if(!strcmp(text, MOTIVATE_REPLACE))  
  {
    static int loop_motives = 0;
#if !defined(LOOP_TEXT)
    loop_motives = random(MOTIVATES_TEXTS_COUNT);
#endif //!defined(LOOP_TEXT)
    get_text_from_pm_char_ptr_array(replace_text, MAX_CHARS_TO_DISPLAY_STR,
                                    motivates_texts, MOTIVATES_TEXTS_COUNT, loop_motives++);
    ret_text = replace_text;
  }
  else
  if(!strcmp(text, BPM_REPLACE))  
  {
    if(last_computed_heart_rate != (-1))
    {
      itoa(last_computed_heart_rate, replace_text, 10);
      strcat (replace_text," BPM");
      ret_text = replace_text;
    }
    else
    {
      //Something a little better than the tagged replacement text.
      ret_text = "Be Heart SMart!";
    }
  }
  
  return ret_text;
}


void adjust_string(const char * text, char * out_text, boolean * out_decimals)
{
  char replace_text[MAX_CHARS_TO_DISPLAY_STR];
  //Clear
  for(int i = 0; i<MAX_CHARS_TO_DISPLAY_STR; i++)
  {
    replace_text[i] = '\0';
  }

  //SERIAL_DEBUG_PRINT_F( "Text = " );
  //SERIAL_DEBUG_PRINTLN( text );

  //Replace motivation or name strings etc
  text = replace_special_strings(text, replace_text);
  SERIAL_DEBUG_PRINT_F( "Adj. = " );
  SERIAL_DEBUG_PRINTLN( text );

  mydisplay.modify_string_for_better_display(text, out_text, out_decimals, MAX_CHARS_TO_DISPLAY);
  //SERIAL_DEBUG_PRINT_F( "Mod. = " );
  //SERIAL_DEBUG_PRINTLN( out_text );
}




//NOTE: Sleeps with screen off for period -- DELAY_BETWEEN_DISPLAYS_MS
void print_and_delay(const char * text)
{
    adjust_string( text, adjusted_text, temp_string_decimals );
    mydisplay.shutdown(0, false);  // Turns on display
    mydisplay.setDisplayAndScroll(0, adjusted_text, temp_string_decimals, MAX_CHARS_TO_DISPLAY, DISPLAY_DURATION_MS, my_delay_function );
    mydisplay.clearDisplay(0);
    mydisplay.shutdown(0, true);  // Turns off display (saving battery)
    my_delay_function( DELAY_BETWEEN_DISPLAYS_MS );
}



// **************************************************************************************************
// ***********************************  ANT+  *******************************************************
// **************************************************************************************************


// SDM -- 6.2.2
//Distance, time and stride count
int update_sdm_rollover( int MessageValue, int * Cumulative, int * PreviousMessageValue )
{
  if((*PreviousMessageValue) == -1)
  {
    (*PreviousMessageValue) = MessageValue;
  }
  (*Cumulative) += (MessageValue - (*PreviousMessageValue));

  if ((*PreviousMessageValue) > MessageValue)
  {
    (*Cumulative) += 256; //All fields rollover on this amount
  }
  (*PreviousMessageValue) = MessageValue;
  
  return (*Cumulative);
}


void process_packet( ANT_Packet * packet )
{
#if defined(USE_SERIAL_CONSOLE) && defined(ANTPLUS_DEBUG)
  //This function internally uses Serial.println
  //Only use it if the console is available and if the ANTPLUS library is in debug mode
  antplus.printPacket( packet, false );
#endif //defined(USE_SERIAL_CONSOLE) && defined(ANTPLUS_DEBUG)
   
  switch ( packet->msg_id )
  {
    case MESG_BROADCAST_DATA_ID:
    {
      const ANT_Broadcast * broadcast = (const ANT_Broadcast *) packet->data;
      SERIAL_DEBUG_PRINT_F( "CHAN " );
      SERIAL_DEBUG_PRINT( broadcast->channel_number );
      SERIAL_DEBUG_PRINT_F( " " );
      const ANT_DataPage * dp = (const ANT_DataPage *) broadcast->data;
      
      //To determine the device type -- and the data pages -- check channel setups
      if(channel_device[broadcast->channel_number] == DEVCE_TYPE_HRM)
      {
          switch(dp->data_page_number)
          {
            case DATA_PAGE_HEART_RATE_0:
            case DATA_PAGE_HEART_RATE_0ALT:
            case DATA_PAGE_HEART_RATE_1:
            case DATA_PAGE_HEART_RATE_1ALT:
            case DATA_PAGE_HEART_RATE_2:
            case DATA_PAGE_HEART_RATE_2ALT:
            case DATA_PAGE_HEART_RATE_3:
            case DATA_PAGE_HEART_RATE_3ALT:
            case DATA_PAGE_HEART_RATE_4:
            case DATA_PAGE_HEART_RATE_4ALT:
            {
              const ANT_HRMDataPage * hrm_dp = (const ANT_HRMDataPage *) dp;
              gReceivedHRMData = true;
              SERIAL_INFO_PRINT_F( "HR (BPM) = ");
              SERIAL_INFO_PRINTLN( hrm_dp->computed_heart_rate );
              last_computed_heart_rate = hrm_dp->computed_heart_rate;
            }
            break;

            default:
              //SERIAL_DEBUG_PRINTLN_F( "....");
              break;
          }
      }
      else
      if(channel_device[broadcast->channel_number] == DEVCE_TYPE_SDM)
      {
            switch(dp->data_page_number)
            {
              case DATA_PAGE_SPEED_DISTANCE_1:
              {
                const ANT_SDMDataPage1 * sdm_dp = (const ANT_SDMDataPage1 *) dp;
                gReceivedSDMData = true;
                //SERIAL_DEBUG_PRINT_F( "SDM Stride count..... = ");
                //SERIAL_DEBUG_PRINTLN( sdm_dp->stride_count );
                //SERIAL_DEBUG_PRINT_F( "SDM distance..... = ");
                //SERIAL_DEBUG_PRINTLN( sdm_dp->distance_int );
                SERIAL_INFO_PRINT_F( "Strides = ");
                update_sdm_rollover( sdm_dp->stride_count, &gCumulativeStrideCount, &gPreviousMessageStrideCount );
                SERIAL_INFO_PRINTLN( gCumulativeStrideCount );
                SERIAL_INFO_PRINT_F( "Distance = ");
                update_sdm_rollover( sdm_dp->distance_int, &gCumulativeDistance, &gPreviousMessageDistance );
                SERIAL_INFO_PRINTLN( gCumulativeDistance );
              }
              break;
  
              default:
                //TODO: Other pages....
                //SERIAL_DEBUG_PRINTLN_F("....");
                break;
            }
        }
    }
    break;
    
    default:
      //SERIAL_DEBUG_PRINTLN_F("...");
      break;
  }
}

// **************************************************************************************************
// ******************************  Loop functions  **************************************************
// **************************************************************************************************

void loop_display()
{
  static int counter = 0;

  char text[MAX_CHARS_TO_DISPLAY_STR];
  
  //SERIAL_DEBUG_PRINT( counter );
  //SERIAL_DEBUG_PRINT_F(" | ");
  //SERIAL_DEBUG_PRINT( LOOP_TEXTS_COUNT );
  //SERIAL_DEBUG_PRINT_F(" | ");
  //SERIAL_DEBUG_PRINTLN( counter % LOOP_TEXTS_COUNT );
  
  //Pull a string from the loop texts
  //These are one per loop() and some of them get replaced if they are magic texts
  get_text_from_pm_char_ptr_array(text, MAX_CHARS_TO_DISPLAY_STR, loop_texts, LOOP_TEXTS_COUNT, counter++);

  //Delay is in here
  print_and_delay( text );
  
  //Make the final values stick
  if(gCumulativeDistance > metres_today)
  {
    gCumulativeDistance = metres_today;
  }
  metres_left = metres_today - gCumulativeDistance;
  if(metres_left < 0)
  {
    metres_left = 0;
  }

  //HACK: Remove after sorted out SDM
  if(!gReceivedSDMData)
  {
      if(last_computed_heart_rate != (-1))
      {
        gCumulativeDistance     += last_computed_heart_rate;
        gCumulativeStrideCount  += last_computed_heart_rate/2;
      }
      else
      {
        gCumulativeDistance     += 1157;
        gCumulativeStrideCount  += 1157/2;
      }
  }
}

// ******************************************************************************************************

void loop_antplus()
{
  byte packet_buffer[MAXPACKETLEN];
  ANT_Packet * packet = (ANT_Packet *) packet_buffer;
  MESSAGE_READ ret_val = MESSAGE_READ_NONE;
  
  if(rts_ant_received == 1)
  {
#if 0
    SERIAL_DEBUG_PRINTLN_F("Received RTS Interrupt. ");
#endif
    antplus.rTSHighAssertion();
    //Clear the ISR flag
    rts_ant_received = 0;
  }

  //TODO: Make this a while loop to drain buffer
  ret_val = antplus.readPacket(packet, MAXPACKETLEN, 0 );

  if((ret_val == MESSAGE_READ_EXPECTED) || (ret_val == MESSAGE_READ_OTHER))
  {
#if 0
    if( (ret_val == MESSAGE_READ_EXPECTED) )
    {
      SERIAL_DEBUG_PRINTLN_F( "Expected" );
    }
    else
    if( (ret_val == MESSAGE_READ_OTHER) )
    {
      SERIAL_DEBUG_PRINTLN_F( "Other" );
    }
#endif
    process_packet(packet);
  }

  //Only one channel can progrss at a time though
  //HRM channel first
  if(ret_val_ce_hrm != ANT_CHANNEL_ESTABLISH_COMPLETE)
  {
    ret_val_ce_hrm = antplus.progress_setup_channel( &hrm_channel );
    if(ret_val_ce_hrm == ANT_CHANNEL_ESTABLISH_COMPLETE)
    {
      SERIAL_INFO_PRINTLN_F( "HRM established." );
    }
    else
    if(ret_val_ce_hrm == ANT_CHANNEL_ESTABLISH_PROGRESSING)
    {
      //SERIAL_DEBUG_PRINTLN_F( "HRM progressing." );
    }
    else
    {
      SERIAL_WARN_PRINTLN_F( "HRM ERROR!" );
    }
  }

  if(ret_val_ce_hrm == ANT_CHANNEL_ESTABLISH_COMPLETE)
  {
    //If HRM is up, then we progress SDM
    if(ret_val_ce_sdm != ANT_CHANNEL_ESTABLISH_COMPLETE)
    {
      ret_val_ce_sdm = antplus.progress_setup_channel( &sdm_channel );
      if(ret_val_ce_sdm == ANT_CHANNEL_ESTABLISH_COMPLETE)
      {
        SERIAL_INFO_PRINTLN_F( "SDM established." );
      }
      else
      if(ret_val_ce_sdm == ANT_CHANNEL_ESTABLISH_PROGRESSING)
      {
        //SERIAL_DEBUG_PRINTLN_F( "SDM progressing." );
      }
      else
      {
        SERIAL_WARN_PRINTLN_F( "SDM ERROR!" );
      }
    }
  }
}



// **************************************************************************************************
// ************************************  Setup  *****************************************************
// **************************************************************************************************
void setup()
{
#if defined(USE_SERIAL_CONSOLE)
  Serial.begin(CONSOLE_BAUD_RATE); 
#endif //defined(USE_SERIAL_CONSOLE)

  SERIAL_INFO_PRINTLN("CanToo Runner!");
  SERIAL_INFO_PRINTLN_F("Setup.");

  mydisplay.shutdown(0, false);  // turns on display
  mydisplay.setIntensity(0, DISPLAY_INTENSITY); // 0..15 = brightest
  mydisplay.clearDisplay(0);
  mydisplay.shutdown(0, true);  // turns off display

#if defined(USE_NARCOLEPTIC_DISABLE)
  Narcoleptic.disableTimer2();
//  Narcoleptic.disableTimer1(); //Needed for SPI
//  Narcoleptic.disableMillis();
#if !defined(USE_SERIAL_CONSOLE)
  //Don't need serial if not debugging [NOTE: This may have implications later on USART usage...]
//TODO:XXX:Come back to this  Narcoleptic.disableSerial();
#endif //!defined(USE_SERIAL_CONSOLE)
  Narcoleptic.disableADC();

#endif

  //Print the startup messages
  for(unsigned int counter_setup = 0;
      counter_setup < STARTUP_TEXTS_COUNT; counter_setup++)
  {
    char text[MAX_CHARS_TO_DISPLAY_STR];
    get_text_from_pm_char_ptr_array(text, MAX_CHARS_TO_DISPLAY_STR,
                                    startup_texts, STARTUP_TEXTS_COUNT, counter_setup);
    print_and_delay( text );
  }

  SERIAL_DEBUG_PRINTLN_F("ANT+ Config.");

  //We setup an interrupt to detect when the RTS is received from the ANT chip.
  //This is a 50 usec HIGH signal at the end of each valid ANT message received from the host at the chip
  attachInterrupt(RTS_PIN_INT, isr_rts_ant, RISING);


#if defined(ANTPLUS_ON_HW_UART)
  //Using hardware UART
  Serial.begin(ANTPLUS_BAUD_RATE); 
  antplus.begin( Serial );
#else
  //Using soft serial
  ant_serial.begin( ANTPLUS_BAUD_RATE ); 
  antplus.begin( ant_serial );
#endif

  //This should not be strictly necessary - the device should always come up by itself....
  //But let's make sure we didn't miss the first RTS in a power-up race
  antplus.hardwareReset();

  SERIAL_DEBUG_PRINTLN_F("ANT+ Config Finished.");
  SERIAL_INFO_PRINTLN_F("Setup Finished.");
  
}

// **************************************************************************************************
// ************************************  Loop *******************************************************
// **************************************************************************************************


void loop()
{
  //TODO: We execute a few of these to ensure the buffers clear fast enough (as loop_display() also delays)...
  //TODO: Make it so that inside the function it loops until buffer is empty
  for(int i = 0; i< 10; i++)
  {
    loop_antplus();
  }

  //Get the ANT+ channels up quickly
  // then send some display messages
  if((ret_val_ce_hrm == ANT_CHANNEL_ESTABLISH_COMPLETE)
      &&
     (ret_val_ce_sdm == ANT_CHANNEL_ESTABLISH_COMPLETE))
  {
    loop_display();
  }
  else
  {
    //Number cycling will happen whilst the channels are starting up
    static byte counter = 0;
    mydisplay.shutdown(0, false);  // Turns on display
    mydisplay.clearDisplay(0);
    mydisplay.setChar(0, 0, counter++%0xF, false);
    my_delay_function( 50 );
  }
}

