//-----------------------------------------------------------------------------
// File:        cute_avr32.c
//
// Description: Embedded AVR32 code for the CUTE cryostat positioning system
//
// Revisions:   2011/06/03 - P.Harvey created
//              2013/06/06 - PH v1.03 - added ability to specify channel ranges in pa/pb
//              2013/10/04 - PH v1.04 - updated for Steve's new MANIP readout circuit
//              2013/10/24 - PH v1.05 - added ability to invert motor on/dir signals
//              2014/07/04 - PH v1.06 - added code to turn off DEAP valves on startup
//              2014/07/15 - PH v1.07 - disable rs232 debugging due to port conflict,
//                                      and disable WDT LED for DEAP resurfacer
//              2014/10/14 - PH v1.08 - clear more outputs on startup
//              2016/02/29 - PH v1.09 - added SNO+ digital i/o commands and changed
//                                      minimum motor speed to 25 steps/sec
//              2016/03/01 - PH v1.10 - added motor "run" command
//              2016/03/15 - PH v1.11 - check MAX197 INT signal to indicate conversion
//                                      and added "p6" command
//              2016/03/17 - PH v1.12 - changed motor/pwm "run" command to "spd"
//              2017/01/30 - PH v1.13 - added ability to specify ADC range
//              2018/07/19 - PH v1.14 - added motor step command
//
// Notes:       The embedded software is C because the inherent indirect
//              addressing of C++ objects is too inefficient
//
//              The watchdog timer is set to 1 second by default, and will
//              reset the AVR32 if a command is not received in this time.
//              On reset, LED3 will light until a "wdt SECS" command is received. (manip only)
//
// Disclaimer:  This code is very ugly and contains many duplicated sections
//              because it was found that using array lookups to reduce duplicate
//              code slowed things down significantly
//-----------------------------------------------------------------------------

//_____  I N C L U D E S ___________________________________________________

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "compiler.h"
#include "board.h"
#include "print_funcs.h"
#include "intc.h"
#include "power_clocks_lib.h"
#include "gpio.h"
#include "adc.h"
#include "conf_usb.h"
#include "usb_task.h"
#include "tc.h"
#include "wdt.h"
#include "pwm.h"
#include "usb_drv.h"
#include "usb_descriptors.h"
#include "usb_standard_request.h"

//#define MANIP       // for use with SNO+ manip
#define CUTE   		// use for CUTE expt

//#define DEBUG       // enable debugging code
#define VERSION		1.14

#define NUM_MOTORS          3
#define NUM_ADCS            4

#define TC0_CHANNEL      	0
#define TC1_CHANNEL      	1
#define TC2_CHANNEL      	2

#define IO_CHANNELS         44      // number of available I/O channels (PA0-PA31,PB0-PB11)

#define PKT_SIZE            64      // maximum USB packet size

#define FPBA              	FOSC0   // 12 MHz

#if defined(MANIP) || defined(CUTE)
// SNO+ manip AVR channels
#define BYSEL       16  // read byte select
#define XWR         17  // write strobe (active low)
#define XRD         20  // read strobe (active low)
#define XRST        21  // counter reset (active low)
#define DEV0        23  // device ID bit 0
#define INT         24  // MAX197 adc interrupt (active low)
#define DEV1        25  // device ID bit 1
#define BRD0        26  // board ID bit 0
#define BRD1        27  // board ID bit 1
#define BRDSEL      28  // board select
#define ENCP        29  // encoder power
#define WDAT        8   // bit 0 of write data
#define RDAT        0   // bit 0 of read data
#elif !defined(CUTE)
// DEAP output pins to clear on startup
#define kNumClearPins	14
static int sClearPin[kNumClearPins] = { 0,1,7,5,6,2,20,21,22,23,24,25,26,27 };
#endif

//_____ watchdog timer ______________________________________________
// watchdog timer delay in microseconds (note: value is rounded up to nearest available time)
#define WDT_MAX_VALUE_US   1 * 1000000L

// To specify which current Watchdog value
volatile U32 current_wdt_value = WDT_MAX_VALUE_US;

//_____ D E F I N I T I O N S ______________________________________________

#define kClockFreq		 12000000L // frequency for default tc clock source (3)
#define kPrescale		 8      // prescale for default clock source (3)
#define kMinTop 		 5      // limits maximum speed
#define kMotorAccDefault 4000   // default motor acceleration in steps/sec/sec
#define kMotorAccMin     1000   // minimum motor acceleration (steps/sec/sec)
#define kMotorAccMax     10000  // maximum motor acceleration (steps/sec/sec)
#define kMinSpeed        25     // minimum motor speed (steps/sec)
#define kInitRC          kClockFreq / (kPrescale * (long)kMinSpeed) // initial RC value

#define kMaxWaitConv     40     // maximum number of loops to wait for ADC conversion

#define OUT_SIZE 		1024	// size of message response buffer

__attribute__((__interrupt__)) static void m0_irq(void);
__attribute__((__interrupt__)) static void m1_irq(void);
__attribute__((__interrupt__)) static void m2_irq(void);

//_____ D E C L A R A T I O N S ____________________________________________

// motor TC initialization constants
static struct {
    int channel;
    int pin;
    int function;
    int rc;
    int irqNum;
    void (*irq)(void);
    int dir;    // PA channel for direction signal
    int on;     // PA channel for on signal
    char dirInv; // flag for direction signal inverted
    char onInv;  // flag for windings on signal inverted
} sMotor[NUM_MOTORS] = {
#if defined(MANIP) || defined(CUTE)
//
// Motor definitions for SNO+ MANIP and CUTE
//
{
    .channel    = 0, // TC channel for this motor
    .pin        = AVR32_TC_A0_0_0_PIN,      // PA32 (=PB00)
    .function   = AVR32_TC_A0_0_0_FUNCTION,
    .rc         = kInitRC,
    .irqNum     = AVR32_TC_IRQ0,
    .irq        = &m0_irq,
    .dir        = 33,                       // PB01
    .on         = 37,                       // PB05
    // aux output PB04
    .dirInv     = 0,
    .onInv      = 1
},{
    .channel    = 1,
    //.pin        = AVR32_TC_A1_0_0_PIN,      // PA21
    //.function   = AVR32_TC_A1_0_0_FUNCTION,
    .pin        = AVR32_TC_A1_0_1_PIN,      // PA34 (=PB02)
    .function   = AVR32_TC_A1_0_1_FUNCTION,
    .rc         = kInitRC,
    .irqNum     = AVR32_TC_IRQ1,
    .irq        = &m1_irq,
    .dir        = 35,                       // PB03
    .on         = 38,                       // PB06
    // aux output PB07
    .dirInv     = 0,
    .onInv      = 1
},{
    .channel    = 2,
    .pin        = AVR32_TC_A2_0_1_PIN,      // PB10
    .function   = AVR32_TC_A2_0_1_FUNCTION,
    .rc         = kInitRC,
    .irqNum     = AVR32_TC_IRQ2,
    .irq        = &m2_irq,
    .dir        = 43,                       // PB11
    .on         = 40,                       // PB08
    // aux output PB09
    .dirInv     = 0,
    .onInv      = 1
}
#else
//
// Motor definitions for DEAP resurfacer
//
{
    .channel    = 0, // TC channel for this motor
    .pin        = AVR32_TC_A0_0_0_PIN,      // PA32 (=PB00)
    .function   = AVR32_TC_A0_0_0_FUNCTION,
    .rc         = kInitRC,
    .irqNum     = AVR32_TC_IRQ0,
    .irq        = &m0_irq,
    .dir        = 15,                       // PA15
    .on         = 14,                       // PA14
    .dirInv     = 0,
    .onInv      = 0
},{
    .channel    = 1,
    //.pin        = AVR32_TC_A1_0_0_PIN,      // PA21
    //.function   = AVR32_TC_A1_0_0_FUNCTION,
    .pin        = AVR32_TC_A1_0_1_PIN,      // PA34 (=PB02)
    .function   = AVR32_TC_A1_0_1_FUNCTION,
    .rc         = kInitRC,
    .irqNum     = AVR32_TC_IRQ1,
    .irq        = &m1_irq,
    .dir        = 13,                       // PA13
    .on         = 12,                       // PA12
    .dirInv     = 0,
    .onInv      = 0
},{
    .channel    = 2,
    .pin        = AVR32_TC_A2_0_0_PIN,      // PA11
    .function   = AVR32_TC_A2_0_0_FUNCTION,
    .rc         = kInitRC,
    .irqNum     = AVR32_TC_IRQ2,
    .irq        = &m2_irq,
    .dir        = 10,                       // PA10
    .on         = 9,                        // PA09
    .dirInv     = 0,
    .onInv      = 0
}
#endif
};

static struct {
    int channel;
    int pin;
    int function;
} sADC[NUM_ADCS] = {
{
    .channel    = 0,
    .pin        = AVR32_ADC_AD_0_PIN,       // PA03
    .function   = AVR32_ADC_AD_0_FUNCTION,
},{
    .channel    = 1,
    .pin        = AVR32_ADC_AD_1_PIN,       // PA04
    .function   = AVR32_ADC_AD_1_FUNCTION,
},{
    .channel    = 6, // 6=light sensor
    .pin        = AVR32_ADC_AD_6_PIN,       // PA30
    .function   = AVR32_ADC_AD_6_FUNCTION,
},{
    .channel    = 7, // 7=temperature
    .pin        = AVR32_ADC_AD_7_PIN,       // PA31
    .function   = AVR32_ADC_AD_7_FUNCTION,
}};

static U16  sof_cnt;
static U16  data_length;
static char has_data;
static char wdt_flag = 0;   // 0=not enabled, 1=power up, 2=WDT reset
static char pwm_flag = 0;   // 0=not initialized, 1=stopped, 2=running

// PIO channel output modes (0=input, 1=output, 2=input /w pull-up, 3=other function)
static char output_mode[64] = { 0 };

// configuration
#if defined(MANIP) || defined(CUTE)
#define kNumAdrLines 	4
int cfg_adr[kNumAdrLines] = {
    // default lines for addressing devices (add 32 for PB channels)
    DEV0, DEV1, BRD0, BRD1
};
#define kNumDatLines	8
int cfg_dat[kNumDatLines] = {
    // default data lines (add 32 for PB channels)
    RDAT, RDAT+1, RDAT+2, RDAT+3, RDAT+4, RDAT+5, RDAT+6, RDAT+7
};
#define kNumDelay       6
int cfg_del[kNumDelay] = {
    0,  // counter delay after raising RST and after enable output
    0,  // counter delay for data to stabilize (min 65ns)
    0,  // adc delay before initiate conversion (min 50ns)
    0,  // adc delay for conversion (min 10us) [now wait for INT to go low]
    0,  // adc delay for data to stabilize (min 150ns)
    0,  // adc delay before reading low byte (min ??)
};

#define A0      cfg_adr[0]
#define A1      cfg_adr[1]
#define A2      cfg_adr[2]
#define A3      cfg_adr[3]

int dig_out[4] = { 0 };     // digital output bytes for Steve's modified board
#endif

int motor_src[5] = {
    TC_CLOCK_SOURCE_TC1,
    TC_CLOCK_SOURCE_TC2,
    TC_CLOCK_SOURCE_TC3,
    TC_CLOCK_SOURCE_TC4,
    TC_CLOCK_SOURCE_TC5
};
/*int motor_freq[5] = {
    32000,
    12000000,
    12000000,
    12000000,
    12000000
};
int motor_prescale[5] = {
    1,
    2,
    8,
    32,
    128
};*/
// actual frequency for each tc clock source
int motor_actClock[5] = {
    32768,      // 32 kHz / 1
    6000000,    // 12 MHz / 2
    1500000,    // 12 MHz / 8
    375000,     // 12 MHz / 32
    93750       // 12 MHz / 128
};

// motor variables
// NOTE: "ISR" variables are changed in interrupt routine!
long            m0_motorPos = 0;        // ISR motor position count
unsigned char   m0_motorDir = 0;        // motor direction flag
unsigned char   m0_motorOn = 0;         // motor on flag
unsigned char   m0_ramping = 0;         // ISR ramping flag
unsigned int    m0_rampTo = 0;          // ramp to this RC value
unsigned char   m0_rampFlag = 0;        // ISR flag to start ramping motor (2=stop,3=halt)
unsigned char   m0_running = 0;         // ISR flag that motor is running
unsigned        m0_curSpeed = kMinSpeed;// ISR current speed in Hz
unsigned        m0_endSpeed;            // ISR ramp end speed in Hz
unsigned        m0_curRC = kInitRC;     // ISR current counter RC value
#ifdef DEBUG
unsigned        m0_latency = 0;         // ISR maximum latency of interrupt
unsigned        m0_lastCount = 0;       // ISR latency of last interrupt
#endif
unsigned long   m0_actClock = 1500000;  // actual clock freq = kClockFreq / kPrescale
unsigned        m0_startSpeed;          // motor start speed
unsigned int    m0_acc = kMotorAccDefault;  // motor acceleration (steps/s/s)
long            m0_rampTime;            // time motor has been ramping (TC ticks)
long            m0_rampEndTime;         // time at end of ramp (TC ticks)
float           m0_rampScl = (float)(kClockFreq / kPrescale) / kMotorAccDefault;
int             m0_minSpeed = kMinSpeed;
unsigned char   m0_stopFlag = 0;        // ISR flag to stop motor
unsigned char   m0_stepMode = 0;        // 0=done, 1=ramp up, 2=cruise, 3=ramp down
int             m0_src = 3;             // source clock
long            m0_stepFrom;            // step start
long            m0_stepTo;              // step end
long            m0_rampEnd;             // step where ramp was completed
long            m0_stepNext;            // step where we have to change something

long            m1_motorPos = 0;
unsigned char   m1_motorDir = 0;
unsigned char   m1_motorOn = 0;
unsigned char   m1_ramping = 0;
unsigned int    m1_rampTo = 0;
unsigned char   m1_rampFlag = 0;
unsigned char   m1_running = 0;
unsigned        m1_curSpeed = kMinSpeed;
unsigned        m1_endSpeed;
unsigned        m1_curRC = kInitRC;
#ifdef DEBUG
unsigned        m1_latency = 0;
unsigned        m1_lastCount = 0;
#endif
unsigned long   m1_actClock = 1500000;
unsigned        m1_startSpeed;
unsigned int    m1_acc = kMotorAccDefault;
long            m1_rampTime;
long            m1_rampEndTime;
float           m1_rampScl = (float)(kClockFreq / kPrescale) / kMotorAccDefault;
int             m1_minSpeed = kMinSpeed;
unsigned char   m1_stopFlag = 0;
int             m1_src = 3;             // source clock

long            m2_motorPos = 0;
unsigned char   m2_motorDir = 0;
unsigned char   m2_motorOn = 0;
unsigned char   m2_ramping = 0;
unsigned int    m2_rampTo = 0;
unsigned char   m2_rampFlag = 0;
unsigned char   m2_running = 0;
unsigned        m2_curSpeed = kMinSpeed;
unsigned        m2_endSpeed;
unsigned        m2_curRC = kInitRC;
#ifdef DEBUG
unsigned        m2_latency = 0;
unsigned        m2_lastCount = 0;
#endif
unsigned long   m2_actClock = 1500000;
unsigned        m2_startSpeed;
unsigned int    m2_acc = kMotorAccDefault;
long            m2_rampTime;
long            m2_rampEndTime;
float           m2_rampScl = (float)(kClockFreq / kPrescale) / kMotorAccDefault;
int             m2_minSpeed = kMinSpeed;
unsigned char   m2_stopFlag = 0;
int             m2_src = 3;             // source clock

static tc_waveform_opt_t waveform_opt[NUM_MOTORS] = {
{
    .channel  = TC0_CHANNEL,        // Channel selection.

    .bswtrg   = TC_EVT_EFFECT_NOOP,           // Software trigger effect on TIOB.
    .beevt    = TC_EVT_EFFECT_NOOP,           // External event effect on TIOB.
    .bcpc     = TC_EVT_EFFECT_NOOP,           // RC compare effect on TIOB.
    .bcpb     = TC_EVT_EFFECT_NOOP,           // RB compare effect on TIOB.

    .aswtrg   = TC_EVT_EFFECT_NOOP,           // Software trigger effect on TIOA.
    .aeevt    = TC_EVT_EFFECT_NOOP,           // External event effect on TIOA.
    .acpc     = TC_EVT_EFFECT_CLEAR,          // RC compare effect on TIOA: clear output.
    .acpa     = TC_EVT_EFFECT_SET,            // RA compare effect on TIOA: set (possibilities are none, toggle, set and clear).

    .wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,      // Waveform selection: Up mode with automatic trigger on RC compare.
    .enetrg   = FALSE,                        // External event trigger enable.
    .eevt     = TC_EXT_EVENT_SEL_TIOB_INPUT,  // External event selection.
    .eevtedg  = TC_SEL_NO_EDGE,               // External event edge selection.
    .cpcdis   = FALSE,                        // Counter disable when RC compare.
    .cpcstop  = FALSE,                        // Counter clock stopped with RC compare.

    .burst    = TC_BURST_NOT_GATED,           // Burst signal selection.
    .clki     = TC_CLOCK_RISING_EDGE,         // Clock inversion.
    .tcclks   = TC_CLOCK_SOURCE_TC3           // Internal source clock 3, connected to fPBA / 8. (pg 522)
},{
    .channel  = TC1_CHANNEL,        // Channel selection.

    .bswtrg   = TC_EVT_EFFECT_NOOP,           // Software trigger effect on TIOB.
    .beevt    = TC_EVT_EFFECT_NOOP,           // External event effect on TIOB.
    .bcpc     = TC_EVT_EFFECT_NOOP,           // RC compare effect on TIOB.
    .bcpb     = TC_EVT_EFFECT_NOOP,           // RB compare effect on TIOB.

    .aswtrg   = TC_EVT_EFFECT_NOOP,           // Software trigger effect on TIOA.
    .aeevt    = TC_EVT_EFFECT_NOOP,           // External event effect on TIOA.
    .acpc     = TC_EVT_EFFECT_CLEAR,          // RC compare effect on TIOA: clear output.
    .acpa     = TC_EVT_EFFECT_SET,            // RA compare effect on TIOA: set (possibilities are none, toggle, set and clear).

    .wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,      // Waveform selection: Up mode with automatic trigger on RC compare.
    .enetrg   = FALSE,                        // External event trigger enable.
    .eevt     = TC_EXT_EVENT_SEL_TIOB_INPUT,  // External event selection.
    .eevtedg  = TC_SEL_NO_EDGE,               // External event edge selection.
    .cpcdis   = FALSE,                        // Counter disable when RC compare.
    .cpcstop  = FALSE,                        // Counter clock stopped with RC compare.

    .burst    = TC_BURST_NOT_GATED,           // Burst signal selection.
    .clki     = TC_CLOCK_RISING_EDGE,         // Clock inversion.
    .tcclks   = TC_CLOCK_SOURCE_TC3           // Internal source clock 3, connected to fPBA / 8. (pg 522)
},{
    .channel  = TC2_CHANNEL,        // Channel selection.

    .bswtrg   = TC_EVT_EFFECT_NOOP,           // Software trigger effect on TIOB.
    .beevt    = TC_EVT_EFFECT_NOOP,           // External event effect on TIOB.
    .bcpc     = TC_EVT_EFFECT_NOOP,           // RC compare effect on TIOB.
    .bcpb     = TC_EVT_EFFECT_NOOP,           // RB compare effect on TIOB.

    .aswtrg   = TC_EVT_EFFECT_NOOP,           // Software trigger effect on TIOA.
    .aeevt    = TC_EVT_EFFECT_NOOP,           // External event effect on TIOA.
    .acpc     = TC_EVT_EFFECT_CLEAR,          // RC compare effect on TIOA: clear output.
    .acpa     = TC_EVT_EFFECT_SET,            // RA compare effect on TIOA: set (possibilities are none, toggle, set and clear).

    .wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,      // Waveform selection: Up mode with automatic trigger on RC compare.
    .enetrg   = FALSE,                        // External event trigger enable.
    .eevt     = TC_EXT_EVENT_SEL_TIOB_INPUT,  // External event selection.
    .eevtedg  = TC_SEL_NO_EDGE,               // External event edge selection.
    .cpcdis   = FALSE,                        // Counter disable when RC compare.
    .cpcstop  = FALSE,                        // Counter clock stopped with RC compare.

    .burst    = TC_BURST_NOT_GATED,           // Burst signal selection.
    .clki     = TC_CLOCK_RISING_EDGE,         // Clock inversion.
    .tcclks   = TC_CLOCK_SOURCE_TC3           // Internal source clock 3, connected to fPBA / 8. (pg 522)
}};

//-----------------------------------------------------------------------------
/*! \brief timer interrupt for motor
 */
__attribute__((__interrupt__))
static void m0_irq(void)
{
   static char in = 0;

   // clear the interrupt flag by reading the TC status register
   tc_read_sr(&AVR32_TC, TC0_CHANNEL);

#ifdef DEBUG
   static unsigned long ticks = 0;
   ticks++;
   // toggle an LED to indicate we are running
   if (! (ticks % (16*200))) gpio_tgl_gpio_pin(LED0_GPIO);
#endif

   // keep track of motor position
   if (m0_motorOn) {
       if (m0_motorDir) {
           --m0_motorPos;
       } else {
           ++m0_motorPos;
       }
   }
   // return now if already inside interrupt
   if (in) return;
   in = 1;
   
   // re-enable interrupts so we don't miss a count
   Enable_interrupt_level(0);

   if (m0_stepMode && ((long)((m0_motorPos - m0_stepNext) * (1 - 2 * (long)m0_motorDir)) > -2)) {
       switch (m0_stepMode) {
          case 1:   // ramp down in middle of ramping up
          case 2:   // ramp down from cruising
             m0_stepMode = 3;
             m0_stepNext = m0_stepTo;
             m0_rampTo = 60000; // = m0_actClock / m0_minSpeed
             m0_rampFlag = 2;
             break;
          case 3:   // time to stop
             m0_stepMode = 0;
             m0_rampFlag = 3;   // halt!
             break;
       }
   }
   if (m0_rampFlag) {
       m0_stopFlag = m0_rampFlag;
       m0_rampFlag = 0;
       if (m0_rampTo != m0_curRC || m0_stopFlag >= 2) {
           m0_curSpeed = (unsigned)(kClockFreq / (kPrescale * (long)m0_curRC));
           m0_endSpeed = (unsigned)(kClockFreq / (kPrescale * (long)m0_rampTo));
           m0_startSpeed = m0_curSpeed;
           if (m0_stopFlag == 3) {
        	   m0_rampEndTime = 0;
           } else if (m0_startSpeed < m0_endSpeed) {
               m0_rampEndTime = (long)(((int)m0_endSpeed - (int)m0_startSpeed) * m0_rampScl);
           } else {
               m0_rampEndTime = (long)(((int)m0_startSpeed - (int)m0_endSpeed) * m0_rampScl);
           }
           m0_rampTime = 0;
           m0_ramping = 1;
#ifdef DEBUG
           m0_latency = 0;
#endif
       }
	} else if (m0_ramping) {
	   m0_rampTime += m0_curRC;
	   if (m0_rampTime >= m0_rampEndTime) {
		   // stop TC if stopping motor
		   if (m0_stopFlag >= 2) {
		       if ((m0_stopFlag == 2) && m0_stepMode) {
		          // don't stop until we reach our end point
		          in = 0;
		          return;
		       }
		       tc_stop(&AVR32_TC, TC0_CHANNEL);
		       m0_curSpeed = kMinSpeed;
		       m0_running = 0;
               m0_stepMode = 0;
		   } else {
		       m0_curSpeed = m0_endSpeed;
		       if (m0_stepMode) {
		          m0_stepMode = 2;
		          // next mode is when we have to start ramping down
		          m0_stepNext = m0_stepTo - (m0_motorPos - m0_stepFrom);
		       }
		   }
		   m0_ramping = 0;
	   } else {
	       // weird: things go funny if the last m0_startSpeed is cast to (int).  why??
		   m0_curSpeed = (unsigned)((int)(((int)m0_endSpeed - (int)m0_startSpeed)
		               * (float)m0_rampTime / (float)m0_rampEndTime) + m0_startSpeed);
		   if (!m0_curSpeed) m0_curSpeed = 1;
	   }
	   m0_curRC = kClockFreq / (kPrescale * (unsigned long)m0_curSpeed);
	   // set RA/RC for new frequency
	   tc_write_ra(&AVR32_TC, TC0_CHANNEL, m0_curRC >> 1);
	   tc_write_rc(&AVR32_TC, TC0_CHANNEL, m0_curRC);
#ifdef DEBUG
	   unsigned int lat = tc_read_tc(&AVR32_TC, TC0_CHANNEL);
	   if (lat > m0_latency) m0_latency = lat;
#endif
	}
	
#ifdef DEBUG
	m0_lastCount = tc_read_tc(&AVR32_TC, TC0_CHANNEL);
#endif
    in = 0;
}

//-----------------------------------------------------------------------------
/*! \brief timer interrupt for motor
 */
__attribute__((__interrupt__))
static void m1_irq(void)
{
   static char in = 0;

   // clear the interrupt flag by reading the TC status register
   tc_read_sr(&AVR32_TC, TC1_CHANNEL);

#ifdef DEBUG
   static unsigned long ticks = 0;
   ticks++;
   // toggle an LED to indicate we are running
   if (! (ticks % (16*200))) gpio_tgl_gpio_pin(LED1_GPIO);
#endif

   // keep track of motor position
   if (m1_motorOn) {
       if (m1_motorDir) {
           --m1_motorPos;
       } else {
           ++m1_motorPos;
       }
   }
   // return now if already inside interrupt
   if (in) return;
   in = 1;
   
   // re-enable interrupts so we don't miss a count
   Enable_interrupt_level(0);

   if (m1_rampFlag) {
       m1_stopFlag = m1_rampFlag;
       m1_rampFlag = 0;
       if (m1_rampTo != m1_curRC || m1_stopFlag >= 2) {
           m1_curSpeed = (unsigned)(kClockFreq / (kPrescale * (long)m1_curRC));
           m1_endSpeed = (unsigned)(kClockFreq / (kPrescale * (long)m1_rampTo));
           m1_startSpeed = m1_curSpeed;
           if (m1_stopFlag == 3) {
        	   m1_rampEndTime = 0;
           } else if (m1_startSpeed < m1_endSpeed) {
               m1_rampEndTime = (long)(((int)m1_endSpeed - (int)m1_startSpeed) * m1_rampScl);
           } else {
               m1_rampEndTime = (long)(((int)m1_startSpeed - (int)m1_endSpeed) * m1_rampScl);
           }
           m1_rampTime = 0;
           m1_ramping = 1;
#ifdef DEBUG
           m1_latency = 0;
#endif
       }
	} else if (m1_ramping) {
	   m1_rampTime += m1_curRC;
	   if (m1_rampTime >= m1_rampEndTime) {
		   // stop TC if stopping motor
		   if (m1_stopFlag >= 2) {
		       tc_stop(&AVR32_TC, TC1_CHANNEL);
		       m1_curSpeed = kMinSpeed;
		       m1_running = 0;
		   } else {
		       m1_curSpeed = m1_endSpeed;
		   }
		   m1_ramping = 0;
	   } else {
	       // weird: things go funny if the last m1_startSpeed is cast to (int).  why??
		   m1_curSpeed = (unsigned)((int)(((int)m1_endSpeed - (int)m1_startSpeed)
		               * (float)m1_rampTime / (float)m1_rampEndTime) + m1_startSpeed);
		   if (!m1_curSpeed) m1_curSpeed = 1;
	   }
	   m1_curRC = kClockFreq / (kPrescale * (unsigned long)m1_curSpeed);
	   // set RA/RC for new frequency
	   tc_write_ra(&AVR32_TC, TC1_CHANNEL, m1_curRC >> 1);
	   tc_write_rc(&AVR32_TC, TC1_CHANNEL, m1_curRC);
#ifdef DEBUG
	   unsigned int lat = tc_read_tc(&AVR32_TC, TC1_CHANNEL);
	   if (lat > m1_latency) m1_latency = lat;
#endif
	}
	
#ifdef DEBUG
	m1_lastCount = tc_read_tc(&AVR32_TC, TC1_CHANNEL);
#endif
    in = 0;
}

//-----------------------------------------------------------------------------
/*! \brief timer interrupt for motor
 */
__attribute__((__interrupt__))
static void m2_irq(void)
{
   static char in = 0;

   // clear the interrupt flag by reading the TC status register
   tc_read_sr(&AVR32_TC, TC2_CHANNEL);

#ifdef DEBUG
   static unsigned long ticks = 0;
   ticks++;
   // toggle an LED to indicate we are running
   if (! (ticks % (16*200))) gpio_tgl_gpio_pin(LED2_GPIO);
#endif

   // keep track of motor position
   if (m2_motorOn) {
       if (m2_motorDir) {
           --m2_motorPos;
       } else {
           ++m2_motorPos;
       }
   }
   // return now if already inside interrupt
   if (in) return;
   in = 1;
   
   // re-enable interrupts so we don't miss a count
   Enable_interrupt_level(0);

   if (m2_rampFlag) {
       m2_stopFlag = m2_rampFlag;
       m2_rampFlag = 0;
       if (m2_rampTo != m2_curRC || m2_stopFlag >= 2) {
           m2_curSpeed = (unsigned)(kClockFreq / (kPrescale * (long)m2_curRC));
           m2_endSpeed = (unsigned)(kClockFreq / (kPrescale * (long)m2_rampTo));
           m2_startSpeed = m2_curSpeed;
           if (m2_stopFlag == 3) {
        	   m2_rampEndTime = 0;
           } else if (m2_startSpeed < m2_endSpeed) {
               m2_rampEndTime = (long)(((int)m2_endSpeed - (int)m2_startSpeed) * m2_rampScl);
           } else {
               m2_rampEndTime = (long)(((int)m2_startSpeed - (int)m2_endSpeed) * m2_rampScl);
           }
           m2_rampTime = 0;
           m2_ramping = 1;
#ifdef DEBUG
           m2_latency = 0;
#endif
       }
	} else if (m2_ramping) {
	   m2_rampTime += m2_curRC;
	   if (m2_rampTime >= m2_rampEndTime) {
		   // stop TC if stopping motor
		   if (m2_stopFlag >= 2) {
		       tc_stop(&AVR32_TC, TC2_CHANNEL);
		       m2_curSpeed = kMinSpeed;
		       m2_running = 0;
		   } else {
		       m2_curSpeed = m2_endSpeed;
		   }
		   m2_ramping = 0;
	   } else {
	       // weird: things go funny if the last m2_startSpeed is cast to (int).  why??
		   m2_curSpeed = (unsigned)((int)(((int)m2_endSpeed - (int)m2_startSpeed)
		               * (float)m2_rampTime / (float)m2_rampEndTime) + m2_startSpeed);
		   if (!m2_curSpeed) m2_curSpeed = 1;
	   }
	   m2_curRC = kClockFreq / (kPrescale * (unsigned long)m2_curSpeed);
	   // set RA/RC for new frequency
	   tc_write_ra(&AVR32_TC, TC2_CHANNEL, m2_curRC >> 1);
	   tc_write_rc(&AVR32_TC, TC2_CHANNEL, m2_curRC);
#ifdef DEBUG
	   unsigned int lat = tc_read_tc(&AVR32_TC, TC2_CHANNEL);
	   if (lat > m2_latency) m2_latency = lat;
#endif
	}
	
#ifdef DEBUG
	m2_lastCount = tc_read_tc(&AVR32_TC, TC2_CHANNEL);
#endif
    in = 0;
}

//-----------------------------------------------------------------------------
void setPin(int n, int val)
{
    if (output_mode[n] != 1) {
        switch (output_mode[n]) {
    	  case 3:
    	    gpio_enable_gpio_pin(n);
    	    // fall through!
          case 2:
    	    gpio_disable_pin_pull_up(n);
    	    break;
    	}
        output_mode[n] = 1;
    }
    if (val) {
        gpio_set_gpio_pin(n);
    } else {
        gpio_clr_gpio_pin(n);
    }
}

//-----------------------------------------------------------------------------
void resurfacer_task_init(void)
 {
   sof_cnt = 0;
   data_length = 0;
   has_data = 0;
 	  // Don't need this - PH
 	  //Usb_enable_sof_interrupt();
 }

//-----------------------------------------------------------------------------
void usb_sof_action(void)
{
	sof_cnt++;
}

//-----------------------------------------------------------------------------
// initialize the watchdog timer status LED
void wdt_init(void)
{
#if defined(MANIP) || defined(CUTE)
  if (AVR32_PM.RCAUSE.wdt) {
      setPin(LED3_GPIO, 0); // turn on LED3
  }
#endif
  wdt_disable();
}

//-----------------------------------------------------------------------------
// initialize the PWM output
// PWM channel initialization
//
#define PWM_CLK     187500   // PWM clock rate Hz (187500 = 12MHz / 64)
#define PWM_CHAN    6        // use PWM6
#define PWM_PIN     AVR32_PWM_6_2_PIN
#define PWM_FN      AVR32_PWM_6_2_FUNCTION
#define PWM_WID     2        // pulse width (clock ticks, must be less than kMinTop)

static pwm_opt_t pwm_opt = {
    // PWM controller configuration.
    .diva = AVR32_PWM_DIVA_CLK_OFF,
    .divb = AVR32_PWM_DIVB_CLK_OFF,
    .prea = AVR32_PWM_PREA_MCK,
    .preb = AVR32_PWM_PREB_MCK
};
static avr32_pwm_channel_t pwm_channel = {
//    .CMR.calg = PWM_MODE_LEFT_ALIGNED,       // Channel mode.
//    .CMR.cpol = PWM_POLARITY_LOW,            // Channel polarity.
//    .CMR.cpd = PWM_UPDATE_PERIOD,            // Not used the first time.
//    .CMR.cpre = AVR32_PWM_CPRE_MCK_DIV_64,   // Channel prescaler.
    .cdty = PWM_WID,// Channel duty cycle, should be < CPRD (2 --> 10 microsec)
    .cprd = 20,     // Channel period (set later)
    .cupd = 0,      // Channel update is not used here
    .ccnt = 0
    // With these settings, the output waveform rate will be : (12000000/64)/20
};

// run PWM at specified rate (0 = stop)
void pwm_spd(float rate)
{
	// (i can't figure out how to initialize these in the variable definition)
    pwm_channel.CMR.calg = PWM_MODE_LEFT_ALIGNED;       // Channel mode.
    pwm_channel.CMR.cpol = PWM_POLARITY_HIGH;           // Channel polarity.
    pwm_channel.CMR.cpd  = PWM_UPDATE_PERIOD;           // Not used the first time.
    pwm_channel.CMR.cpre = AVR32_PWM_CPRE_MCK_DIV_64;   // Channel prescaler.

    if (rate) {
        // initialize the PWM if necessary
        if (!pwm_flag) {
            pwm_init(&pwm_opt);
            pwm_flag = 1;   // (stopped)
        }
        // calculate PWM period
        unsigned long rcl = (unsigned long)(PWM_CLK / rate);
        if (rcl > 0xfffff) rcl = 0xfffff;   // cprd is 20 bits
        if (rcl < kMinTop) rcl = kMinTop;
        pwm_channel.cprd = rcl;
        if (pwm_flag == 1) { // stopped?
            // init this channel and start the PWM
            pwm_channel_init(PWM_CHAN, &pwm_channel);
            // enable PWM function pin if necessary
            if (output_mode[PWM_PIN] != 3) {
                output_mode[PWM_PIN] = 3;
                gpio_enable_module_pin(PWM_PIN, PWM_FN);
            }
            pwm_start_channels(1 << PWM_CHAN);  // start pwm
            pwm_flag = 2; // (running)
        } else {
            // already running, so just update the period
            AVR32_PWM.channel[PWM_CHAN].cupd = rcl; // (will update period on the next cycle)
        }
    } else if (pwm_flag == 2) {
        // drive the output pin low
        gpio_clr_gpio_pin(PWM_PIN); // (set the pin low first to try to avoid a transient pulse)
        gpio_enable_gpio_pin(PWM_PIN);
        gpio_clr_gpio_pin(PWM_PIN); // (must clear pin again, since enable_gpio_pin apparently resets this)
        output_mode[PWM_PIN] = 1;
        // stop the PWM
        pwm_stop_channels(1 << PWM_CHAN);
        pwm_flag = 1; // (stopped)
    }
}

//-----------------------------------------------------------------------------
// activate the watchdog timer
void wdt_scheduler(void)
{
  // If Reset Cause is due to a Watchdog reset just relaunch Watchdog and turn
  // on LED3 on to let user know that a new wdt reset has occured (MANIP/CUTE only).
  if (AVR32_PM.RCAUSE.wdt) {
      wdt_reenable();
      wdt_flag = 2;
  // If Reset Cause is due to a Power On or external reset...
//  } else if (AVR32_PM.RCAUSE.por || AVR32_PM.RCAUSE.ext) {
  } else {
      // Save current value in GPLP register (preserved across reset)
    //  pm_write_gplp(&AVR32_PM,0,current_wdt_value);
      // enable the watchdog timer
      wdt_enable(current_wdt_value);
      wdt_flag = 1;
  }
}

// read comma-separated channels from string
// returns error string, or NULL on success
char *getValues(int *chan, char *str, int num, int max)
{
    int i,j,n;
    for (i=0,j=0,n=0; ; ++j) {
        char ch = str[j];
        if (ch == ',' || !ch) {
            if (n >= max) return "value out of range";
            if (i >= num) return "too many channels";
            chan[i++] = n;
            n = 0;
            if (!ch) break;
        } else if (ch >= '0' && ch <= '9') {
            n = 10 * n + ch - '0';
        } else {
            return "invalid channel";
        }
    }
    return NULL;
}

// delay loop
// (apparently there is are delay_ms() and delay_us() functions defined
//  in delay.h that I could use instead of just looping here)
#if defined(MANIP) || defined(CUTE)
int delay(int del_num)
{
    int i, n;
    int dummy=0;
    if (del_num < kNumDelay) {
        n = cfg_del[del_num];
        for (i=0; i<n; ++i) ++dummy;
    }
    return dummy;
}
#endif

//-----------------------------------------------------------------------------
// this is the task that handles incoming commands over USB, executes them, and sends a response
void resurfacer_task()
{
    const int bsiz = 256;
    int len, pos, i, j, n, n2;
    char msg_buff[512];
    char cmd_buff[bsiz];
    static char out_buff[OUT_SIZE];
    static char buf[EP_SIZE_TEMP2];

    if (!Is_device_enumerated()) return;            // Check if USB HID is enumerated

    if (Is_usb_out_received(EP_TEMP_OUT)) {

    	// clear the watchdog timer because we are alive
       if (!wdt_flag) {
    	  wdt_scheduler();	// enable watchdog timer
   	   } else {
   	   	  wdt_clear();
       }

       Usb_reset_endpoint_fifo_access(EP_TEMP_OUT);
       len = Usb_byte_count(EP_TEMP_OUT);

       usb_read_ep_rxpacket(EP_TEMP_OUT, buf, len, NULL);
       Usb_ack_out_received_free(EP_TEMP_OUT);
       pos = 0;

       for (;;) {     // loop through commands
		  char *cmd = cmd_buff;
 		  char *dat;
          char *err = (char *)0;
		  char idx = '\0';
          int ok = 0;
          msg_buff[0] = '\0';

          for (;;) {    // (a cheap goto)
 			// read command
 			while (pos < len) {
 				char ch = buf[pos++];
 				if (ch == '\n' || ch == ';') break;   // semicolon is command terminator
 				*(cmd++) = ch;
 				if (cmd - cmd_buff >= bsiz) { err = "cmd too big"; break; }
 				if (pos >= len) { err = "no cmd"; break; }
 			}
 			if (err) break;
 			*cmd = '\0';

			// decode the command
 			cmd = strtok(cmd_buff, " ");
 			if (!cmd) { err = "no cmd"; break; }
 			dat = strtok(NULL, " ");
 			
 			if (cmd[0] && (cmd[1] == '.')) {
 			    idx = cmd[0];
 			    cmd += 2;   // skip index
 			} else {
 			    idx = '\0';
 			}

 			if (cmd[0]=='m' && cmd[1]>='0' && cmd[1]-'0'<NUM_MOTORS && !cmd[2]) {
 			
 			    // Command: m# CMD - motor commands
 				int mot_num = cmd[1] - '0';
 				cmd = dat;
 				dat = strtok(NULL, " ");
 				// motor commands
				if (!cmd || !strcmp(cmd,"stat")) {   // get motor status
				    int spd;
				    char dir;
				    long pos;
				    int src;
#ifdef DEBUG
                    int lat, count;
				    unsigned rc;
#endif
				    switch (mot_num) {
				      case 0:
                        spd   = (m0_running && m0_motorOn) ? (int)m0_curSpeed : 0;
                        dir   = m0_motorDir ? '-' : '+';
				        pos   = m0_motorPos;
				        src   = m0_src;
#ifdef DEBUG
				        rc    = m0_curRC;
				        lat   = m0_latency;
				        count = m0_lastCount;
#endif
                        break;
				      case 1:
                        spd   = (m1_running && m1_motorOn) ? (int)m1_curSpeed : 0;
                        dir   = m1_motorDir ? '-' : '+';
				        pos   = m1_motorPos;
				        src   = m1_src;
#ifdef DEBUG
				        rc    = m1_curRC;
				        lat   = m1_latency;
				        count = m1_lastCount;
#endif
                        break;
				      case 2:
                        spd   = (m2_running && m2_motorOn) ? (int)m2_curSpeed : 0;
                        dir   = m2_motorDir ? '-' : '+';
				        pos   = m2_motorPos;
				        src   = m2_src;
#ifdef DEBUG
				        rc    = m2_curRC;
				        lat   = m2_latency;
				        count = m2_lastCount;
#endif
                        break;
                    }
#ifdef DEBUG
                    sprintf(msg_buff, "m%d SPD=%c%d POS=%ld CLK=%d RC=%u LAT=%d CNT=%d",
						mot_num, dir, spd, pos, src, rc, lat, count);
#else
                    if (m0_stepMode && mot_num == 0) {
                       sprintf(msg_buff, "m%d SPD=%c%d POS=%ld MOD=%d NXT=%ld",
                            mot_num, dir, spd, pos, (int)m0_stepMode, m0_stepNext);
                    } else {
                       sprintf(msg_buff, "m%d SPD=%c%d POS=%ld CLK=%d",
						    mot_num, dir, spd, pos, src);
			        }
#endif
					ok = 1;
				} else if (!strcmp(cmd,"stop") || !strcmp(cmd,"ramp") || !strcmp(cmd,"step")) {
					int speed, step=0;
					long dest;
				    if (!strcmp(cmd,"stop")) {
					    speed = 0;
					} else if (!strcmp(cmd,"step")) {
                        if (m0_running) { err = "already running"; break; }
					    if (!dat) { err = "no destination"; break; }
    					if (!sscanf(dat, "%ld", &dest)) {
    					    err = "invalid destination";
    					    break;
    					}
    					dat = strtok(NULL, " ");
    					if (!dat) { err = "no speed"; break; }
    					if (!sscanf(dat, "%d", &speed)) {
    					    err = "invalid speed";
    					    break;
    					}
    					step = 1;
					} else {
					    if (!dat) { err = "no speed"; break; }
    					if (!sscanf(dat, "%d", &speed)) {
    					    err = "invalid speed";
    					    break;
    					}
					}
					m0_stepMode = 0;    // make sure step mode is off initially
                    unsigned char rampFlag = 1;
                    unsigned int rc;
                    unsigned long clock, rcl;
                    switch (mot_num) {
                      case 0:
                        if (speed <= 0) {
                            if (!m0_running) break;     // nothing to do if we aren't running
                            speed = m0_curSpeed < m0_minSpeed ? m0_curSpeed : m0_minSpeed;
                            ++rampFlag;
                        } else if (!m0_motorOn) {
                            err = "m0 is not on";
                            break;
                        } else if (step) {
                            if (dest == m0_motorPos) {
                                err = "at destination";
                                break;
                            }
                            unsigned char dir = (dest - m0_motorPos > 0) ? 0 : 1;
                            if (m0_motorDir != dir) {
                                m0_motorDir = dir;
                                setPin(sMotor[0].dir, m0_motorDir ^ sMotor[0].dirInv);
                            }
                            m0_stepMode = 1;
                            m0_stepFrom = m0_motorPos;
                            m0_stepTo = dest;
                            m0_stepNext = (m0_stepTo + m0_stepFrom) / 2;
                        }
                        clock = m0_actClock;
                        rcl = clock / speed;
                        if (rcl > 0xffff) rcl = 0xffff;
                        rc = (unsigned int)rcl;
                        if (rc < kMinTop) rc = kMinTop;
                        m0_rampTo = rc;
                        m0_rampFlag = rampFlag;
                        if (!m0_running) {
                            m0_running = 1;
                            tc_start(&AVR32_TC, TC0_CHANNEL);   // Start the timer/counter
                        }
                        ok = 1;
                        break;
                      case 1:
                        if (speed <= 0) {
                            if (!m1_running) break;     // nothing to do if we aren't running
                            speed = m1_curSpeed < m1_minSpeed ? m1_curSpeed : m1_minSpeed;
                            ++rampFlag;
                        } else if (!m1_motorOn) {
                            err = "m1 is not on";
                            break;
                        } else if (step) {
                            err = "m1 doesn't step";
                            break;
                        }
                        clock = m1_actClock;
                        rcl = clock / speed;
                        if (rcl > 0xffff) rcl = 0xffff;
                        rc = (unsigned int)rcl;
                        if (rc < kMinTop) rc = kMinTop;
                        m1_rampTo = rc;
                        m1_rampFlag = rampFlag;
                        if (!m1_running) {
                            m1_running = 1;
                            tc_start(&AVR32_TC, TC1_CHANNEL);   // Start the timer/counter
                        }
                        ok = 1;
                        break;
                      case 2:
                        if (speed <= 0) {
                            if (!m2_running) break;     // nothing to do if we aren't running
                            speed = m2_curSpeed < m2_minSpeed ? m2_curSpeed : m2_minSpeed;
                            ++rampFlag;
                        } else if (!m2_motorOn) {
                            err = "m2 is not on";
                            break;
                        } else if (step) {
                            err = "m2 doesn't step";
                            break;
                        }
                        clock = m2_actClock;
                        rcl = clock / speed;
                        if (rcl > 0xffff) rcl = 0xffff;
                        rc = (unsigned int)rcl;
                        if (rc < kMinTop) rc = kMinTop;
                        m2_rampTo = rc;
                        m2_rampFlag = rampFlag;
                        if (!m2_running) {
                            m2_running = 1;
                            tc_start(&AVR32_TC, TC2_CHANNEL);   // Start the timer/counter
                        }
                        ok = 1;
                        break;
                    }
                    if (ok) {
                        speed = (int)(clock / rc);
                        sprintf(msg_buff,"m%d RAMP=%d (rc=%u)",mot_num,speed,rc);
                    } else if (!err) {
                        // the do-nothing response
                        sprintf(msg_buff,"m%d RAMP=0",mot_num);
                        ok = 1;
                    }
                } else if (!strcmp(cmd,"spd")) {        // run motor at specified speed
					float speed;
					if (!dat) { err = "no speed"; break; }
                    if (!sscanf(dat, "%f", &speed)) {
                        err = "invalid speed";
                        break;
                    }
                    char *pt = strtok(NULL," ");
                    int src = 0;
                    if (pt) {
                        src = atoi(pt);
                        if (src<1 || src>5) { err = "bad clk"; break; }
                    }
                    unsigned int rc;
                    unsigned long clock, rcl;
                    int stopped = 0;
                    switch (mot_num) {
                      case 0:
                        if (speed <= 0) {
                            if (m0_running) {
                                tc_stop(&AVR32_TC, TC0_CHANNEL);
                                m0_running = 0;
                            }
		                    speed = kMinSpeed;
                            stopped = 1;
                        } else if (!m0_motorOn) {
                            err = "m0 is not on";
                            break;
                        }
                        if (!src) {
                            src = m0_src;
                        } else if (m0_src != src) {
                            if (m0_running) {
                                tc_stop(&AVR32_TC, TC0_CHANNEL);    // stop the tc before initializing
                                m0_running = 0;
                            }
                            m0_src = src;
                            waveform_opt[0].tcclks = motor_src[src-1];
	                        tc_init_waveform(&AVR32_TC, &waveform_opt[0]);  // re-initialize the timer/counter waveform.
                        }
                        clock = motor_actClock[src-1];
                        rcl = (long)(clock / speed + 0.5);
                        if (rcl > 0xffff) rcl = 0xffff;
                        rc = (unsigned int)rcl;
                        if (rc < kMinTop) rc = kMinTop;
                        speed = clock / (float)rc;
                        m0_curRC = rc;
                        m0_curSpeed = (unsigned)speed;
                        m0_rampFlag = 0;
                        m0_ramping = 0;
	                    tc_write_ra(&AVR32_TC, TC0_CHANNEL, rc >> 1);
	                    tc_write_rc(&AVR32_TC, TC0_CHANNEL, rc);
                        if (!m0_running && !stopped) {
                            m0_running = 1;
                            tc_start(&AVR32_TC, TC0_CHANNEL);   // Start the timer/counter
                        }
                        ok = 1;
                        break;
                      case 1:
                        if (speed <= 0) {
                            if (m1_running) {
                                tc_stop(&AVR32_TC, TC1_CHANNEL);
                                m1_running = 0;
                            }
		                    speed = kMinSpeed;
                            stopped = 1;
                        } else if (!m1_motorOn) {
                            err = "m1 is not on";
                            break;
                        }
                        if (!src) {
                            src = m1_src;
                        } else if (m1_src != src) {
                            if (m1_running) {
                                tc_stop(&AVR32_TC, TC1_CHANNEL);    // stop the tc before initializing
                                m1_running = 0;
                            }
                            m1_src = src;
                            waveform_opt[1].tcclks = motor_src[src-1];
	                        tc_init_waveform(&AVR32_TC, &waveform_opt[1]);  // re-initialize the timer/counter waveform.
                        }
                        clock = motor_actClock[src-1];
                        rcl = (long)(clock / speed + 0.5);
                        if (rcl > 0xffff) rcl = 0xffff;
                        rc = (unsigned int)rcl;
                        if (rc < kMinTop) rc = kMinTop;
                        speed = clock / (float)rc;
                        m1_curRC = rc;
                        m1_curSpeed = (unsigned)speed;
                        m1_rampFlag = 0;
                        m1_ramping = 0;
	                    tc_write_ra(&AVR32_TC, TC1_CHANNEL, rc >> 1);
	                    tc_write_rc(&AVR32_TC, TC1_CHANNEL, rc);
                        if (!m1_running && !stopped) {
                            m1_running = 1;
                            tc_start(&AVR32_TC, TC1_CHANNEL);   // Start the timer/counter
                        }
                        ok = 1;
                        break;
                      case 2:
                        if (speed <= 0) {
                            if (m2_running) {
                                tc_stop(&AVR32_TC, TC2_CHANNEL);
                                m2_running = 0;
                            }
		                    speed = kMinSpeed;
                            stopped = 1;
                        } else if (!m2_motorOn) {
                            err = "m2 is not on";
                            break;
                        }
                        if (!src) {
                            src = m2_src;
                        } else if (m2_src != src) {
                            if (m2_running) {
                                tc_stop(&AVR32_TC, TC2_CHANNEL);    // stop the tc before initializing
                                m2_running = 0;
                            }
                            m2_src = src;
                            waveform_opt[2].tcclks = motor_src[src-1];
	                        tc_init_waveform(&AVR32_TC, &waveform_opt[2]);  // re-initialize the timer/counter waveform.
                        }
                        clock = motor_actClock[src-1];
                        rcl = (long)(clock / speed + 0.5);
                        if (rcl > 0xffff) rcl = 0xffff;
                        rc = (unsigned int)rcl;
                        if (rc < kMinTop) rc = kMinTop;
                        speed = clock / (float)rc;
                        m2_curRC = rc;
                        m2_curSpeed = (unsigned)speed;
                        m2_rampFlag = 0;
                        m2_ramping = 0;
	                    tc_write_ra(&AVR32_TC, TC2_CHANNEL, rc >> 1);
	                    tc_write_rc(&AVR32_TC, TC2_CHANNEL, rc);
                        if (!m2_running && !stopped) {
                            m2_running = 1;
                            tc_start(&AVR32_TC, TC2_CHANNEL);   // Start the timer/counter
                        }
                        ok = 1;
                        break;
                    }
                    if (ok) {
                        if (stopped) {
                            sprintf(msg_buff,"m%d STOPPED (clk=%d)",mot_num,src);
                        } else {
                            sprintf(msg_buff,"m%d SPD=%.6g (rc=%u)",mot_num,speed,rc);
                        }
                    } else if (!err) {
                        // the do-nothing response
                        sprintf(msg_buff,"m%d SPD=0",mot_num);
                        ok = 1;
                    }
                } else if (!strcmp(cmd,"halt")) {       // halt motor immediately
                    switch (mot_num) {
                      case 0:
                        m0_rampTo = (unsigned int)(m0_actClock / m0_minSpeed);
                        m0_rampFlag = 3;
                        break;
                      case 1:
                        m1_rampTo = (unsigned int)(m1_actClock / m1_minSpeed);
                        m1_rampFlag = 3;
                        break;
                      case 2:
                        m2_rampTo = (unsigned int)(m2_actClock / m2_minSpeed);
                        m2_rampFlag = 3;
                        break;
                    }
                    sprintf(msg_buff,"m%d HALTED",mot_num);
                    ok = 1;
				} else if (!strcmp(cmd,"dir")) {        // get/set motor direction flag
				    int n = sMotor[mot_num].dir;
                    if (dat) {
				        unsigned char *dirPt;
                        switch (mot_num) {
                          case 0:
                            dirPt = &m0_motorDir;
                            break;
                          case 1:
                            dirPt = &m1_motorDir;
                            break;
                          case 2:
                            dirPt = &m2_motorDir;
                            break;
                        }
                        if (dat[0] == '0' || dat[0] == '1') {
                            int val = dat[0] - '0';
                            setPin(n, val ^ sMotor[mot_num].dirInv);
                            *dirPt = val;
                        } else if (dat[0] == '+' || dat[0] == '-') {
                            sMotor[mot_num].dirInv = (dat[0] == '+' ? 0 : 1);
                            setPin(n, *dirPt ^ sMotor[mot_num].dirInv);
                        } else {
                            err = "must set to 0, 1, + or -";
                            break;
                        }
                    } else {
                        int val = gpio_get_pin_value(n);
                        char *inv = sMotor[mot_num].dirInv ? " (inv)" : "";
                        sprintf(msg_buff,"pa%d VAL=%d%s", n, val, inv);
                    }
                    ok = 1;
				} else if (!strcmp(cmd,"on")) {     // get/set motor on status
				    int n = sMotor[mot_num].on;
                    if (dat) {
                        unsigned char *onPt;
                        switch (mot_num) {
                          case 0:
                            onPt = &m0_motorOn;
                            break;
                          case 1:
                            onPt = &m1_motorOn;
                            break;
                          case 2:
                            onPt = &m2_motorOn;
                            break;
                        }
                        if (dat[0] == '0' || dat[0] == '1') {
                            int val = dat[0] - '0';
                            setPin(n, val ^ sMotor[mot_num].onInv);
                            *onPt = val;
                        } else if (dat[0] == '+' || dat[0] == '-') {
                            sMotor[mot_num].onInv = (dat[0] == '+' ? 0 : 1);
                            setPin(n, *onPt ^ sMotor[mot_num].onInv);
                        } else {
                            err = "must set to 0, 1, + or -";
                            break;
                        }
                    } else {
                        int val = gpio_get_pin_value(n);
                        char *inv = sMotor[mot_num].onInv ? " (inv)" : "";
                        sprintf(msg_buff,"pa%d VAL=%d%s", n, val, inv);
                    }
                    ok = 1;
				} else if (!strcmp(cmd,"pos")) {    // get set motor position
					long pos;
					if (!dat) {
						switch (mot_num) {
						  case 0:
						    pos = m0_motorPos;
						    break;
						  case 1:
						    pos = m1_motorPos;
						    break;
						  case 2:
						    pos = m2_motorPos;
						    break;
						}
						sprintf(msg_buff,"m%d POS=%ld",mot_num,pos);
						ok = 1;
					} else if (sscanf(dat, "%ld", &pos) > 0) {
                        switch (mot_num) {
                          case 0:
                            m0_motorPos = pos;
                            break;
                          case 1:
                            m1_motorPos = pos;
                            break;
                          case 2:
                            m2_motorPos = pos;
                            break;
                        }
                        sprintf(msg_buff,"m%d POS=%ld",mot_num,pos);
                        ok = 1;
                    }
				} else if (!strcmp(cmd,"acc")) {    // get/set acceleration
					unsigned int acc;
					if (!dat) {
						switch (mot_num) {
						  case 0:
						    acc = m0_acc;
						    break;
						  case 1:
						    acc = m1_acc;
						    break;
						  case 2:
						    acc = m2_acc;
						    break;
						}
						sprintf(msg_buff,"m%d ACC=%u",mot_num,acc);
						ok = 1;
					} else if (sscanf(dat, "%u", &acc) > 0) {
					    if (acc < kMotorAccMin) acc = kMotorAccMin;
					    if (acc > kMotorAccMax) acc = kMotorAccMax;
                        switch (mot_num) {
                          case 0:
                            m0_acc = acc;
                            m0_rampScl = (float)(kClockFreq / kPrescale) / acc;
                            break;
                          case 1:
                            m1_acc = acc;
                            m1_rampScl = (float)(kClockFreq / kPrescale) / acc;
                            break;
                          case 2:
                            m2_acc = acc;
                            m2_rampScl = (float)(kClockFreq / kPrescale) / acc;
                            break;
                        }
                        sprintf(msg_buff,"m%d ACC=%u",mot_num,acc);
                        ok = 1;
                    }
				}
			} else if (cmd[0]=='p' && (cmd[1]=='a' || cmd[1]=='b')) {

                // Command: pa# [0|1|-|+] - input/output digital value
                if (!isdigit(cmd[2])) break;
                n = cmd[2] - '0';
                i = 3;
                if (isdigit(cmd[i])) {
                    n = n * 10 + cmd[i] - '0';
                    ++i;
                }
                if (cmd[i]) {
                    if (cmd[i] != '-') break;
                    ++i;
                    if (!isdigit(cmd[i])) break;
                    n2 = cmd[i] - '0';
                    ++i;
                    if (isdigit(cmd[i])) {
                        n2 = n2 * 10 + cmd[i] - '0';
                        ++i;
                    }
                    if (cmd[i]) break;
                } else {
                    n2 = n;
                }
                if (cmd[1]=='b') { n += 32; n2 += 32; }
                if (n >= IO_CHANNELS || n2 >= IO_CHANNELS) { err = "channel out of range"; break; }
                ok = 1;
                if (dat) {
                    j = 0;
                    for (;;) {
                        if (dat[j] == '0') {
                            setPin(n, 0);
                        } else if (dat[j] == '1') {
                            setPin(n, 1);
                        } else if (dat[j] == '-') {
                            gpio_enable_gpio_pin(n);// gpio module controls pin (also enables output driver, which we don't want)
                            gpio_local_disable_pin_output_driver(n);
                            gpio_disable_pin_pull_up(n);
                            output_mode[n] = 0;
                        } else if (dat[j] == '+') {
                            gpio_enable_gpio_pin(n);// gpio module controls pin (also enables output driver, which we don't want)
                            gpio_local_disable_pin_output_driver(n);
                            gpio_enable_pin_pull_up(n);
                            output_mode[n] = 2;
                        } else {
                            err = "must set to 0, 1, - or +";
                            ok = 0;
                            break;
                        }
                        if (n == n2) break;
                        if (n < n2) {
                            ++n;
                        } else {
                            --n;
                        }
                        if (dat[j+1]) ++j;
                    }
                } else {
                    char val_str[256];
                    if (n == n2) {
                        sprintf(val_str, "%d", gpio_get_pin_value(n));
                        switch (output_mode[n]) {
                          case 1:
                            strcat(val_str, " (output)");
                            break;
                          case 2:
                            strcat(val_str, " (pull up)");
                            break;
                          case 3:
                            strcat(val_str, " (function)");
                            break;
                        }
                    } else {
                        j = 0;
                        for (i=n;;) {
                            j += sprintf(val_str + j, "%d", gpio_get_pin_value(i));
                            if (i == n2) break;
                            if (i < n2) {
                                ++i;
                            } else {
                                --i;
                            }
                            // put a space every 8 bits
                            if (!((i-n)&0x07)) {
                            	strcpy(val_str+j, " ");
                            	++j;
                            }
                        }
                    }
                    char c;
                    if (n >= 32 && n2 >= 32) {
                        n -= 32;
                        n2 -= 32;
                        c = 'b';
                    } else {
                        c = 'a';
                    }
                    if (n2 == n) {
                        sprintf(msg_buff,"p%c%d VAL=%s", c, n, val_str);
                    } else {
                        sprintf(msg_buff,"p%c%d-%d VAL=%s", c, n, n2, val_str);
                    }
                }

#if defined(MANIP) || defined(CUTE)
            } else if (cmd[0]=='c' && cmd[1]>='0' && cmd[1]<='3' && !cmd[2]) {

                // Command: c## - read Steve's encoder counter (SNO+)
                int brd = cmd[1] - '0';
                unsigned count = 0;
                setPin(DEV0, 1);    // device 1
                setPin(BRD0, brd & 0x01); // address the board
                setPin(BRD1, brd & 0x02);
                setPin(BRDSEL, 1);  // select the board
                setPin(XRD, 0);     // read data
                delay(0);           // wait for data to stabilize
                // read high byte
                for (i=0; i<kNumDatLines; ++i) {
                    count |= (gpio_get_pin_value(cfg_dat[i]) << (8+i));
                }
                setPin(BYSEL, 1);   // select low byte
                delay(1);           // wait for data to stabilize
                // read low byte
                for (i=0; i<kNumDatLines; ++i) {
                    count |= (gpio_get_pin_value(cfg_dat[i]) << i);
                }
                // return outputs to their defaults
                setPin(XRD, 1);     // completes the inihibit logic
                setPin(BRDSEL, 0);
                setPin(DEV0, 0);
                setPin(BYSEL, 0);
                sprintf(msg_buff,"%s VAL=%u (0x%.4x)",cmd,count,count);
                ok = 1;

            } else if (cmd[0]=='a' && cmd[1]>='0' && cmd[1]<='3' &&
                       cmd[2]>='0' && cmd[2]<='7' && (!cmd[3] || !cmd[4]))
            {
                // Command: a## - read Steve's MAX197 12-bit adc (SNO+)
                int brd = cmd[1] - '0';
                int adc = cmd[2] - '0';
                int rng = cmd[3] & 0x03;
                unsigned count = 0;
                
                setPin(BRD0, brd & 0x01);   // address the board
                setPin(BRD1, brd & 0x02);
                // set the MAX197 control byte
                // bits 0-2 - adc select
                for (i=0; i<3; ++i) {
                    if (adc & (1 << i)) setPin(WDAT+i, 1);
                }
                // bit 3    - 0=unipolar, 1=bipoloar operation
                // bit 4    - 0=5V range, 1=10V range
                if (rng) {
                    if (rng & 0x01) setPin(WDAT+3, 1);
                    if (rng & 0x02) setPin(WDAT+4, 1);
                }
                // bit 5    - 0 (0=internally controlled acquisition)
                // bits 6-7 - 0/0 (0/0=normal operation/external clock)
                setPin(BRDSEL, 1);  // select the board
                setPin(XWR, 0);     // write the control register (initiates conversion)
                setPin(XWR, 1);
                // wait for conversion (INT goes low)
                for (i=0; ; ++i) {
                    if (gpio_get_pin_value(INT)) {
                        if (i < kMaxWaitConv) continue;
                        err = "conversion error";
                    }
                    break;
                }
                setPin(XRD, 0);     // read data
                delay(4);           // wait for data to stabilize
                // read low byte
                for (i=0; i<kNumDatLines; ++i) {
                    count |= (gpio_get_pin_value(cfg_dat[i]) << i);
                }
                setPin(BYSEL, 1);   // select high byte
                delay(5);           // wait a bit
                // read high byte
                for (i=0; i<kNumDatLines; ++i) {
                    count |= (gpio_get_pin_value(cfg_dat[i]) << (8+i));
                }
                // return outputs to their defaults
                setPin(XRD, 1);
                setPin(BRDSEL, 0);
                setPin(BYSEL, 0);
                for (i=0; i<3; ++i) {
                    if (adc & (1 << i)) setPin(WDAT+i, 0);
                }
                if (rng) {
                    if (rng & 0x01) setPin(WDAT+3, 0);
                    if (rng & 0x02) setPin(WDAT+4, 0);
                }
                sprintf(msg_buff,"%s VAL=%u (0x%.4x)",cmd,count,count);
                ok = 1;

            } else if (cmd[0]=='d' && cmd[1]>='0' && cmd[1]<='3' &&
                (!cmd[2] || (cmd[2]>='0' && cmd[2]<='7' && !cmd[3]))) {

                // Command: d# or d## - read/write Steve's digital i/o (SNO+)
                int brd = cmd[1] - '0';
                int bit = cmd[2];
                unsigned val = 0;
                setPin(BRD0, brd & 0x01);   // address the board
                setPin(BRD1, brd & 0x02);
                if (dat) {
                    // write output bits (output device = 0)
                    val = atoi(dat);
                    if (bit) {
                        bit -= '0';
                        int mask = (1 << bit);
                        if (val) {
                            val = (dig_out[brd] | mask);
                        } else {
                            val = (dig_out[brd] & ~mask);
                        }
                    }
                    dig_out[brd] = val;
                    // set the state of all high output bits
                    for (i=0; i<8; ++i) {
                        if (val & (1 << i)) setPin(WDAT+i, 1);
                    }
                    setPin(BRDSEL, 1);  // select the board
                    setPin(XWR, 0);     // write the bits
                    setPin(XWR, 1);
                    setPin(BRDSEL, 0);  // deselect the board
                    // return write lines to their defaults
                    for (i=0; i<8; ++i) {
                        if (val & (1 << i)) setPin(WDAT+i, 0);
                    }

                } else {
                    // read input bit(s)
                    setPin(DEV0, 1);    // input device = 1
                    setPin(BRDSEL, 1);  // select the board
                    setPin(XRD, 0);     // read data
                    if (bit) {
                        // read one bit
                        bit -= '0';
                        val = gpio_get_pin_value(cfg_dat[bit]);
                        sprintf(msg_buff,"%s VAL=%u",cmd,val);
                    } else {
                        // read all 8 bits
                        for (i=0; i<8; ++i) {
                            val |= (gpio_get_pin_value(cfg_dat[i]) << i);
                        }
                        sprintf(msg_buff,"%s VAL=%u (0x%.4x)",cmd,val,val);
                    }
                    setPin(XRD, 1);     // complete the read
                    setPin(BRDSEL, 0);  // deselect the board
                    setPin(DEV0, 0);
                }
                ok = 1;

            } else if (cmd[0]=='s' && cmd[1]>='0' && cmd[1]<='3' &&
                (!cmd[2] || (cmd[2]>='0' && cmd[2]<='3' && !cmd[3]))) {

                // Command: s# or s## - read Steve's switches (SNO+)
                int brd = cmd[1] - '0';
                int bit = cmd[2];
                unsigned val = 0;
                
                setPin(BRD0, brd & 0x01);   // address the board
                setPin(BRD1, brd & 0x02);
                setPin(DEV0, 1);
                setPin(DEV1, 1);
                setPin(BRDSEL, 1);  // select the board
                if (bit) {
                    // read one switch
                    bit -= '0';
                    val = gpio_get_pin_value(cfg_dat[bit]);
                    sprintf(msg_buff,"%s VAL=%u",cmd,val);
                } else {
                    // read all switches (low 4 bits)
                    for (i=0; i<4; ++i) {
                        val |= (gpio_get_pin_value(cfg_dat[i]) << i);
                    }
                    sprintf(msg_buff,"%s VAL=%u (0x%.4x)",cmd,val,val);
                }
                // return outputs to their defaults
                setPin(BRDSEL, 0);  // deselect the board
                setPin(DEV0, 0);
                setPin(DEV1, 0);
                ok = 1;

            } else if (!strcmp(cmd, "cfg")) {

                // Command: cfg OPTS - configure up/down counter and ADC i/o (SNO+)
                if (dat) {
                    while (dat) {
                        char *e;
                        if (strlen(dat) < 2 || dat[1] != '=') {
                            err = "invalid argument";
                            break;
                        }
                        if (dat[0] == 'a') {
                        	e = getValues(cfg_adr, dat+2, kNumAdrLines, IO_CHANNELS);
                            if (e) err = e;
                        } else if (dat[0] == 'd') {
                            e = getValues(cfg_dat, dat+2, kNumDatLines, IO_CHANNELS);
                            if (e) err = e;
                        } else if (dat[0] == 'x') {
                            e = getValues(cfg_del, dat+2, kNumDelay, 0x7fffffff);
                            if (e) err = e;
                        } else {
                            err = "unknown argument";
                        }
                        dat = strtok(NULL," ");
                    }
                } else {
                    for (i=0, n=0; i<kNumAdrLines; ++i) {
                        n += sprintf(msg_buff+n, "%s%d", (i ? "," : "A="), cfg_adr[i]);
                    }
                    for (i=0; i<kNumDatLines; ++i) {
                        n += sprintf(msg_buff+n, "%s%d", (i ? "," : " D="), cfg_dat[i]);
                    }
                    for (i=0; i<kNumDelay; ++i) {
                        n += sprintf(msg_buff+n, "%s%d", (i ? "," : " X="), cfg_del[i]);
                    }
                }
                ok = 1;
#endif // MANIP

            } else if (cmd[0]=='p' && cmd[1]>='0' && cmd[1]<='6' && !cmd[2]) {

                int pwm_num = cmd[1] - '0';
                if (pwm_num != 6) { err = "invalid pwm"; break; }   // only support PWM6 for now
 				cmd = dat;
 				dat = strtok(NULL, " ");
                if (!cmd || !strcmp(cmd,"stat")) {  // get PWM status
                    float spd = pwm_flag == 2 ? ((float)PWM_CLK / pwm_channel.cprd) : 0;
                    sprintf(msg_buff, "p%d SPD=%.6g", pwm_num, spd);
                    ok = 1;
 				} else {
 				    if (!strcmp(cmd,"spd")) {
                        if (!dat) { err = "no speed"; break; }
                        pwm_spd(atof(dat));
                        ok = 1;
                    } else if (!strcmp(cmd,"halt") || !strcmp(cmd,"stop")) {
                        pwm_spd(0);
                        ok = 1;
                    }
                    if (pwm_flag == 2) {
                        float spd = (float)PWM_CLK / pwm_channel.cprd;
                        sprintf(msg_buff,"p%d SPD=%.6g (rc=%lu)", pwm_num, spd, pwm_channel.cprd);
                    } else {
                        sprintf(msg_buff,"p%d STOPPED", pwm_num);
                    }
                }

            } else if (cmd[0]=='a' && cmd[1]=='d' && cmd[2]=='c' &&
                       cmd[3]>='0' && cmd[3]-'0'<NUM_ADCS && !cmd[4]) {

                // Command: adc# - read specified ADC
                short n = cmd[3] - '0';
                short chan = sADC[n].channel;
                short pin = sADC[n].pin;
                // enable ADC if not done already
                if (output_mode[pin] != 3) {
                    gpio_enable_module_pin(pin, sADC[i].function);
                    adc_enable(&AVR32_ADC, chan);
                    output_mode[pin] = 3;
                }
                // read and discard old value if necessary
                if (adc_check_eoc(&AVR32_ADC, chan) == HIGH) {
                    adc_get_value(&AVR32_ADC, chan);
                }
                // start new conversion
                adc_start(&AVR32_ADC);
                signed val = adc_get_value(&AVR32_ADC, chan);
                sprintf(msg_buff,"%s VAL=%d", cmd, val);
                ok = 1;
			        
			} else if (!strcmp(cmd,"halt")) {

                // Command: halt - stop all motors immediately
                m0_rampTo = (unsigned int)(m0_actClock / m0_minSpeed);
                m0_rampFlag = 3;
                m1_rampTo = (unsigned int)(m1_actClock / m1_minSpeed);
                m1_rampFlag = 3;
                m2_rampTo = (unsigned int)(m2_actClock / m2_minSpeed);
                m2_rampFlag = 3;
                strcpy(msg_buff, "HALTED");
                ok = 1;

            } else if (!strcmp(cmd,"ser")) {

                // Command: ser - get serial number
                volatile unsigned int *id = (unsigned int *)0x80800204; 
                sprintf(msg_buff, "%.8x%.8x%.8x%.6x",id[0],id[1],id[2],id[3]>>8);
            	ok = 1;

            } else if (!strcmp(cmd,"help")) {

                // Command: help - show command help
                strcpy(msg_buff, "Available commands:\n"
#ifdef MANIP
                                 "pa#; pb#; adc#; a##; c#; d#[#]; s#[#]; cfg\n"
#else
                                 "pa#; pb#; adc#\n"
#endif
                                 "m# [ramp,spd,stop,halt,stat,pos,on,dir,acc]\n"
                                 "p# [spd,stop,halt,stat]; nop; ver; ser; help");
            	ok = 1;

            } else if (!strcmp(cmd,"wdt")) {

                // Command: wdt - get/set watchdog timer
                int secs;
                if (dat) {
                    wdt_disable();
                    wdt_flag = 1;
#if defined(MANIP) || defined(CUTE)
                    setPin(LED3_GPIO, 1);   // make sure LED3 is off
#endif
                    secs = atoi(dat);
                    current_wdt_value = secs * 1000000L;
                    if (secs) wdt_enable(current_wdt_value);
                } else {
                    secs = (int)(current_wdt_value / 1000000L);
                }
                if (secs) {
                    char *rmsg = wdt_flag == 2 ? " (RESET OCCURRED!)" : "";
                    sprintf(msg_buff, "WDT set to %d seconds%s", secs, rmsg);
                } else {
                    strcpy(msg_buff, "WDT disabled");
                }
        		ok = 1;
 			} else if (!strcmp(cmd,"ver")) {
#ifdef MANIP
 				sprintf(msg_buff, "Version %.2f (SNO+ MANIP)", VERSION);
#elif defined(CUTE)
                sprintf(msg_buff, "Version %.2f (CUTE)", VERSION);
#else
                sprintf(msg_buff, "Version %.2f (DEAP)", VERSION);
#endif
 				ok = 1;
 			} else if (!strcmp(cmd,"nop")) {
 			    // Command: nop - do nothing
 			    ok = 1;
 			}
 			break;
 		  }
 		  if (err || !ok) {
 		     ok = 0;
 		     if (!err) err = "unknown cmd";
 		     strcpy(msg_buff, err);
 		  }
 		  // add this response to the returned message
 		  n = strlen(msg_buff);
 		  // (save room for "X.BAD " header and "\0" terminator)
 		  if (data_length + n + 7 < OUT_SIZE) {
             // prefix response with command index if provided
             if (idx) {
                out_buff[data_length++] = idx;
                out_buff[data_length++] = '.';
             }
             if (ok) {
                strcpy(out_buff + data_length, "OK");  data_length += 2;
             } else {
			    strcpy(out_buff + data_length, "BAD"); data_length += 3;
             }
             if (n) {
                out_buff[data_length++] = ' ';
                memcpy(out_buff + data_length, msg_buff, n);  data_length += n;
             }
             out_buff[data_length++] = '\n';
             out_buff[data_length] = '\0';  // null-terminate response
             has_data = 1;
 		  }
 		  if (pos >= len) break;
 	   }
    }

    // Load the IN endpoint with the command response
    // (max PKT_SIZE bytes per packet)
    if (has_data && Is_usb_in_ready(EP_TEMP_IN))
    {
        if (data_length < PKT_SIZE-1) {
            n = data_length + 1;    // also send terminator (not counted in data_length)
            data_length = 0;
            has_data = 0;
        } else if (data_length == PKT_SIZE-1) {
        	// this is really weird, and probably a bug in the AVR32 USB library, but
        	// the write will fail if we send only one packet with a length of exactly
        	// 64 bytes, so in this special case send the terminator separately
        	n = data_length;
        	data_length = 0;
        } else {
            n = PKT_SIZE;
            data_length -= PKT_SIZE;
        }
        Usb_reset_endpoint_fifo_access(EP_TEMP_IN);
        usb_write_ep_txpacket(EP_TEMP_IN, out_buff, n, NULL);
        Usb_ack_in_ready_send(EP_TEMP_IN);
        // move data to start of buffer if we have more to send
        if (has_data) memmove(out_buff, out_buff+n, data_length+1);
    }

    return;

}

//-----------------------------------------------------------------------------
/*! \brief Main function. Execution starts here.
 *
 * \retval 42 Fatal error.
 */
int main(void)
{
    // The timer/counter instance and channel number are used in several functions.
    // It's defined as local variable for ease-of-use causes and readability.
    volatile avr32_tc_t *tc = &AVR32_TC;
    
    // Options for waveform generation.
    static const tc_interrupt_t tc_interrupt[NUM_MOTORS] = {
    {
	    .etrgs = 0,
	    .ldrbs = 0,
	    .ldras = 0,
	    .cpcs  = 1,
	    .cpbs  = 0,
	    .cpas  = 0,
	    .lovrs = 0,
	    .covfs = 0
	},{
	    .etrgs = 0,
	    .ldrbs = 0,
	    .ldras = 0,
	    .cpcs  = 1,
	    .cpbs  = 0,
	    .cpas  = 0,
	    .lovrs = 0,
	    .covfs = 0
    },{
	    .etrgs = 0,
	    .ldrbs = 0,
	    .ldras = 0,
	    .cpcs  = 1,
	    .cpbs  = 0,
	    .cpas  = 0,
	    .lovrs = 0,
	    .covfs = 0
	}};

    int i;

	Disable_global_interrupt();

	// The INTC driver has to be used only for GNU GCC for AVR32.
	// Initialize interrupt vectors.
	INTC_init_interrupts();

    // enable the 32-kHz oscillator (for clock source 1 of motor "spd" command)
    // (perhaps this is bad -- it may use PA11/PA12 - Steve)
    //pm_enable_osc32_crystal(&AVR32_PM);
    //pm_enable_clk32_no_wait(&AVR32_PM, AVR32_PM_OSCCTRL32_STARTUP_0_RCOSC);

	// Configure Osc0 in crystal mode (i.e. use of an external crystal source, with
	// frequency FOSC0) with an appropriate startup time then switch the main clock
	// source to Osc0.
    pcl_switch_to_osc(PCL_OSC0, FOSC0, OSC0_STARTUP);

    // initialize digital outputs used for motor control
    gpio_local_init();
    setPin(sMotor[0].on,  m0_motorOn ^ sMotor[0].onInv);
    setPin(sMotor[1].on,  m1_motorOn ^ sMotor[1].onInv);
    setPin(sMotor[2].on,  m2_motorOn ^ sMotor[2].onInv);
    setPin(sMotor[0].dir, m0_motorDir ^ sMotor[0].dirInv);
    setPin(sMotor[1].dir, m1_motorDir ^ sMotor[1].dirInv);
    setPin(sMotor[2].dir, m2_motorDir ^ sMotor[2].dirInv);

#if defined(MANIP)
    // initialize MANIP digital outputs
    setPin(BYSEL, 0);
    setPin(XWR, 1);
    setPin(XRD, 1);
    setPin(XRST, 1);
    setPin(DEV0, 0);
    setPin(DEV1, 0);
    setPin(BRD0, 0);
    setPin(BRD1, 0);
    setPin(BRDSEL, 0);
    setPin(ENCP, 1);
    setPin(PWM_PIN, 0);
    for (i=0; i<8; ++i) {
        // set write bit to zero
        setPin(WDAT+i, 0); 
    }
#elif !defined(CUTE)
    // clear necessary DEAP output pins
    for (i=0; i<kNumClearPins; ++i) {
    	setPin(sClearPin[i], 0);
    }
#endif

	for (i=0; i<NUM_MOTORS; ++i) {
    	// Register the RTC interrupt handler to the interrupt controller.
        INTC_register_interrupt(sMotor[i].irq, sMotor[i].irqNum, AVR32_INTC_INT0);
	    // Assign I/O to timer/counter channel pin & function.
	    gpio_enable_module_pin(sMotor[i].pin, sMotor[i].function);
	    output_mode[sMotor[i].pin] = 3;
    	// Initialize the timer/counter.
	    tc_init_waveform(tc, &waveform_opt[i]);  // Initialize the timer/counter waveform.
    }
	Enable_global_interrupt();

	for (i=0; i<NUM_MOTORS; ++i) {
        // Set the compare triggers.
        tc_write_ra(tc, sMotor[i].channel, sMotor[i].rc >> 1);    // Set RA value.
        tc_write_rc(tc, sMotor[i].channel, sMotor[i].rc);         // Set RC value.
        // configure the TC interrupts
	    tc_configure_interrupts(tc, sMotor[i].channel, &tc_interrupt[i]);
    }

    // configure ADC - lower the ADC clock to match the ADC characteristics
    // (because we configured the CPU clock to 12MHz and the ADC clock characteristics
    // are usually lower)
    AVR32_ADC.mr |= 0x1 << AVR32_ADC_MR_PRESCAL_OFFSET;
    adc_configure(&AVR32_ADC);
    
    Enable_global_exception();
    // THIS IS BAD BECAUSE WE ARE USING USART PINS FOR OTHER PURPOSES
    //init_dbg_rs232(FOSC0);
    pcl_configure_usb_clock();
    usb_task_init();
    resurfacer_task_init();

    wdt_init();

    while (TRUE) {
        usb_task();
        resurfacer_task();
    }
}

