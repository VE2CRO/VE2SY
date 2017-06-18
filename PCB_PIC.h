#include <16F690.h>
#include <stddef.h>

#define NOOP		0
#define MORSE_ECHO	1
#define SET_AUXOUT1	2
#define SET_AUXOUT2	3
#define ENABLE_REPEATER	4
#define ENABLE_RADIO1	5
#define ENABLE_RADIO2	6
#define ENABLE_RADIO3	7
#define TX_MORSE_ID     8
#define MCHAR(c)	c-'a'+10
#define AUX_DELAY_S(c) c*1831/60
// WAIT/NOWAIT are used for morseStart function
// When WAIT is specified, morse will only begin
// after COR_IN falls or after a specified delay
// has expired.
#define WAIT 1
#define NOWAIT 0

#include "SITE_08.h"

//#define CCP_PWM_MODE 0x00
//#define CCP_PWM_HH   0x0C
//#byte CCP1CON = 0x17

// Preprocessor variables {{{
#define CMD_MODE_TIMEOUT 2*1831 // 1 minute 
#define DTMFSeqSize 	14
#define DTMFArgPtr  	5
#define PTT_PWM_USER_DELAY 2000 // 1.25s 
#define PTT_PWM_BEGIN_DELAY 550
#define PTT_PWM_END_DELAY 	900

#define AUX_OUT_REG	64
#define TEMP_REG	65
#define COR_PER_SEC_REG 66
#define TX_MORSE_ID_FLAG 70
// The TIMER2 is spending 252 instructions between two calls to set_pwm_duty.
// At 8MHz, this corresponds to 31.5us.
// Two PWM periods corresponds to 126us.
// This means the TIMER2 interrupt accounts for 25% of the time spent during
// the PWM output is generated.
#define TIMER2_DELAY_FACTOR 1/6 // Based on experimentation with trace. 
// 1/6 gives a delay of 0.01ms per unit...
//#define TIMER2_DELAY_FACTOR 1/3 --> This one works really great!
#define DAH_DURATION_RATIO 3
#define MORSE_MULTIPLIER_ISR_ON 10 * TIMER2_DELAY_FACTOR
#define MORSE_MULTIPLIER_ISR_OFF 10 
#define SITE_ID_TX_DELAY 30
#define MORSE_WORD_DELAY morse_word_delay()
//#define MORSE_WORD_DELAY delay_ms(DitDelay * 7 * MorseMultiplier)

// }}}

// SITE_ID and SITE_GID parameter values {{{

#define PWM_DEFAULT_AMPLITUDE 2 // 4 = Full amplitude.

// }}}

#device ADC=10
#fuses INTRC
#fuses NOPROTECT
#fuses BROWNOUT
#fuses NOMCLR
#fuses NOCPD
#fuses NOWDT // WDT controlled by sw
#fuses NOPUT
#fuses NOFCMEN
#fuses NOIESO
#case

#use delay(internal=8M,restart_wdt)
//#use delay(clock=8000000,internal,restart_wdt)
//#use delay(internal=8MHZ,restart_wdt)

#use fast_io(A)
#use fast_io(B)
#use fast_io(C)

// Declare EEPROM register locations {{{

#define GET_RAM_PTR(id) 	Offset_##id
#define GET_EEPROM_PTR(id)	Offset_##id
#define GET_DEFAULT_VAL(id)  	DefaultValue_##id
#define GET_REG_SIZE(id)	RegSize_##id

const int8 RegCnt = 0;

#define REG_DEFINE_ARRAY(name,dv,size) \
	int8 name[size]; \
	const int8 Offset_##name = RegCnt; \
	const int8 DefaultValue_##name = dv ; \
	const int8 RCT = RegCnt; \
	#undef RegCnt \
	const int8 RegCnt = (RCT + size); \
	#undef RCT

//	const int8 RegSize_##name = size; \

#define REG_DEFINE(name,dv,size) \
	int8 name; \
	const int8 Offset_##name = RegCnt; \
	const int8 DefaultValue_##name = dv ; \
	const int8 RCT = RegCnt; \
	#undef RegCnt \
	const int8 RegCnt = (RCT + size); \
	#undef RCT 

//	const int8 RegSize_##name = size; \
	
#define SET_DEF_VAL(name,dv) \
	const int8 DefaultValue_##name = dv;

#define BASE_PTR &AdminPwd[0]



//
// Register name		     DefaultValue 	        Size     // Reg   EEPtr
REG_DEFINE_ARRAY(AdminPwd    , 0x01                  , 8     ) // 1     0
SET_DEF_VAL(AdminPwd1        , 0x22                          ) // 1     1
SET_DEF_VAL(AdminPwd2        , 0x00                          ) // 1     2
SET_DEF_VAL(AdminPwd3        , 0x00                          ) // 1     3
SET_DEF_VAL(AdminPwd4        , 0x00                          ) // 1     4
SET_DEF_VAL(AdminPwd5        , 0x00                          ) // 1     5
SET_DEF_VAL(AdminPwd6        , 0x00                          ) // 1     6
SET_DEF_VAL(AdminPwd7        , 0x00                          ) // 1     7
REG_DEFINE(Enable            , 0x03                  , 1     ) // 1     8
REG_DEFINE(Polarity          , POLARITY_DEF_VAL      , 1     ) // 2     9
REG_DEFINE(RX0PTT            , 0x02                  , 1     ) // 3     A
REG_DEFINE(RX1PTT            , 0x03                  , 1     ) // 4     B
REG_DEFINE(RX2PTT            , 0x00                  , 1     ) // 5     C
REG_DEFINE(PWMPTT            , 0x03                  , 1     ) // 6     D
REG_DEFINE(DitDelay          , 6                     , 1     ) // 7     E
REG_DEFINE(LinkTimeout       , LINK_TIMEOUT          , 1     ) // 8     F
REG_DEFINE(TempLowOp         , 0x00                  , 1     ) // 9     10
REG_DEFINE(TempHighOp        , 0x00                  , 1     ) // 10    11
REG_DEFINE(TempNormOp        , 0x00                  , 1     ) // 11    12
REG_DEFINE(AuxInAOp          , AUXINA_OP             , 1     ) // 12    13
REG_DEFINE(AuxInBOp          , AUXINB_OP             , 1     ) // 13    14
REG_DEFINE(PUAuxOutValue     , 0x00                  , 1     ) // 14    15
REG_DEFINE(SiteID            , SITE_ID_VAL           , 1     ) // 15    16
REG_DEFINE(MaxCorPerMinute   , MAX_COR_PER_MINUTE    , 1     ) // 16    17
REG_DEFINE_ARRAY(SiteIDMorse , MORSEID0              , 6     ) // 17 V  18
SET_DEF_VAL(SiteIDMorse1     , MORSEID1                      ) // 18 E  19
SET_DEF_VAL(SiteIDMorse2     , MORSEID2                      ) // 19 2  1A
SET_DEF_VAL(SiteIDMorse3     , MORSEID3                      ) // 20 R  1B
SET_DEF_VAL(SiteIDMorse4     , MORSEID4                      ) // 21 E  1C
SET_DEF_VAL(SiteIDMorse5     , MORSEID5                      ) // 22 H  1D
REG_DEFINE(TempLow           , 50                    , 1     ) // 23    1E (10degC) 
REG_DEFINE(TempHigh          , 68                    , 1     ) // 24    1F (28degC) 
// AuxCfg  Aux2  	Aux1
// --------------------------
// 00	   Output	Output 
// 01	   Output	Input
// 10	   Input	Output
// 11	   Input	Input
REG_DEFINE(AuxCfg            , 0x01                  , 1     ) // 25    20
REG_DEFINE(EnableMorseIDTx   , ENABLE_MORSE_ID_TX    , 1     ) // 26    21
REG_DEFINE(EnableSiteIDTx    , 0x00                  , 1     ) // 27    22
REG_DEFINE(PWMAmplitude      , PWM_DEFAULT_AMPLITUDE , 1     ) // 28    23
REG_DEFINE(UserFunction1Reg  , USER_FCT1_REG         , 1     ) // 29    24
REG_DEFINE(UserFunction1Op   , USER_FCT1_OP          , 1     ) // 30    25
REG_DEFINE(UserFunction2Reg  , USER_FCT2_REG         , 1     ) // 31    26
REG_DEFINE(UserFunction2Op   , USER_FCT2_OP          , 1     ) // 32    27
REG_DEFINE(UserFunction3Reg  , USER_FCT3_REG         , 1     ) // 33    28
REG_DEFINE(UserFunction3Op   , USER_FCT3_OP          , 1     ) // 34    29
REG_DEFINE(UserFunction4Reg  , USER_FCT4_REG         , 1     ) // 35    2A
REG_DEFINE(UserFunction4Op   , USER_FCT4_OP          , 1     ) // 36    2B
REG_DEFINE(PTTTimeout        , PTT_TIMEOUT           , 1     ) // 37    2C
REG_DEFINE(EEPROM_VALID      , 0xA5                  , 1     ) // 38    2D

// End of test. }}}

typedef short bit;

// Specifying the AddressMod type for EEPROM variables {{{
//void DataEE_Read(int32 addr, int8 * ram, int bytes) {
//
//   int i;
//
//   for(i=0;i<=bytes;i++,ram++,addr++)
//
//     *ram=read_eeprom(addr);
//
//}

//void DataEE_Write(int32 addr, int8 * ram, int bytes) {
//
//   int i;
//
//   for(i=0;i<=bytes;i++,ram++,addr++)
//
//     write_eeprom(addr,*ram);
//
//}

//addressmod (eeprom,DataEE_Read,DataEE_Write,1,0xFE);
// }}}

// DTMF Key maps {{{
//
#define d1 0x01
#define d2 0x02
#define d3 0x03
#define d4 0x04
#define d5 0x05
#define d6 0x06
#define d7 0x07
#define d8 0x08
#define d9 0x09
#define d0 0x0a
#define ds 0x0b
#define dp 0x0c
#define da 0x0d
#define db 0x0e
#define dc 0x0f
#define dd 0x00
// }}}

//
//Timer2 period
//
// PWM period en fonction de TIMER2_PERIOD:
// 125 --> 15.873KHz
// 63 --> 31.250KHz
// Timer2Period = TOSC * 4 * (PR2+1)
// Sine wave has 8 points so the PWM frequency must be divided by 8.
// Then, the PWMFreqDivider is used to further divide this period.
// This is used to speed up the timer2 interrupts.
#define TIMER2_PERIOD 125 // 63us
//#define TIMER2_PERIOD 63 // 32us
#define PWMFreqDivider	2

#bit TRISC3 = 0x87.3

#bit RB4 = 0x06.4
#bit RA5 = 0x05.5

// PI
#define PI 3.1415926

//
// State machine modes
//
#define IDLE 	0
#define ADMIN 	1
#define IO	2
#define HWRESET 3



// Define structures {{{
// sDTMF character structure 
//
typedef struct { 
			int Key 	: 4;
			int Strobe	: 1;
			int Last	: 1;
			int UNUSED	: 2;
} sDTMF;

typedef struct {
		int RX0 : 1;
		int RX1 : 1;
		int RX2 : 1;
} sCOR;


typedef struct tsRegSize {
		int * ptr;
		int size;
} sRegSize;		

//typedef struct tsActiveBits {
//	int LinkActive : 1;
//	int LocalActive : 1;
//	int LocalBusy : 1;
//	int UNUSED : 5;
//} ActiveBits;
//
//#define LinkActive ActiveBits.LinkActive;
//#define LocalActive ActiveBits.LocalActive;
//#define LocalBusy ActiveBits.LocalBusy;
// }}}

// Function headers {{{
// 
void timer0interrupt(void);
void resetAdminPwd(void);
void processUserFunctions(int Key);
//void StartPWM(int div);
//void StopPWM(void);
void DelayMs(int);
void ProcessDTMF(int Key);
void ClearDTMFSeq(void);
void DTMFStates(void);
void dit(void);
void dah(void);
void beep(void);
void ptt();
void ProcessCorIn(void);
void ProcessAuxOut(void);
void morsepause(void);
void morse(int reg);
void morseStart(int wait);
void morseStop(void);
void TempCtrl(void);
void AdminStates(void);
void waits(int sec);
void MasterHWReset(void);
void ClearWord(int reg );
void SetWord(int reg, int arg);
void SetArray(int reg, sDTMF * arg);
void SetBits(int reg,sDTMF sbit );
void ClearBits(int reg,sDTMF sbit );
void ExecOP(int op,int port);
void SetAuxCfg(void);
void reset_eeprom(void);
void write_ee(int x,int val);
void morse_word_delay(void);
void ProcessCorInPort(void);

int get_sin_val(void);
int CheckPWDResetPassword (void);
//int* getRamPtr(int id);
int eeprom_cksum(int NumAddr);
//int dec2hex(int c1,int c2);
int CheckPassword(void);
// }}}
	
// Volatile variables {{{
// These variables can be used in 
// normal functions and ISR routies.
//
int Last_COR;
int DiagTailPTT;
volatile int PTT;
volatile int PWMPtr; // PWM sine wave sample pocharer 0..7
volatile int PWMDiv; // PWM freq divider
volatile int PWMDivCnt; // Divider counter
volatile int16 Temp;
volatile unsigned int16 min;
volatile int hr_2;
volatile sDTMF DTMF; // ISR writes to this register.
volatile int COR_IN;
volatile bit DTMFStrb;
volatile bit TXMorseID; // 30 min timeout for MorseID
volatile long int CommandModeTimeout;
volatile bit TempUpdate;
volatile bit Tail;
volatile bit DiagTail;
volatile bit ClearDTMFFlag;

// }}}

// Declare SIN tables {{{
#define PWMOffset TIMER2_PERIOD/2 // 125/2 = 63.5 (63)
#define Offset    0 // 31.5
#define Amplitude (TIMER2_PERIOD/2)*0.85 // 85% of max swing
//#define Amplitude 63/27 // 7
const float sin[] = {	0,
			0.70711,
			1,
			0.70711,
			0,
			-0.70711,
			-1,
			-0.70711};

const int8 sinval[] =  {Offset + (int8)(sin[0]*Amplitude),
			Offset + (int8)(sin[1]*Amplitude),
			Offset + (int8)(sin[2]*Amplitude),
			Offset + (int8)(sin[3]*Amplitude),
			Offset + (int8)(sin[4]*Amplitude),
			Offset + (int8)(sin[5]*Amplitude),
			Offset + (int8)(sin[6]*Amplitude),
			Offset + (int8)(sin[7]*Amplitude)
			};
//}}}

// Global variables {{{

bit PWM_ACTIVE;
bit AuxInTail;

// Define CORE_STATEs
#define COR_IDLE 	0
#define COR_LINK 	1
#define COR_LOCAL 	2
#define COR_PWM 	3


sDTMF DTMFSeq[DTMFSeqSize];
sDTMF * DTMFPtr;
sDTMF * LastDTMF;
//int RXEnable; // RX2,RX1,RX0
int STATE;
int PTT_ENABLE;
volatile int AuxTimer;
int AuxInTailChar;
int AuxOut;
int AdminPtt;
const int SiteGID = SITE_GID_VAL;
int LinkTimeoutTimer;
int PTTTimeoutTimer;
int COR_Per_Minute;
unsigned int16 delay;

int PoundChars;
sCOR COR;
signed int TempC;

//const int mul10[] = {0,10,20,30,40,50,60,70,80,90,100,110,120};

// }}}

// Morse characters binary mapping {{{
// Dit = 01
// Dah = 10
// Stop = 00
// Word is read from right to left (LSB to MSB)
const int8 cMorseChar[] = {
	0b10101010, // 0 (dah dah dah dah dah)	0
	0b01101010, // 1 (dit dah dah dah dah)	1
	0b01011010, // 2 (dit dit dah dah dah)	2
	0b01010110, // 3 (dit dit dit dah dah)	3
	0b01010101, // 4 (dit dit dit dit dah)	4
	0b01010101, // 5 (dit dit dit dit dit)	5
	0b10010101, // 6 (dah dit dit dit dit)	6
	0b10100101, // 7 (dah dah dit dit dit)	7
	0b10101001, // 8 (dah dah dah dit dit)	8
	0b10101010, // 9 (dah dah dah dah dit)	9
	0b01100000, // a (dit dah)		10
	0b10010101, // b (dah dit dit dit)	11
	0b10011001, // c (dah dit dah dit)	12
	0b10010100, // d (dah dit dit)		13
	0b01000000, // e (dit)			14
	0b01011001, // f (dit dit dah dit)	15
	0b10100100, // g (dah dah dit) 		16
	0b01010101, // h (dit dit dit dit)	17
	0b01010000, // i (dit dit)		18
	0b01101010, // j (dit dah dah dah)	19
	0b10011000, // k (dah dit dah)		20
	0b01100101, // l (dit dah dit dit)	21
	0b10100000, // m (dah dah)		22
	0b10010000, // n (dah dit)		23
	0b10101000, // o (dah dah dah)		24
	0b01101001, // p (dit dah dah dit)	25
	0b10100110, // q (dah dah dit dah)	26
	0b01100100, // r (dit dah dit)		27
	0b01010100, // s (dit dit dit)		28
	0b10000000, // t (dah)			29
	0b01011000, // u (dit dit dah)		30
	0b01010110, // v (dit dit dit dah)	31
	0b01101000, // w (dit dah dah)		32
	0b10010110, // x (dah dit dit dah)	33
	0b10011010, // y (dah dit dah dah)	34
	0b10100101 // z (dah dah dit dit)	35
}; // }}}

// Define circuit port symbols {{{
//

#bit RC0=07.0
#bit RC1=07.1
#bit RC2=07.2
#bit RC4=07.4
#bit RC5=07.5
#bit RC6=07.6

#define COR0 RB5
#define COR1 RB6
#define COR2 RB7

#define RX_EN0	PIN_C0
#define RX_EN1	PIN_C1
#define RX_EN2	PIN_C2



#define PTT0_PIN PIN_C4
#define PTT1_PIN PIN_C5
#define PTT2_PIN PIN_C6
bit PTT0;
bit PTT1;
bit PTT2;

#define AUX0 RB4
#define AUX1 RA5

#define AUX0_PIN PIN_B4
#define AUX1_PIN PIN_A5
#define AUX0_IN   PIN_A5
#define AUX1_IN  PIN_B4

#bit TRISB4 = 0x86.4
#bit TRISA5 = 0x85.5

#define AUX1_INPUT_PIN PIN_A5
#define AUX2_INPUT_PIN PIN_B4
#define AUX1_OUTPUT_PIN PIN_B4
#define AUX2_OUTPUT_PIN PIN_A5

#define SET_AUX1_INPUT TRISA5=1
#define SET_AUX2_INPUT TRISB4=1
#define SET_AUX1_OUTPUT TRISB4=0
#define SET_AUX2_OUTPUT TRISA5=0

//#define TRISAUX0 TRISB4
//#define TRISAUX1 TRISA5
// }}}

#define 	EEPROM_CKSUM	     0xFF

typedef struct sRegInfo_t {
	int8 ptr;		// Pointer in EEPROM
	int8 * ramPtr;
	int8 size;
}	sRegInfo;

const sRegInfo RegInfo[] = {
	//								ID	ADDR
	//---------------------------------------------------------------------------
	{GET_EEPROM_PTR(AdminPwd)         , &AdminPwd[0]      , 8}            , // 0	0:7
	{GET_EEPROM_PTR(Enable)           , &Enable           , 1}            , // 1	8
	{GET_EEPROM_PTR(Polarity)         , &Polarity         , 1}            , // 2	9
	{GET_EEPROM_PTR(RX0PTT)           , &RX0PTT           , 1}            , // 3	A
	{GET_EEPROM_PTR(RX1PTT)           , &RX1PTT           , 1}            , // 4	B
	{GET_EEPROM_PTR(RX2PTT)           , &RX2PTT           , 1}            , // 5	C
	{GET_EEPROM_PTR(PWMPTT)           , &PWMPTT           , 1}            , // 6	D
	{GET_EEPROM_PTR(DitDelay)         , &DitDelay         , 1}            , // 7	E
	{GET_EEPROM_PTR(LinkTimeout)      , &LinkTimeout      , 1}            , // 8	F
	{GET_EEPROM_PTR(TempLowOp)        , &TempLowOp        , 1}            , // 9	10
	{GET_EEPROM_PTR(TempHighOp)       , &TempHighOp       , 1}            , // 10	11
	{GET_EEPROM_PTR(TempNormOp)       , &TempNormOp       , 1}            , // 11	12
	{GET_EEPROM_PTR(AuxInAOp)         , &AuxInAOp         , 1}            , // 12	13
	{GET_EEPROM_PTR(AuxInBOp)         , &AuxInBOp         , 1}            , // 13	14
	{GET_EEPROM_PTR(PUAuxOutValue)    , &PUAuxOutValue    , 1}            , // 14	15
	{GET_EEPROM_PTR(SiteID)           , &SiteID           , 1}            , // 15	16
	{GET_EEPROM_PTR(MaxCorPerMinute)  , &MaxCorPerMinute  , 1}            , // 16	17
	{GET_EEPROM_PTR(SiteIDMorse)      , &SiteIDMorse[0]   , 6}            , // 17	18
	{GET_EEPROM_PTR(SiteIDMorse)+1    , &SiteIDMorse[1]   , 1}            , // 18	19
	{GET_EEPROM_PTR(SiteIDMorse)+2    , &SiteIDMorse[2]   , 1}            , // 19	1A
	{GET_EEPROM_PTR(SiteIDMorse)+3    , &SiteIDMorse[3]   , 1}            , // 20	1B
	{GET_EEPROM_PTR(SiteIDMorse)+4    , &SiteIDMorse[4]   , 1}            , // 21	1C
	{GET_EEPROM_PTR(SiteIDMorse)+5    , &SiteIDMorse[5]   , 1}            , // 22	1D
	{GET_EEPROM_PTR(TempLow)          , &TempLow          , 1}            , // 23	1E
	{GET_EEPROM_PTR(TempHigh)         , &TempHigh         , 1}            , // 24	1F
	{GET_EEPROM_PTR(AuxCfg)           , &AuxCfg           , 1}            , // 25	20
	{GET_EEPROM_PTR(EnableMorseIDTx)  , &EnableMorseIDTx  , 1}            , // 26	21
	{GET_EEPROM_PTR(EnableSiteIDTx)   , &EnableSiteIDTx   , 1}            , // 27	22
	{GET_EEPROM_PTR(PWMAmplitude)     , &PWMAmplitude     , 1}            , // 28	23
	{GET_EEPROM_PTR(UserFunction1Reg) , &UserFunction1Reg , 1}            , // 29   24
	{GET_EEPROM_PTR(UserFunction1Op)  , &UserFunction1Op  , 1}            , // 30   25
	{GET_EEPROM_PTR(UserFunction2Reg) , &UserFunction2Reg , 1}            , // 31   26
	{GET_EEPROM_PTR(UserFunction2Op)  , &UserFunction2Op  , 1}            , // 32   27
	{GET_EEPROM_PTR(UserFunction3Reg) , &UserFunction3Reg , 1}            , // 33   28
	{GET_EEPROM_PTR(UserFunction3Op)  , &UserFunction3Op  , 1}            , // 34   29
	{GET_EEPROM_PTR(UserFunction4Reg) , &UserFunction4Reg , 1}            , // 35   2A
	{GET_EEPROM_PTR(UserFunction4Op)  , &UserFunction4Op  , 1}            , // 36   2B
	{GET_EEPROM_PTR(PTTTimeout)       , &PTTTimeout       , 1}            , // 37   2C
	{GET_EEPROM_PTR(EEPROM_VALID)     , &EEPROM_VALID     , 1}              // 38   2D
};

const int RegInfoSize = sizeof(RegInfo)/sizeof(sRegInfo);

