#include "PCB_PIC.h"

//
// Set _CONFIG bits
//
//__CONFIG(INTIO & WDTEN & PWRTDIS & 
//MCLRDIS & UNPROTECT & BORDIS & IESODIS & FCMDIS);

#INT_AD
void adinterrupt(void) { // {{{
	Temp=read_adc(ADC_READ_ONLY);
        TempUpdate=1;
} // }}}

#INT_RTCC
void timer0interrupt(void) { // {{{
  if ( CommandModeTimeout ) {
    CommandModeTimeout--;
  }
  if ( min ) {
    min--;
	}
	else {
		COR_Per_Minute=0;
		if ( hr_2 ) {
			hr_2--;
		}
		else {
			hr_2 = SITE_ID_TX_DELAY; // Transmit every 30 mins 
			TXMorseID=1;
		}
		// Is the link timeout timer enabled?
		if ( LinkTimeout ) {
			if ( LinkTimeoutTimer ) {
				LinkTimeoutTimer--;
			} else { // Timeout reached
				Enable &= 0xFE; // Disable link radio
			}
		}
    if ( PTTTimeout ) {
      if ( PTTTimeoutTimer ) {
        PTTTimeoutTimer--;
      }
    }
		read_adc(ADC_START_ONLY); 
		min = 1831; // 915.527 TMR0 interrupts per minute @4MHz.
	}		
	if (AuxTimer) {
		AuxTimer--;
	}
} // }}}

//#INT_RA
//void rainterrupt(void) { // {{{
	//if ( input(PIN_A4) && !DTMFStrb ) {
		//DTMF.Key = input_a() & 0xF; // Keep only 5 bits.
		//DTMF.Strobe = 1;
	//}
	//DTMFStrb = input(PIN_A4);
//} // }}}

#INT_RB
void rbinterrupt(void) { // {{{
	if ( input(PIN_A4) && !DTMFStrb ) {
		DTMF.Key = input_a() & 0xF; // Keep only 4 bits.
		DTMF.Strobe = 1;
	}
	DTMFStrb = input(PIN_A4);
	ProcessCorInPort();
} // }}}

void ProcessCorInPort(void) { // {{{
	bit LocalBusy;
	bit NO_COR;
	bit COR_Negedge;
	bit RX0_DEFAULT;

	COR_IN = input_b();
	COR_IN &= 0xE0;
	COR_IN = COR_IN >> 5;
	COR = (sCOR)(COR_IN ^ Polarity);
	
	// LocalBusy means that a QSO on Radio 2 or Radio 3 is on
	// LocalBusy = (((int8)COR & Last_COR & 0x06) != 0);
	LocalBusy = (((int8)COR & Last_COR & 0x06) != 0x00);

	//
	// All enabled COR signals just became Low, then enable the tail:
	//
	NO_COR = ((int8)COR == 0x00);
	COR_Negedge = NO_COR && ((Enable & Last_COR) != 0x00);
	if ( COR_Negedge ) { // COR from radios falls 
		// Only act on selected radios when COR fall
		DiagTail = ((Last_COR & EnableSiteIDTx) != 0x00); 
		// Don't send diag tail back to radio that just transmitted
    		DiagTailPTT = ~Last_COR & 0x07;
//		Tail = DiagTail || AuxInTail;
		COR_Per_Minute++;
	}
  if ( NO_COR ) {
    PTTTimeoutTimer = PTTTimeout;
    ClearDTMFFlag=1;
  }
	//
	// Only process COR_IN if Last_COR is 0
	// This garantees that a current conversation is not
	// cut by a new COR_IN
	//

	//if ( (Last_COR == 0) || ((int8)COR==0) ) { 
	// !LocalBusy --> None of the local radios are being retransmitted
	// (LocalActive | LinkActive) -> None of the radios' COR signals are high
	// COR == 0 --> If all COR's have dropped, then go thru this to clear the PTT state and drop RX relays
	if ( !LocalBusy || (int8)COR==0 ) { 
		// Stop interference
		if ( MaxCorPerMinute && (COR_Per_Minute > MaxCorPerMinute )) {
			// This inhibits the PTTs for about 30 seconds in case of
			// interference.
			// Also, in order to protect the relays, the RX0 relay will remain
			// on for about 30 seconds
			PTT_ENABLE = 0x00;
			RX0_DEFAULT = 1;
			(int8)COR = 0x00;
		} else {
			PTT_ENABLE = Enable;
			RX0_DEFAULT = 0;
		}
		// COR on RX2 and (That's the only COR or the RX2 is enabled or no other radio is enabled) then enter
		if ( COR.RX2 && ( (int8)COR == 0x04 || (Enable & 0x04) || !Enable)) { // RX2
			output_bit(RX_EN0,0);
			output_bit(RX_EN1,0);
			output_bit(RX_EN2,1);
			Last_COR = 0x04;
			Tail = 0; // avoid tail if conversation starts
			if ( Enable & 0x04 ) {
				PTT = RX2PTT;
			} else {
				PTT = 0x00;
			}
		}
		else if ( COR.RX1 && ((int8)COR == 0x02 || (Enable & 0x02) || !Enable)) { // RX1
			output_bit(RX_EN0,0);
			output_bit(RX_EN2,0);
			output_bit(RX_EN1,1);
			Last_COR = 0x02;
			Tail = 0; // avoid tail if conversation starts
			if ( Enable & 0x02 ) {
				PTT = RX1PTT;
			} else {
				PTT = 0x00;
			}
		}
		else if ( COR.RX0 && ((int8)COR == 0x01 || (Enable & 0x01) || !Enable)) { // RX0
			output_bit(RX_EN1,0);
			output_bit(RX_EN2,0);
			output_bit(RX_EN0,1);
			// Give no priority to link radio.
			// Any other radio may interrupt the link radio.
			// This is required for the phone patch.
			//
			// Also, data from this radio does not have the
			// siteID diagnostic tail appended to it.
			//
			Last_COR = 0x01; // Give no priority to link radio.
			Tail = 0; // avoid tail if conversation starts
			if ( Enable & 0x01 ) {
				PTT = RX0PTT;
			} else {
				PTT = 0x00;
			}
		}
		else {
			output_bit(RX_EN1,0);
			output_bit(RX_EN2,0);
			output_bit(RX_EN0,RX0_DEFAULT); 
			Last_COR = 0;
			PTT = 0 ; 
		}
    if ( PTTTimeout && !PTTTimeoutTimer ) {
      PTT=0;
    }
		ptt();	
	}
	else {  // Ongoing QSO 
		Last_COR &= (int)COR;
    if ( PTTTimeout && !PTTTimeoutTimer ) {
      PTT=0;
      ptt();
    }
	}
} // }}}

#INT_TIMER2
void timer2interrupt(void) { // {{{
    int amplitude;
    int SinPtr;
    signed int Sin;
    
	PWMDivCnt--;
	if ( !PWMDivCnt ) {
		PWMPtr++;
		PWMPtr &= 0x07;
        SinPtr = PWMPtr & 0x03;
        Sin = sinval[SinPtr];
    	switch(PWMAmplitude) {
	       	case(0x00):amplitude=0;break;
	       	case(0x01):amplitude=Sin>>3;break;
	    	case(0x02):amplitude=Sin>>2;break;
	    	case(0x03):amplitude=Sin>>1;break;
	    	default:amplitude=Sin;
	    }
        if ( PWMPtr & 0x04) {
                amplitude = -amplitude;
        }
    amplitude+=PWMOffset;
    set_pwm1_duty(amplitude);
    PWMDivCnt = PWMDiv;
	} 
} // }}}
// eeprom_init_check {{{

void eeprom_init_check (void) { 
	int8 x,y,eePtr;
	int Valid,cksum,EECKSUM;
	int Val;
	int RamPtr;
	
	y=Offset_EEPROM_VALID;
	Valid = read_eeprom(Offset_EEPROM_VALID);
	EECKSUM = read_eeprom(EEPROM_CKSUM);
	cksum = 0;
	
	if ( Valid == 0xA5 ) { // EEPROM content is valid
		//for(x=0;x<sizeof(DefaultValue);x++) {
    eePtr=0;
    for(x=0;x<RegInfo[0].size;x++) { // Process AdminPwd
	    RamPtr = RegInfo[0].ramPtr+x;
  		restart_wdt();
  		Val = read_eeprom(eePtr);
		 	*RamPtr = Val;
	  	cksum ^= Val;
      eePtr++;
    }
		for(x=1;x<RegInfoSize;x++) { // Process other registers
			  // Fetch ROM values into RAM (they may not be good however)
		  	//RamPtr = BASE_PTR + x;
	  		RamPtr = RegInfo[x].ramPtr;
  			restart_wdt();
  			Val = read_eeprom(eePtr);
		  	*RamPtr = Val;
	  		cksum ^= Val;
        eePtr++;
		}
		cksum ^= EECKSUM; // This should be 0 if EEPROM is clean
		if ( cksum ) {
			Valid = 0;
		}
	}
	else {
		Valid = 0;
	}
	if ( ! Valid ) { // Initialise EEPROM 
			AdminPwd[0]     =GET_DEFAULT_VAL(AdminPwd);
			AdminPwd[1]     =GET_DEFAULT_VAL(AdminPwd1);
			AdminPwd[2]     =GET_DEFAULT_VAL(AdminPwd2);
			AdminPwd[3]     =GET_DEFAULT_VAL(AdminPwd3);
			AdminPwd[4]     =GET_DEFAULT_VAL(AdminPwd4);
			AdminPwd[5]     =GET_DEFAULT_VAL(AdminPwd5);
			AdminPwd[6]     =GET_DEFAULT_VAL(AdminPwd6);
			AdminPwd[7]     =GET_DEFAULT_VAL(AdminPwd7);
			Enable	        =GET_DEFAULT_VAL(Enable);
			Polarity	=GET_DEFAULT_VAL(Polarity);
			RX0PTT		=GET_DEFAULT_VAL(RX0PTT);
			RX1PTT		=GET_DEFAULT_VAL(RX1PTT);
			RX2PTT		=GET_DEFAULT_VAL(RX2PTT);
			PWMPTT		=GET_DEFAULT_VAL(PWMPTT);
			DitDelay	=GET_DEFAULT_VAL(DitDelay		);
			LinkTimeout     =GET_DEFAULT_VAL(LinkTimeout);
			TempLowOp	=GET_DEFAULT_VAL(TempLowOp		);
			TempHighOp	=GET_DEFAULT_VAL(TempHighOp		);
			TempNormOp	=GET_DEFAULT_VAL(TempNormOp		);
			AuxInAOp	=GET_DEFAULT_VAL(AuxInAOp		);
			AuxInBOp	=GET_DEFAULT_VAL(AuxInBOp		);
			PUAuxOutValue	=GET_DEFAULT_VAL(PUAuxOutValue	);
			SiteID		=GET_DEFAULT_VAL(SiteID		);
			MaxCorPerMinute =GET_DEFAULT_VAL(MaxCorPerMinute		);
			SiteIDMorse[0]	=GET_DEFAULT_VAL(SiteIDMorse	);
			SiteIDMorse[1]	=GET_DEFAULT_VAL(SiteIDMorse1	);
			SiteIDMorse[2]	=GET_DEFAULT_VAL(SiteIDMorse2	);
			SiteIDMorse[3]	=GET_DEFAULT_VAL(SiteIDMorse3	);
			SiteIDMorse[4]	=GET_DEFAULT_VAL(SiteIDMorse4	);
			SiteIDMorse[5]	=GET_DEFAULT_VAL(SiteIDMorse5	);
			TempLow		=GET_DEFAULT_VAL(TempLow	);
			TempHigh	=GET_DEFAULT_VAL(TempHigh	);
			AuxCfg		=GET_DEFAULT_VAL(AuxCfg		);
			EnableMorseIDTx	=GET_DEFAULT_VAL(EnableMorseIDTx);
			EnableSiteIDTx	=GET_DEFAULT_VAL(EnableSiteIDTx	);
			PWMAmplitude	=GET_DEFAULT_VAL(PWMAmplitude);
			UserFunction1Reg=GET_DEFAULT_VAL(UserFunction1Reg);
			UserFunction1Op =GET_DEFAULT_VAL(UserFunction1Op);
			UserFunction2Reg=GET_DEFAULT_VAL(UserFunction2Reg);
			UserFunction2Op =GET_DEFAULT_VAL(UserFunction2Op);
			UserFunction3Reg=GET_DEFAULT_VAL(UserFunction3Reg);
			UserFunction3Op =GET_DEFAULT_VAL(UserFunction3Op);
			UserFunction4Reg=GET_DEFAULT_VAL(UserFunction4Reg);
			UserFunction4Op =GET_DEFAULT_VAL(UserFunction4Op);
			PTTTimeout      =GET_DEFAULT_VAL(PTTTimeout);
			EEPROM_VALID	=GET_DEFAULT_VAL(EEPROM_VALID   );
			cksum = 0;
			
    eePtr=0;
		for(x=0;x<RegInfo[0].size;x++) { // Process AdminPwd
	  	RamPtr = RegInfo[0].ramPtr+x;
			restart_wdt();
  		Val = *RamPtr;
  		write_ee(eePtr,Val); // Store Default value in EEPROM
  		cksum ^= Val;
      eePtr++;
    }
		for(x=1;x<RegInfoSize;x++) {
		  // Fetch ROM values into RAM (they may not be good however)
		 	//RamPtr = BASE_PTR + x;
	  	RamPtr = RegInfo[x].ramPtr;
		  restart_wdt();
  		Val = *RamPtr;
  		write_ee(eePtr,Val); // Store Default value in EEPROM
  		cksum ^= Val;
      eePtr++;
		}
		write_ee(EEPROM_CKSUM,cksum);
	}

} // }}}
// Function init (void) {{{
//========================
//Function : init
//Inputs	 : None
//Outputs	 : None
//
//This function performs various chip initialisations:
//PWM
//INPUTS
//OUTPUTS
//  ========================= 
void init (void) { 
	// Setup TIMER0 as the main delay counter
	//OPTION = 0b0111;	// prescale by 256
	//	RABPU  = 1;		// Disable RA and RB pullups
	//	T0CS   = 0;		// select internal clock
	setup_timer_0 (RTCC_DIV_256|RTCC_INTERNAL);
	set_timer0(0);
	enable_interrupts(INT_RTCC);
	//
	// Setup ADC
	//
	setup_adc(ADC_CLOCK_INTERNAL);
	setup_adc_ports ( sAN9 );
	set_adc_channel ( 9 );
	enable_interrupts(INT_AD);
	setup_wdt(WDT_ON | WDT_TIMES_128);
	//setup_wdt(WDT_ON);
	set_tris_c(0b010001000);
	eeprom_init_check(); // Initialise RAM variables
	PWMPtr = 0;
	ClearDTMFSeq();
	COR_Per_Minute = 0;
	STATE = IDLE;
	AdminPtt = 0x00;
	DiagTailPTT=0;
	set_tris_a(0x1F);
	set_tris_b(0xE0);
	AuxInTailChar = 0;
	if(AuxCfg & 0x01) {
		SET_AUX1_INPUT;
	}
	else {
		SET_AUX1_OUTPUT;
	}
	if(AuxCfg & 0x02) {
		SET_AUX2_INPUT;
	}
	else {
		SET_AUX2_OUTPUT;
	}
	AuxOut = PUAuxOutValue;
  ClearDTMFFlag = 0;
	// PWM Configuration
	// Setting P1A,P1B,P1C,P1D are active high
	setup_ccp1(CCP_PWM | CCP_PULSE_STEERING_C,CCP_PWM_H_H );
	setup_timer_2(T2_DISABLED,TIMER2_PERIOD,1); // period = 125, post-scale = 1
	// PWM Period = [(PR2)+1]*4*TOSC*(TMR2 prescale value)
	// PWM Duty Cycle = (CCPR1L:CCP1CON<5:4>) * TOSC * (TMR2 prescale value)
	// Initialise aux outputs
	//SetAuxCfg();
	PWM_ACTIVE = 0;
	TXMorseID = 1; // Transmit station morse ID on power-up
	hr_2 = SITE_ID_TX_DELAY; 
	min = 1831;
	enable_interrupts(INT_RB5|INT_RB6|INT_RB7); 
	enable_interrupts(INT_RA4);
	enable_interrupts(GLOBAL);
} // }}}
void main (void) { // {{{
	int x;
//	int MorsePTT;
	int1 MorseTransmit;
	init();

   	// Main loop	
#ignore_warnings 203
		while(1) {
#ignore_warnings NONE
		restart_wdt();
		// Check COR_IN inputs
		ProcessCorInPort(); 
		restart_wdt();
    if ( ClearDTMFFlag ) {
      ClearDTMFSeq();
      ClearDTMFFlag = 0;
    }
		if (DTMF.Strobe) {
			processUserFunctions(DTMF.Key);
			if (DTMF.Key == ds) {
				ClearDTMFSeq();
				PoundChars=0x00;
			}
			else {
				ProcessDTMF(DTMF.Key);
			}
			if(DTMF.Key == dp) {
				DTMFStates();
			}
			DTMF.Strobe = 0;
		}
		restart_wdt();
		// Check Temperature and AuxIn control
		TempCtrl();
		restart_wdt();
		// TXMorseID values:
		// 0 -- No Morse ID transmission
		// 1 -- Always transmit Morse ID 
		// 2 -- Transmit Morse ID when repeater is disabled
		if ( TXMorseID ) {
			MorseTransmit = 0;
			if ( (EnableMorseIDTx & 0x01) || ((EnableMorseIDTx & 0x02) && ((Enable & 0x01) == 0x00)  ) ) {
				PWMPTT = read_eeprom(GET_EEPROM_PTR(PWMPTT));
				MorseTransmit = 1;
			} else {
			  if ( EnableMorseIDTx & 0xF0 ) {
			// Transmit morse every 30 mins on selected radios
			// TXMorseID 6 5 4 3 2 1 0
			//           | | |     | |-> On/Off
			//           | | |     |---> TX when Link radio is Off
			//           | | |---------> Override setting --> Send every 1/2 hr 
					PWMPTT = ((EnableMorseIDTx >> 4) & 0x07);
					MorseTransmit = 1;
			  }
			}
			if ( MorseTransmit ) {
				// Transmit Morse ID
				morseStart(NOWAIT);
				for (x=0;x<6;x++) {
					morse(SiteIDMorse[x]);
					MORSE_WORD_DELAY;
					restart_wdt();
				}
				morseStop();
				PWMPTT = read_eeprom(GET_EEPROM_PTR(PWMPTT));
				MorseTransmit = 0;
			}
			TXMorseID = 0;
		}
		restart_wdt();
		// Transmit SiteID on tail. This is used for diagnostic
		if ( Tail ) {
			morseStart(NOWAIT);
			if ( AuxInTail ) { // Transmit AuxIn tail before siteID
				morse(AuxInTailChar); // This is either 0 or 1
				MORSE_WORD_DELAY;
				AuxInTail = 0;
			}
			if (DiagTail) {
                // Send a long "BEEP" 
	 		    morse(MCHAR('t'));
//	 		    morse(SiteID);
			    DiagTail=0;
                        }
            		MORSE_WORD_DELAY;
			Tail = 0; // Clear Tail bit
			morseStop();
		}
	}
} // }}}
void TransmitDigit(signed int digit) { // {{{
	int idigit;
	int dizaine;
	
	if ( digit < 0 ) {
		morse('n'-'a'+10); // Transmit 'n' for negative
		idigit = -digit;
	} else {
		idigit = digit;
	}	
	// Send 
	dizaine=0;
	while( idigit > 9 ) {
		idigit=idigit-10;
		dizaine++;
	}
	// Send remainder
	if ( dizaine ) {
		morse(dizaine);
	}
	morse(idigit);
} // }}}

void ClearDTMFSeq(void) { // {{{
	char x;
	for(x=0;x<DTMFSeqSize;x++) {
		DTMFSeq[x].Key = 0x00;
		DTMFSeq[x].Last = 0;
		DTMFSeq[x].Strobe = 0;
	}
	DTMFPtr = &DTMFSeq[0];
	LastDTMF = NULL;
	DTMF.Key = 0x00;
	DTMF.Last = 0;
	DTMF.Strobe = 0;
}	// }}}
void StartPWM(int div) { // {{{
	bit COR_ON;
	PWMDiv = div;
	PWMDivCnt = div;
//	CCP1CON = CCP_PWM_MODE | CCP_PWM_HH;	
	disable_interrupts(INT_RTCC);
	setup_timer_2(T2_DIV_BY_1,TIMER2_PERIOD,1); // period = 125, post-scale = 1
	enable_interrupts(INT_TIMER2);
	// Cut the PWM by half when a COR is enabled
	output_drive(PIN_C3);
	COR_ON = ((int8)COR !=0);
	if ( COR_ON ) {
		PWMAmplitude = (read_eeprom(GET_EEPROM_PTR(PWMAmplitude))-1) & 0x07;
	    //output_float(PIN_C3);
	} else {
		PWMAmplitude = read_eeprom(GET_EEPROM_PTR(PWMAmplitude));
  	    //output_drive(PIN_C3);
	}
}	// }}}
void StopPWM(void) { // {{{
	setup_timer_2(T2_DISABLED,TIMER2_PERIOD,1); // period = 125, post-scale = 1
	setup_ccp1(CCP_OFF|CCP_PULSE_STEERING_C,CCP_PWM_H_H);
	disable_interrupts(INT_TIMER2);
	enable_interrupts(INT_RTCC);
	output_float(PIN_C3);
}	// }}}
void ClearWord(int reg ) { // {{{
	int8 * w;
	int8 eePtr,EECKSUM;

	w = RegInfo[reg].ramPtr;
	eePtr = RegInfo[reg].ptr;
	// Update EEPROM Ckecksum
	EECKSUM = read_eeprom(EEPROM_CKSUM);
	EECKSUM ^= read_eeprom(eePtr);
	write_ee(EEPROM_CKSUM,EECKSUM);
	*w = 0x00;
	write_ee(eePtr,0x00);
}	// }}}
void SetWord(int reg,int arg ) { // {{{
	int8 * w;
	int8 eePtr,EECKSUM;
	w = RegInfo[reg].ramPtr;
	eePtr = RegInfo[reg].ptr;
	// Update EEPROM Ckecksum
	EECKSUM = read_eeprom(EEPROM_CKSUM);
	EECKSUM ^= read_eeprom(eePtr) ^ arg;
	write_ee(EEPROM_CKSUM,EECKSUM);

	*w = arg;
	write_ee(eePtr,arg);
} // }}}
void SetArray(int reg,sDTMF * arg) { // {{{
	int8 * w;
	int x,eePtr,EECKSUM;
	int1 done;
	sDTMF wreg;
	int num;
	
	w = RegInfo[reg].ramPtr;
	eePtr = RegInfo[reg].ptr;
	num = RegInfo[reg].size;
	done=0;
	
	// Update EEPROM Ckecksum
	EECKSUM = read_eeprom(EEPROM_CKSUM);
	for(x=0;x<num;x++) {
		if ( !done ) {
			restart_wdt();
			//wreg = arg->Key;
			//wreg |= 0x10 && (arg->Strobe != 0);
			//wreg |= 0x20 && (arg->Last != 0);
			wreg = *arg;
			wreg.Strobe = 0; // Don't keep strobe bit
			done = arg->Last;
			*w = wreg;
			EECKSUM ^= read_eeprom(eePtr+x);
			write_ee(eePtr+x,(int)wreg);
			EECKSUM ^= (int)wreg;
			w++;
			arg++;
		}
	} 
	write_ee(EEPROM_CKSUM,EECKSUM);
}// }}}
void SetBits(int reg,sDTMF sbit ) { // {{{
	int8 * w;
	int wreg,eePtr,EECKSUM;
	w = RegInfo[reg].ramPtr;
	eePtr = RegInfo[reg].ptr;
	wreg = *w | sbit.Key;
	*w = wreg;
	// Update EEPROM Ckecksum
	EECKSUM = read_eeprom(EEPROM_CKSUM);
	EECKSUM ^= read_eeprom(eePtr) ^ wreg;
	write_ee(EEPROM_CKSUM,EECKSUM);

	write_ee(eePtr,wreg);
} // }}}
void ClearBits(int reg,sDTMF sbit ) { // {{{
	int8 * w;
	int wreg,eePtr,EECKSUM;
	w = RegInfo[reg].ramPtr;
	eePtr = RegInfo[reg].ptr;
	wreg = *w & (~ sbit.Key);
	*w = wreg;
	// Update EEPROM Ckecksum
	EECKSUM = read_eeprom(EEPROM_CKSUM);
	EECKSUM ^= read_eeprom(eePtr) ^ wreg;
	write_ee(EEPROM_CKSUM,EECKSUM);

	write_ee(eePtr,wreg);
} // }}}

int dec2hex(int c1, int c2) { // {{{
	int r;
	//r = mul10[c1] + c2;
	r = (c1 * 10) + c2;
	return (r);
}	// }}}

void ProcessDTMF (int Key) { // {{{
    if((DTMFPtr <= &DTMFSeq[DTMFSeqSize-1]) && (DTMFPtr >= &DTMFSeq[0])) {
	switch(Key) {
		case(d0): DTMFPtr->Key = 0x00; break;
		case(da): DTMFPtr->Key = 0x0a; break;
		case(db): DTMFPtr->Key = 0x0b; break;
		case(dc): DTMFPtr->Key = 0x0c; break;
		case(dd): DTMFPtr->Key = 0x0d; break;
		case(ds): DTMFPtr->Key = 0x0e; break;
		case(dp): DTMFPtr->Key = 0x0f; break;
		default : DTMFPtr->Key = Key ; break;
	}
	if (Key == dp) {
		LastDTMF->Last = 1;
		PoundChars = (PoundChars << 1) + 1;
	}
	else {
		LastDTMF->Last = 0;
		PoundChars = 0x00;
	}
	DTMFPtr->Strobe = 1;
	LastDTMF = DTMFPtr;
	DTMFPtr++;
    }
}	// }}}

void DTMFStates ( void ) { // {{{
	int ADDR;
	bit ADDR_VALID;

	ADDR = dec2hex(DTMFSeq[0].Key , DTMFSeq[1].Key);
	ADDR_VALID = (ADDR == SiteID) || (ADDR == SiteGID) ;

	switch(STATE) {
		case(IDLE):
			if ( ADDR_VALID && CheckPassword() ) {
				STATE = ADMIN;
                    AdminPtt = (int8)COR; // Send DTMF on radio that does the admin stuff
		    morseStart(WAIT);
				morse(0x0a);
				MORSE_WORD_DELAY;
				morseStop();
				CommandModeTimeout = CMD_MODE_TIMEOUT;
			}
			// 5 # consecutive characters resets the password
			if ( PoundChars == 0x1F ) {
				resetAdminPwd();
			}
			break;
		case(ADMIN):
			if ( CommandModeTimeout && ADDR_VALID ) { // Variable is active low
				AdminStates();
				CommandModeTimeout = CMD_MODE_TIMEOUT;
			} 
			else {
				STATE = IDLE;
			} 			
			break;
		default: 
			STATE = IDLE;
			break;
	}
	ClearDTMFSeq();
	return;
} // }}}

void AdminStates(void) { // {{{
	int REG;
	int OP;
	int rcnt;
	int Argument0;
	int Argument1;
	int *TargetRamReg;
	int *RamPtr;
	bit ExitCondition,ArgumentsPresent;
	bit RamRegisterAccess;
	static bit eepromReg;
	//struct {int eeprom * ptr,
	//	int size} 
	// AuxOut address = 64

	//
	// Input syntax:
	//  ADDR  0	        1	  2    3       4      5    6 
	//  *    ADDR[1] ADDR[0] [OP] <REG[1] REG[0] <ARGx ARGy ...> >
	//
	REG  = dec2hex(DTMFSeq[3].Key , DTMFSeq[4].Key);
	OP   = DTMFSeq[2].Key;
	ArgumentsPresent = (LastDTMF > &DTMFSeq[4]);
	ExitCondition = (LastDTMF == &DTMFSeq[3]);
	eepromReg = (REG < RegInfoSize) || OP==0x00;
	restart_wdt();
	if ( ArgumentsPresent ) {
		Argument0 = DTMFSeq[5].Key;
		Argument1 = DTMFSeq[6].Key;
	}
	else {
		Argument0 = 0x00;
		Argument1 = 0x00;
	}
  if ( OP == 0x01 && ExitCondition) {
		morseStart(WAIT);
		morse(MCHAR('r'));
		morse(MCHAR('b'));
		morse(MCHAR('t'));
    MORSE_WORD_DELAY;
    morseStop();
  	reset_cpu();
  }
	// Matching SiteID or GroupID {{{
	if ( eepromReg ) {
		// EEPROM registers {{{
		switch(OP) {
			case(0x00): // Enter & Exit admin mode
				morseStart(WAIT);
				if ( ExitCondition ) {
					// Exit admin state on *010#
					morse(0x0e);
					STATE = IDLE;
				} else {
					// Stay in ADMIN state and confirm it with "A"
					morse(0x0a);
				}
				MORSE_WORD_DELAY;
				morseStop();
				break;
			case(0x02): // Set Nibble
				SetWord(REG,DTMFSeq[DTMFArgPtr].Key);
				break;
			case(0x05): // Set Word using 2 decimal value (00,01,02, etc... up to 99)
				SetWord(REG,dec2hex(DTMFSeq[DTMFArgPtr].Key,DTMFSeq[DTMFArgPtr+1].Key));
				break;
			case(0x06): // Set array of nibbles
				SetArray(REG,&DTMFSeq[DTMFArgPtr]);
				break;
			case(0x07):
				morseStart(WAIT);
				if ( REG == 0x00 ) {
					morse(0x0E); // ERROR! Don't echo AdminPwd
          morse(0x03); // ERROR! Don't echo AdminPwd
          MORSE_WORD_DELAY;
				}
				else {
          RamPtr = RegInfo[REG].ramPtr;
					for(rcnt=0;rcnt< RegInfo[REG].size;rcnt++) {
						if ( REG >= 17 && REG <= 22 ) {
		          morse(*RamPtr);
						} 
						else {
							TransmitDigit((signed int)*RamPtr);
						}
            RamPtr++;
					}
          MORSE_WORD_DELAY;
				}
				morseStop();
				break;
			case(0x08):
				MasterHWReset();
				break;
			default:
				morseStart(WAIT);
				morse(0x0e); // Error
				morse(0x01);
	      MORSE_WORD_DELAY;
				morseStop();
				break;
		}
				// }}}
	}
	else { // RAM registers
		// RAM registers {{{
		switch(REG) {
			case(AUX_OUT_REG):
					TargetRamReg = &AuxOut;
					RamRegisterAccess = 1;
					break;
			case(COR_PER_SEC_REG):
					TargetRamReg = &COR_Per_Minute;
					RamRegisterAccess = 1;
					break;
			case(TX_MORSE_ID_FLAG): // emulate 1/2 delay
					STATE = IDLE;
					TXMorseID = 1;
					break;					
			default:
					TargetRamReg = NULL;
					RamRegisterAccess = 0;
					break;
		}
		if ( RamRegisterAccess ) { // Auxiliary register --> Set to 64
			switch(OP) {
				case(0x01):
					*TargetRamReg = 0x00;
					break;
				case(0x02): 
					*TargetRamReg = DTMFSeq[DTMFArgPtr].Key;
					break;
				case(0x03):
					*TargetRamReg |= DTMFSeq[DTMFArgPtr].Key;
					break;
				case(0x04):
					*TargetRamReg &= ~DTMFSeq[DTMFArgPtr].Key;
					break;
				case(0x07):
					morseStart(WAIT);
					TransmitDigit((signed int)*TargetRamReg);
					MORSE_WORD_DELAY;
					morseStop();
					break;
				default:
					morseStart(WAIT);
					morse(0x0e); // Error
					morse(0x01);
					MORSE_WORD_DELAY;
					morseStop();
				    break;
			}
		}
		else if ( REG == TEMP_REG ) {
			if ( OP == 0x07 ) {
				morseStart(WAIT);
				TransmitDigit(TempC);
				MORSE_WORD_DELAY;
				morseStop();
			} else {
				morseStart(WAIT);
				morse(0x0e); // Error
				morse(0x01);
				MORSE_WORD_DELAY;
				morseStop();
			}
		}
		else {
			morseStart(WAIT);
			morse(0x0e); // Error
			morse(0x02);
			MORSE_WORD_DELAY;
			morseStop();
		}
		// }}}
	}
	if ( OP == 0x02 || OP == 0x05 ) {
			morseStart(WAIT);
			morse(Argument0); // Send dixaine
			if ( OP == 0x05 ) {
				morse(Argument1); // Send unit
			}
			MORSE_WORD_DELAY;
			morseStop();
	}
	// }}}
	restart_wdt();
} // }}}

void MasterHWReset(void) { // {{{
	write_ee(GET_EEPROM_PTR(EEPROM_VALID),0x00);
	eeprom_init_check(); // Initialise RAM variables
} // }}}

void resetAdminPwd(void) { // {{{
	// The Admin password is reset in RAM only (Not EEPROM).
	AdminPwd[0] = GET_DEFAULT_VAL(AdminPwd);
	AdminPwd[1] = GET_DEFAULT_VAL(AdminPwd1);
	AdminPwd[2] = GET_DEFAULT_VAL(AdminPwd2);
	AdminPwd[3] = GET_DEFAULT_VAL(AdminPwd3);
	AdminPwd[4] = GET_DEFAULT_VAL(AdminPwd4);
	AdminPwd[5] = GET_DEFAULT_VAL(AdminPwd5);
	AdminPwd[6] = GET_DEFAULT_VAL(AdminPwd6);
	AdminPwd[7] = GET_DEFAULT_VAL(AdminPwd7);
	// Make sure to update the CKSUM if we are going to write to EEPROM.
	//write_ee(GET_EEPROM_PTR(AdminPwd),GET_DEFAULT_VAL(AdminPwd));
	//write_ee(GET_EEPROM_PTR(AdminPwd)+1,GET_DEFAULT_VAL(AdminPwd1));
	//write_ee(GET_EEPROM_PTR(AdminPwd)+2,GET_DEFAULT_VAL(AdminPwd2));
	//write_ee(GET_EEPROM_PTR(AdminPwd)+3,GET_DEFAULT_VAL(AdminPwd3));
	//write_ee(GET_EEPROM_PTR(AdminPwd)+4,GET_DEFAULT_VAL(AdminPwd4));
	//write_ee(GET_EEPROM_PTR(AdminPwd)+5,GET_DEFAULT_VAL(AdminPwd5));
	//write_ee(GET_EEPROM_PTR(AdminPwd)+6,GET_DEFAULT_VAL(AdminPwd6));
	//write_ee(GET_EEPROM_PTR(AdminPwd)+7,GET_DEFAULT_VAL(AdminPwd7));
} // }}}

int CheckPassword(void) { // {{{
	int x;
	sDTMF PWD;
	sDTMF * dtmfPtr;
	int GoodPwd,Done;
//	int ADDR;
	int keyIn;
	bit LastDigit;
	bit PwdEntry;

	
//	ADDR = dec2hex(DTMFSeq[0].Key , DTMFSeq[1].Key);
	
	dtmfPtr = &DTMFSeq[0];	

	// Syntax : <SiteID0><SiteID1><0><Pwd1><Pwd2>...<#>
	//
	PwdEntry = (DTMFSeq[2].Key == 0x00);

	GoodPwd = 1;
	Done=0;
	restart_wdt();
//	if ( ADDR == SiteID ) { // 0x00 is the broadcast address
    if (PwdEntry) {
	    for(x=0;x<8;x++) {
	    	if ( !Done ) {
	    		dtmfPtr = &DTMFSeq[x+3];
	    		// EEPROM words are stored in 14 bits and look as if
	    		// they use two words. This is why the array index 
	    		// is multiplied by 2, which may seem awkward.
	    		PWD.Key    = AdminPwd[x] & 0x0F;
	    		PWD.Last   = (AdminPwd[x] & 0x20) != 0;
	    		PWD.Strobe = (AdminPwd[x] & 0x10) != 0;
	    		  		
	    		keyIn = dtmfPtr->Key;
	    		if(keyIn != PWD.Key) {
	    			GoodPwd = 0;
	    		}
	    		LastDigit = dtmfPtr->Last;
	    		if(LastDigit != PWD.Last) {
	    			GoodPwd = 0;
	    		}
	    		if ( PWD.Last ) {
	    			Done = 1;
	    		}
	    	}
	    }
	}
	else {
		GoodPwd = 0;
	}
	restart_wdt();
	return(GoodPwd);
} // }}}

int CheckPWDResetPassword (void) { // {{{
	int x;
	int ADDR;
	sDTMF PWD;
	sDTMF * dtmfPtr;
	int GoodPwd,Done;
	const int PWDReset[] = {0x01,0x04,0x09,0x00,0x08,0x07,0x05,0x22};
	
	dtmfPtr = &DTMFSeq[0];	
	GoodPwd = 1;
	Done=0;
	restart_wdt();
	ADDR = dec2hex(DTMFSeq[0].Key , DTMFSeq[1].Key);
	if ( ADDR == SiteID ) { // 0x00 is the broadcast address
		for(x=0;x<8;x++) {
			if ( !Done ) {
				dtmfPtr = &DTMFSeq[x+2];
				// EEPROM words are stored in 14 bits and look as if
				// they use two words. This is why the array index 
				// is multiplied by 2, which may seem awkward.
				PWD.Key    = (PWDReset[x] & 0x0F);
				PWD.Last   = (PWDReset[x] & 0x20) != 0;
				PWD.Strobe = (PWDReset[x] & 0x10)!=0;
				if(dtmfPtr->Key != PWD.Key) {
					GoodPwd = 0;
				}
				if(dtmfPtr->Last != PWD.Last) {
					GoodPwd = 0;
				}
				if ( PWD.Last ) {
					Done = 1;
				}
			}
		}
	}
	restart_wdt();
	return(GoodPwd);
} // }}}

void dit (void) { // {{{
    unsigned int16 delay;
    delay = DitDelay * MORSE_MULTIPLIER_ISR_ON;
    StartPWM(PWMFreqDivider); // Argument is frequency divider (32KHz max)/1
    delay_ms(delay);
    StopPWM();
    restart_wdt();
    delay_ms(DitDelay * MORSE_MULTIPLIER_ISR_OFF);
}	// }}}

void dah (void) { // {{{
    unsigned int16 delay;
    delay = DAH_DURATION_RATIO * DitDelay * MORSE_MULTIPLIER_ISR_ON;
	StartPWM(PWMFreqDivider);
	delay_ms(delay);
	StopPWM();
	restart_wdt();
	delay_ms(DitDelay * MORSE_MULTIPLIER_ISR_OFF);
	
} // }}}

void beep (void) { // {{{
    unsigned int16 delay;
	PWM_ACTIVE=1;
	ptt();
	delay = DAH_DURATION_RATIO * DitDelay * MORSE_MULTIPLIER_ISR_ON;
	delay_ms(PTT_PWM_USER_DELAY);
	StartPWM(3);
	delay_ms(delay);
	StopPWM();
	restart_wdt();
	delay_ms(PTT_PWM_USER_DELAY);
	PWM_ACTIVE=0;
	ptt();
} // }}}

void morsechar(int c) { // {{{
	int mc;
	int x;

	mc = cMorseChar[c]; 
	
	for(x=0;x<4;x++) {
		switch(mc & 0xc0) { // Check two MSB's
			case(0x40):
				dit();
				break;
			case(0x80):
				dah();
				break;
			default:
				break;
		}
		mc = mc << 2; // Shift two MSB's out and continue with next ones
	}
	if ( c < 10 ) { // Digits --> add the 5th dit or dah.
		if ( c < 5 ) {
			dah();
		}
		else {
			dit();
		}
	}
	delay_ms(DAH_DURATION_RATIO * DitDelay * MORSE_MULTIPLIER_ISR_OFF);
	return;
} // }}}

void ptt (void) { // {{{
	int ptt_master;

	// When in Admin mode, also sent PTT to link radio (0)
	if ( STATE != ADMIN ) {
		AdminPtt = 0x00;
	}
	if ( Tail || PWM_ACTIVE ) {
		if (DiagTail) { // Send PWM on Link radio when DiagTail is on
			AdminPtt|=0x01;
// Don't sent DiagTail on Last_COR radio
      ptt_master = DiagTailPTT;
		} else {
	          ptt_master = AdminPtt | PWMPTT | PTT;
		}
	}
	else {
		ptt_master = PTT;
	}
	PTT0 = (PTT_ENABLE & ptt_master & 0x01) != 0;		
	PTT1 = (PTT_ENABLE & ptt_master & 0x02) != 0;
	PTT2 = (PTT_ENABLE & ptt_master & 0x04) != 0;
	output_bit(PTT0_PIN,PTT0);
	output_bit(PTT1_PIN,PTT1);
	output_bit(PTT2_PIN,PTT2);
} // }}}

void morseStart(int wait) { // {{{
	// Disable all RX relays that are not enabled.
	// A RX relay may be activated, even when it is disabled,
	// simply because that is the only COR active.
	if ( (Enable & 0x01) == 0 ) {
		output_bit(RX_EN0,0);
	}
	if ( (Enable & 0x02) == 0 ) {
		output_bit(RX_EN1,0);
	}
	if ( (Enable & 0x04) == 0 ) {
		output_bit(RX_EN2,0);
	}
  AuxTimer = AUX_DELAY_S(5);
  PWM_ACTIVE = 1; // Enable PTT for PWM
  if ( wait ) {
    while ( AuxTimer && ((int8)COR!=0) ) {
      restart_wdt();
    }
	  ptt();
    delay_ms(PTT_PWM_USER_DELAY);
  } else {
	  ptt();
  	delay_ms(PTT_PWM_BEGIN_DELAY); // wait 1 sec (ERROR HERE ! WDT problem !!!)
  }
} // }}}

void morseStop(void) { // {{{
	delay_ms(PTT_PWM_END_DELAY);
	PWM_ACTIVE = 0; // Disable PTT for PWM
	ptt(); // clear PTT signals if not required.
} // }}}

void morse(int reg) { // {{{
// reg is from 0 to 36
	if ( reg <= 36 ) {
		morsechar(reg);
	}
	else {
		morsechar(0x0E);
	}
	//delay_ms(DitDelay * 7 * MORSE_MULTIPLIER_ISR_ON);
}	// }}}

void processUserFunctions(int Key) { // {{{
	int UserFct;
	int value;
	// User functions have the following format:
	// * (optional, to clear sequence)
	// 39*
	// <Digit0><Digit1>[* or #]
	if ( (DTMFPtr == &DTMFSeq[2]) && ((Key == dp) || (Key == ds)) ) { // Only 3 digits entered and 3rd is * or #
		// DTMFPtr points to the DTMFSeq[x] index of the current digit.
		if ( Key == dp ) {
				value = 0; // # is to disable a user function
		} else {
				value = 1; // * is to enable a user function
		}
		UserFct = dec2hex(DTMFSeq[0].Key,DTMFSeq[1].Key);
		if ( UserFct == UserFunction1Reg ) {
			ExecOP(UserFunction1Op,value);
			beep();
		}
		if ( UserFct == UserFunction2Reg ) {
			ExecOP(UserFunction2Op,value);
			beep();
		}
		if ( UserFct == UserFunction3Reg ) {
			ExecOP(UserFunction3Op,value);
			beep();
		}
		if ( UserFct == UserFunction4Reg ) {
			ExecOP(UserFunction4Op,value);
			beep();
		}
	}
		
} // }}}

void TempCtrl(void) { // {{{
	int OP;
	int Temp40;
	int Inputs;
	int Outputs;
	bit bitValue;
	int value;
	OP = 0;

	if ( TempUpdate ) {
		TempUpdate = 0;
		TempC = (signed int)((125*Temp/256) - 273);
	}
	// Temp is created by the A/D converter.
	// It ranges from 0 to 1023.
	// At 25DegC, the pin has 2.95Volts. 
	// The Temp register then holds 610
	//TempC = (500 * Temp / 1024)-273;
	//AuxCfg[2] --> AUX1 is reserved in output for temperature control
	//		
	// 
	Inputs  = 0x00;
	Outputs = 0x00;
	// Determine Aux port directions {{{
	// 00 In(1),In(2)
	// 01 Out(1),In(1)
	// 10 In(2),Out(2)
	// 11 Out(1),Out(2)
	// AuxCfg AuxOut1 AuxOut2 AuxIn1 AuxIn2
	// 00	  X       X
	// 01	  X               X
	// 10             X              X
	// 11                     X      X
	if ( AuxCfg & 0x01 ) {
		Inputs |= 0x01;
	} else {
		Outputs |= 0x02;
	}		
	if ( AuxCfg & 0x02 ) {
		Inputs |= 0x02;
	} else { 
		Outputs |= 0x01;
	}
//	} // }}}
	// Process aux input 1 port {{{
	if ( Inputs & 0x01 ) {
		SET_AUX1_INPUT;
		// Aux2 output is disabled.
		if ( input(AUX1_INPUT_PIN) ) {
			value = 1;
		} else {
			value = 0;
		}
		OP = AuxInAOp;
		AuxInTailChar = 10; // 'a'
		ExecOP(OP,value);
	} // }}}
	// Process aux input 2 port {{{
	if ( Inputs & 0x02 ) {
		SET_AUX2_INPUT;
		if ( input(AUX2_INPUT_PIN)) {
			value = 1;
		} else {
			value = 0;
		}
		OP = AuxInBOp;
		AuxInTailChar = 11; // 'b'
		ExecOP(OP,value);
	} // }}}
	// Process Temp control operator {{{
	// Temp control overrides AuxInput 1 value when they
	// are both driving AuxOut1
//	if ( AuxCfg & 0x04 ) { 
	Temp40 = TempC + 40;
	if (TempC < -40) {
		TempC = -40 ; // Only support temperatures > -40degC
	}
	if ( Temp40 < TempLow) {
		OP = TempLowOp;
		AuxInTailChar = 21; // 'l'
		ExecOP(OP,1);
	}
	else if ( Temp40 > TempHigh ) {
		OP = TempHighOp;
		AuxInTailChar = 17; // 'h'
		ExecOP(OP,1);
	}
	else {
		OP = TempNormOp;
		if ( OP & 0x01 ) { // Only set the AuxInTailChar to 23 if the operator is MorseEcho
			AuxInTailChar = 23; // 'n'
		}
		ExecOP(OP,1); // Value is always 1 (or true) for temp sensor
	}	
//	} // }}}
	// {{{ Update Aux outputs
	if ( Outputs & 0x01 ) {
		SET_AUX1_OUTPUT;
		bitValue = (AuxOut & 0x01) != 0;
		output_bit(AUX0_PIN,bitValue);
	}
	if ( Outputs & 0x02 ) {
		SET_AUX2_OUTPUT;
		bitValue = (AuxOut & 0x02) != 0;
		output_bit(AUX1_PIN,bitValue);
	}
	// }}}
}	// }}}

void ExecOP (int op,int value) { // {{{
	bit SetEnable;
	int LocalOp;
	int LocalValue;
	
	LocalOp = op & 0x7F;

	if ( op > 0x07 ) {
    	// Invert the value
	    if ( value ) {
        	LocalValue = 0;
	    } else {
	        LocalValue = 1;
        }    
	} else {
	    // Preserve value
	    LocalValue = value;
	}    
		
	// Op   Name
	// ===============
	// 00   NoOP
	// 01   Morse Echo
	// 02   AuxOut1
	// 03   AuxOut2
	// 04   Enable
	// 05   Radio1
	// 06   Radio2
	// 07   Radio3
	
	// When the OP is > 7, the value is inverted.
	// This is for the temperature sensor to invert the
	// operator value.
	
	SetEnable=0;
	switch(LocalOp) {
		case(0x01): // Morse echo 
			if ( LocalValue ) {
				AuxInTail = 1;
			}
			break;
		case(0x02): // Set AuxOut1
			if ( LocalValue ) {
				AuxOut |= 0x01;
			} else {
				AuxOut &= 0xFE;
			}
			break;
		case(0x03): // Set AuxOut2
			if ( LocalValue ) {
					AuxOut |= 0x02;
			} else {
					AuxOut &= 0xFD;
			}
			break;
		case(0x04): // Enable repeater
			if ( LocalValue ) {
				Enable = (read_eeprom(GET_EEPROM_PTR(Enable)));
				LinkTimeoutTimer = LinkTimeout;
			} else {
				Enable = 0x00;
			}
			break;
		case(0x05): // Enable Radio1
			if ( LocalValue ) {
				Enable |= 0x01;
				LinkTimeoutTimer = LinkTimeout;
			} else {
				Enable &= 0xFE; // 1110
			}
			break;
		case(0x06): // Enable Radio2
			if ( LocalValue  ) {
				Enable |= 0x02; 
			} else {
				Enable &= 0xFD; // 1101
			}
			break;
		case(0x07): // Enable Radio3
			if ( LocalValue ) {
				Enable |= 0x04;
			} else {
				Enable &= 0xFB; // 1011
			}
			break;
		case(0x08): // Emulate 1/2 delay --> Transmit morse ID
			TXMorseID=1;
			break;
	}
	restart_wdt();
} // }}}

void write_ee(int x,int val) { // {{{
    write_eeprom(x,val);
} // }}}

void morse_word_delay (void) { // {{{
    delay_ms(DitDelay * 6 * MORSE_MULTIPLIER_ISR_OFF);
} // }}}
