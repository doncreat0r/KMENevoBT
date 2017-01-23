///////////////////////////////////////////////////////////////////////////////////////////////////
// Serial/Bluetooth adapter for KME Nevo (Pro) LPG ECU
// (c) Dmitry 'Creat0r' Bobrik, 2015
//
// HW req: ATMega324P/PA or similar with 2 UARTs, Bluetooth HC-04/05/06 module (optional)

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

#include "avrlibdefs.h"
#include "avrlibtypes.h"
#include "tables.h"
#include "onewire.h"
 
#define T0delay (255 - 108)
#define T2delay (255 - 108)

// modes
// 1, 2, 3, 4, ... - current cmd to xmit
// and modeWait Nth bit
#define modeZero 0
#define modeSearch 1				// automatic search for KME Nevo ECU 
#define modeGetOBD 2				// send "GET OBD Config" command
#define modeSetOBD 3				// set required PIDs for OBD (may be wait until OBD initialization?)
#define modeReadLPG 4				// transmit 'read current values' command
#define modeReadOBD 5				// transmit 'read OBD' command
#define modeReadOSA 6				// transmit 'read OSA Bank 1' command
#define modeWait 8					// wait for response or timeout, than switch to modeSearching or modeReadXXX based on response buffer 
#define modeTransparent 32			// transparent passsthrough mode
#define modePassthrough 48			// passthrough mode until reboot (explicitly selectable, for tuning the Bluetooth module, etc.)
#define modeParkAssist 64			// used only for DATA[2]	

#define modeMask 0b111				// modeWait-1

#define pcNone 0
#define pcRcvCmd 1					// receiving cmd from PC
#define pcCmdOK 2					// entire cmd received from PC

#define pcCmdRead 0
#define pcCmdAddLPG 1
#define pcCmdAddPET 2
#define pcCmdResetTrip 3
#define pcCmdSetLPGInjFlow 8
#define pcCmdSetPETInjFLow 9
#define pcCmdSetSpeedCorr 10
#define pcCmdReboot 0xFF

// what to read from LPG ECU
#define PCreadLive 1				// read live LPG and OBD data
#define PCreadOSA 2					// read OSA table as well

// Why all vars are volatile? 
// Because I need to know total memory consumption w/o any optimizations 
// in case of a large chuck of code being commented out for testing purposes

// work mode
volatile u08 PTmode = modeSearch;	// current work mode

// KME communication
volatile u08 KMEBuff[150];			// buffer to receive data from KME Nevo ECU
volatile u08 KMEidx = 0;			// current index in buffer
volatile u08 KMEsnd = 0;			// send buffer current byte2send index
volatile u08 KMEcs = 0;				// checksum
volatile u08 KMEgr = 0;				// 1 - got full and valid response to any KME request
//volatile s08 KMEcmd = -1;			// pending request cmd
volatile u08 KMEsendbuf = 0;		// send PCBuff contents to KME ECU (during 1st command catch)
volatile u08 KMEtimeout = 0;		// timeout in 10 ms ticks, up to 2.55 s
volatile u16 KMEtimeLPG = 0;		// calculated time between two LPG responces (in 10ms ticks, usually ~50 = 0.5s)
volatile u16 KMEtimeOBD = 0;		// calculated time between two OBD responces

// PC communication
volatile u08 PCBuff[150];
volatile u16 PPCBuff = (u16)&PCBuff[0];
volatile u08 PCsend[100];
volatile u08 PChead = 0, PCtail = 0;
volatile u08 PCidx = 0;
volatile u08 PCcs = 0;
volatile u08 PCgr = 0;
volatile u08 PCtimeout = 0;
volatile u08 PCCmdTimeout = 0;
// response queue
volatile u08 PCreq = 0;				// bitmask of responses pending
volatile u08 PCread = 0;			// when PC read > 0 we're reading data from ECU and constantly send responses to PC

//
volatile u08 startCalc = 0; 		// start calc after RPM > 0
volatile u08 tempMode = 0;			// tempMode: 0 - skiprom/begin measure, 1 - wait for measure end, 2 - read temp value

// park assist decoder
volatile u08 PAstep = 0; // steps are: sync, pause, 1st byte, 2nd byte, pause, 3rd byte, 4th byte, last bit, finish
volatile u08 PAcnt = 0;
volatile u08 PAchanged = 0;
volatile u16 PAcur[2]; // raw data from PA unit
volatile u16 PAold[2]; // 

// EEPROM vars
volatile u08 eepromDeadZone[202] EEMEM = {209, 176, 0};
volatile u32 eepromLPGTank EEMEM = 0;
volatile u32 eepromPETTank EEMEM = 0;
volatile u16 eepromUpdCount EEMEM = 0;
volatile u08 eepromDeadZone2[200] EEMEM = {0, 0, 0};
volatile s08 eepromSpeedCor EEMEM = 1;
volatile u08 eepromLPGFlow EEMEM = 225;
volatile u08 eepromPETFlow EEMEM = 177;

// ======================================================
// Send byte to PC
void PCSend(u08 data)
{
	u08 _sreg = SREG; 
	cli();
	PCsend[PChead] = data;
	PChead++;
	if (PChead >= sizeof(PCsend)) PChead = 0;
	SREG = _sreg;
}

static inline void PCSendInt(u08 data)	// send called from ISR handler
{
	PCsend[PChead] = data;
	PChead++;
	if (PChead >= sizeof(PCsend)) PChead = 0;
}

static inline void StartT0(void) {
	cbi(TIMSK0, TOIE0);	   // disable overflow
	sbi(TIFR0, TOV0);						// reset overflow flag
	PCtimeout = 0;
	TCNT0 = T0delay; // reload timer overflow
	sbi(TIMSK0, TOIE0);	   // enable overflow
}

static inline void StartT2(void) {
	cbi(TIMSK2, TOIE2);	   // disable overflow
	sbi(TIFR2, TOV2);						// reset overflow flag
	KMEtimeout = 0;
	TCNT2 = T2delay; // reload timer overflow
	sbi(TIMSK2, TOIE2);	   // enable overflow
}

// ======================================================
ISR(PCINT3_vect) {
	u16 tmr = TCNT1;
	u08 prk = PIND & BV(PARK);
	TCNT1 = 0;		// always rearm the timer
//	if (!prk)  sbi(DF.LPGstatusBits, STATUS_PARKMODE_ACTIVE);  // don't activate on zero due to some random interference
	switch (PAstep) {
		case 0: // no sync, got 1st positive front
			if (!prk) {
				PAstep++;  PAcnt = 0;  // next step is sync count, zero sync counter 
			}
			break;
		case 1:
			if (tmr > 12 && tmr < 28)  // if time from last change between 0,37 ms and 0,64 ms
				PAcnt++;
			else
				PAstep = 0;
			if (PAcnt > 9)  PAstep++;
			break;
		case 2: // expect a long pulse
			if (tmr > 40 && tmr < 48 && (prk))  // long pulse is 1ms
				PAstep++;
			else 
				PAstep = 0;
			break;
		case 3: // expect a 2ms pause - assuming we are at 1st positive edge of 1st message bit
			if (tmr > 80 && tmr < 92 && !prk) {  // long pause is 2ms
				PAstep++;
				PAcnt = 0;
				PAcur[0] = 0;
			} else 	
				PAstep = 0;
			break;
		case 4: // decode 1st word - assuming we are at Nth bit negative edge
			if ((prk)) {
				if (tmr > 10 && tmr < 22) {  // zerobit is about 0.33 ms, onebit is about 0.66 ms
					PAcur[0] |= (1<<PAcnt);
					PAcnt++;
				} else {
					if (tmr > 23 && tmr < 33)   // zerobit is about 0.33 ms, onebit is about 0.66 ms
						PAcnt++;
					else 
						PAstep = 0;
				}
			}
			if (PAcnt > 15)  PAstep++;
			if (tmr > 200)  PAstep = 0;
			break;
		case 5:  // expect another pause for ~4.3 ms
			if (tmr > 172 && tmr < 212 && !prk) {  // another long pause is ~4.3ms
				PAcnt = 0;
				PAcur[1] = 0;
				PAstep++;
			} else 
				PAstep = 0;
			break;
		case 6:
			if ((prk)) {
				if (tmr > 10 && tmr < 22) {  // zerobit is about 0.33 ms, onebit is about 0.66 ms
					PAcur[1] |= (1<<PAcnt);
					PAcnt++;
				} else {
					if (tmr > 23 && tmr < 33)   // zerobit is about 0.33 ms, onebit is about 0.66 ms
						PAcnt++;
					else 
						PAstep = 0;
				}
			}
			if (PAcnt > 15)  PAstep++;
			if (tmr > 200)  PAstep = 0;
			break;
		case 7:
			DP.P1 = PAcur[0];
			DP.P2 = PAcur[1];
			PAchanged = 1;
			PAstep++;
			sbi(DF.LPGstatusBits, STATUS_PARKMODE_ACTIVE);  // activate PARKMODE_ACTIVE only when got the actual data
			break;
		default:
			PAstep = 0;
			PAcnt = 0;
	}
}

// ======================================================
// ======================================================
ISR(USART1_RX_vect)
{
	u08 RS;

	RS = UDR1;
	if (PTmode >= modeTransparent && !(PTmode & modeWait))  UDR0 = RS;   // pass bytes to PC directly while in transparent/passthrough mode
	if (PTmode < modePassthrough) {
		// if we're in modeSearch and receiving the response - store the response to "search" cmd into the buffer
		if ((PTmode & modeMask) == modeSearch && KMEidx < 20)  KMEcmds[modeSearch][KMEidx + 16] = RS;
		KMEBuff[KMEidx++] = RS;  // ... try to process the response as well
		// we have a response with a valid checksum and num of bytes received
		if ((!KMEgr) && (KMEBuff[0] == bRespKME) && (KMEidx > 1) && (KMEidx >= KMEBuff[1]) && (RS == KMEcs)) {
			KMEidx = 0;  
			KMEcs = 0;  
			KMEgr = 1;  // "got response" signal processed in main cycle
		} else {
			KMEcs += RS;
		}
		// prevent buffer overflow
		if (KMEidx >= sizeof(KMEBuff)) {
			KMEidx = 0;
			KMEcs = 0;
		}
	}
}

// ======================================================
ISR(USART1_TX_vect)
{
}

// ======================================================
ISR(USART1_UDRE_vect)
{
	u08 cmd;

	cmd = (PTmode & modeMask);
	// if not in transparent mode - send cmd 
	if ((PTmode < modeTransparent) && (KMEsnd < KMEcmds[cmd][1])) {
		if (!KMEsnd) StartT0();
		UDR1 = KMEcmds[cmd][KMEsnd++];  // don't need to prevent overflow, we're not writing to the buffer
	} else {
		cbi(UCSR1B, UDRIE1);   // disable UDR Empty Interrupt
		KMEidx = 0; KMEcs = 0; // also reset response buffer indexes
		KMEsnd = 0;
//		PCSend(PTmode + 0x40);
	}
}

// ======================================================
// ======================================================
// ======================================================
ISR(USART0_UDRE_vect)
{
	if (PCtail != PChead)
	{
		UDR0 = PCsend[PCtail++];
		if (PCtail >= sizeof(PCsend)) PCtail = 0;
	}
	else
		cbi(UCSR0B, UDRIE0);  // disable UDR Empty Interrupt
}

// ======================================================
ISR(USART0_TX_vect)
{
	cbi(UCSR0B, UDRIE0);  // disable UDR Empty Interrupt
}


// ======================================================
// Received byte from PC interrupt handler
ISR(USART0_RX_vect)
{
	u08 RS, i;

	RS = UDR0;
	PCBuff[PCidx++] = RS; 
	// if not transparent mode and got 1st byte and its the request start - go to transparent mode (temporarly)
	if ((PTmode < modeTransparent) && (PCidx == 1)) {
		switch (RS) {
	  		case bReqKME: PTmode = modeTransparent | modeWait; PCread = 0; break;
	  		case  bReqPC: PTmode = modePassthrough; PCread = 0; break;
			case bReqBuf: PCgr = pcRcvCmd; break;
			default: PCidx = 0; break; // just ignore the garbage as a 1st symbol
	  	}
	}
	// if receiving cmd from own custom software
	if (PCgr) {
		if (PCidx >= 7) {  // if got required 7 bytes
			if (RS == PCcs) {
				PCgr = pcCmdOK;
			}
			PCcs = 0; PCidx = 0;
		} else {
			PCcs += RS;
		}
	}
	// if receiving data from KME Nevo software
	if (PTmode >= modeTransparent) {
		if (!(PTmode & modeWait))  UDR1 = RS;  // just feed the byte strait to ECU
		if (PTmode & modeTransparent) {
			// if we got enough bytes according to the request data
			if ((PCidx > 1) && (PCidx >= PCBuff[1])) {
				// if we have valid checksum - set signal "got request", else reset PTmode
				if (RS == PCcs) {
					StartT0();             // start T0 timer, so if no requests from KME software in 1s - revert to own requests
					KMEidx = 0; KMEcs = 0; // also reset response buffer indexes
					// if we've received 0x01 cmd - just send the response and switch to pure modeTransparent
					if (PCBuff[2] == 0x01 && (PTmode & modeWait)) {
						PTmode = modeTransparent;
						for (i = 16; i < 34; i++ )  PCSendInt(KMEcmds[modeSearch][i]);
					}
				}
				PCcs = 0;  PCidx = 0;   // reinit index and checksum 
			} else {
				PCcs += RS;       // update checksum
			}
		}
	}
	// prevent PCbuff overflow
	if (PCidx >= sizeof(PCBuff)) {
		PCidx = 0;
		PCcs = 0;
	}
}

/////////////////////////////////////////////////////////
void InitParams(void) {

	UBRR1L = LO(brd9600);
	UBRR1H = HI(brd9600);
	UCSR1A = 0;
	UCSR1B = 1<<RXEN1|1<<TXEN1|1<<RXCIE1|1<<TXCIE1;
	UCSR1C = 1<<UCSZ01|1<<UCSZ00|0<<USBS1; 

	// Init USART0 - PC comm. 38400/8N1
	UBRR0L = LO(brd9600);
	UBRR0H = HI(brd9600);
	UCSR0A = 0;
	UCSR0B = 1<<RXEN0|1<<TXEN0|1<<RXCIE0|1<<TXCIE0;
	UCSR0C = 1<<UCSZ01|1<<UCSZ00|0<<USBS1; 

	DATAtimer = 100*28;
	DR.LPGinjFlow = 225;  
	DR.PETinjFlow = 177; // doubled in the formula
	DR.SpeedCorr = 0;
	DF.LPGstatusBits = 0;

	DF.id = 0x42;
	DF.length = sizeof(DF);
	DF.type = BV(RESP_FAST_BIT);

	DS.id = 0x42;
	DS.length = sizeof(DS);
	DS.type = BV(RESP_SLOW_BIT);

	DR.id = 0x42;
	DR.length = sizeof(DR);
	DR.type = BV(RESP_RARE_BIT);

	DP.id = 0x42;
	DP.length = sizeof(DP);
	DP.type = BV(RESP_PARK_BIT);

	DO.id = 0x42;
	DO.length = sizeof(DO);
	DO.type = BV(RESP_OSA1_BIT);

}

/////////////////////////////////////////////////////////
void ReadParamsEEPROM(void) {
	u08 tmp;
	u32 tmp32;

	eeprom_busy_wait();
	tmp = eeprom_read_byte((u08*)&eepromLPGFlow);
	if (tmp < 0xFF)  DR.LPGinjFlow = tmp;
	tmp = eeprom_read_byte((u08*)&eepromPETFlow);
	if (tmp < 0xFF)  DR.PETinjFlow = tmp;
	tmp = eeprom_read_byte((u08*)&eepromSpeedCor);
	if (tmp < 0xFF)  DR.SpeedCorr = tmp;
	DR.eepromUpdateCount = eeprom_read_word((uint16_t*)&eepromUpdCount);
	tmp32 = eeprom_read_dword((u32*)&eepromLPGTank);
	if (tmp32 < 0xFFFFFFFF)  DS.totalLPGInTank = tmp32;
	tmp32 = eeprom_read_dword((u32*)&eepromPETTank);
	if (tmp32 < 0xFFFFFFFF)  DS.totalPETInTank = tmp32;
}

/////////////////////////////////////////////////////////
void WriteParamsEEPROM(int withFlow) {
	u08 _sreg = SREG, i = 0;
	u32 _totalLPGInTank = 0, _totalPETInTank = 0;
	u16 _eepromUpdateCount = 0;
	cli();
	// slow down the clock while writing to EEPROM
	CLKPR = (1<<CLKPCE);
	CLKPR = (1<<CLKPS2);  // set clock divider to 16
	nop();nop();
	if (withFlow) {
		eeprom_busy_wait();
		eeprom_update_byte((u08*)&eepromLPGFlow, DR.LPGinjFlow);
		eeprom_busy_wait();
		eeprom_update_byte((u08*)&eepromPETFlow, DR.PETinjFlow);
		eeprom_busy_wait();
		eeprom_update_byte((u08*)&eepromSpeedCor, DR.SpeedCorr);
	}
	// trying to write to EEPROM up to three times if case any of vars was read back with a wrong value
	do {
		eeprom_busy_wait();
		eeprom_update_dword((u32*)&eepromLPGTank, DS.totalLPGInTank);
		eeprom_busy_wait();
		eeprom_update_dword((u32*)&eepromPETTank, DS.totalPETInTank);
		eeprom_busy_wait();
		eeprom_update_word((uint16_t*)&eepromUpdCount, ++DR.eepromUpdateCount);
		eeprom_busy_wait();
		_totalLPGInTank = eeprom_read_dword((u32*)&eepromLPGTank);
		_totalPETInTank = eeprom_read_dword((u32*)&eepromPETTank);
		_eepromUpdateCount = eeprom_read_word((uint16_t*)&eepromUpdCount);
	} while ((DS.totalLPGInTank != _totalLPGInTank || DS.totalPETInTank != _totalPETInTank || DR.eepromUpdateCount != _eepromUpdateCount) && (++i <= 3));
	// clock back to nominal
	CLKPR = (1<<CLKPCE);
	CLKPR = 0;
	EEAR = 0xFF;			// atmega324 doesn't have any errata, but we'll try this trick anyway
	sbi(PCreq, RESP_RARE_BIT);
	SREG = _sreg;
}

/////////////////////////////////////////////////////////
ISR(SIG_OVERFLOW0) {

	TCNT0 = T0delay;  // reload timer overflow
	PCtimeout++;

	// if we have timeout reached
	if (PCtimeout > 200) {  // 1s timeout
		PCtimeout = 0;
		PCidx = 0;  PCcs = 0;
		cbi(TIMSK0, TOIE0);	 // disable overflow
		// now next cmd
		if (PTmode & modeWait) {
			PTmode = PTmode & modeMask;  // if we're in transparent&wait - we'll go to modeZero with this code
		} else {
			if (PTmode == modeTransparent)  PTmode = modeSearch;  // if timeout in transparent - means no frames, so switch to search mode
		}
	}
}

/////////////////////////////////////////////////////////
ISR(SIG_OVERFLOW1) {
	PAstep = 0;
	PAcnt = 0;
}

/////////////////////////////////////////////////////////
ISR(SIG_OVERFLOW2) {

	TCNT2 = T2delay;  // reload timer overflow

	if (((PTmode & modeMask) > modeSetOBD) || (PTmode == modeTransparent)) {
		KMEtimeLPG++;
		KMEtimeOBD++;
		// if got timeout receiving 
		if (KMEtimeLPG > 100*10)  {
			PTmode = modeSearch;
			if (startCalc) {
				startCalc = 0;
				DF.LPGRPM = 0;
				WriteParamsEEPROM(0);
			}
		}
	}
	// timeout receiving cmd from own custom software
	if (PCgr) {
		PCCmdTimeout++;
	}
	if (PCCmdTimeout > 10) {
		PCCmdTimeout = 0;
		PCidx = 0; PCgr = pcNone; PCcs = 0;
	}
	// used for 1wire temperature readings
	DATAtimer++;
}

/////////////////////////////////////////////////////////
ISR(BADISR_vect)
{
// user code here
}

/////////////////////////////////////////////////////////
u16 GetInjMax(u16 inj1, u16 inj2, u16 inj3, u16 inj4) {
	u16 tmp = inj1;
	if (inj2 > tmp) tmp = inj2;
	if (inj3 > tmp) tmp = inj3;
	if (inj4 > tmp) tmp = inj4;
	return tmp;
}
/////////////////////////////////////////////////////////
u16 GetInjMin(u16 inj1, u16 inj2, u16 inj3, u16 inj4) {
	u16 tmp = inj1;
	if (inj2 < tmp) tmp = inj2;
	if (inj3 < tmp) tmp = inj3;
	if (inj4 < tmp) tmp = inj4;
	return tmp;
}

/////////////////////////////////////////////////////////
static inline void ParseLPGResponse() {
	u16 tmp, inj1, inj2, inj3, inj4;
	
	cli(); // disable int for critical params
	// reset timer counter
	tmp = KMEtimeLPG;  KMEtimeLPG = 0;
	// process LPG data ---------------
	// injector time recalc
	inj1 = (KMEBuff[3]<<8) + KMEBuff[4]; 
	inj2 = (KMEBuff[5]<<8) + KMEBuff[6]; 
	inj3 = (KMEBuff[7]<<8) + KMEBuff[8]; 
	inj4 = (KMEBuff[9]<<8) + KMEBuff[10]; 
	DF.PETsuminjtime = inj1 + inj2 + inj3 + inj4;
	// if injector max time much longer than others OR min time shorter than others - set alarm bit
	if ( DF.PETsuminjtime && 
		(GetInjMax(inj1, inj2, inj3, inj4) / (DF.PETsuminjtime/40) > MaxInjDeviation || 
		 GetInjMin(inj1, inj2, inj3, inj4) / (DF.PETsuminjtime/40) < MinInjDeviation) ) 
		sbi(DF.LPGstatusBits, STATUS_PET_TIME_MISMATCH);
	else
		cbi(DF.LPGstatusBits, STATUS_PET_TIME_MISMATCH);
	inj1 = (KMEBuff[19]<<8) + KMEBuff[20]; 
	inj2 = (KMEBuff[21]<<8) + KMEBuff[22]; 
	inj3 = (KMEBuff[23]<<8) + KMEBuff[24]; 
	inj4 = (KMEBuff[25]<<8) + KMEBuff[26]; 
	DF.LPGsuminjtime = inj1 + inj2 + inj3 + inj4;
	if ( DF.LPGsuminjtime && 
		(GetInjMax(inj1, inj2, inj3, inj4) / (DF.LPGsuminjtime/40) > MaxInjDeviation || 
		 GetInjMin(inj1, inj2, inj3, inj4) / (DF.LPGsuminjtime/40) < MinInjDeviation) ) 
		sbi(DF.LPGstatusBits, STATUS_LPG_TIME_MISMATCH);
	else
		cbi(DF.LPGstatusBits, STATUS_LPG_TIME_MISMATCH);
	DF.LPGRPM = (KMEBuff[35]<<8) + KMEBuff[36];
	DF.LPGPcol = (KMEBuff[41]<<8) + KMEBuff[42];
	DF.LPGPsys = (KMEBuff[43]<<8) + KMEBuff[44];
	DS.LPGTred = (KMEBuff[47]<<8) + KMEBuff[48];
	DS.LPGTgas = (KMEBuff[49]<<8) + KMEBuff[50];
	DF.LPGVbat = (KMEBuff[53]<<8) + KMEBuff[54];
	DF.LPGStatus = KMEBuff[58];
	corrPres = KMEBuff[106];
	corrTemp = KMEBuff[107];
	DI.LPGLoad = KMEBuff[112];
	// petrol switch strategies
	if ((KMEBuff[96] & 0x08) || (KMEBuff[97] & 0x3F) || (KMEBuff[98] & 0x06)) {
		sbi(DF.LPGstatusBits, STATUS_PETROL_SWITCH);
		DF.LPGStatus = 4;  // force  "petrol warming" mode
	} else
		cbi(DF.LPGstatusBits, STATUS_PETROL_SWITCH);
	sei();
	// start calculating fuel consumption if engine load > 0 (engine started)
	// stop calculating if load = 0 and RPM < 1000 (engine idling), fuel cutoff mode shouldn't activate below 1000
	// save calculated to EEPROM if calculation was started
	if (DI.LPGLoad) {
		startCalc = 1;
	} else 
		if (DF.LPGRPM < 1000) {
			if (startCalc) { WriteParamsEEPROM(0); }
			startCalc = 0;
		}
	if (startCalc)  CalcFuel(tmp);
	sbi(PINC, PLED);
}

/////////////////////////////////////////////////////////
static inline void ParseOBDResponse() {

	cli();   // disable int for critical params
	KMEtimeOBD = 0;
	DF.OBDRPM = (KMEBuff[31]<<8) + KMEBuff[32];
	DF.OBDSpeed = KMEBuff[33];
	// if speed correction value available
	if (DR.SpeedCorr) DF.OBDSpeed += (KMEBuff[33] + ((50 / DR.SpeedCorr) & 0x7F)) / (100 / DR.SpeedCorr);
	sei();
	DF.OBDLoad  = KMEBuff[34];
	DS.OBDECT   = KMEBuff[35];
	DF.OBDMAP   = KMEBuff[36];
	DF.OBDTA    = KMEBuff[37];
	DS.OBDIAT   = KMEBuff[38];
	DF.OBDSTFT  = KMEBuff[27];
	DF.OBDLTFT  = KMEBuff[28];
	DF.OBDerror = KMEBuff[26];
	DF.OBDTPS   = KMEBuff[41];
	if ((KMEBuff[42] & 0x0F) == 2)
		sbi(DF.LPGstatusBits, STATUS_OBD_CLOSED_LOOP);
	else
		cbi(DF.LPGstatusBits, STATUS_OBD_CLOSED_LOOP);
	sbi(PINC, PLED);
}
/////////////////////////////////////////////////////////
static inline void ParseOSAResponse() {
// copy the OSA Bank 1 to KMEcmds after the request bytes
	for (u08 j = 0; j < 25; j++) {
		DO.OSA[j] = KMEBuff[j + 3];
	}
}

/////////////////////////////////////////////////////////
void ReadTemperature() {
	u08 temp[9];
	u08 i;

	if (DATAtimer > 100*30){
		// time to read temperature
		switch (tempMode) {
    		case 0: 	ow_reset();
						write_byte(THERM_CMD_SKIPROM);
						write_byte(THERM_CMD_CONVERTTEMP);
						tempMode = 1;
						break;
		}
	}
	if (DATAtimer > 100*32){
		ow_reset();
		write_byte(THERM_CMD_SKIPROM);
		write_byte(THERM_CMD_RSCRATCHPAD);
		for (i=0;i<9;i++)  temp[i] = read_byte();
		// if CRC matches - convert T and put it into buffer
		if (temp[8] == calc_crc(&temp[0], 8)) {
			DS.outsideTemp = (temp[0]<<8) + temp[1];
		} 
		DATAtimer = 0;
		tempMode = 0;
	}
}

/////////////////////////////////////////////////////////
void ReadParkAssist() {
	u08 crc1, crc2;

	cli();
	PAchanged = 0;
	//if (PAold[0] != PAcur[0] || PAold[1] != PAcur[1]) 
	{
		PAold[0] = PAcur[0];
		PAold[1] = PAcur[1];
		sei();  // rls ints ASAP
		DP.CM = 0x0F - ( ((PAold[0]>>2) & 0x0C) + ((PAold[0]>>12) & 0x03) );

		DP.A = (PAold[1]>>12) & 0x0F;
		DP.B = ((PAold[0] & 7)<<2) + ((PAold[0]>>14) & 0x03);
 		DP.C = ((PAold[0]>>6) & 0x03) + (((PAold[0]>>8) & 0x07)<<2);
 		DP.D = ((PAold[1]>>8) & 0x0F);

		crc1 = (((DP.C & 0x0F) + (DP.B & 0x0F) + DP.D) - 1); 
		crc2 = ((crc1>>4) ^ 3) - (((DP.A>>2) & 0x0F) + ((DP.B>>2) & 0x0F) + ((DP.D>>4) & 0x0F));

		DP.A = 0x0F - DP.A;
		DP.B = 0x1F - DP.B;
		DP.C = 0x1F - DP.C;
		DP.D = 0x0F - DP.D;

		DP.status = 0;
		DP.status |= (DP.A != 0x0F - 2)<<0;
		DP.status |= (DP.B != 0x1F - 4)<<1;
		DP.status |= (DP.C != 0x1F - 4)<<2;
		DP.status |= (DP.D != 0x0F - 2)<<3;

		DP.status |= ((crc1 & 0x0F) == (PAold[1] & 0x0F))<<4;
		DP.status |= ((crc2 & 0x0F) == ((PAold[1]>>4) & 0x0F))<<5;

		sbi(PCreq, RESP_PARK_BIT); // send PA buffer
	}
	sei();
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
int main (void) {

	u08 i, tmp, tempEnabled;
	u08 fastCnt = 0;

	wdt_disable();
	InitParams();
	ReadParamsEEPROM();
	// is BODLEVEL enabling the brown-out detection?
	//ReadParamsEEPROM();

	cbi(DDRD, PARK);
	sbi(DDRC, PLED);
	sbi(DDRA, P_DS);
	sbi(PORTA, P_DS);           
	// timers
	TCCR0A = 0;
	TCCR0B = 0b101;		// prescaler/1024
	cbi(TIMSK0, TOIE0);	 // disable overflow
	sbi(TIFR0, TOV0);
	TCCR2A = 0;
	TCCR2B = 0b111;		// BEWARE! Different set of bits for prescaler/1024 on T0 and T2 !!!
	cbi(TIMSK2, TOIE2);	 // disable overflow
	sbi(TIFR2, TOV2);

	// park assist init
	PCMSK3 = (1<<PCINT28);
	PCICR = (1<<PCIE3);

	TCCR1A = 0;
	TCCR1B = 0b100;		// prescaler/256
	sbi(TIFR1, TOV1);						// reset overflow flag
	TCNT1 = 0; // reload timer overflow
	sbi(TIMSK1, TOIE1);	   // enable overflow

	sei();
	StartT2();  // start 10ms counting helper timer immediately

	tempEnabled = ow_reset();
	if (!tempEnabled)  tempEnabled = ow_reset();  // 2nd attempt
		

	while (1) {
		if (tempEnabled)  ReadTemperature();
		if (DATAtimer > 100*40){
			DATAtimer = 0;
			tempMode = 0;
		}
		// park assist data
		if (PAchanged)  ReadParkAssist();
		if ((DF.LPGstatusBits & BV(STATUS_PARKMODE_ACTIVE)) && TCNT1 > 30000) {
			cbi(DF.LPGstatusBits, STATUS_PARKMODE_ACTIVE);  DP.status = 0; 
		}
		// incoming commands
		if (PCgr == pcCmdOK) {
			cli();  tmp = PCBuff[1];  PCgr = pcNone;  sei();
			switch (tmp) {
				case pcCmdRead: 		PCread = PCBuff[2];   // 0 - stop sending data to PC, 1 - readLive, 2 - readOSA
										sbi(PCreq, RESP_RARE_BIT); break; 
				case pcCmdAddLPG: 		cli(); DS.totalLPGInTank += DWORD(PPCBuff, 2); WriteParamsEEPROM(0); sei(); 
										break;
				case pcCmdAddPET: 		cli(); DS.totalPETInTank += DWORD(PPCBuff, 2); WriteParamsEEPROM(0); sei(); 
										break;
				case pcCmdResetTrip: 	cli();
										DS.tripLPGSpent = 0; DS.tripLPGTime = 0; DS.tripLPGDist = 0; 
										DS.tripPETSpent = 0; DS.tripPETTime = 0; DS.tripPETDist = 0;
										sei();
										break;
				case pcCmdSetLPGInjFlow: 
										DR.LPGinjFlow = PCBuff[2]; 
										WriteParamsEEPROM(1); 
										sbi(PCreq, RESP_RARE_BIT); break;
				case pcCmdSetPETInjFLow: 
										DR.PETinjFlow = PCBuff[2]; 
										WriteParamsEEPROM(1);
										sbi(PCreq, RESP_RARE_BIT); break;
				case pcCmdSetSpeedCorr:
										DR.SpeedCorr = PCBuff[2]; 
										WriteParamsEEPROM(1);
										sbi(PCreq, RESP_RARE_BIT); break;
				case pcCmdReboot: 		WriteParamsEEPROM(0);   //save spent fuel before reboot
										PCSend('$');
										sbi(UCSR0B, UDRIE0);  				// explicitly enable UDRIE0 before reset
										wdt_enable(1); cli(); while(1); break;

			} // switch
		}
		// this status is set if we've got valid response from KME, doesn't matter who's requested it
		if (KMEgr) {
			KMEgr = 0;
			switch (KMEBuff[2]) {
				case bRespLPG:  // got 0x06 (LPG info) 
					ParseLPGResponse();
					sbi(PCreq, RESP_FAST_BIT);
					fastCnt++; // byte overflow 
					// also transmit SLOW data after each four FAST data
					if ((fastCnt & 0b11) == 0b11) 
						sbi(PCreq, RESP_SLOW_BIT);
				    // also transmit PARK data if parmode after each eight FAST data
					if ((DF.LPGstatusBits & BV(STATUS_PARKMODE_ACTIVE)) && ((fastCnt & 0b111) == 0b111))
						sbi(PCreq, RESP_PARK_BIT); // send PA buffer
					break;
				case bRespOBD:  // got 0xB1 (OBD info) 
					ParseOBDResponse();
					break;
				case bRespOSA:
					ParseOSAResponse();
					sbi(PCreq, RESP_OSA1_BIT);
					// TODO: May be check if OSA table has been changed before just sending it everytime?
					break;
			}
			// check for valid response to our own request and switch to next request 
			if (PTmode < modeTransparent) {
			    PTmode = PTmode & modeMask;
				switch (PTmode) {
					case modeSearch: if ((KMEBuff[0] == bRespKME) && (KMEBuff[1] == 0x11) && (KMEBuff[2] == 0x01)) PTmode++; break;
					case modeGetOBD:
						if ((KMEBuff[0] == bRespKME) && (KMEBuff[1] == 0x26) && (KMEBuff[2] == 0xB3)) {
							cli();
							// now copy current OBD config (except 3 bytes of PID config) to SetOBD request
							KMEcmds[modeSetOBD][37] = 0;
							for (i=0; i<37; i++) {
								if (i > 5) KMEcmds[modeSetOBD][i] = KMEBuff[i];
								KMEcmds[modeSetOBD][37] += KMEcmds[modeSetOBD][i];
							}
						 	PTmode++; 
							sei();
						}
						break;
					case modeReadOBD: if (PCread == PCreadOSA)  PTmode = modeReadOSA; else PTmode = modeReadLPG; break;
					case modeReadOSA: PTmode = modeReadLPG; break;
					default: PTmode++; break;
				}
			}
		}  // KMEgr
		// if current mode w/o wait bit - set wait bit and initiate request sending
		if (PTmode < modeWait) {
			PTmode = PTmode | modeWait;
			sbi(UCSR1B, UDRIE1);
		}
		// handling PC response buffer sending
		// TODO: some kind of a function here...
		if (PCread) {
			if (PCreq & BV(RESP_PARK_BIT)) {
				DP.checkSum = 0;
				for (i = 0; i < DP.length; i++) {
					if (!PCread) break;
					tmp = *(volatile u08*)((void *)&DP + i);
					PCSend(tmp);
					DP.checkSum += tmp;
				}
				cbi(PCreq, RESP_PARK_BIT);
			} else
			if (PCreq & BV(RESP_RARE_BIT)) {
				DR.checkSum = 0;
				for (i = 0; i < DR.length; i++) {
					if (!PCread) break;
					tmp = *(volatile u08*)((void *)&DR + i);
					PCSend(tmp);
					DR.checkSum += tmp;
				}
				cbi(PCreq, RESP_RARE_BIT);
			} else
			if (PCreq & BV(RESP_SLOW_BIT)) {
				DS.checkSum = 0;
				for (i = 0; i < DS.length; i++) {
					if (!PCread) break;
					tmp = *(volatile u08*)((void *)&DS + i);
					PCSend(tmp);
					DS.checkSum += tmp;
				}
				cbi(PCreq, RESP_SLOW_BIT);
			} else
			if (PCreq & BV(RESP_FAST_BIT)) {
				DF.checkSum = 0;
				DF.workMode = PTmode;
				for (i = 0; i < DF.length; i++) {
					if (!PCread) break;
					tmp = *(volatile u08*)((void *)&DF + i);
					PCSend(tmp);
					DF.checkSum += tmp;  // so the idea is to send the last byte as checksum BEFORE adding it to itself
				}
				cbi(PCreq, RESP_FAST_BIT);
			}
			if (PCreq & BV(RESP_OSA1_BIT)) {
				DO.checkSum = 0;
				DO.workMode = PTmode;
				for (i = 0; i < DO.length; i++) {
					if (!PCread) break;
					tmp = *(volatile u08*)((void *)&DO + i);
					PCSend(tmp);
					DO.checkSum += tmp;  // so the idea is to send the last byte as checksum BEFORE adding it to itself
				}
				cbi(PCreq, RESP_OSA1_BIT);
			}
		} // PCread
		if (PChead != PCtail)  sbi(UCSR0B, UDRIE0);  // enable UDR Empty Interrupt
	}
	return 0;
}
