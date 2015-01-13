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
// 0, 1, 2, 3 - current cmd to xmit
// 4 - modeWait bit
#define modeSearch 0				// automatic search for KME Nevo ECU
#define modeSetOBD 1				// set required PIDs for OBD (may be wait until OBD initialization?)
#define modeReadLPG 2				// transmit 'read current values' command
#define modeReadOBD 3				// transmit 'read OBD' command
#define modeWait 4					// wait for response or timeout, than switch to modeSearching or modeReadXXX based on response buffer 
#define modeTransparent 32			// transparent passsthrough mode
#define modePassthrough 48			// passthrough mode until reboot (explicitly selectable, for tuning the Bluetooth module, etc.)

#define pcNone 0
#define pcRcvCmd 1					// receiving cmd from PC
#define pcCmdOK 2					// entire cmd received from PC

#define pcCmdRead 0
#define pcCmdAddLPG 1
#define pcCmdAddPET 2
#define pcCmdResetTrip 3
#define pcCmdSetLPGInjFlow 8
#define pcCmdSetPETInjFLow 9
#define pcCmdReboot 0xFF

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

//
volatile u08 startCalc = 0L; 		// start calc after RPM > 0
volatile u16 EEPROMUpdates = 0L;	// number of EEPROM updates
volatile u08 tempMode = 0;			// tempMode: 0 - skiprom/begin measure, 1 - wait for measure end, 2 - read temp value

// ======================================================
// Send byte to PC
void PCSend(u08 data)
{
	u08 _sreg;

	_sreg = SREG; cli();
	PCsend[PChead] = data;
	PChead++;
	if (PChead >= sizeof(PCsend)) PChead = 0;
	SREG = _sreg;
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
// ======================================================
// ======================================================
ISR(USART1_RX_vect)
{
	unsigned char RS;

	RS = UDR1;
	if (PTmode >= modeTransparent)  UDR0 = RS;   // pass bytes to PC directly while in transparent/passthrough mode
	if (PTmode < modePassthrough) {
		KMEBuff[KMEidx++] = RS;  // ... but try to process the response as well
		// we have a response with a valid checksum and num of bytes received
		if ((!KMEgr) && (KMEBuff[0] == bRespKME) && (KMEidx > 1) && (KMEidx >= KMEBuff[1]) && (RS == KMEcs)) {
			KMEidx = 0;  KMEcs = 0;  KMEgr = 1;  // "got response" signal processed in main cycle
		} else {
			KMEcs = KMEcs + RS;
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

	cmd = PTmode & 0b11;
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
		UDR0 = PCsend[PCtail];
		PCtail++;
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
	unsigned char RS;

	RS = UDR0;
	PCBuff[PCidx++] = RS; 
	// if not transparent mode and got 1st byte and its the request start - go to transparent mode (temporarly)
	if ((PTmode < modeTransparent) && (PCidx == 1)) {
		switch (RS) {
	  		case bReqKME: PTmode = modeTransparent; break;
	  		case  bReqPC: PTmode = modePassthrough; break;
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
		UDR1 = RS;
		if (PTmode == modeTransparent) {
			// if we got enough bytes according to the request data
			if ((PCidx > 1) && (PCidx >= PCBuff[1])) {
				// if we have valid checksum - set signal "got request", else reset PTmode
				if (RS == PCcs) {
					StartT0();             // start T0 timer, so if no requests from KME software in 1s - revert to own requests
					KMEidx = 0; KMEcs = 0; // also reset response buffer indexes
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

	DATA[0] = 0x42;
	DATA[1] = 0x24;

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
}

/////////////////////////////////////////////////////////
void ReadParamsEEPROM(void) {
	u08 tmp;
	u32 tmp32;

	tmp = eeprom_read_byte((u08*)0);
	if (tmp < 0xFF)  injFlowLPG = tmp;
	tmp = eeprom_read_byte((u08*)1);
	if (tmp < 0xFF)  injFlowPET = tmp;
	EEPROMUpdates = eeprom_read_word((uint16_t*)10);

	tmp32 = eeprom_read_dword((uint32_t*)2);
	if (tmp32 < 0xFFFFFFFF)  DWORD(PDATA, totalLPGInTank) = tmp32;
	tmp32 = eeprom_read_dword((uint32_t*)6);
	if (tmp32 < 0xFFFFFFFF)  DWORD(PDATA, totalPETInTank) = tmp32;

	WORD(PDATA, eepromUpdateCount) = EEPROMUpdates;
}

/////////////////////////////////////////////////////////
void WriteParamsEEPROM(void) {
	eeprom_busy_wait();

	eeprom_update_dword((uint32_t*)2, (uint32_t)DWORD(PDATA, totalLPGInTank));
	eeprom_update_dword((uint32_t*)6, (uint32_t)DWORD(PDATA, totalPETInTank));
	EEPROMUpdates++; eeprom_update_word((uint16_t*)10, EEPROMUpdates);
	WORD(PDATA, eepromUpdateCount) = EEPROMUpdates;
}

/////////////////////////////////////////////////////////
ISR(SIG_OVERFLOW0) {

	TCNT0 = T0delay;  // reload timer overflow

	PCtimeout++;

	// if we have timeout reached
	if (PCtimeout > 25) {
		PCtimeout = 0;
		PCidx = 0;  PCcs = 0;
		cbi(TIMSK0, TOIE0);	 // disable overflow
		// now next cmd
		if (PTmode & modeWait) {
			PTmode = PTmode & 0b11;
		} else {
			if (PTmode == modeTransparent)  PTmode = modeSearch;  // if timeout in transparent - means no frames, so switch to search mode
		}
	}
}

/////////////////////////////////////////////////////////
ISR(SIG_OVERFLOW2) {

	TCNT2 = T2delay;  // reload timer overflow

	if (((PTmode & 0b11) > modeSetOBD) || (PTmode == modeTransparent)) {
		KMEtimeLPG++;
		KMEtimeOBD++;
		// if got timeout receiving 
		if (KMEtimeLPG > 100*10)  {
			PTmode = modeSearch;
			if (startCalc) {
				startCalc = 0;
				WORD(PDATA, LPGRPM) = 0;
				WriteParamsEEPROM();
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
ISR(PCINT0_vect) {
	PCSend('!');
	tempMode = 2;
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
int main (void) {

	u16 tmp;
	u08 i, cmd, tempEnabled;
	u08 temp[9];

	wdt_disable();
	InitParams();

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
	ReadParamsEEPROM();

	sei();
	StartT2();  // start 10ms counting helper timer immediately

	tempEnabled = ow_reset();

	while (1) {
		if (tempEnabled) {
			if (DATAtimer > 100*30){
				// time to read temperature
				//PCICR = (0<<PCIE0);
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
					//PCSend(Nibble2Hex((crc>>4)&0xF));
					//PCSend(Nibble2Hex((crc>>0)&0xF));
					WORD(PDATA, outsideTemp) = (temp[0]<<8) + temp[1];
				} 
				DATAtimer = 0;
				tempMode = 0;
			}
		}  // tempEnabled
		if (DATAtimer > 100*40){
			DATAtimer = 0;
			tempMode = 0;
		}
		if (PCgr == pcCmdOK) {
			cli();  cmd = PCBuff[1];  PCgr = pcNone;  sei();
			switch (cmd) {
				case pcCmdRead: for (i=0; i<90; i++)  PCSend(DATA[i]); break;
				case pcCmdAddLPG: 		cli(); DWORD(PDATA, totalLPGInTank) += DWORD(PPCBuff, 2); sei(); PCSend(pcCmdAddLPG); break;
				case pcCmdAddPET: 		cli(); DWORD(PDATA, totalPETInTank) += DWORD(PPCBuff, 2); sei(); PCSend(pcCmdAddPET); break;
				case pcCmdResetTrip: 	cli();
										DWORD(PDATA, tripLPGSpent) = 0; DWORD(PDATA, tripLPGTime) = 0; DWORD(PDATA, tripLPGDist) = 0; 
										DWORD(PDATA, tripPETSpent) = 0; DWORD(PDATA, tripPETTime) = 0; DWORD(PDATA, tripPETDist) = 0;
										sei();
										PCSend(pcCmdResetTrip);
										break;
				case pcCmdSetLPGInjFlow: injFlowLPG = PCBuff[2]; eeprom_busy_wait(); eeprom_update_byte((u08*)0, injFlowLPG); PCSend(pcCmdSetLPGInjFlow); break;
				case pcCmdSetPETInjFLow: injFlowPET = PCBuff[2]; eeprom_busy_wait(); eeprom_update_byte((u08*)1, injFlowPET); PCSend(pcCmdSetPETInjFLow); break;
				case pcCmdReboot: 		PCSend('$');
										sbi(UCSR0B, UDRIE0);  				// explicitly enable UDRIE0 before reset
										wdt_enable(1); cli(); while(1); break;

			} // switch
		}
		if (KMEgr) {
			KMEgr = 0;
			// if we got 0x06 (LPG info) response - parse it
			if (KMEBuff[2] == bRespLPG) {
				cli(); // disable int for critical params
				tmp = KMEtimeLPG;  KMEtimeLPG = 0;
				// process LPG data
				WORD(PDATA, PETsuminjtime) = 
				       (KMEBuff[3]<<8) + KMEBuff[4] + (KMEBuff[5]<<8) + KMEBuff[6] + (KMEBuff[7]<<8) + KMEBuff[8] + (KMEBuff[9]<<8) + KMEBuff[10];
				WORD(PDATA, LPGsuminjtime) = 
				       (KMEBuff[19]<<8) + KMEBuff[20] + (KMEBuff[21]<<8) + KMEBuff[22] + (KMEBuff[23]<<8) + KMEBuff[24] + (KMEBuff[25]<<8) + KMEBuff[26];
				WORD(PDATA, LPGRPM) = (KMEBuff[35]<<8) + KMEBuff[36];
				WORD(PDATA, LPGPcol) = (KMEBuff[41]<<8) + KMEBuff[42];
				WORD(PDATA, LPGPsys) = (KMEBuff[43]<<8) + KMEBuff[44];
				WORD(PDATA, LPGTred) = (KMEBuff[47]<<8) + KMEBuff[48];
				WORD(PDATA, LPGTgas) = (KMEBuff[49]<<8) + KMEBuff[50];
				WORD(PDATA, LPGVbat) = (KMEBuff[53]<<8) + KMEBuff[54];
				DATA[LPGStatus] = KMEBuff[58];
				sei();
				// start calculating fuel consumption if RPM becomes nonzero
				// save calculated to EEPROM if RPM becomes zero (engine stopped)
				if (WORD(PDATA, LPGRPM)) {
					startCalc = 1;
				} else {
					if (startCalc) { WriteParamsEEPROM(); }
					startCalc = 0;
				}
				if (startCalc)  CalcFuel(tmp);
				sbi(PINC, PLED);
			}
			// if we got 0xB1 (OBD info) response - parse it
			if (KMEBuff[2] == bRespOBD) {
				cli();   // disable int for critical params
				KMEtimeOBD = 0;
				WORD(PDATA, OBDRPM) = (KMEBuff[31]<<8) + KMEBuff[32];
				DATA[OBDSpeed] = KMEBuff[33];
				sei();
				DATA[OBDLoad]  = KMEBuff[34];
				DATA[OBDECT]   = KMEBuff[35];
				DATA[OBDMAP]   = KMEBuff[36];
				DATA[OBDTA]    = KMEBuff[37];
				DATA[OBDIAT]   = KMEBuff[38];
				DATA[OBDSTFT]  = KMEBuff[27];
				DATA[OBDLTFT]  = KMEBuff[28];
				DATA[OBDerror] = KMEBuff[26];
				DATA[OBDTPS]   = KMEBuff[41];
				sbi(PINC, PLED);
			}
			// check for valid response
			if (PTmode < modeTransparent) {
			    PTmode = PTmode & 0b11;
				switch (PTmode) {
					case modeSearch: if ((KMEBuff[0] == bRespKME) && (KMEBuff[1] == 0x11) && (KMEBuff[2] == 0x01))  PTmode++; break;
					case modeReadOBD: PTmode = modeReadLPG; break;
					default: PTmode++; break;
				}
			}
		}  // KMEgr
		// handling current work mode
		if (PTmode < modeWait) {
			PTmode = PTmode | modeWait;
			sbi(UCSR1B, UDRIE1);
		}
		if (PChead != PCtail)
		{
			DATA[workMode] = PTmode;
			sbi(UCSR0B, UDRIE0);  // enable UDR Empty Interrupt
		}
	}
	return 0;
}
