#ifndef TABLES_H
#define TABLES_H

#define brc9600 9600L						// default comm speed
#define brd9600 (F_CPU/(brc9600*16L)-1)		

#define HI(x) ((x)>>8)
#define LO(x) ((x)& 0xFF)

#define WORD(arr, addr) (*(volatile u16*)((addr)+arr))
#define DWORD(arr, addr) (*(volatile u32*)((addr)+arr))

#define PLED	PC3

#define bReqKME 0xC3
#define bRespKME 0xD3
#define bRespLPG 0x06
#define bRespOBD 0xB1
#define bReqPC 0x42
#define bReqBuf 0x62

#define workMode 1

// offsets in buffer for 16 bits
#define PETsuminjtime 2
#define LPGsuminjtime 4
#define LPGRPM 6
#define LPGPcol 8
#define LPGPsys 10
#define LPGTred 12
#define LPGTgas 14
#define LPGVbat 16
#define OBDRPM 18

// offsets in buffer for 8 bits
#define LPGStatus 20
#define OBDSpeed 21
#define OBDLoad 22
#define OBDECT 23
#define OBDMAP 24
#define OBDTA 25
#define OBDIAT 26
#define OBDSTFT 27
#define OBDLTFT 28
#define OBDerror 29
#define OBDTPS 30
// error bits:
// 0 - some LPG injector time much longer or shorter than others
// 1 - some PET injector time much longer or shorter than others
#define LPGerrBits 31

// offsets for calculated values
#define cycleAvgLPGPerHour 32
#define cycleAvgPETPerHour 34
#define cycleAvgLPGPer100 36
#define cycleAvgPETPer100 38

#define outsideTemp 40

#define bufAvgLPGPer100 42
#define bufAvgPETPer100 44

// 32bit - tank fill
#define totalLPGInTank 46
#define totalPETInTank 50

// debug
#define db1 54
#define db2 56

// avg per trip
// avg consumption per 100 km calculated from Spent and Dist in the client!
#define tripLPGSpent 58
#define tripPETSpent 62
#define tripLPGDist 66
#define tripPETDist 70
#define tripLPGTime 74
#define tripPETTime 80

#define eepromUpdateCount 84
#define LPGinjFlow 86
#define PETinjFlow 87

// KME Nevo requests
static volatile u08 KMEcmds[4][40] = {
			{0xC3, 0x04, 0x01, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			{0xC3, 0x26, 0xB4, 0x3F, 0xBE, 0x99, 0x80, 0x80, 0x00, 0x11, 0x00, 0x04, 0xCB, 0x01, 0x00, 0x02, 
			 0x00, 0x00, 0x02, 0x19, 0x04, 0xB0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x20, 0x04, 0x30, 0x06,
			 0x00, 0x07, 0x00, 0x00, 0x00, 0x4A},
			{0xC3, 0x0C, 0x06, 0xC2, 0x8D, 0xC7, 0x28, 0x7D, 0xEC, 0xDB, 0x10, 0x67, 0x00, 0x00, 0x00, 0x00, 
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			{0xC3, 0x0C, 0xB1, 0xC2, 0x8D, 0xC7, 0x28, 0x7D, 0xEC, 0xDB, 0x10, 0x12, 0x00, 0x00, 0x00, 0x00, 
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};

// some error ranges
#define MaxInjDeviation 12
#define MinInjDeviation 10 - (MaxInjDeviation - 10)

// LPG/OBD data buffers
volatile u08 DATA[200];
volatile u16 PDATA = (u16)&DATA[0];
volatile u16 DATAtimer = 0;

// 1st buffer - ~8s depth
volatile u16 cycleInjTime[16];
volatile u16 cycleDelay[16];
volatile u08 cycleVspeed[16];
volatile u08 cycleStatus[16];

volatile u08 cIdx = 0;    // current index must be always limited to & 0x0F

volatile u16 cycleTotalLPG = 0L;
volatile u16 cycleTotalPET = 0L;
volatile u16 cycleTotalLPGDelay = 0L;
volatile u16 cycleTotalPETDelay = 0L;
volatile u16 cycleTotalLPGDist = 0L;
volatile u16 cycleTotalPETDist = 0L;

// 2nd buffer - ~128s depth
volatile u16 bufLPGTime[16];
volatile u16 bufPETTime[16];
//volatile u16 bufLPGDelay[16];
//volatile u16 bufPETDelay[16];
volatile u16 bufLPGDist[16];
volatile u16 bufPETDist[16];

volatile u08 bIdx = 0;

volatile u32 bufTotalLPG = 0L;
volatile u32 bufTotalPET = 0L;
volatile u16 bufTotalLPGDelay = 0L;
volatile u16 bufTotalPETDelay = 0L;
volatile u16 bufTotalLPGDist = 0L;
volatile u16 bufTotalPETDist = 0L;

inline u08 Nibble2Hex(u08 nibble)
{
	return ( (nibble <= 0x09)?(nibble + 0x30):(nibble - 9 + 0x40) );
}

////////////////
// While calling this function we have some params in DATA buffer:
// current Injector time for both LPG and PET
// current RPM
// current Speed (from previous B1 frame data)
// current delay (via function parameter)
// current status - working on LPG or Petrol
static inline void CalcFuel(u16 cDelay) {
	u08 cTail, bTail;
	u32 cRPMs, cInjT;
	u32 cDist, cSpent;

	cIdx = cIdx & 0x0F;    // ensure we're within buffer boundaries
	cTail = (cIdx + 1) & 0x0F;  // buffer tail
	cycleDelay[cIdx] = cDelay;
	cycleVspeed[cIdx] = DATA[OBDSpeed];
	cycleStatus[cIdx] = DATA[LPGStatus];
	// counting totals based on current fuel source - LPG/Petrol
	cRPMs = (u32)(WORD(PDATA, LPGRPM) / 5 * cDelay);
	cDist = (cycleVspeed[cIdx] * cDelay * 10 / 36);  // in centimeters!
	if (cycleStatus[cIdx] == 5) {
		cInjT = (u32)WORD(PDATA, LPGsuminjtime) * cRPMs;
		cSpent = (u32)(cInjT / 2692800L * DATA[LPGinjFlow]);
		cycleInjTime[cIdx] = (u16)( cInjT / 448800L );  // /10 would be milliseconds
		DWORD(PDATA, totalLPGInTank) -= cSpent;
		DWORD(PDATA, tripLPGSpent) += cSpent;
		cycleTotalLPG += cycleInjTime[cIdx];
		cycleTotalLPGDelay += cDelay;
		DWORD(PDATA, tripLPGTime) += cDelay;
		cycleTotalLPGDist += cDist;  // in centimeters!!
		DWORD(PDATA, tripLPGDist) += cDist; 
	} else if (cycleStatus[cIdx] > 2) {  // status = 3 - driving on petrol, status = 4 - warming on petrol
		cInjT = (u32)WORD(PDATA, PETsuminjtime) * cRPMs;
		cSpent = (u32)(cInjT / 2692800L * DATA[PETinjFlow]);
		cycleInjTime[cIdx] = (u16)( cInjT / 448800L );
		DWORD(PDATA, totalPETInTank) -= cSpent;
		DWORD(PDATA, tripPETSpent) += cSpent;
		cycleTotalPET += cycleInjTime[cIdx];
		cycleTotalPETDelay += cDelay;
		DWORD(PDATA, tripPETTime) += cDelay;
		cycleTotalPETDist += cDist;  // in centimeters
		DWORD(PDATA, tripPETDist) += cDist; 
	}
	// remove the tail values from counters
	if (cycleStatus[cTail] == 5){
		cycleTotalLPG -= cycleInjTime[cTail];
		cycleTotalLPGDelay -= cycleDelay[cTail];
		cycleTotalLPGDist -= (cycleVspeed[cTail] * cycleDelay[cTail] * 10 / 36);  // cm
	} else if (cycleStatus[cTail] > 2) {
		cycleTotalPET -= cycleInjTime[cTail];
		cycleTotalPETDelay -= cycleDelay[cTail];
		cycleTotalPETDist -= (cycleVspeed[cTail] * cycleDelay[cTail] * 10 / 36);  // cm
	}

	WORD(PDATA, db1) = cycleTotalLPG;
	WORD(PDATA, db2) = cycleTotalLPGDelay;

	// now calc some LPG
	if (cycleTotalLPGDelay) {
		WORD(PDATA, cycleAvgLPGPerHour) = (u16)( (u32)(cycleTotalLPG) * (u32)(DATA[LPGinjFlow] * 6) / (u32)cycleTotalLPGDelay);
	} else  WORD(PDATA, cycleAvgLPGPerHour) = 0L;
	if (cycleTotalLPGDist) {
		WORD(PDATA, cycleAvgLPGPer100) = (u16)( (u32)cycleTotalLPG * 10 * (u32)DATA[LPGinjFlow] / (u32)(cycleTotalLPGDist / 100 ) / 6 );  // changed to centimeters
	} else  WORD(PDATA, cycleAvgLPGPer100) = 0L;
	// now calc some Petrol
	if (cycleTotalPETDelay) {
		WORD(PDATA, cycleAvgPETPerHour) = (u16)( (u32)(cycleTotalPET) * (u32)(DATA[PETinjFlow] * 6) / (u32)cycleTotalPETDelay);
	} else  WORD(PDATA, cycleAvgPETPerHour) = 0L;
	if (cycleTotalPETDist) {
		WORD(PDATA, cycleAvgPETPer100) = (u16)( (u32)cycleTotalPET * 10 * (u32)DATA[PETinjFlow] / (u32)(cycleTotalPETDist / 100) / 6 );
	} else  WORD(PDATA, cycleAvgPETPer100) = 0L;

	// now fill 2nd buffer if 16th step filling 1st is active
	if (cIdx == 0x0F) {
		bIdx = bIdx & 0xF;
		bTail = (bIdx + 1) & 0x0F;  // buffer tail
		bufLPGTime[bIdx]  = cycleTotalLPG;
		bufTotalLPG      += cycleTotalLPG;
//		bufLPGDelay[bIdx] = cycleTotalLPGDelay;
//		bufTotalLPGDelay += cycleTotalLPGDelay;
		bufLPGDist[bIdx]  = cycleTotalLPGDist;
		bufTotalLPGDist  += cycleTotalLPGDist;
		bufPETTime[bIdx]  = cycleTotalPET;
		bufTotalPET      += cycleTotalPET;
//		bufPETDelay[bIdx] = cycleTotalPETDelay;
//		bufTotalPETDelay += cycleTotalPETDelay;
		bufPETDist[bIdx]  = cycleTotalPETDist;
		bufTotalPETDist  += cycleTotalPETDist;

		bufTotalLPG      -= bufLPGTime[bTail];
//		bufTotalLPGDelay -= bufLPGDelay[bTail];
		bufTotalLPGDist  -= bufLPGDist[bTail];
		bufTotalPET      -= bufPETTime[bTail];
//		bufTotalPETDelay -= bufPETDelay[bTail];
		bufTotalPETDist  -= bufPETDist[bTail];

		// now calc some LPG
		if (bufTotalLPGDist) {
			WORD(PDATA, bufAvgLPGPer100) = (u16)( (u32)bufTotalLPG * 10 * (u32)DATA[LPGinjFlow] / (u32)(bufTotalLPGDist / 100) / 6 );
		} else  WORD(PDATA, bufAvgLPGPer100) = 0L;
		// petrol
		if (bufTotalPETDist) {
			WORD(PDATA, bufAvgPETPer100) = (u16)( (u32)bufTotalPET * 10 * (u32)DATA[PETinjFlow] / (u32)(bufTotalPETDist / 100) / 6 );
		} else  WORD(PDATA, bufAvgPETPer100) = 0L;

		bIdx++;
	}

	cIdx++;
}

#endif
