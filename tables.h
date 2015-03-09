#ifndef TABLES_H
#define TABLES_H

#define brc9600 9600L						// default comm speed
#define brd9600 (F_CPU/(brc9600*16L)-1)		

#define HI(x) ((x)>>8)
#define LO(x) ((x)& 0xFF)

#define BYTE(arr, addr) (*(volatile u08*)((addr)+arr))
#define WORD(arr, addr) (*(volatile u16*)((addr)+arr))
#define DWORD(arr, addr) (*(volatile u32*)((addr)+arr))

#define PLED	PC3
#define PARK	PD4

#define bReqKME 0xC3
#define bRespKME 0xD3
#define bRespLPG 0x06
#define bRespOBD 0xB1
#define bRespOSA 0x29
#define bReqPC 0x42
#define bReqBuf 0x62

// new separate responses
#define RESP_FAST_BIT 0		// fast updating response (inj time, pressures, RPM, part assist, etc.)
#define RESP_SLOW_BIT 1		// slow responses (temperature, fuel tank level, avg fuel consumption, outside temp, etc.)
#define RESP_RARE_BIT 2		// rare updating response (EEPROM cnt, inj flow)
#define RESP_PARK_BIT 3		// park assist response
#define RESP_OSA1_BIT 4		// OSA 1 Bank response

// fast updating structure (twice a sec or so)
struct strucResponseFast {
	u08 id;					// id = 0x42
	u08 length;				// data length
	u08 type;				// response type (RESP_XXX)
	u08 workMode;
	//
	u16 PETsuminjtime;
	u16 LPGsuminjtime;
	u16 LPGRPM;
	u16 LPGPcol;
	u16 LPGPsys;
	u16 LPGVbat;
	u16 OBDRPM;
	u08 LPGStatus;
	u08 OBDSpeed;
	u08 OBDMAP;
	u08 OBDTA;
	u08 OBDSTFT;
	u08 OBDLTFT;
	u08 OBDerror;
	u08 OBDTPS;
	u08 OBDLoad;
	// error bits:
	// 0 - some LPG injector time much longer or shorter than others
	// 1 - some PET injector time much longer or shorter than others
	u08 LPGerrBits;
	// fast updating fuel consumptions
	u16 cycleAvgLPGPerHour;
	u16 cycleAvgPETPerHour;
	//
	u08 checkSum;
}__attribute__((__packed__ )) DF;

// slow updating structure (once in 2 sec, sent just before Fast struct)
struct strucResponseSlow {
	u08 id;
	u08 length;
	u08 type;				// response type (RESP_XXX)
	u08 workMode;
	//
	u16 outsideTemp;
	u16 LPGTred;
	u16 LPGTgas;
	u16 OBDECT;
	u16 OBDIAT;
 	u16 cycleAvgLPGPer100;
	u16 cycleAvgPETPer100;
	u16 bufAvgLPGPer100;
	u16 bufAvgPETPer100;
	u32 totalLPGInTank;
	u32 totalPETInTank;
	// avg consumption per 100 km calculated from Spent and Dist in the client!
	u32 tripLPGSpent;
	u32 tripPETSpent;
	u32 tripLPGDist;
	u32 tripPETDist;
	u32 tripLPGTime;
	u32 tripPETTime;
	//
	u08 checkSum;
}__attribute__((__packed__ )) DS;

// rare updating values (sent if needed)
struct strucResponseRare {
	u08 id;
	u08 length;
	u08 type;				// response type (RESP_XXX)
	u08 workMode;
	//
	u16 eepromUpdateCount;
	u08 LPGinjFlow;
	u08 PETinjFlow;
	//
	u08 checkSum;
}__attribute__((__packed__ )) DR;

// park assist values (sent if PA is active)
struct strucResponsePark {
	u08 id;
	u08 length;
	u08 type;				// response type (RESP_XXX)
	u08 workMode;
	//
	u16 P1;
	u16 P2;
	//
	u08 checkSum;
}__attribute__((__packed__ )) DP;

// OSA Bank 1 table
struct strucResponseOSA1 {
	u08 id;
	u08 length;
	u08 type;				// response type (RESP_XXX)
	u08 workMode;
	//
	u08 OSA[25];
	//
	u08 checkSum;
}__attribute__((__packed__ )) DO;

// KME Nevo requests
// response to 1st request (get hw conf) kept in the request buffer (offset+16)
// we'll send it to the KME software while KME Nevo finishing it's previous transmission
static volatile u08 KMEcmds[7][38] = {
			{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			{0xC3, 0x04, 0x01, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0xD3, 0x11, 0x01, 0x40, 0x48, 0x07, 0x63, 0x03, 0xC3, 0x02, 0x0A, 0x01, 0x02, 0x0A, 0x01, 0x23, 
			 0xDA, 0x00, 0x00, 0x00, 0x00, 0x00},
			{0xC3, 0x04, 0xB3, 0x7A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
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
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
// request to read OSA Bank 1 table
			{0xC3, 0x04, 0x29, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};

// some error ranges
#define MaxInjDeviation 12
#define MinInjDeviation 10 - (MaxInjDeviation - 10)

// LPG corrections affecting injection time/liquid phase consumption relation, signed %
volatile s08 corrPres, corrTemp; 

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
volatile u32 bufTotalLPGDist = 0L;
volatile u32 bufTotalPETDist = 0L;

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
	cycleVspeed[cIdx] = DF.OBDSpeed;
	cycleStatus[cIdx] = DF.LPGStatus;
	// counting totals based on current fuel source - LPG/Petrol
	cRPMs = (u32)(DF.LPGRPM / 5 * cDelay);
	cDist = (cycleVspeed[cIdx] * cDelay * 10 / 36);  // in centimeters!
	if (cycleStatus[cIdx] == 5) {
		cInjT = (u32)(DF.LPGsuminjtime) * cRPMs / 100 * (100 - corrPres) / 100 * (100 - corrTemp);
		cSpent = (u32)(cInjT / 2692800L * DR.LPGinjFlow);
		cycleInjTime[cIdx] = (u16)( cInjT / 448800L );  // /10 would be milliseconds
		DS.totalLPGInTank -= cSpent;
		DS.tripLPGSpent += cSpent;
		cycleTotalLPG += cycleInjTime[cIdx];
		cycleTotalLPGDelay += cDelay;
		DS.tripLPGTime += cDelay;
		cycleTotalLPGDist += cDist;  // in centimeters!!
		DS.tripLPGDist += cDist; 
	} else if (cycleStatus[cIdx] > 2 && cRPMs > 0) {  // if RPM > 0 and status = 3 - driving on petrol, status = 4 - warming on petrol
		cInjT = (u32)(DF.PETsuminjtime) * cRPMs * 2; // doubling to fit pet injflow to 1 byte
		cSpent = (u32)(cInjT / 2692800L * DR.PETinjFlow);
		cycleInjTime[cIdx] = (u16)( cInjT / 448800L );
		DS.totalPETInTank -= cSpent;
		DS.tripPETSpent += cSpent;
		cycleTotalPET += cycleInjTime[cIdx];
		cycleTotalPETDelay += cDelay;
		DS.tripPETTime += cDelay;
		cycleTotalPETDist += cDist;  // in centimeters
		DS.tripPETDist += cDist; 
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

	// now calc some LPG
	if (cycleTotalLPGDelay) {
		DF.cycleAvgLPGPerHour = (u16)( (u32)(cycleTotalLPG) * (u32)(DR.LPGinjFlow * 6) / (u32)cycleTotalLPGDelay);
	} else  DF.cycleAvgLPGPerHour = 0L;
	if (cycleTotalLPGDist) {
		DS.cycleAvgLPGPer100 = (u16)( (u32)cycleTotalLPG * 10 * (u32)(DR.LPGinjFlow) / (u32)(cycleTotalLPGDist / 100 ) / 6 );  // changed to centimeters
	} else  DS.cycleAvgLPGPer100 = 0L;
	// now calc some Petrol
	if (cycleTotalPETDelay) {
		DF.cycleAvgPETPerHour = (u16)( (u32)(cycleTotalPET) * (u32)(DR.PETinjFlow * 6) / (u32)cycleTotalPETDelay);
	} else  DF.cycleAvgPETPerHour = 0L;
	if (cycleTotalPETDist) {
		DS.cycleAvgPETPer100 = (u16)( (u32)cycleTotalPET * 10 * (u32)(DR.PETinjFlow) / (u32)(cycleTotalPETDist / 100) / 6 );
	} else  DS.cycleAvgPETPer100 = 0L;

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
			DS.bufAvgLPGPer100 = (u16)( (u32)bufTotalLPG * 10 * (u32)(DR.LPGinjFlow) / (u32)(bufTotalLPGDist / 100) / 6 );
		} else  DS.bufAvgLPGPer100 = 0L;
		// petrol
		if (bufTotalPETDist) {
			DS.bufAvgPETPer100 = (u16)( (u32)bufTotalPET * 10 * (u32)(DR.PETinjFlow) / (u32)(bufTotalPETDist / 100) / 6 );
		} else  DS.bufAvgPETPer100 = 0L;

		bIdx++;
	}

	cIdx++;
}

#endif
