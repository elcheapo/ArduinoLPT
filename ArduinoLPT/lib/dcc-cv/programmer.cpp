//----------------------------------------------------------------
//-----------------------------------------------------------------
//
// file:      programmer.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-06-02 V0.1 started
//            2006-10-24 V0.2 bugfixes in register mode and
//                            write routines (reported by Volker Bosch)
//            2007-01-24 V0.3 IB routines removed and put
//                            in ibox_programmer.c
//            2007-04-05 V0.4 added programmer_busy()
//            2007-04-20 V0.5 test with Train Programmer successfully
//            
//           
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   genereates the dcc servicemode messages and checks for
//            acknowledge.
//
// interface upstream:
//            init_programmer(void)     // 
//
// interface downstream:
//
//                                      // to organizer
//
// 2do:       
//-----------------------------------------------------------------

// LINK:      http://ruppweb.dyndns.org/xray/comp/decoder.htm
//            (calculate long addresses)

#include "Arduino.h"
#include "organizer.h"
#include "dcc_timer.h"                // next message

//#include "lenz_parser.h"
#include "programmer.h"

//#define DEBUG_UART3
#undef DEBUG_UART3
#undef DEBUG_ACK

//=====================================================================
//
// Data Structures
//
//--------------------------------------------------------------------

// Overview of Programming Modes:
//
// Mode:        Leading PagePre Mid     Command Post
//              resets          resets          resets (r/w)
//-------------------------------------------------------------------
// Direct         3       -       -       5       1 / 6
// Addr, only     3       5       9       7       1 / 10
// Reg. Mode      3       5       9       7       1 / 10
// Paged Mode     same as Reg. Mode, but page access in advance        
//


//typedef struct                      // this is the structure where we handle different modes
//  {
//    t_prog_mode mode;
//    uint8_t cycles[5];        // we have 5 different cycles - see table above
//  } t_prog_ctrl;

// t_prog_ctrl prog_ctrl;
 
// t_prog_ctrl const direct_ctrl       PROGMEM  = {P_READ, {3, 0, 0, 5,  6}};
// t_prog_ctrl const direct_ctrl       PROGMEM  = {P_READ, {7, 0, 0, 5,  6}};
// t_prog_ctrl const registermode_ctrl PROGMEM  = {P_READ, {3, 5, 9, 7, 10}};


//--------------------------------------------------------------------
//                            rep, size, type, DCC
extern message DCC_Reset;
message prog_message;

#define ACK_LEVEL 15
#define MIN_ACK 8
#define MAX_ACK 400
#define NB_ACK_SEEN 1
#define TIMEOUT_ACK 50
#define STEP_ACK 1
#undef DEBUG_UART3

DCC_timer * timer;
uint8_t prog_adc_channel;
//t_adc prog_adc_mode;
uint8_t ack_level;

bool set_programmer (DCC_timer * _timer, uint8_t _adc_channel) {
	uint8_t ret;
	uint8_t i,j;
	timer = _timer;
	prog_adc_channel = _adc_channel;
//	prog_adc_mode = _adc_mode;
//	uint8_t count = 0;
	uint8_t ack0, ack1;

	// Start with default ack Level
	ack_level = ACK_LEVEL;

	ret = false;
//	while(1) {
	for (i=0; (i<4) & (ret == false); i++){
		for(j=0; (j<5) & (ret == false); j++) {
			// Try to set a proper ACK level
			if (direct_mode_bit_verify(8, 7, 1)) {
				ack1 = 1;
			} else {
				ack1 = 0;
			}
			if (direct_mode_bit_verify(8, 7, 0)) {
				ack0 = 1;
			} else {
				ack0 = 0;
			}
			if (  ((ack0 == 0) & (ack1 == 1)) | (( ack0 == 1) & (ack1 == 0 )) ){
				// We see an ack properly
				ret = true;
			}
			delay( 100 );
		}
		// No ack seen lower detection level
		if (ret == false) ack_level -= 1;
	}
	Serial.print(F("ACK LEVEL DETECTED : "));
	Serial.println(ack_level,10);
	return ret;
}

void set_ack_level(uint8_t level) {
	if (level >= MIN_ACK) {
		ack_level = level;
	}
}

uint8_t get_ack_level(void) {
	return ack_level;
}

//
//===================================================================================
//
//   Support Routines for programmer
//
//--------------------------------------------------------------------------------
// program in direct mode ...

int8_t programmer(message * prog_message) {
	int8_t ret = false;
	uint16_t value;
#ifdef DEBUG_ACK
	uint16_t ack_adc = 0;
	int16_t time_stamps = 0;
	int16_t time_start;
#endif
#ifdef DEBUG_UART3
	Serial3.println(F("\npgm start"));
#endif
	timer->begin(digital);
    timer->set_direct();
    // First send 20 resets to prime including 3 for direct mode ...
    DCC_Reset.repeat = 20;
    while (timer->dcc_busy() != 0);
    timer->send_direct_dcc_packet(&DCC_Reset);
    while (timer->dcc_busy() != 0);
    DCC_Reset.repeat = 1;
#ifdef DEBUG_UART3
	Serial.println(F("DCC_Reset sent"));
#endif
   	prog_message->repeat = 4; // Should be sent twice before we see the ACK ?
//    xSemaphoreTake(timer->ready_for_acknowledge, 0);
#ifdef DEBUG_UART3
			Serial3.println();
			Serial3.print(prog_message->repeat,10);
			Serial3.print(',');
			for (uint8_t i=0; i < prog_message->size; i++) {
				Serial3.print(prog_message->dcc[i],16);
				Serial3.print(':');
			}
#endif
    while (timer->dcc_busy() != 0);
	Serial.println(F("---------"));
    timer->send_direct_dcc_packet(prog_message);

   //  watch for ACK on correct channel

	//	ack_valid_counter = 0;
	for (uint8_t i=0; i< TIMEOUT_ACK/STEP_ACK; i++) {
		value = analogRead(prog_adc_channel);
//		Serial.print(F("ACK read at "));
		Serial.println(value);
		if (value > ack_level) { // Ack detected
			ret = true;
			break;
		}
		delay( STEP_ACK );
	}
	if (ret == true) {
		/* Wait until we don't see the ack anymore - just wait 200 ms so we don't get stuck here */
		delay( 200 );

#ifdef DEBUG_ACK
	} else {
		Serial3.println(F("NO ACK"));
#endif
	}

#ifdef DEBUG_UART3
	Serial3.println();
	if (ret == true) {
		Serial3.println(F("Acknowledge"));
	} else {
		Serial3.println(F("Nak"));
	}
#endif
	// Send RESET packet according to spec (direct mode)
	DCC_Reset.repeat = 5;     // send reset packets
//    xSemaphoreTake(timer->packet_sent,0); // Make sure we clear the packet_sent semaphore
    while (timer->dcc_busy() != 0);
	timer->send_direct_dcc_packet(&DCC_Reset);
    while (timer->dcc_busy() != 0);

    DCC_Reset.repeat = 1;
#ifdef DEBUG_UART3
	Serial3.print(F("DCC_Reset2 Ret="));
	Serial3.println(ret,HEX);
#endif
	timer->end();
	return ret;
  }


/// CV-Mode, write
//
//  Write Paket
//  preamble 0 0111 CCAA 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1 mit CC=11

uint8_t direct_mode_write(uint16_t cv, uint8_t data)
  {
     uint16_t cv_adr;
#ifdef DEBUG_UART3
	Serial3.println(F(">direct_mode_write"));
#endif

     cv_adr = cv-1;
     prog_message.repeat = 1;
     prog_message.size   = 3;
     prog_message.type   = is_prog;
     prog_message.dcc[0] = 0b01110000 | 0b00001100 | (uint8_t)((cv_adr >> 8) & 0b11);
     prog_message.dcc[1] = (uint8_t)(cv_adr & 0xFF);
     prog_message.dcc[2] = data;

     return programmer(&prog_message);
  }

/// CV-Mode, verify
//
//  Verify Paket
//  preamble 0 0111 CCAA 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1 mit CC=01

uint8_t direct_mode_verify(uint16_t cv, uint8_t data)  {
     uint16_t cv_adr;
     uint8_t ret;

     cv_adr = cv-1;
     prog_message.repeat = 1;
     prog_message.size   = 3;
     prog_message.type   = is_prog;
     prog_message.dcc[0] = 0b01110000 | 0b00000100 | (uint8_t)((cv_adr >> 8) & 0b11);
     prog_message.dcc[1] = (uint8_t)(cv_adr & 0xFF);
     prog_message.dcc[2] = data;
     
     ret = programmer(&prog_message);
     if (ret == true) return ret;
     // If not try a second time ...
     return programmer(&prog_message);
  }


/// CV-Mode, verify single bit
//
//  Verify Paket
//  preamble 0 0111 CCAA 0 AAAAAAAA 0 111KDBBB 0 EEEEEEEE 1 mit CC=10
//  K = (1=write, 0=verify) D = Bitvalue, BBB = bitpos
//  return: true if verified

uint8_t direct_mode_bit_verify(uint16_t cv, uint8_t bitpos, uint8_t mybit) {
     uint16_t cv_adr;

     cv_adr = cv-1;
     prog_message.repeat = 1;
     prog_message.size   = 3;
     prog_message.type   = is_prog;
     prog_message.dcc[0] = 0b01110000 | 0b00001000 | (uint8_t)((cv_adr >> 8) & 0b11);
     prog_message.dcc[1] = (uint8_t)(cv_adr & 0xFF);
     prog_message.dcc[2] = 0b11100000 | ((mybit &0b1) << 3) | (bitpos & 0b111);

     return programmer(&prog_message);
  }

/// CV-Mode, write single bit
//
//  Write Paket
//  preamble 0 0111 CCAA 0 AAAAAAAA 0 111KDBBB 0 EEEEEEEE 1 mit CC=10
//  K = (1=write, 0=verify) D = Bitvalue, BBB = bitpos

uint8_t direct_mode_bit_write(uint16_t cv, uint8_t bitpos, uint8_t mybit) {
     uint16_t cv_adr;

     cv_adr = cv-1;
     prog_message.repeat = 1;
     prog_message.size   = 3;
     prog_message.type   = is_prog;
     prog_message.dcc[0] = 0b01110000 | 0b00001000 | (uint8_t)((cv_adr >> 8) & 0b11);
     prog_message.dcc[1] = (uint8_t)(cv_adr & 0xFF);
     prog_message.dcc[2] = 0b11110000 | ((mybit &0b1) << 3) | (bitpos & 0b111);
     return programmer(&prog_message);
}



// returns byte;
// returns -1 if verify of byte failed

int16_t direct_mode_read(uint16_t cv) {
     uint8_t i;
     uint8_t result=0;
     for (i=0; i<8; i++) {
         if (direct_mode_bit_verify(cv, i, 1))
        	 result |= (1<<i);
       }
     if (direct_mode_verify(cv, result)) return(result);
     else { // Try once more
         for (i=0; i<8; i++) {
             if (direct_mode_bit_verify(cv, i, 1))
            	 result |= (1<<i);
           }
         if (direct_mode_verify(cv, result)) return(result);
    	 return(-1);
     }

  }


uint8_t test_direct_mode(void) {
	// test auf MSB von CV8
	if (direct_mode_bit_verify(8, 7, 1)) return(1);
	if (direct_mode_bit_verify(8, 7, 0)) return(1);
	return(0);
}

/*
 * Factory Reset Command ...
 *
 */
uint8_t factory_reset(void) {
     return programmer(&DCC_Factory_Reset);
}


//===========================================================================================
//
// Lenzinterface (upstream)
//
// Achtung: Lenz kann nur 256 Adressen!
// Liste der unterstützten Lenzbefehle (x=nyi):

// x _ Prog.-Lesen Register 0x22 0x11 REG X-Or
// x _ Prog.-Lesen CV 0x22 0x15 CV X-Or
// x _ Prog.-Lesen Paging 0x22 0x14 CV X-Or
// x _ Prog.-Ergebnis anfordern 0x21 0x10 0x31
// x _ Prog.-Schreiben Register 0x23 0x12 REG DAT X-Or
// x _ Prog.-Schreiben CV 0x23 0x16 CV DAT X-Or
// x _ Prog.-Schreiben Paging 0x23 0x17 CV DAT X-Or

// x _ Prog. on Main Byte ab V3 0xE6 0x30 ADR High ADR Low 0xEC + C CV DAT X-Or
// x _ Prog. on Main Bit ab V3 0xE6 0x30 ADR High ADR Low 0xE8 + C CV DAT X-Or
#if 0
t_lenz_result lenz_result;          // This stores the last programming result
uint8_t lenz_result_adr;
uint8_t lenz_result_data;

uint8_t pcm_lenz_prog[] = {0x63, 0x21, 0x30, 0x00};     // LZ100 Zentrale in Version 3.0
uint8_t *pcm_lenz_ptr;


void lprog_send_prog_result(void){
	// Messages:
	// 61 11: ready
	// 61 12: short - Kurzschluss
	// 61 13: cant read - Daten nicht gefunden
	// 61 1f: busy
	// 63 10 EE D: EE=adr, D=Daten; nur f�r Register oder Pagemode, wenn bei cv diese Antwort, dann kein cv!
	// 63 14 CV D: CV=cv, D=Daten: nur wenn cv gelesen wurde
	switch (lenz_result)
	{
	case VOID:
		pcm_lenz_prog[0] = 0x61;
		pcm_lenz_prog[1] = 0x13;
		pc_send_lenz(pcm_lenz_ptr = pcm_lenz_prog);
		break;
	case READY:
		pcm_lenz_prog[0] = 0x61;
		pcm_lenz_prog[1] = 0x11;
		pc_send_lenz(pcm_lenz_ptr = pcm_lenz_prog);
		break;
	default:
	case BUSY:
		pcm_lenz_prog[0] = 0x61;
		pcm_lenz_prog[1] = 0x1f;
		pc_send_lenz(pcm_lenz_ptr = pcm_lenz_prog);
		break;
	case REGMODE:
		pcm_lenz_prog[0] = 0x63;
		pcm_lenz_prog[1] = 0x10;
		pcm_lenz_prog[2] = lenz_result_adr;
		pcm_lenz_prog[3] = lenz_result_data;
		pc_send_lenz(pcm_lenz_ptr = pcm_lenz_prog);
		break;
	case CVMODE:
		pcm_lenz_prog[0] = 0x63;
		pcm_lenz_prog[1] = 0x14;
		pcm_lenz_prog[2] = lenz_result_adr;
		pcm_lenz_prog[3] = lenz_result_data;
		pc_send_lenz(pcm_lenz_ptr = pcm_lenz_prog);
		break;
	case SHORT:
		pcm_lenz_prog[0] = 0x61;
		pcm_lenz_prog[1] = 0x12;
		pc_send_lenz(pcm_lenz_ptr = pcm_lenz_prog);
		break;

	}
}


void lprog_read_register(uint8_t regnr) {
	int retval;
	lenz_result_adr = regnr;
	if ((retval=direct_mode_read(regnr-1)) == -1) {       // regadr:0..7, regnr: 1..8
		// failed
		lenz_result = VOID;
	} else {
		lenz_result_data = (uint8_t) retval;
		lenz_result = REGMODE;
	}
}


void lprog_write_register(uint8_t regnr, uint8_t data) {
	lenz_result_adr = regnr;
	direct_mode_write(regnr - 1, data);
	lenz_result = REGMODE;
}

// lenz_cv = 1...256, 256 is coded as 0
//
void lprog_read_cv(uint8_t lenz_cv) {
	uint16_t i;
	int retval;
	lenz_result_adr = lenz_cv;
	if (lenz_cv == 0) i = 256;
	else i = lenz_cv;
	if ((retval=direct_mode_read(i)) == -1) {  // cv as is
		// failed
		lenz_result = VOID;
	} else {
		lenz_result_data = (uint8_t) retval;
		lenz_result = CVMODE;
	}
}

void lprog_write_cv(uint8_t lenz_cv, uint8_t data) {
	uint16_t i;
#ifdef DEBUG_UART3
	Serial3.println(F(">lprog_write_cv"));
#endif
	lenz_result_adr = lenz_cv;
	if (lenz_cv == 0) i = 256;
	else i = lenz_cv;

	direct_mode_write(i, data);
	lenz_result_data = data;
	lenz_result = CVMODE;
#ifdef DEBUG_UART3
	Serial3.println(F("<lprog_write_cv"));
#endif
}
#endif
