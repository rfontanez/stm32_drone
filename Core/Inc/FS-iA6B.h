/*
 * FS-iA6B.h
 *
 *  Created on: Oct 16, 2025
 *      Author: ryanfontanez
 */

#ifndef __FSIA6B_H
#define __FSIA6B_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

typedef struct _FSiA6B_iBus
{
	unsigned short RH; //Right horizontal stick
	unsigned short RV; //Right vertical stick
	unsigned short LV; //Left horizontal stick
	unsigned short LH; //Left vertical stick
	unsigned short SwA;
	unsigned short SwB;
	unsigned short SwC;
	unsigned short SwD;
	unsigned short VrA;
	unsigned short VrB;

	unsigned char FailSafe;
} FSiA6B_iBus;

extern FSiA6B_iBus iBus;

unsigned char ibus_Check_CHKSUM(unsigned char* data, unsigned char len);
void iBus_Parsing(unsigned char* data, FSiA6B_iBus* iBus);
void FSiA6B_UART5_Initialization(void);
unsigned char iBus_isActiveFailsafe(FSiA6B_iBus* iBus);

#ifdef __cplusplus
}
#endif
#endif /*__FSIA6B_H */
