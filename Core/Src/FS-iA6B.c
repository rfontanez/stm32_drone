/*
 * FS-iA6B.c
 *
 *  Created on: Oct 16, 2025
 *      Author: ryanfontanez
 */


#include "FS-iA6B.h"

unsigned char ibus_Check_CHKSUM(unsigned char* data, unsigned char len)
{
	//ibus checksum is calculated by subtracting each byte one by one from 0xffff (except for the checksum bytes)
	unsigned short chksum = 0xffff;
	for (int i = 0; i < (len-2); i++)
	{
		chksum = chksum - data[i];
	}
	//checksum is stored in little endian mode, so the data[30] is lsb and data[31] is msb
	return (((chksum & 0x00ff) == data[30]) && ((chksum>>8) == data[31]));
}
