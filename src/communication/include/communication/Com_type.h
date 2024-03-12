/*
 * Com_type.h
 *
 *  Created on: Feb 23, 2024
 *      Author: OptiTrack2
 */

#ifndef COMMUNICATE_COM_TYPE_H_
#define COMMUNICATE_COM_TYPE_H_

#include <stdint.h>

#define NEWFIXTYPE(TYPE,I,F)							\
	struct												\
	{													\
		TYPE sign			:	1;						\
		TYPE IntegerPart	:	I;						\
		TYPE FractionPart	:	F;						\
	}

#define NEWUFIXTYPE(TYPE,I,F)							\
	struct												\
	{													\
		TYPE IntegerPart	:	I;						\
		TYPE FractionPart	:	F;						\
	}

typedef NEWUFIXTYPE(uint8_t,4,4) ufix8_t;
typedef NEWFIXTYPE(uint16_t,4,11) fix16_t;

static inline float cvtUFix8Float(ufix8_t num)
{
	return (float)num.IntegerPart + 1.0 / 16.0 * (float)num.FractionPart;
}
static inline ufix8_t cvtFloatUFix8(float num)
{
	ufix8_t temp;
	temp.IntegerPart = (uint8_t)num & 0xF;
	temp.FractionPart = (num - (int)num) * 16;
	return temp;
}
static inline float cvtFix16Float(fix16_t num)
{
	float s = num.sign?-1:1;
	return ((float)num.IntegerPart + 1.0 / 2048.0 * (float)num.FractionPart)*s;
}
static inline fix16_t cvtFloatFix16(float num)
{
	fix16_t temp;
	//temp.sign = num>0?0:1;
	if(num<0)
	{
		temp.sign = 1;
		num = -num;
	}
	else
	{
		temp.sign = 0;
	}
	temp.IntegerPart = (uint8_t)num & 0xF;
	temp.FractionPart = (num - (int)num) * 2048;
	return temp;
}

/*typedef struct
{
	uint8_t sign			:	1;
	uint8_t IntegerPart		:	4;
	uint8_t FractionPart	:	3;
}fix8_4_t;

typedef struct
{
	uint8_t IntegerPart		:	4;
	uint8_t FractionPart	:	4;
}ufix8_4_t;

typedef struct
{
	uint16_t sign			:	1;
	uint16_t IntegerPart	:	3;
	uint16_t FractionPart	:	12;
}fix16_3_t;

typedef struct
{
	uint16_t IntegerPart	:	3;
	uint16_t FractionPart	:	13;
}ufix16_3_t;*/

#endif /* COMMUNICATE_COM_TYPE_H_ */
