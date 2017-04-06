/* Warning: This file is automatically generated by AX-RadioLAB.
   Manual changes are overwritten! */

#include "../COMMON/easyax5043.h"
#include <libmftypes.h>
#include <libmfcrc.h>

#define M433
//#define M868
#define EXT_POWER_AMP

#ifdef M868

// TX: fcarrier=868.300MHz dev=  1.600kHz br=  4.800kBit/s pwr= 15.0dBm
// RX: fcarrier=868.300MHz bw=  7.200kHz br=  4.800kBit/s

__reentrantb void ax5043_set_registers(void) __reentrant
{
	AX5043_MODULATION              = 0x08;
	AX5043_ENCODING                = 0x00;
	AX5043_FRAMING                 = 0x16;
	AX5043_PINFUNCSYSCLK           = 0x01;
	AX5043_PINFUNCDCLK             = 0x01;
	AX5043_PINFUNCDATA             = 0x01;
	AX5043_PINFUNCANTSEL           = 0x04;
#ifdef EXT_POWER_AMP
    AX5043_PINFUNCPWRAMP           = 0x06;
#else
	AX5043_PINFUNCPWRAMP           = 0x07;  //0x07;
#endif // EXT_POWER_AMP
	AX5043_PWRAMP                  = 0x00;  //default Set OFF(0)
	AX5043_WAKEUPXOEARLY           = 0x01;
	AX5043_IFFREQ1                 = 0x01;
	AX5043_IFFREQ0                 = 0x06;
	AX5043_DECIMATION              = 0x29;
	AX5043_RXDATARATE2             = 0x00;
	AX5043_RXDATARATE1             = 0x3C;
	AX5043_RXDATARATE0             = 0xF9;
	AX5043_MAXDROFFSET2            = 0x00;
	AX5043_MAXDROFFSET1            = 0x00;
	AX5043_MAXDROFFSET0            = 0x00;
	AX5043_MAXRFOFFSET2            = 0x80;
	AX5043_MAXRFOFFSET1            = 0x00;  //0x02;	//0x00;
	AX5043_MAXRFOFFSET0            = 0x64;  //0x5F;	//0x64;
	AX5043_FSKDMAX1                = 0x00;
	AX5043_FSKDMAX0                = 0xA6;
	AX5043_FSKDMIN1                = 0xFF;
	AX5043_FSKDMIN0                = 0x5A;
	AX5043_AMPLFILTER              = 0x00;
	AX5043_RXPARAMSETS             = 0xF4;
	AX5043_AGCGAIN0                = 0xD6;
	AX5043_AGCTARGET0              = 0x84;
	AX5043_TIMEGAIN0               = 0xF8;
	AX5043_DRGAIN0                 = 0xF2;
	AX5043_PHASEGAIN0              = 0xC3;
	AX5043_FREQUENCYGAINA0         = 0x0F;
	AX5043_FREQUENCYGAINB0         = 0x1F;
	AX5043_FREQUENCYGAINC0         = 0x09;
	AX5043_FREQUENCYGAIND0         = 0x09;
	AX5043_AMPLITUDEGAIN0          = 0x06;
	AX5043_FREQDEV10               = 0x00;
	AX5043_FREQDEV00               = 0x00;
	AX5043_BBOFFSRES0              = 0x00;
	AX5043_AGCGAIN1                = 0xD6;
	AX5043_AGCTARGET1              = 0x84;
	AX5043_AGCAHYST1               = 0x00;
	AX5043_AGCMINMAX1              = 0x00;
	AX5043_TIMEGAIN1               = 0xF6;
	AX5043_DRGAIN1                 = 0xF1;
	AX5043_PHASEGAIN1              = 0xC3;
	AX5043_FREQUENCYGAINA1         = 0x0F;
	AX5043_FREQUENCYGAINB1         = 0x1F;
	AX5043_FREQUENCYGAINC1         = 0x09;
	AX5043_FREQUENCYGAIND1         = 0x09;
	AX5043_AMPLITUDEGAIN1          = 0x06;
	AX5043_FREQDEV11               = 0x00;
	AX5043_FREQDEV01               = 0x43;
	AX5043_FOURFSK1                = 0x16;
	AX5043_BBOFFSRES1              = 0x00;
	AX5043_AGCGAIN3                = 0xFF;
	AX5043_AGCTARGET3              = 0x84;
	AX5043_AGCAHYST3               = 0x00;
	AX5043_AGCMINMAX3              = 0x00;
	AX5043_TIMEGAIN3               = 0xF5;
	AX5043_DRGAIN3                 = 0xF0;
	AX5043_PHASEGAIN3              = 0xC3;
	AX5043_FREQUENCYGAINA3         = 0x0F;
	AX5043_FREQUENCYGAINB3         = 0x1F;
	AX5043_FREQUENCYGAINC3         = 0x0D;
	AX5043_FREQUENCYGAIND3         = 0x0D;
	AX5043_AMPLITUDEGAIN3          = 0x06;
	AX5043_FREQDEV13               = 0x00;
	AX5043_FREQDEV03               = 0x43;
	AX5043_FOURFSK3                = 0x16;
	AX5043_BBOFFSRES3              = 0x00;
	AX5043_MODCFGF                 = 0x03;
	AX5043_FSKDEV2                 = 0x00;
	AX5043_FSKDEV1                 = 0x02;
	AX5043_FSKDEV0                 = 0x2F;
	AX5043_MODCFGA                 = 0x05;  //default diff transmitter  0x05;
	AX5043_TXRATE2                 = 0x00;
	AX5043_TXRATE1                 = 0x06;
	AX5043_TXRATE0                 = 0x8E;
	AX5043_TXPWRCOEFFB1            = 0x0F;
	AX5043_TXPWRCOEFFB0            = 0xFF;
	AX5043_PLLVCOI                 = 0x99;
	AX5043_PLLRNGCLK               = 0x05;
	AX5043_BBTUNE                  = 0x0F;
	AX5043_BBOFFSCAP               = 0x77;
	AX5043_PKTADDRCFG              = 0x01;
	AX5043_PKTLENCFG               = 0x80;
	AX5043_PKTLENOFFSET            = 0x00;
	AX5043_PKTMAXLEN               = 0xC8;
	AX5043_MATCH0PAT3              = 0xAA;
	AX5043_MATCH0PAT2              = 0xCC;
	AX5043_MATCH0PAT1              = 0xAA;
	AX5043_MATCH0PAT0              = 0xCC;
	AX5043_MATCH0LEN               = 0x9F;
	AX5043_MATCH0MAX               = 0x1F;
	AX5043_MATCH1PAT1              = 0x55;
	AX5043_MATCH1PAT0              = 0x55;
	AX5043_MATCH1LEN               = 0x8A;
	AX5043_MATCH1MAX               = 0x0A;
	AX5043_TMGTXBOOST              = 0x5B;
	AX5043_TMGTXSETTLE             = 0x3E;
	AX5043_TMGRXOFFSACQ            = 0x00;
	AX5043_TMGRXCOARSEAGC          = 0x9C;
	AX5043_TMGRXRSSI               = 0x03;
	AX5043_TMGRXPREAMBLE2          = 0x35;
	AX5043_RSSIABSTHR              = 0xC8;  //0xCD;
	AX5043_BGNDRSSITHR             = 0x00;
	AX5043_PKTCHUNKSIZE            = 0x0D;
	AX5043_PKTACCEPTFLAGS          = 0x20;
	AX5043_DACVALUE1               = 0x00;  //0x00;
	AX5043_DACVALUE0               = 0x00;  //0x00;
	AX5043_DACCONFIG               = 0x00;  //0x00;
	AX5043_REF                     = 0x03;
	AX5043_XTALOSC                 = 0x04;
	AX5043_XTALAMPL                = 0x00;
	AX5043_0xF1C                   = 0x07;
	AX5043_0xF21                   = 0x68;
	AX5043_0xF22                   = 0xFF;
	AX5043_0xF23                   = 0x84;
	AX5043_0xF26                   = 0x98;
	AX5043_0xF34                   = 0x08;
	AX5043_0xF35                   = 0x11;
	AX5043_0xF44                   = 0x25;
}


__reentrantb void ax5043_set_registers_tx(void) __reentrant
{
	AX5043_PLLLOOP                 = 0x07;
	AX5043_PLLCPI                  = 0x12;
	AX5043_PLLVCODIV               = 0x20;
	AX5043_XTALCAP                 = 0x00;
	AX5043_0xF00                   = 0x0F;
	AX5043_0xF18                   = 0x06;
}


__reentrantb void ax5043_set_registers_rx(void) __reentrant
{
	AX5043_PLLLOOP                 = 0x07;
	AX5043_PLLCPI                  = 0x08;
	AX5043_PLLVCODIV               = 0x20;
	AX5043_XTALCAP                 = 0x00;
	AX5043_0xF00                   = 0x0F;
	AX5043_0xF18                   = 0x06;
}


__reentrantb void ax5043_set_registers_rxwor(void) __reentrant
{
	AX5043_TMGRXAGC                = 0x7E;
	AX5043_TMGRXPREAMBLE1          = 0x0F;  //0x1A;	//0x0F;
	AX5043_PKTMISCFLAGS            = 0x05;
	AX5043_AGCGAIN0                = 0x83;
	AX5043_AGCGAIN1                = 0x83;
}


__reentrantb void ax5043_set_registers_rxcont(void) __reentrant
{
	AX5043_TMGRXAGC                = 0x00;
	AX5043_TMGRXPREAMBLE1          = 0x00;
	AX5043_PKTMISCFLAGS            = 0x00;
	AX5043_AGCGAIN0                = 0xD6;
	AX5043_AGCGAIN1                = 0xD6;
}


__reentrantb void ax5043_set_registers_rxcont_singleparamset(void) __reentrant
{
	AX5043_RXPARAMSETS             = 0xFF;
	AX5043_FREQDEV13               = 0x00;
	AX5043_FREQDEV03               = 0x00;
	AX5043_AGCGAIN3                = 0xE7;
}



#endif // M868

#ifdef M433

// TX: fcarrier=433.000MHz dev=  1.600kHz br=  4.800kBit/s pwr= 15.0dBm
// RX: fcarrier=433.000MHz bw=  7.200kHz br=  4.800kBit/s

__reentrantb void ax5043_set_registers(void) __reentrant
{
	AX5043_MODULATION              = 0x08;
	AX5043_ENCODING                = 0x00;
	AX5043_FRAMING                 = 0x16;
	AX5043_PINFUNCSYSCLK           = 0x01;
	AX5043_PINFUNCDCLK             = 0x01;
	AX5043_PINFUNCDATA             = 0x01;
	AX5043_PINFUNCANTSEL           = 0x04;
#ifdef EXT_POWER_AMP
    AX5043_PINFUNCPWRAMP           = 0x06;
    //AX5043_PINFUNCPWRAMP           = 0x05;
#else
	AX5043_PINFUNCPWRAMP           = 0x07;  //0x07;
#endif // EXT_POWER_AMP
	AX5043_PWRAMP                  = 0x00;  //default Set OFF(0)
AX5043_WAKEUPXOEARLY           = 0x01;
	AX5043_IFFREQ1                 = 0x01;
	AX5043_IFFREQ0                 = 0xE4;
	AX5043_DECIMATION              = 0x16;
	AX5043_RXDATARATE2             = 0x00;
	AX5043_RXDATARATE1             = 0x3D;
	AX5043_RXDATARATE0             = 0x8D;
	AX5043_MAXDROFFSET2            = 0x00;
	AX5043_MAXDROFFSET1            = 0x00;
	AX5043_MAXDROFFSET0            = 0x00;
	AX5043_MAXRFOFFSET2            = 0x80;
	AX5043_MAXRFOFFSET1            = 0x02;
	AX5043_MAXRFOFFSET0            = 0x2F;
	AX5043_FSKDMAX1                = 0x00;
	AX5043_FSKDMAX0                = 0xA6;
	AX5043_FSKDMIN1                = 0xFF;
	AX5043_FSKDMIN0                = 0x5A;
	AX5043_AMPLFILTER              = 0x00;
	AX5043_RXPARAMSETS             = 0xF4;
	AX5043_AGCGAIN0                = 0xC5;
	AX5043_AGCTARGET0              = 0x84;
	AX5043_TIMEGAIN0               = 0xF8;
	AX5043_DRGAIN0                 = 0xF2;
	AX5043_PHASEGAIN0              = 0xC3;
	AX5043_FREQUENCYGAINA0         = 0x0F;
	AX5043_FREQUENCYGAINB0         = 0x1F;
	AX5043_FREQUENCYGAINC0         = 0x08;
	AX5043_FREQUENCYGAIND0         = 0x08;
	AX5043_AMPLITUDEGAIN0          = 0x06;
	AX5043_FREQDEV10               = 0x00;
	AX5043_FREQDEV00               = 0x00;
	AX5043_BBOFFSRES0              = 0x00;
	AX5043_AGCGAIN1                = 0xC5;
	AX5043_AGCTARGET1              = 0x84;
	AX5043_AGCAHYST1               = 0x00;
	AX5043_AGCMINMAX1              = 0x00;
	AX5043_TIMEGAIN1               = 0xF6;
	AX5043_DRGAIN1                 = 0xF1;
	AX5043_PHASEGAIN1              = 0xC3;
	AX5043_FREQUENCYGAINA1         = 0x0F;
	AX5043_FREQUENCYGAINB1         = 0x1F;
	AX5043_FREQUENCYGAINC1         = 0x08;
	AX5043_FREQUENCYGAIND1         = 0x08;
	AX5043_AMPLITUDEGAIN1          = 0x06;
	AX5043_FREQDEV11               = 0x00;
	AX5043_FREQDEV01               = 0x43;
	AX5043_FOURFSK1                = 0x16;
	AX5043_BBOFFSRES1              = 0x00;
	AX5043_AGCGAIN3                = 0xFF;
	AX5043_AGCTARGET3              = 0x84;
	AX5043_AGCAHYST3               = 0x00;
	AX5043_AGCMINMAX3              = 0x00;
	AX5043_TIMEGAIN3               = 0xF5;
	AX5043_DRGAIN3                 = 0xF0;
	AX5043_PHASEGAIN3              = 0xC3;
	AX5043_FREQUENCYGAINA3         = 0x0F;
	AX5043_FREQUENCYGAINB3         = 0x1F;
	AX5043_FREQUENCYGAINC3         = 0x0C;
	AX5043_FREQUENCYGAIND3         = 0x0C;
	AX5043_AMPLITUDEGAIN3          = 0x06;
	AX5043_FREQDEV13               = 0x00;
	AX5043_FREQDEV03               = 0x43;
	AX5043_FOURFSK3                = 0x16;
	AX5043_BBOFFSRES3              = 0x00;
	AX5043_MODCFGF                 = 0x03;
	AX5043_FSKDEV2                 = 0x00;
	AX5043_FSKDEV1                 = 0x04;
	AX5043_FSKDEV0                 = 0x08;
	AX5043_MODCFGA                 = 0x06;//0x04 | 0x01;  //default diff transmitter  0x05;  Enable Single Ended Transmitter 0x02;
	AX5043_TXRATE2                 = 0x00;
	AX5043_TXRATE1                 = 0x0C;
	AX5043_TXRATE0                 = 0x19;
	AX5043_TXPWRCOEFFB1            = 0x0F;
	AX5043_TXPWRCOEFFB0            = 0xFF;
	AX5043_PLLVCOI                 = 0x99;
	AX5043_PLLRNGCLK               = 0x04;
	AX5043_BBTUNE                  = 0x0F;
	AX5043_BBOFFSCAP               = 0x77;
	AX5043_PKTADDRCFG              = 0x01;
	AX5043_PKTLENCFG               = 0x80;
	AX5043_PKTLENOFFSET            = 0x00;
	AX5043_PKTMAXLEN               = 0xC8;
	AX5043_MATCH0PAT3              = 0xAA;
	AX5043_MATCH0PAT2              = 0xCC;
	AX5043_MATCH0PAT1              = 0xAA;
	AX5043_MATCH0PAT0              = 0xCC;
	AX5043_MATCH0LEN               = 0x9F;
	AX5043_MATCH0MAX               = 0x1F;
	AX5043_MATCH1PAT1              = 0x55;
	AX5043_MATCH1PAT0              = 0x55;
	AX5043_MATCH1LEN               = 0x8A;
	AX5043_MATCH1MAX               = 0x0A;
	AX5043_TMGTXBOOST              = 0x3E;
	AX5043_TMGTXSETTLE             = 0x31;
	AX5043_TMGRXBOOST              = 0x3E;
	AX5043_TMGRXSETTLE             = 0x31;
	AX5043_TMGRXOFFSACQ            = 0x00;
	AX5043_TMGRXCOARSEAGC          = 0x7F;
	AX5043_TMGRXRSSI               = 0x03;
	AX5043_TMGRXPREAMBLE2          = 0x35;
	AX5043_RSSIABSTHR              = 0xCD;  //0xCD;
	AX5043_BGNDRSSITHR             = 0x00;
	AX5043_PKTCHUNKSIZE            = 0x0D;
	AX5043_PKTACCEPTFLAGS          = 0x20;
	AX5043_DACVALUE1               = 0x00;
	AX5043_DACVALUE0               = 0x00;
	AX5043_DACCONFIG               = 0x00;
	AX5043_REF                     = 0x03;
	AX5043_XTALOSC                 = 0x04;
	AX5043_XTALAMPL                = 0x00;
	AX5043_0xF1C                   = 0x07;
	AX5043_0xF21                   = 0x68;
	AX5043_0xF22                   = 0xFF;
	AX5043_0xF23                   = 0x84;
	AX5043_0xF26                   = 0x98;
	AX5043_0xF34                   = 0x28;
	AX5043_0xF35                   = 0x11;
	AX5043_0xF44                   = 0x25;
}



__reentrantb void ax5043_set_registers_tx(void) __reentrant
{
	AX5043_PLLLOOP                 = 0x0B;
	AX5043_PLLCPI                  = 0x10;
	AX5043_PLLVCODIV               = 0x24;
	//AX5043_XTALCAP                 = 0x00;
    AX5043_XTALCAP                 = 0x0E;
	AX5043_0xF00                   = 0x0F;
	AX5043_0xF18                   = 0x06;
}


__reentrantb void ax5043_set_registers_rx(void) __reentrant
{
	AX5043_PLLLOOP                 = 0x0B;
	AX5043_PLLCPI                  = 0x10;
	AX5043_PLLVCODIV               = 0x24;
	//AX5043_XTALCAP                 = 0x00;
    AX5043_XTALCAP                 = 0x0E;
	AX5043_0xF00                   = 0x0F;
	AX5043_0xF18                   = 0x02;
}


__reentrantb void ax5043_set_registers_rxwor(void) __reentrant
{
	AX5043_TMGRXAGC                = 0x71;
	AX5043_TMGRXPREAMBLE1          = 0x15;
	AX5043_PKTMISCFLAGS            = 0x05;
	AX5043_AGCGAIN0                = 0x83;
	AX5043_AGCGAIN1                = 0x83;
}


__reentrantb void ax5043_set_registers_rxcont(void) __reentrant
{
	AX5043_TMGRXAGC                = 0x00;
	AX5043_TMGRXPREAMBLE1          = 0x00;
	AX5043_PKTMISCFLAGS            = 0x00;
	AX5043_AGCGAIN0                = 0xC5;
	AX5043_AGCGAIN1                = 0xC5;
}


__reentrantb void ax5043_set_registers_rxcont_singleparamset(void) __reentrant
{
	AX5043_RXPARAMSETS             = 0xFF;
	AX5043_FREQDEV13               = 0x00;
	AX5043_FREQDEV03               = 0x00;
	AX5043_AGCGAIN3                = 0xE7;
}


#endif // M433



__reentrantb void axradio_setup_pincfg1(void) __reentrant
{
	PALTRADIO = 0xc0; //pass through PWRAMP -> PB2 ANTSEL -> PB3
}

__reentrantb void axradio_setup_pincfg2(void) __reentrant
{
	PORTR = (PORTR & 0x3F) | 0x00; //AX8052F143 --> no pull-ups on PR6, PR7
}



#if defined SDCC

#define CONST #

#define CONSTMULFIX24(x)			\
	__asm					\
	mov	r0,dpl				\
	mov	r1,dph				\
	mov	r2,b				\
	mov	r3,a				\
	push	acc				\
	jnb	acc.7,00000$			\
	clr	c				\
	clr	a				\
	subb	a,r0				\
	mov	r0,a				\
	clr	a				\
	subb	a,r1				\
	mov	r1,a				\
	clr	a				\
	subb	a,r2				\
	mov	r2,a				\
	clr	a				\
	subb	a,r3				\
	mov	r3,a				\
00000$:	clr	a				\
	mov	r4,a				\
	mov	r5,a				\
	mov	r6,a				\
	mov	r7,a				\
	;; stage -1				\
	.if	(((x)>>16)&0xff)		\
	mov	a,CONST (((x)>>16)&0xff)	\
	mov	b,r0				\
	mul	ab				\
	mov	r7,a				\
	mov	r4,b				\
	.endif					\
	.if	(((x)>>8)&0xff)			\
	mov	a,CONST (((x)>>8)&0xff)		\
	mov	b,r1				\
	mul	ab				\
	.if	(((x)>>16)&0xff)		\
	add	a,r7				\
	mov	r7,a				\
	mov	a,b				\
	addc	a,r4				\
	mov	r4,a				\
	clr	a				\
	addc	a,r5				\
	mov	r5,a				\
	.else					\
	mov	r7,a				\
	mov	r4,b				\
	.endif					\
	.endif					\
	.if	((x)&0xff)			\
	mov	a,CONST ((x)&0xff)		\
	mov	b,r2				\
	mul	ab				\
	.if	(((x)>>8)&0xffff)		\
	add	a,r7				\
	mov	r7,a				\
	mov	a,b				\
	addc	a,r4				\
	mov	r4,a				\
	clr	a				\
	addc	a,r5				\
	mov	r5,a				\
	.else					\
	mov	r7,a				\
	mov	r4,b				\
	.endif					\
	.endif					\
	;; clear precision extension		\
	clr	a				\
	mov	r7,a				\
	;; stage 0				\
	.if	(((x)>>24)&0xff)		\
	mov	a,CONST (((x)>>24)&0xff)	\
	mov	b,r0				\
	mul	ab				\
	add	a,r4				\
	mov	r4,a				\
	mov	a,b				\
	addc	a,r5				\
	mov	r5,a				\
	clr	a				\
	addc	a,r6				\
	mov	r6,a				\
	.endif					\
	.if	(((x)>>16)&0xff)		\
	mov	a,CONST (((x)>>16)&0xff)	\
	mov	b,r1				\
	mul	ab				\
	add	a,r4				\
	mov	r4,a				\
	mov	a,b				\
	addc	a,r5				\
	mov	r5,a				\
	clr	a				\
	addc	a,r6				\
	mov	r6,a				\
	.endif					\
	.if	(((x)>>8)&0xff)			\
	mov	a,CONST (((x)>>8)&0xff)		\
	mov	b,r2				\
	mul	ab				\
	add	a,r4				\
	mov	r4,a				\
	mov	a,b				\
	addc	a,r5				\
	mov	r5,a				\
	clr	a				\
	addc	a,r6				\
	mov	r6,a				\
	.endif					\
	.if	((x)&0xff)			\
	mov	a,CONST ((x)&0xff)		\
	mov	b,r3				\
	mul	ab				\
	add	a,r4				\
	mov	r4,a				\
	mov	a,b				\
	addc	a,r5				\
	mov	r5,a				\
	clr	a				\
	addc	a,r6				\
	mov	r6,a				\
	.endif					\
	;; stage 1				\
	.if	(((x)>>24)&0xff)		\
	mov	a,CONST (((x)>>24)&0xff)	\
	mov	b,r1				\
	mul	ab				\
	add	a,r5				\
	mov	r5,a				\
	mov	a,b				\
	addc	a,r6				\
	mov	r6,a				\
	clr	a				\
	addc	a,r7				\
	mov	r7,a				\
	.endif					\
	.if	(((x)>>16)&0xff)		\
	mov	a,CONST (((x)>>16)&0xff)	\
	mov	b,r2				\
	mul	ab				\
	add	a,r5				\
	mov	r5,a				\
	mov	a,b				\
	addc	a,r6				\
	mov	r6,a				\
	clr	a				\
	addc	a,r7				\
	mov	r7,a				\
	.endif					\
	.if	(((x)>>8)&0xff)			\
	mov	a,CONST (((x)>>8)&0xff)		\
	mov	b,r3				\
	mul	ab				\
	add	a,r5				\
	mov	r5,a				\
	mov	a,b				\
	addc	a,r6				\
	mov	r6,a				\
	clr	a				\
	addc	a,r7				\
	mov	r7,a				\
	.endif					\
	;; stage 2				\
	.if	(((x)>>24)&0xff)		\
	mov	a,CONST (((x)>>24)&0xff)	\
	mov	b,r2				\
	mul	ab				\
	add	a,r6				\
	mov	r6,a				\
	mov	a,b				\
	addc	a,r7				\
	mov	r7,a				\
	.endif					\
	.if	(((x)>>16)&0xff)		\
	mov	a,CONST (((x)>>16)&0xff)	\
	mov	b,r3				\
	mul	ab				\
	add	a,r6				\
	mov	r6,a				\
	mov	a,b				\
	addc	a,r7				\
	mov	r7,a				\
	.endif					\
	;; stage 3				\
	.if	(((x)>>24)&0xff)		\
	mov	a,CONST (((x)>>24)&0xff)	\
	mov	b,r3				\
	mul	ab				\
	add	a,r7				\
	mov	r7,a				\
	.endif					\
	pop	acc				\
	jnb	acc.7,00001$			\
	clr	c				\
	clr	a				\
	subb	a,r4				\
	mov	dpl,a				\
	clr	a				\
	subb	a,r5				\
	mov	dph,a				\
	clr	a				\
	subb	a,r6				\
	mov	b,a				\
	clr	a				\
	subb	a,r7				\
	sjmp	00002$				\
00001$:	mov	dpl,r4				\
	mov	dph,r5				\
	mov	b,r6				\
	mov	a,r7				\
00002$:						\
	__endasm

#define CONSTMULFIX16(x)			\
	__asm					\
	mov	r0,dpl				\
	mov	a,dph				\
	mov	r1,a				\
	push	acc				\
	jnb	acc.7,00000$			\
	clr	c				\
	clr	a				\
	subb	a,r0				\
	mov	r0,a				\
	clr	a				\
	subb	a,r1				\
	mov	r1,a				\
00000$:	clr	a				\
	mov	r4,a				\
	mov	r5,a				\
	mov	r6,a				\
	mov	r7,a				\
	;; stage -1				\
	.if	(((x)>>16)&0xff)		\
	mov	a,CONST (((x)>>16)&0xff)	\
	mov	b,r0				\
	mul	ab				\
	mov	r7,a				\
	mov	r4,b				\
	.endif					\
	.if	(((x)>>8)&0xff)			\
	mov	a,CONST (((x)>>8)&0xff)		\
	mov	b,r1				\
	mul	ab				\
	.if	(((x)>>16)&0xff)		\
	add	a,r7				\
	mov	r7,a				\
	mov	a,b				\
	addc	a,r4				\
	mov	r4,a				\
	clr	a				\
	addc	a,r5				\
	mov	r5,a				\
	.else					\
	mov	r7,a				\
	mov	r4,b				\
	.endif					\
	.endif					\
	;; clear precision extension		\
	clr	a				\
	mov	r7,a				\
	;; stage 0				\
	.if	(((x)>>24)&0xff)		\
	mov	a,CONST (((x)>>24)&0xff)	\
	mov	b,r0				\
	mul	ab				\
	add	a,r4				\
	mov	r4,a				\
	mov	a,b				\
	addc	a,r5				\
	mov	r5,a				\
	clr	a				\
	addc	a,r6				\
	mov	r6,a				\
	.endif					\
	.if	(((x)>>16)&0xff)		\
	mov	a,CONST (((x)>>16)&0xff)	\
	mov	b,r1				\
	mul	ab				\
	add	a,r4				\
	mov	r4,a				\
	mov	a,b				\
	addc	a,r5				\
	mov	r5,a				\
	clr	a				\
	addc	a,r6				\
	mov	r6,a				\
	.endif					\
	;; stage 1				\
	.if	(((x)>>24)&0xff)		\
	mov	a,CONST (((x)>>24)&0xff)	\
	mov	b,r1				\
	mul	ab				\
	add	a,r5				\
	mov	r5,a				\
	mov	a,b				\
	addc	a,r6				\
	mov	r6,a				\
	clr	a				\
	addc	a,r7				\
	mov	r7,a				\
	.endif					\
	pop	acc				\
	jnb	acc.7,00001$			\
	clr	c				\
	clr	a				\
	subb	a,r4				\
	mov	dpl,a				\
	clr	a				\
	subb	a,r5				\
	mov	dph,a				\
	clr	a				\
	subb	a,r6				\
	mov	b,a				\
	clr	a				\
	subb	a,r7				\
	sjmp	00002$				\
00001$:	mov	dpl,r4				\
	mov	dph,r5				\
	mov	b,r6				\
	mov	a,r7				\
00002$:						\
	__endasm

#else // SDCC

#define MUL8_16(x,y) ((uint8_t)((x)&0xff)*(uint16_t)(uint8_t)((y)&0xff))

#define CONSTMULFIX24(x)					\
	if (f >= 0) {						\
		uint32_t r = MUL8_16((x)>>16,f);		\
		r += MUL8_16((x)>>8,f>>8);			\
		r += MUL8_16((x),f>>16);			\
		r >>= 8;					\
		r += MUL8_16((x)>>24,f);			\
		r += MUL8_16((x)>>16,f>>8);			\
		r += MUL8_16((x)>>8,f>>16);			\
		r += MUL8_16((x),f>>24);			\
		r += ((uint32_t)MUL8_16((x)>>24,f>>8))<<8;	\
		r += ((uint32_t)MUL8_16((x)>>16,f>>16))<<8;	\
		r += ((uint32_t)MUL8_16((x)>>8,f>>24))<<8;	\
		r += ((uint32_t)MUL8_16((x)>>24,f>>16))<<16;	\
		r += ((uint32_t)MUL8_16((x)>>16,f>>24))<<16;	\
		r += ((uint32_t)MUL8_16((x)>>24,f>>24))<<24;	\
		return r;					\
	}							\
	{							\
		int32_t r;					\
		f = -f;						\
		r = -(uint32_t)MUL8_16((x)>>16,f);		\
		r -= (uint32_t)MUL8_16((x)>>8,f>>8);		\
		r -= (uint32_t)MUL8_16((x),f>>16);		\
		r >>= 8;					\
		r -= (uint32_t)MUL8_16((x)>>24,f);		\
		r -= (uint32_t)MUL8_16((x)>>16,f>>8);		\
		r -= (uint32_t)MUL8_16((x)>>8,f>>16);		\
		r -= (uint32_t)MUL8_16((x),f>>24);		\
		r -= ((uint32_t)MUL8_16((x)>>24,f>>8))<<8;	\
		r -= ((uint32_t)MUL8_16((x)>>16,f>>16))<<8;	\
		r -= ((uint32_t)MUL8_16((x)>>8,f>>24))<<8;	\
		r -= ((uint32_t)MUL8_16((x)>>24,f>>16))<<16;	\
		r -= ((uint32_t)MUL8_16((x)>>16,f>>24))<<16;	\
		r -= ((uint32_t)MUL8_16((x)>>24,f>>24))<<24;	\
		return r;					\
	}

#define CONSTMULFIX16(x)					\
	if (f >= 0) {						\
		uint32_t r = MUL8_16((x)>>16,f);		\
		r += MUL8_16((x)>>8,f>>8);			\
		r >>= 8;					\
		r += MUL8_16((x)>>24,f);			\
		r += MUL8_16((x)>>16,f>>8);			\
		r += ((uint32_t)MUL8_16((x)>>24,f>>8))<<8;	\
		return r;					\
	}							\
	{							\
		int32_t r;					\
		f = -f;						\
		r = -(uint32_t)MUL8_16((x)>>16,f);		\
		r -= (uint32_t)MUL8_16((x)>>8,f>>8);		\
		r >>= 8;					\
		r -= (uint32_t)MUL8_16((x)>>24,f);		\
		r -= (uint32_t)MUL8_16((x)>>16,f>>8);		\
		r -= ((uint32_t)MUL8_16((x)>>24,f>>8))<<8;	\
		return r;					\
	}

#endif // SDCC

#if defined SDCC
// do not mark as reentrant, otherwise the register allocator will generate suboptimal code
#pragma nooverlay
int32_t axradio_conv_freq_fromhz(int32_t f)
#else
__reentrantb int32_t axradio_conv_freq_fromhz(int32_t f) __reentrant
#endif
{
    #ifdef M868
	/* scale by 0.349525 (true 0.349525) */
	CONSTMULFIX24(0x597a7e);
    #endif // M868

    #ifdef M433
    /* scale by 0.645278 (true 0.645278) */
	CONSTMULFIX24(0xa530e8);
    #endif // M433
}

#if defined SDCC
// do not mark as reentrant, otherwise the register allocator will generate suboptimal code
#pragma nooverlay
int32_t axradio_conv_freq_tohz(int32_t f)
#else
__reentrantb int32_t axradio_conv_freq_tohz(int32_t f) __reentrant
#endif
{
    #ifdef M868
	/* scale by 2.861023 (true 2.861023) */
	CONSTMULFIX24(0x2dc6c00);
    #endif // M868

    #ifdef M433
	/* scale by 1.549721 (true 1.549721) */
	CONSTMULFIX24(0x18cba80);
    #endif // M433
}

const uint8_t __code axradio_phy_innerfreqloop = 0;

#if defined SDCC
// do not mark as reentrant, otherwise the register allocator will generate suboptimal code
#pragma nooverlay
int32_t axradio_conv_freq_fromreg(int32_t f)
#else
__reentrantb int32_t axradio_conv_freq_fromreg(int32_t f) __reentrant
#endif
{
	/* scale by 1.000000 (true 1.000000) */
	CONSTMULFIX16(0x1000000);
}

#if defined SDCC
// do not mark as reentrant, otherwise the register allocator will generate suboptimal code
#pragma nooverlay
int32_t axradio_conv_timeinterval_totimer0(int32_t dt)
#else
__reentrantb int32_t axradio_conv_timeinterval_totimer0(int32_t dt) __reentrant
#endif
{
	int32_t r;

    #ifdef M868
    /* scale by 0.010864 (true 0.010923) */
	dt >>= 6;
	r = dt;
	dt >>= 2;
	r -= dt;
	dt >>= 2;
	r -= dt;
	dt >>= 3;
	r += dt;
    #endif // M868

    #ifdef M433
    /* scale by 0.020142 (true 0.020165) */
    dt >>= 6;
	r = dt;
	dt >>= 2;
	r += dt;
	dt >>= 3;
	r += dt;
	dt >>= 2;
	r += dt;

    #endif // M433

	return r;
}

__reentrantb uint8_t axradio_byteconv(uint8_t b) __reentrant
{
	return b;
}


__reentrantb void axradio_byteconv_buffer(uint8_t __xdata *buf, uint16_t buflen) __reentrant
{
}

__reentrantb uint16_t axradio_framing_check_crc(uint8_t __xdata *pkt, uint16_t cnt) __reentrant
{
	if (crc_ccitt(pkt, cnt, 0xFFFF) != 0xF0B8)
		return 0;
	return cnt;
}

__reentrantb uint16_t axradio_framing_append_crc(uint8_t __xdata *pkt, uint16_t cnt) __reentrant
{
	uint16_t s = 0xFFFF;
	s = crc_ccitt(pkt, cnt, s);
	pkt += cnt;
	*pkt++ = ~(uint8_t)(s);
	*pkt++ = ~(uint8_t)(s >> 8);
	return cnt + 2;
}





// physical layer
const uint8_t __code axradio_phy_pn9 = 0;
const uint8_t __code axradio_phy_nrchannels = 1;
#ifdef M868
const uint32_t __code axradio_phy_chanfreq[1] = { 0x1216eeef };
#endif // M868
#ifdef M433
//const uint32_t __code axradio_phy_chanfreq[1] = { 0x10a76277 };
const uint32_t __code axradio_phy_chanfreq[1] = { 0x10a76277+194 }; // add frqoffset

#endif // M433

const uint8_t __code axradio_phy_chanpllrnginit[1] = { 0x0a };
const uint8_t __code axradio_phy_chanvcoiinit[1] = { 0x99 };
uint8_t __xdata axradio_phy_chanpllrng[1];
uint8_t __xdata axradio_phy_chanvcoi[1];
const uint8_t __code axradio_phy_vcocalib = 0;

#ifdef M868
const int32_t __code axradio_phy_maxfreqoffset = 1821;	//300;
#endif // M868
#ifdef M433
const int32_t __code axradio_phy_maxfreqoffset = 1676;
#endif // M433

const int8_t __code axradio_phy_rssioffset = 64;
// axradio_phy_rssioffset is added to AX5043_RSSIREFERENCE and subtracted from chip RSSI value to prevent overflows (8bit RSSI only goes down to -128)
// axradio_phy_rssioffset is also added to AX5043_RSSIABSTHR
#ifdef M868
const int8_t __code axradio_phy_rssireference = 0xF6 + 64;
#endif // M868
#ifdef M433
const int8_t __code axradio_phy_rssireference = 0xFA + 64;  //0xFA + 64;
#endif // M433

const int8_t __code axradio_phy_channelbusy = -96 + 64;    //-96 + 64;
const uint16_t __code axradio_phy_cs_period = 7; // timer0 units, 10ms
const uint8_t __code axradio_phy_cs_enabled = 0;
const uint8_t __code axradio_phy_lbt_retries = 0;
const uint8_t __code axradio_phy_lbt_forcetx = 0;

#ifdef M868
const uint16_t __code axradio_phy_preamble_wor_longlen = 3;//4; // wor_longlen + wor_len totals to 240.0ms plus 32bits
const uint16_t __code axradio_phy_preamble_wor_len = 224;//160;
#endif // M868
#ifdef M433
const uint16_t __code axradio_phy_preamble_wor_longlen = 4; // wor_longlen + wor_len totals to 240.0ms plus 32bits
const uint16_t __code axradio_phy_preamble_wor_len = 160;
#endif // M433
const uint16_t __code axradio_phy_preamble_longlen = 0;
const uint16_t __code axradio_phy_preamble_len = 32;
const uint8_t __code axradio_phy_preamble_byte = 0x55;
const uint8_t __code axradio_phy_preamble_flags = 0x38;
const uint8_t __code axradio_phy_preamble_appendbits = 0;
const uint8_t __code axradio_phy_preamble_appendpattern = 0x00;

//framing
const uint8_t __code axradio_framing_maclen = 3;
const uint8_t __code axradio_framing_addrlen = 2;
const uint8_t __code axradio_framing_destaddrpos = 1;
const uint8_t __code axradio_framing_sourceaddrpos = 0xff;
const uint8_t __code axradio_framing_lenpos = 0;
const uint8_t __code axradio_framing_lenoffs = 0;
const uint8_t __code axradio_framing_lenmask = 0xff;
const uint8_t __code axradio_framing_swcrclen = 0;

const uint8_t __code axradio_framing_synclen = 32;
const uint8_t __code axradio_framing_syncword[] = { 0xcc, 0xaa, 0xcc, 0xaa};
#ifdef M868
const uint8_t __code axradio_framing_syncflags = 0x38;
#endif // M868
#ifdef M433
const uint8_t __code axradio_framing_syncflags = 0x38;
#endif // M433
const uint8_t __code axradio_framing_enable_sfdcallback = 0;

const uint32_t __code axradio_framing_ack_timeout = 25; // 37.0ms in wtimer0 units (640Hz)
const uint32_t __code axradio_framing_ack_delay = 313; // 1.0ms in wtimer1 units (20MHz/64)
const uint8_t __code axradio_framing_ack_retransmissions = 0;//3;
const uint8_t __code axradio_framing_ack_seqnrpos = 0xff;

const uint8_t __code axradio_framing_minpayloadlen = 1; // must be set to 1 if the payload directly follows the destination address, and a CRC is configured
//WOR
const uint16_t __code axradio_wor_period = 128;

// synchronous mode
#ifdef M868
const uint32_t __code axradio_sync_period = 32768;//*2; // ACTUALLY FREQ, NOT PERIOD!
const uint32_t __code axradio_sync_xoscstartup = 49;
const uint32_t __code axradio_sync_slave_syncwindow = 98304;//*2; // 3.000s*2
const uint32_t __code axradio_sync_slave_initialsyncwindow = 5898240; //180.000s
const uint32_t __code axradio_sync_slave_syncpause = 19660800; // 600.000s
const int16_t __code axradio_sync_slave_maxperiod = 2020;   //2439;//2020; // in (2^SYNC_K1) * wtimer0 units
const uint8_t __code axradio_sync_slave_resyncloss = 11;  // resyncloss is one more than the number of missed packets to cause a resync
// window 0 is the first window after synchronisation
// window 1 is the window normally used when there are no lost packets
// window 2 is used after one packet is lost, etc
const uint8_t __code axradio_sync_slave_nrrx = 3;
#endif // M868

#ifdef M433
const uint32_t __code axradio_sync_period = 66493; // ACTUALLY FREQ, NOT PERIOD!
const uint32_t __code axradio_sync_xoscstartup = 49;
const uint32_t __code axradio_sync_slave_syncwindow = 199479; // 6.088s
const uint32_t __code axradio_sync_slave_initialsyncwindow = 5898240; //180.000s
const uint32_t __code axradio_sync_slave_syncpause = 19660800; // 600.000s
const int16_t __code axradio_sync_slave_maxperiod = 2452; // in (2^SYNC_K1) * wtimer0 units
const uint8_t __code axradio_sync_slave_resyncloss = 11;  // resyncloss is one more than the number of missed packets to cause a resync
// window 0 is the first window after synchronisation
// window 1 is the window normally used when there are no lost packets
// window 2 is used after one packet is lost, etc
const uint8_t __code axradio_sync_slave_nrrx = 3;
#endif // M433


#if RADIO_MODE==AXRADIO_MODE_WOR_RECEIVE || RADIO_MODE==AXRADIO_MODE_WOR_TRANSMIT

const uint32_t __code axradio_sync_slave_rxadvance[] = { 8569, 8515, 8599 };// 261.494ms, 259.846ms, 262.408ms
const uint32_t __code axradio_sync_slave_rxwindow[] = { 8602, 8494, 8662 }; // 262.501ms, 259.205ms, 264.329ms
const uint32_t __code axradio_sync_slave_rxtimeout = 9845; // 300.4ms, maximum duration of a packet
/*
const uint32_t __code axradio_sync_slave_rxadvance[] = { 8572, 8515, 8599 };// 261.586ms, 259.846ms, 262.408ms
const uint32_t __code axradio_sync_slave_rxwindow[] = { 8608, 8494, 8662 }; // 262.685ms, 259.205ms, 264.329ms
const uint32_t __code axradio_sync_slave_rxtimeout = 9845; // 300.4ms, maximum duration of a packet
*/
#endif

#if RADIO_MODE==AXRADIO_MODE_WOR_ACK_RECEIVE || RADIO_MODE==AXRADIO_MODE_WOR_ACK_TRANSMIT
#ifdef M868
const uint32_t __code axradio_sync_slave_rxadvance[] = { 8569, 8515, 8599 };// 261.494ms, 259.846ms, 262.408ms
const uint32_t __code axradio_sync_slave_rxwindow[] = { 8602, 8494, 8662 }; // 262.501ms, 259.205ms, 264.329ms
const uint32_t __code axradio_sync_slave_rxtimeout = 9845;  //8697.7+Tx_packet_len*54.633; // 300.4ms, maximum duration of a packet
#endif // M868

#ifdef M433
const uint32_t __code axradio_sync_slave_rxadvance[] = { 8569, 8515, 8599 };// 261.499ms, 259.851ms, 262.413ms
const uint32_t __code axradio_sync_slave_rxwindow[] = { 8602, 8494, 8662 }; // 262.506ms, 259.210ms, 264.334ms
const uint32_t __code axradio_sync_slave_rxtimeout = 9845; // 300.4ms, maximum duration of a packet
#endif // M433

#endif


#if RADIO_MODE==AXRADIO_MODE_ASYNC_RECEIVE || RADIO_MODE==AXRADIO_MODE_ASYNC_TRANSMIT

const uint32_t __code axradio_sync_slave_rxadvance[] = { 705, 651, 735 };// 21.494ms, 19.846ms, 22.408ms
const uint32_t __code axradio_sync_slave_rxwindow[] = { 738, 630, 798 }; // 22.501ms, 19.205ms, 24.329ms
const uint32_t __code axradio_sync_slave_rxtimeout = 1980; // 60.4ms, maximum duration of a packet

#endif

