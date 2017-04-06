#ifndef EEPROM_H_INCLUDED
#define EEPROM_H_INCLUDED


#include "../AX_Radio_Lab_output/configms.h"
#include "libmftypes.h"
#include "ax8052.h"
#include "misc.h"

#define SET 1
#define CLR 0

#define EEPROM_LEN 16

#define EE_FQSET 0x0030
#define EE_FQSETPM 0x0031
#define EE_APC 0x0032

#define EE_TP10

#ifdef EE_TP10

// default TP10
#define SDADIR DIRA
#define CLKDIR DIRA

#define SDAPORT PORTA
#define CLKPORT PORTA

#define SDAPIN PINA_1
#define CLKPIN PINA_0

#define SDAPORT_N PORTA_1
#define CLKPORT_N PORTA_0

#define SDAPIN_N BIT1SET
#define CLKPIN_N BIT0SET


#define ep_cs_w_set()	do {CLKPORT |= (1<<0)|(1<<1);} while (0)
#define ep_cs_w_clr()	do {CLKPORT &= (uint8_t)~(1<<0)&(uint8_t)~(1<<1);} while (0)

#define ep_sda_w_set()	do {SDAPORT |= (1<<1);} while (0)
#define ep_sda_w_clr()	do {SDAPORT &= (uint8_t)~(1<<1);} while (0)

#define ep_clk_w_set()	do {CLKPORT |= (1<<0);} while (0)
#define ep_clk_w_clr()	do {CLKPORT &= (uint8_t)~(1<<0);} while (0)

#else
// sample test


#define SDADIR DIRC
#define CLKDIR DIRC

#define SDAPORT PORTC
#define CLKPORT PORTC

#define SDAPIN PINC_0
#define CLKPIN PINC_1

#define SDAPORT_N PORTC_0
#define CLKPORT_N PORTC_1

#define SDAPIN_N BIT0SET
#define CLKPIN_N BIT1SET

#define ep_cs_w_set()	do {CLKPORT |= (1<<0)|(1<<1);} while (0)
#define ep_cs_w_clr()	do {CLKPORT &= (uint8_t)~(1<<0)&(uint8_t)~(1<<1);} while (0)

#define ep_sda_w_set()	do {SDAPORT |= (1<<0);} while (0)
#define ep_sda_w_clr()	do {SDAPORT &= (uint8_t)~(1<<0);} while (0)

#define ep_clk_w_set()	do {CLKPORT |= (1<<1);} while (0)
#define ep_clk_w_clr()	do {CLKPORT &= (uint8_t)~(1<<1);} while (0)

#endif


uint8_t eprd_da=0;
extern uint8_t	Ep_err;

void Ep_start(void);
void Ep_stop(void);
void sendack(void);
void sendnoack(void);
void Ep_err_check(void);


extern  uint8_t wri_ep1b(uint8_t da) ;
extern  void wri_subp(uint16_t addr,uint8_t wrida) ;
extern  void ep_pagew(uint16_t addr,const uint8_t *Txbuf,uint8_t num) ;
extern  uint8_t read_ep1b(void) ;
extern  uint8_t read_subp(uint16_t addr) ;
extern  void ep_pager(uint16_t addr,uint8_t *daddr,uint8_t num) ;



#endif // EEPROM_H_INCLUDED
