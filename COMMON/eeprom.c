
#include <stdlib.h>
#include <string.h>

#include "eeprom.h"

//uint8_t eprd_da=0;
//uint8_t	Ep_err;


/*************************************************
*        EP start
*************************************************/
void Ep_start(void)
{
    SDADIR|=BIT0SET|BIT1SET;

    NOP();
    ep_cs_w_set();
    NOP();
    ep_sda_w_clr();
    NOP();
    ep_clk_w_clr();
    NOP();NOP();NOP();
}


/*************************************************
*        EP stop
*************************************************/

void Ep_stop(void)
{
    SDADIR|=BIT0SET|BIT1SET;
	NOP();
    ep_sda_w_clr();
	NOP();
	ep_clk_w_set();
	NOP();
	ep_sda_w_set();

}

/*************************************************
*        EP aned ack
*        NOT USE
*************************************************/

void sendack(void)
{
    NOP();

    CLKPORT|=BIT0SET;
    CLKDIR&=BIT0CLR;

    //query
    ep_clk_w_set();

    ep_clk_w_clr();
    NOP();

    CLKDIR|=BIT0SET;
    CLKPORT&=BIT0CLR;
}

/*************************************************
*        EP send no ack
*************************************************/

void sendnoack(void)
{

    NOP();NOP();
    ep_sda_w_set();
    ep_clk_w_set();
    NOP();NOP();
    ep_clk_w_clr();
    ep_sda_w_clr();
    NOP();

/*
    NOP();
    ep_sda_w_set();
    NOP();
    ep_clk_w_set();
    NOP();
    ep_clk_w_clr();
    NOP();
    ep_sda_w_clr();
    NOP();
    //ep_sda_w_set();*/

}

/*************************************************
*        EP err check
*        NOT USE
*************************************************/

void Ep_err_check(void)
{
	if(Ep_err==SET)
	{	/*showEp_Err();*/ while(1);	}
	else
	if(Ep_err==CLR)
	{	NOP();	}
}


/*************************************************
*        Write 1byte data to Ep
*************************************************/

uint8_t wri_ep1b(uint8_t da)
{
	uint8_t i=0;uint8_t datmp=0;uint8_t err_cou=0;
    uint8_t ack=0;

    datmp=da;

    SDADIR|=BIT0SET|BIT1SET;

    for(i=0;i<8;i++)
    {
        if(datmp & 0x80)
            {ep_sda_w_set();}
        else
            {ep_sda_w_clr();}

        NOP();
        ep_clk_w_set();
        NOP();NOP();
        ep_clk_w_clr();
        NOP();NOP();
        datmp = datmp<<1;
    }

    ep_cs_w_clr();

    NOP();
    SDADIR&=BIT0CLR;
    ep_clk_w_set();
    NOP();
    ack=SDAPIN&BIT0SET;
    ep_clk_w_clr();
    NOP();
    SDADIR|=BIT0SET;

    return ack;

}


/*************************************************
*	   get addr Write 1byte to Ep
*************************************************/
void wri_subp(uint16_t addr,uint8_t wrida)
{
    uint8_t addrh,addrl;

	addrh = addr >> 8;
	addrl = addr & 0xFF;

	Ep_start();
    Ep_err|=wri_ep1b(0xA0);;
	Ep_err|=wri_ep1b(addrh);
	Ep_err|=wri_ep1b(addrl);
	Ep_err|=wri_ep1b(wrida);
	Ep_stop();

}

/*************************************************
*		Page Wrtie to Ep
*************************************************/

void ep_pagew(uint16_t addr,const uint8_t *Txbuf,uint8_t num)
{
	uint8_t i,addrh,addrl;

	addrh = addr >> 8;
	addrl = addr & 0xFF;

	Ep_start();

    Ep_err|=wri_ep1b(0xA0);; //Write EE
    Ep_err|=wri_ep1b(addrh);
    Ep_err|=wri_ep1b(addrl);

	for(i=num;i>0;i--)
	{
		Ep_err|=wri_ep1b(*Txbuf++);
	}

	Ep_stop();

}

/*************************************************
*        Read 1byte data from Ep
*************************************************/

uint8_t read_ep1b(void)
{
	uint8_t i;
	eprd_da=0;

    SDADIR&=(uint8_t)~(1<<1);SDAPORT|=(1<<1);

    for(i=0;i<8;i++)
    {
        ep_clk_w_clr();
        NOP();NOP();
        ep_clk_w_set();
        NOP();
        eprd_da=eprd_da<<1;
        eprd_da=eprd_da|SDAPIN;
    }
    ep_clk_w_clr();

    SDADIR|=(1<<1);SDAPORT&= (uint8_t)~(1<<1);

    return(eprd_da);
}

/*************************************************
*		give addr read 1byte data from Ep
*************************************************/

uint8_t read_subp(uint16_t addr)
{
    uint8_t addrh,addrl,data;

	addrh = addr >> 8;
	addrl = addr & 0xFF;

	Ep_start();
	Ep_err|=wri_ep1b(0xA0); //Write EE
	Ep_err|=wri_ep1b(addrh);
	Ep_err|=wri_ep1b(addrl);
    Ep_stop();
	Ep_start();
	Ep_err|=wri_ep1b(0xA1); //Read EE
	data=read_ep1b();
    sendnoack();
	Ep_stop();

    return(data);
}

/*************************************************
*		Page Read from Ep
*************************************************/

void ep_pager(uint16_t addr,uint8_t *daddr,uint8_t num)
{
    uint8_t i,addrh,addrl;

	addrh = addr >> 8;
	addrl = addr & 0xFF;

    for(i=0;i<num;i++){

        Ep_start();

        Ep_err|=wri_ep1b(0xA0); //Write EE
        Ep_err|=wri_ep1b(addrh);
        Ep_err|=wri_ep1b(addrl+i);

        Ep_stop();
        Ep_start();
        Ep_err|=wri_ep1b(0xA1); //Read EE

        *daddr++=read_ep1b();
        eprd_da=0;
        sendnoack();
        Ep_stop();

    }

}



