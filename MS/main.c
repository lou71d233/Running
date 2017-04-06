/** \file SLAVE\main.c
*
* \brief Code skeleton for SLAVE module, illustrating reception of packets.
* \brief The packet format is determined by AX-RadioLAB_output\config.c, produced by the AX-RadioLab GUI
*
*/

// Copyright (c) 2007,2008,2009,2010,2011,2012,2013, 2014 AXSEM AG
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1.Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     2.Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     3.Neither the name of AXSEM AG, Duebendorf nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//     4.All advertising materials mentioning features or use of this software
//       must display the following acknowledgement:
//       This product includes software developed by AXSEM AG and its contributors.
//     5.The usage of this source code is only granted for operation with AX5043
//       and AX8052F143. Porting to other radio or communication devices is
//       strictly prohibited.
//
// THIS SOFTWARE IS PROVIDED BY AXSEM AG AND CONTRIBUTORS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL AXSEM AG AND CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "../AX_Radio_Lab_output/configms.h"
//MS add
#include "../COMMON/easyax5043.h"
#include "../COMMON/eeprom.h"
#include "../COMMON/adc.h"
#include "moto.h"

#include <ax8052f143.h>
#include <libmftypes.h>
#include <libmfradio.h>
#include <libmfflash.h>
#include <libmfwtimer.h>
#include <math.h>

#ifdef USE_COM0
#include <libmfuart0.h>
#endif // USE_COM0

#ifdef USE_LCD
#include <libaxlcd2.h>
#endif // USE_LCD

#ifdef USE_DBGLINK
#include <libmfdbglink.h>
#endif // USE_DBGLINK

#define ADJUST_FREQREG
#define FREQOFFS_K 3

#if defined(USE_LCD) || defined(USE_COM0)
#define USE_DISPLAY
#endif // defined(USE_LCD) || defined(USE_COM0)

#include "../COMMON/display_com0.h"

#ifdef MINI_KIT
#include "../COMMON/libminidvkled.h"

#define BUTTON_INTCHG INTCHGC
#define BUTTON_PIN    PINC
#define BUTTON_PIN_N    PINC_4
#define BUTTON_MASK   0x10

#define BUTTON_MASK_EDGE   0xEF

#define BUTTON_PIN_N PINC_4

#else
/*
#include <libdvk2leds.h>

#define BUTTON_INTCHG INTCHGB
#define BUTTON_PIN    PINB
#define BUTTON_MASK   0x04
*/

#include "../COMMON/libminidvkled.h"

#define BUTTON_INTCHG   INTCHGC
#define BUTTON_PIN      PINC
#define BUTTON_PIN_N    PINC_4
#define BUTTON_MASK     0x10

#define BUTTON_MASK_EDGE   0xEF

//SET KEY TO PROMGRAM
#define BUTTON_SET_INTCHG   INTCHGB
#define BUTTON_SET_PIN      PINB
#define BUTTON_SET_PIN_N    PINB_0

#define BUTTON_SET_MASK     0x01

#define BUTTON_SET_MASK_EDGE 0xFE

#endif // MINI_KIT

#include <stdlib.h>
#include <string.h>

#include "../COMMON/misc.h"
#include "../COMMON/configcommon.h"

#if defined(SDCC)
extern uint8_t _start__stack[];
#endif

uint8_t __data coldstart = 1; // caution: initialization with 1 is necessary! Variables are initialized upon _sdcc_external_startup returning 0 -> the coldstart value returned from _sdcc_external startup does not survive in the coldstart case
uint16_t __data pkts_received = 0, pkts_missing = 0;
uint16_t __data pkt_counter = 0;

//#define WDT_EN
#define WDT_EN_ADDR 0x000E

#define TX_ON_DEMAND
#define SWAP(A,B){A^=B;B^=A;A^=B;}
#define RSSI_UP 195
#define RSSI_DOWN 170
#define bb_timeout 7
#define UT_LENS 64

struct axradio_status __xdata *axst;
uint8_t button_busy_times = 0;
uint8_t deglitch_busy = 0;
uint8_t button_busy = 0;
uint32_t button_busy_timeout = 0;

uint8_t lock_wheel = 0;
uint8_t uart0_buff[UT_LENS];
uint8_t trssi=0;
uint8_t __data eep00[16],eep01[16],eep02[16];

#define MOTOR_EN_TIMEOUT 6
uint8_t motor_en_time_st =0;
uint8_t motor_en_timeout =0;

#define PROGRAM_TIMEOUT 6
uint8_t program_st =0;
uint8_t program_timeout =0;

#define VDDIO_TIMEOUT 3
uint8_t vddio_st=0;
uint8_t vddio_timeout=0;

#define rx_save_rate 4
#define rx_save_full 10

struct rx_save
{
    uint8_t ct;
    uint8_t fg;
};

#ifdef TX_ON_DEMAND
#if defined __ICC8051__
//bool deglitch_busy = 0;
#else
//__bit deglitch_busy = 0;
#endif
#endif

struct wtimer_desc __xdata wakeup_desc;

static void change_radio_mode_trx(uint8_t trx);

void set_timer2_onoff(uint8_t onoff);

static void pwrmgmt_irq(void) __interrupt(INT_POWERMGMT)
{
    uint8_t pc = PCON;
    if (!(pc & 0x80))
        return;
    GPIOENABLE = 0;
    IE = EIE = E2IE = 0;
    for (;;)
        PCON |= 0x01;
}

void timer2_irq(void) __interrupt INT_TIMER2
{
    //PORTC^=0x02;
    if((T2STATUS&0x01)/*||(T2STATUS&0x02)*/)
    {
        if(motor_en_time_st)
        {
            if(motor_en_timeout>=MOTOR_EN_TIMEOUT)
            {
                //PORTC^=0x04;
                motor_en_timeout=0;
                set_timer2_onoff(CLR);
                stop_moto_drive();
            }else
                motor_en_timeout++;
        }

        if(program_st>0)
        {
            if(program_timeout>=PROGRAM_TIMEOUT)
            {
                program_timeout=0;
                program_st=2;
            }else{
                display_writestr(".");com0_tx(0x0d);
                program_timeout++;
            }
        }

        if(vddio_st>0)
        {
            if(vddio_timeout>=VDDIO_TIMEOUT)
            {
                vddio_timeout=0;
                vddio_st=2;
            }else{
                vddio_timeout++;
            }
        }
    }
}

//timer 2 init
void timer2_init(void)
{
    //timer2 init
    T2CNT0=0;
    T2CNT1=0;

    T2PERIOD0=255;
    T2PERIOD1=255;

    //T2CLKSRC //T2CLKSYNC : 0 | T2CLKINV : 0 | T2CLKDIV : /64 | T2CLKSRC : FRCOSC
    //set_timer2_onoff(CLR);
    T2CLKSRC = 0b00111000;

    //T2MODE   //T2IRQMU : 1 | T2IRQMO : 1 | T2PRBUF : 11  | T2LBUF : 0 | T2MODE : 011
    T2MODE = 0b11110011;
}

void set_timer2_onoff(uint8_t onoff)
{
    //timer2 init
    T2CNT0=0;
    T2CNT1=0;

    switch(onoff)
    {
        case 0:
            motor_en_time_st=program_st=0;
            break;
        case 1:
            motor_en_timeout=0;
            motor_en_time_st=1;
            break;
        case 2:
            program_st=1;
            break;
        case 3:
            vddio_st=1;
            break;
    }
}


//TX transmit packet
static void transmit_packet(void)
{
    static uint8_t __xdata demo_packet_[sizeof(demo_packet)];

    ++pkt_counter;
#if defined __ICC8051__
    {
        uint8_t c = sizeof(demo_packet);
        if (c) {
            const uint8_t __code *sp = demo_packet;
            uint8_t __xdata *dp = demo_packet_;
            do {
                *dp++ = *sp++;
            } while (--c);
        }
    }
#else
    //memcpy(demo_packet_, demo_packet, sizeof(demo_packet));
    if(lock_wheel==0)memcpy(demo_packet_, packet_lock_wheel, sizeof(packet_lock_wheel));
    else if(lock_wheel==1)memcpy(demo_packet_, packet_unlock_wheel, sizeof(packet_unlock_wheel));

#endif
    if (framing_insert_counter) {
        demo_packet_[framing_counter_pos] = pkt_counter & 0xFF ;
        demo_packet_[framing_counter_pos+1] = (pkt_counter>>8) & 0xFF;
    }

#ifdef EXT_POWER_AMP
    //AX5043_PWRAMP=1;
#endif // EXT_POWER_AMP
    //axradio_transmit(&remoteaddr, demo_packet_, sizeof(demo_packet));
    axradio_transmit(&axradio_default_remoteaddr, demo_packet_, sizeof(demo_packet_));
#ifdef EXT_POWER_AMP
    //AX5043_PWRAMP=0;
#endif // EXT_POWER_AMP
}

static void display_transmit_packet(void)
{
    display_writestr("TX    ");
    display_writenum16(pkt_counter, 4, WRNUM_PADZERO);com0_tx(0x0d);
}

void axradio_statuschange(struct axradio_status __xdata *st)
{
    static uint16_t rssi_ratelimit;

    axst = st;

#if defined(USE_DBGLINK) && defined(DEBUGMSG)
    if (DBGLNKSTAT & 0x10 && st->status != AXRADIO_STAT_CHANNELSTATE) {
        dbglink_writestr("ST: 0x");
        dbglink_writehex16(st->status, 2, WRNUM_PADZERO);
        dbglink_writestr(" ERR: 0x");
        dbglink_writehex16(st->error, 2, WRNUM_PADZERO);
        dbglink_tx('\n');
    }
#endif
    switch (st->status) {

    case AXRADIO_STAT_RECEIVE:
        #ifdef EXT_TRX
        //rf_rx_set();
        rf_tx_clr();
        #else
        led0_on();
        #endif // EXT_TRX
        //button_busy_times=axradio_framing_ack_retransmissions;
        //deglitch_busy = button_busy = button_busy_times = 0;
#ifdef DEBUG_MSG
        display_writestr("AXRADIO_STAT_RECEIVE");com0_tx(0x0d);
#endif // DEBUG_MSG
#if RADIO_MODE == AXRADIO_MODE_SYNC_SLAVE || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_SLAVE
        switch (st->error) {
        case AXRADIO_ERR_TIMEOUT:
            //led2_on();
            // fall through

        case AXRADIO_ERR_NOERROR:
        case AXRADIO_ERR_RESYNCTIMEOUT:
            //led3_off();
            break;

        case AXRADIO_ERR_RESYNC:
            axradio_set_freqoffset(0);
            // fall through

        case AXRADIO_ERR_RECEIVESTART:
            led3_on();
            break;

        default:
            break;
        }

        if (st->error == AXRADIO_ERR_NOERROR) {
            ++pkts_received;
            //led2_off();
#ifdef USE_DISPLAY
            display_received_packet(st);
#endif // USE_DISPLAY
        }
#ifdef USE_DBGLINK
        switch (st->error) {
        case AXRADIO_ERR_RESYNCTIMEOUT:
            if (DBGLNKSTAT & 0x10)
                dbglink_writestr("RESYNC Timeout");com0_tx(0x0d);
            break;

        case AXRADIO_ERR_RESYNC:
            if (DBGLNKSTAT & 0x10)
                dbglink_writestr("RESYNC");com0_tx(0x0d);
            break;

        default:
            break;
        }
#endif // USE_DBGLINK
#else  // RADIO_MODE
        {
            uint8_t retran = (st->error != AXRADIO_ERR_NOERROR);
            ++pkts_received;
            if(moto_machine_drive(eep01)==1)
            {
                #ifdef EXT_TRX
                rf_rx_clr();
                rf_tx_set();
                #else
                led0_on();
                #endif // EXT_TRX
                set_timer2_onoff(SET);
            }
                //led0_toggle();

            memset(axradio_rxbuffer_ext,0,sizeof(axradio_rxbuffer_ext)); // clear axradio_rxbuffer_ext
#ifdef USE_DISPLAY
            if (st->error == AXRADIO_ERR_NOERROR)
                retran = display_received_packet(st) || retran;
#endif // USE_DISPLAY

            /*if (retran)
                led2_on();
            else
                led2_off();
            */
        }
#endif // RADIO_MODE
#ifdef USE_DBGLINK
        if (st->error == AXRADIO_ERR_NOERROR)
            dbglink_received_packet(st);
#endif
        // Frequency Offset Correction
        {
            int32_t foffs = axradio_get_freqoffset();
#if defined(ADJUST_FREQREG)
            foffs -= (st->u.rx.phy.offset)>>(FREQOFFS_K); //adjust RX frequency by low-pass filtered frequency offset
#endif
            // reset if deviation too large
            if (axradio_set_freqoffset(foffs) != AXRADIO_ERR_NOERROR)
                axradio_set_freqoffset(0);
        }
        break;
#if 0
    case AXRADIO_STAT_CHANNELSTATE:
        #ifdef DEBUG_MSG
        display_writestr("AXRADIO_STAT_CHANNELSTATE");com0_tx(0x0d);
        #endif // DEBUG_MSG

        if (st->u.cs.busy)
            led3_on();
        else
            led3_off();
        {
            uint16_t t = wtimer0_curtime();
#if WTIMER0_CLKSRC == CLKSRC_LPXOSC
#define RSSIRATE (uint16_t)(32768/2)
#elif WTIMER0_CLKSRC == CLKSRC_LPOSC
#define RSSIRATE (uint16_t)(640/2)
#else
#error "unknown wtimer0 clock source"
#endif
            if ((uint16_t)(t - rssi_ratelimit) < RSSIRATE)
                break;
            rssi_ratelimit = t;
        }
        display_setpos(0x48);
        display_writestr("R:");
        display_tx(st->u.cs.busy ? '*' : ' ');
        display_writenum16(st->u.cs.rssi, 5, WRNUM_SIGNED);
        break;
#endif

	case AXRADIO_STAT_TRANSMITSTART:
        //led3_on();
        #ifdef DEBUG_MSG
        display_writestr("AXRADIO_STAT_TRANSMITSTART");com0_tx(0x0d);
        #endif // DEBUG_MSG

        #ifdef EXT_TRX
        //rf_rx_clr();
        //rf_tx_set();
        #else
        //led0_on();
        #endif // EXT_TRX
        if (st->error == AXRADIO_ERR_RETRANSMISSION)
            {
                #ifdef DEBUG_MSG
                display_writestr("AXRADIO_ERR_RETRANSMISSION");com0_tx(0x0d);
                #endif // DEBUG_MSG
            }
#ifdef TX_ON_DEMAND
        if( st->error = AXRADIO_ERR_TIMEOUT )
            {
                #ifdef DEBUG_MSG
                display_writestr("AXRADIO_ERR_TIMEOUT");com0_tx(0x0d);
                #endif // DEBUG_MSG
            }
#endif

#if RADIO_MODE == AXRADIO_MODE_SYNC_MASTER || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_MASTER
        display_transmit_packet();
#endif
        break;

    case AXRADIO_STAT_TRANSMITEND:

        #ifdef EXT_TRX
        //rf_tx_clr();
        //rf_rx_clr();
        #else
        //led0_off();
        #endif // EXT_TRX

        #ifdef DEBUG_MSG
        display_writestr("AXRADIO_STAT_TRANSMITEND");com0_tx(0x0d);
        #endif // DEBUG_MSG

        if (st->error == AXRADIO_ERR_TIMEOUT) {
            #ifdef DEBUG_MSG
            display_writestr("AXRADIO_ERR_TIMEOUT");com0_tx(0x0d);
            #endif // DEBUG_MSG
        }

        if (st->error == AXRADIO_ERR_NOERROR) {
            #ifdef DEBUG_MSG
            display_writestr("AXRADIO_ERR_NOERROR");com0_tx(0x0d);
            #endif // DEBUG_MSG
            deglitch_busy = button_busy = button_busy_times = 0;
        }

        if (st->error == AXRADIO_ERR_BUSY) {
                deglitch_busy = button_busy = button_busy_times = 0;
                change_radio_mode_trx(CHANGE_TX2RX);
                axradio_set_mode(axradio_mode);
                #ifdef DEBUG_MSG
                display_writestr("AXRADIO_ERR_BUSY");com0_tx(0x0d);
                #endif // DEBUG_MSG
        }

        break;

#if RADIO_MODE == AXRADIO_MODE_SYNC_MASTER || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_MASTER
    case AXRADIO_STAT_TRANSMITDATA:
        // in SYNC_MASTER mode, transmit data may be prepared between the call to TRANSMITEND until the call to TRANSMITSTART
        // TRANSMITDATA is called when the crystal oscillator is enabled, approximately 1ms before transmission
        transmit_packet();
        #ifdef DEBUG_MSG
        display_writestr("AXRADIO_STAT_TRANSMITDATA");com0_tx(0x0d);
        #endif // DEBUG_MSG
        break;
#endif


    case AXRADIO_STAT_CHANNELSTATE:
        #ifdef DEBUG_MSG
        display_writestr("AXRADIO_STAT_CHANNELSTATE");com0_tx(0x0d);
        #endif // DEBUG_MSG

        //if (st->u.cs.busy)
        //    led3_on();
        //else
        //    led3_off();

        break;
    default:
        break;
    }
}



static void wakeup_callback(struct wtimer_desc __xdata *desc)
{
    desc;
#if defined(WTIMER0_PERIOD)
    wakeup_desc.time += WTIMER0_PERIOD;
    wtimer0_addabsolute(&wakeup_desc);
    transmit_packet();
    display_transmit_packet();
#endif
}



#if defined(__ICC8051__)
//
// If the code model is banked, low_level_init must be declared
// __near_func elsa a ?BRET is performed
//
#if (__CODE_MODEL__ == 2)
__near_func __root char
#else
__root char
#endif
__low_level_init(void) @ "CSTART"
#else
uint8_t _sdcc_external_startup(void)
#endif
{
    LPXOSCGM = 0x8A;
    wtimer0_setclksrc(WTIMER0_CLKSRC, WTIMER0_PRESCALER);
    wtimer1_setclksrc(CLKSRC_FRCOSC, 7);

    LPOSCCONFIG = 0x09; // Slow, PRESC /1, no cal. Does NOT enable LPOSC. LPOSC is enabled upon configuring WTCFGA (MODE_TX_PERIODIC and receive_ack() )

    // caution: coldstart has to be initialized with 1 in it's definition! Variables are initialized upon _sdcc_external_startup returning 0 -> the coldstart value returned from _sdcc_external startup does not survive in the coldstart case
    coldstart = !(PCON & 0x40);

    ANALOGA = 0x18; // PA[3,4] LPXOSC, other PA are used as digital pins
//    PORTA = 0xC0; // pull-up for PA[6,7] which are not bonded, no pull up for PA[3,4] (LPXOSC) and PA[0,1,2,5] (DIP switches to gnd, default on)
#ifndef MINI_KIT
    /*
    PORTA = 0xC0 | (PINA & 0x25); // pull-up for PA[6,7] which are not bonded, no pull up for PA[3,4] (LPXOSC). Output 0 in PA[0,1,2,5] to prevent current consumption in all DIP switch states
    // init LEDs to previous (frozen) state
    PORTB = 0xFE; //PB[0,1]  (LCD RS, LCD RST) are overwritten by lcd2_portinit(), enable pull-ups for PB[2..7]  (PB[2,3] for buttons, PB[4..7] unused)
    PORTC = 0xF3 | (PINC & 0x08); // set PC0 = 1 (LCD SEL), PC1 = 1 (LCD SCK), PC2 = 0 (LCD MOSI), PC3 =0 (LED), enable pull-ups for PC[4..7] which are not bonded Mind: PORTC[0:1] is set to 0x3 by lcd2_portinit()
    // init LEDs to previous (frozen) state
    PORTR = 0xCB; // overwritten by ax5043_reset, ax5043_comminit()
    DIRA = 0x27; // output 0 on PA[0,1,2,5] to prevent current consumption in all DIP switch states. Other PA are inputs, PA[3,4] (LPXOSC) must have disabled digital output drivers
    DIRB = 0x03; // PB[0,1] are outputs (LCD RS, LCD RST), PB[2..7] are inputs (PB[2,3] for buttons,  PB[4..7]  unused)
    DIRC = 0x0F; // PC[0..3] are outputs (LCD SEL, LCD,SCK, LCD MOSI, LED), PC[4..7] are inputs (not bonded).
    DIRR = 0x15; // overwritten by ax5043_reset, ax5043_comminit()
    */


    PORTA = 0xDB; //
    PORTB = 0xFD | (PINB & 0x02); //init RX_EN to previous (frozen) state
    PORTC = 0xF0; //0xFF; //
    PORTR = 0x0B; //

    DIRA = 0x03; //0x00; //PA0 = SCK;   PA1 = SDA;    PA2 = BADET;      PA5 = AD1;
    DIRB = 0x0E; //0x0e; //PB0 = SW;    PB1 = RX_EN;  PB2 = PWRAMP;     PB3 = ANSTSEL;
    DIRC = 0x0F; //0x00; //PC0 = PTT;   PC2 = J7;     PC2 = J8;         PC3 = PTT_SW; PC4 = PTT;
    DIRR = 0x15; // overwritten by ax5043_reset, ax5043_comminit()


#else

    PORTA = 0xFF; //
    PORTB = 0xFD | (PINB & 0x02); //init LEDs to previous (frozen) state
    PORTC = 0xFF; //
    PORTR = 0x0B; //

    DIRA = 0x00; //
    DIRB = 0x0E; //  PB1 = LED; PB2 / PB3 are outputs (in case PWRAMP / ANSTSEL are used)
    DIRC = 0x00; //  PC4 = Switch
    DIRR = 0x15; //

#endif // else MINI_KIT

    axradio_setup_pincfg1();
    DPS = 0;
    IE = 0x40;
    EIE = 0x40 | 0x04; //uart0 int
    E2IE = 0x00;
    display_portinit();
    GPIOENABLE = 1; // unfreeze GPIO

    //ADCCLKSRC = 0x26;
    //ADCCH0CONFIG = 0xD9;
    //ADCCONV = ADCCONV | 7;

#if defined(__ICC8051__)
    return coldstart;
#else
    return !coldstart; // coldstart -> return 0 -> var initialization; start from sleep -> return 1 -> no var initialization
#endif
}

static void change_radio_mode_trx(uint8_t trx)
{
    //trx
    //change_rx2tx:0
    //change_tx2rx:1
    //none:2
    //uint8_t i;

    switch(axradio_mode){

        #if RADIO_MODE==AXRADIO_MODE_WOR_RECEIVE || RADIO_MODE==AXRADIO_MODE_WOR_TRANSMIT
        if((trx==CHANGE_RX2TX)||(trx==CHANGE_NONE)){
            case AXRADIO_MODE_WOR_RECEIVE:
            case AXRADIO_MODE_UNINIT:
                    axradio_mode = AXRADIO_MODE_WOR_TRANSMIT;
                    axradio_set_addr_Switch_Transmit(eep01);
                    break;
        }else if((trx==CHANGE_TX2RX)||(trx==CHANGE_NONE)){
            case AXRADIO_MODE_WOR_TRANSMIT:
            case AXRADIO_MODE_OFF:
                    axradio_mode = AXRADIO_MODE_WOR_RECEIVE;
                    axradio_set_addr_Switch_Receive(eep01);
                    break;
        }
        #endif // RADIO_MODE

        #if RADIO_MODE==AXRADIO_MODE_WOR_ACK_RECEIVE || RADIO_MODE==AXRADIO_MODE_WOR_ACK_TRANSMIT
        if((trx==CHANGE_RX2TX)||(trx==CHANGE_NONE)){
            case AXRADIO_MODE_WOR_ACK_RECEIVE:
            case AXRADIO_MODE_UNINIT:
                    axradio_mode = AXRADIO_MODE_WOR_ACK_TRANSMIT;
                    axradio_set_addr_Switch_Transmit(eep01);
                    break;
        }else if((trx==CHANGE_TX2RX)||(trx==CHANGE_NONE)){
            case AXRADIO_MODE_WOR_ACK_TRANSMIT:
            case AXRADIO_MODE_OFF:
                    axradio_mode = AXRADIO_MODE_WOR_ACK_RECEIVE;
                    axradio_set_addr_Switch_Receive(eep01);
                    break;
        }
        #endif // RADIO_MODE

        #if RADIO_MODE==AXRADIO_MODE_ASYNC_RECEIVE || RADIO_MODE==AXRADIO_MODE_ASYNC_TRANSMIT
        if((trx==CHANGE_RX2TX)||(trx==CHANGE_NONE)){
            case AXRADIO_MODE_ASYNC_RECEIVE:
            case AXRADIO_MODE_UNINIT:
                axradio_mode = AXRADIO_MODE_ASYNC_TRANSMIT;
                axradio_set_addr_Switch_Transmit(eep01);
                break;
        }else if((trx==CHANGE_TX2RX)||(trx==CHANGE_NONE)){
            case AXRADIO_MODE_ASYNC_TRANSMIT:
            case AXRADIO_MODE_OFF:
                axradio_mode = AXRADIO_MODE_ASYNC_RECEIVE;
                axradio_set_addr_Switch_Receive(eep01);
                break;
        }
        #endif // RADIO_MODE
    }

}

void motor_drive_stop(void)
{
    static uint8_t __data adc_stop=0;
    //MOTOR Drive
    if(get_moto_drive_flag()!=MOTO_STOP){
        adc_stop = adc_work(motor_ch);
        //display_writestr("adc_stop=");display_writenum16(adc_stop,4,1);com0_tx(0x0d);
        if(adc_stop>140 && (get_moto_drive_flag()==MOTO_RUNNING_R)) // about 1.5V stop motor lock
        {
            stop_moto_drive();
            //continue;
        }

        if(adc_stop>135 && (get_moto_drive_flag()==MOTO_RUNNING_L)) // about 1.46V stop motor unlock
        {
            stop_moto_drive();
            //continue;
        }

        if(adc_stop>110 /*(get_moto_drive_flag()==MOTO_R || get_moto_drive_flag()==MOTO_L)*/) // about 1.21V first step
        {
            if(get_moto_drive_flag()==MOTO_R)
                set_moto_drive_flag(MOTO_RUNNING_R);
            if((get_moto_drive_flag()==MOTO_L))
                set_moto_drive_flag(MOTO_RUNNING_L);
            //continue;
        }
    }
}

void led_flash_dsiplay(void)
{
    uint8_t i;

    for(i=0;i<3;i++)
    {
        rf_rx_set();
        rf_tx_set();
        delay_ms(200);
        rf_rx_clr();
        rf_tx_clr();
        delay_ms(200);
    }
}


void main(void)
{
    //static uint8_t __data saved_button_state = 0x10;
    static uint8_t __data saved_button_state = 1;
    static uint8_t __data buttonedge,button_set;
    static uint8_t __data adc_vddio=0;
    uint8_t i;
    struct rx_save rx_sv;
    uint8_t addrtvl;
    int8_t freqoffsetplus;

#if !defined(SDCC) && !defined(__ICC8051__)
    _sdcc_external_startup();
#endif

#if defined(SDCC)
    __asm
    G$_start__stack$0$0 = __start__stack
    .globl G$_start__stack$0$0
    __endasm;
#endif

#ifdef USE_DBGLINK
    dbglink_init();
#endif

    EA = 1;
    flash_apply_calibration();
    CLKCON = 0x00;
    wtimer_init();
    adc_init();
    timer2_init();

    IE_6=1;
    #ifdef EXT_TRX
    rf_tx_set();
    rf_rx_set();
    #endif // EXT_TRX

    //watch_init //WDG TIMER DIV /256 about 400ms
    WDTCFG=0x08;

    #ifdef WDT_EN
    //watch_start
    //if(read_subp(WDT_EN_ADDR)==1)
    //    WDTCFG|=0x10;
    #endif // WDT_EN

    if (coldstart) {
        #ifndef EXT_TRX
        led0_off();
        #endif // EXT_TRX

        wakeup_desc.handler = wakeup_callback;
#ifdef TX_ON_DEMAND
        BUTTON_INTCHG |= BUTTON_MASK; //interrupt on button changed (button SW5 on DVK-2) for wake on button pressed
#endif // TX_ON_DEMAND

        display_init();
        #ifndef PROGRAM_MODE
        display_setpos(0);
        #endif // PROGRAM_MODE
        WDTRESET=0xAE;

        i = axradio_init();
        if (i != AXRADIO_ERR_NOERROR) {
            if (i == AXRADIO_ERR_NOCHIP) {
                display_writestr("No AX5043 RF\nchip found");
#ifdef USE_DBGLINK
                if (DBGLNKSTAT & 0x10)
                    dbglink_writestr("No AX5043 RF chip found");com0_tx(0x0d);
#endif // USE_DBGLINK
                goto terminate_error;
            }
            goto terminate_radio_error;
        }
        display_writestr("found AX5043");com0_tx(0x0d);
#ifdef USE_DBGLINK
        if (DBGLNKSTAT & 0x10)
            {dbglink_writestr("found AX5043");com0_tx(0x0d);}
#endif // USE_DBGLINK
        //axradio_set_local_address(&localaddr);
        //axradio_set_default_remote_address(&remoteaddr);

        ep_pager(0x0010,eep01,sizeof(eep01));
        axradio_set_addr_Switch_Receive(eep01);

        axradio_set_local_address(&axradio_localaddr);
        axradio_set_default_remote_address(&axradio_default_remoteaddr);

        WDTRESET=0xAE;
//RX Only
#if RADIO_MODE == AXRADIO_MODE_WOR_RECEIVE || RADIO_MODE == AXRADIO_MODE_WOR_ACK_RECEIVE
        // LPOSC Calibrations needs full control of the radio, so it must run while the radio is idle otherwise
        calibrate_lposc();
#endif
#if RADIO_MODE == MODE_SYNC_MASTER || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_MASTER || RADIO_MODE == AXRADIO_MODE_SYNC_SLAVE || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_SLAVE
        display_writestr("settle LPXOSC");
#ifdef USE_DBGLINK
        if (DBGLNKSTAT & 0x10)
            {dbglink_writestr("settle LPXOSC");com0_tx(0x0d);}
#endif // USE_DBGLINK
        delay_ms(lpxosc_settlingtime);
        display_clear(0x40, 16);
        display_setpos(0x40);
        display_writestr("MASTER & SLAVE");

#endif  // RADIO_MODE

#ifdef USE_DISPLAY
        display_writestr("RNG=");
        display_writenum16(axradio_get_pllrange(), 2, 0);
        {
            uint8_t x = axradio_get_pllvcoi();
            if (x & 0x80) {
                display_writestr(" VCOI=");
                display_writehex16(x, 2, 0);
            }
        }
        WDTRESET=0xAE;
        delay_ms(1000); // just to show PLL RNG

        display_clear(0, 16);
        display_clear(0x40, 16);
        display_setpos(0);
#endif // USE_DISPLAY

#ifdef USE_DBGLINK
        if (DBGLNKSTAT & 0x10) {
            dbglink_writestr("RNG = ");
            dbglink_writenum16(axradio_get_pllrange(), 2, 0);
            {
                uint8_t x = axradio_get_pllvcoi();
                if (x & 0x80) {
                    dbglink_writestr("\nVCOI = ");
                    dbglink_writehex16(x, 2, 0);
                }
            }
            dbglink_writestr("\nMASTER & SLAVE");com0_tx(0x0d);
        }
#endif // USE_DBGLINK

#if RADIO_MODE == AXRADIO_MODE_WOR_RECEIVE || RADIO_MODE == AXRADIO_MODE_WOR_ACK_RECEIVE
        display_writestr("SLAVE RX WOR");com0_tx(0x0d);
#ifdef USE_DBGLINK
        if (DBGLNKSTAT & 0x10)
            {dbglink_writestr("SLAVE RX WOR");com0_tx(0x0d);}
#endif // USE_DBGLINK

#elif RADIO_MODE == AXRADIO_MODE_SYNC_SLAVE || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_SLAVE
        {display_writestr("SLAVE  RX SYNC");com0_tx(0x0d);}
#ifdef USE_DBGLINK
            if (DBGLNKSTAT & 0x10)
        {dbglink_writestr("SLAVE  RX SYNC");com0_tx(0x0d);}
#endif // USE_DBGLINK

#else  // RADIO_MODE
        display_writestr("SLAVE RX CONT");com0_tx(0x0d);
#ifdef USE_DBGLINK
        if (DBGLNKSTAT & 0x10)
            {dbglink_writestr("SLAVE RX CONT");com0_tx(0x0d);}
#endif // USE_DBGLINK

#endif // else RADIO_MODE

        i = axradio_set_mode(RADIO_MODE);
        if (i != AXRADIO_ERR_NOERROR)
            goto terminate_radio_error;

#ifdef DBGPKT
        AX5043_IRQMASK0 = 0x41;
        AX5043_RADIOEVENTMASK0 =0x0C; // radio state changed | RXPS changed
#endif

//TX Only
#if defined(WTIMER0_PERIOD)
        wakeup_desc.time = WTIMER0_PERIOD;
        wtimer0_addrelative(&wakeup_desc);
#endif

    } else {
        ax5043_commsleepexit();
        IE_4 = 1; // enable radio interrupt
    }
    axradio_setup_pincfg2();

    IP|= (1<<4)|(1<<6); //power manage & radio int priority hi

    //watch_init //WDG TIMER DIV /256 about 400ms
    WDTCFG=0x20;
    WDTCFG=0x09;
    #ifdef WDT_EN
    //watch_start
    if(read_subp(WDT_EN_ADDR)==1)
        WDTCFG|=0x10;
    #endif // WDT_EN

    //reset
    button_busy=0;
    lock_wheel=0xFF;
    set_moto_drive_flag(MOTO_STOP);
    moto_en_clr();

    #ifdef EXT_TRX
    rx_sv.fg=rx_sv.ct=0;
    rf_tx_clr();
    rf_rx_clr();
    rf_rx_set();
    #endif // EXT_TRX

    #ifdef TX_ON_DEMAND
    BUTTON_INTCHG |= BUTTON_MASK;
    #endif // TX_ON_DEMAND

/*
    //#ifndef MINI_KIT
    BUTTON_SET_INTCHG |=BUTTON_SET_MASK;
    button_set = BUTTON_SET_PIN_N;
    if(!button_set)
        goto PROGRAMMING_MODE;
    //#endif // MINI_KIT
 */

    set_timer2_onoff(PM_SET);
    wri_subp(0x000F,VERSION);

    for(;;){
        uint8_t checksum,addrtvl,i;
        uint8_t arrtemp[EEPROM_LEN];
        uint16_t addrtemp=0x0000;

        WDTRESET=0xAE;
        if(program_st==2)
            goto MAIN_LOOP;

        if((uart0_rxcount()>0))
        {
            uart0_buff[0]=uart0_rx();

            for(i=1;i<uart0_buff[0]+1;i++){
                uart0_buff[i] = uart0_rx();
            }
            checksum=0;
            for(i=0;i<uart0_buff[0];i++)
                checksum+=uart0_buff[i];

            if((uart0_buff[1]==0x41)&&uart0_buff[(uart0_buff[0])]==checksum)
            {
                switch(uart0_buff[2])
                {
                    case 0xF0: //go to programming mode
                        if((uart0_buff[3]==0xFF)&&(uart0_buff[3]==0xFF)&&(uart0_buff[3]==0xFF))
                           goto PROGRAMMING_MODE;
                        break;
                }
            }
            memset(uart0_buff,0,sizeof(uart0_buff));
        }
    }

MAIN_LOOP:

    program_st=0;
    display_writestr("MAIN_LOOP MODE");com0_tx(0x0d);
    set_timer2_onoff(VD_SET);

    addrtvl=read_subp(0x0030);
    delay_ms(5);
    freqoffsetplus=read_subp(0x0031);
    if(freqoffsetplus==1)
        freqoffsetplus = -1 *addrtvl;
    else if(freqoffsetplus==0)
        freqoffsetplus=addrtvl;
    axradio_set_freqoffset(freqoffsetplus);

    for(;;) {
    	// warmstart
        wtimer_runcallbacks();
        //watch_reset
        WDTRESET=0xAE;
        EA = 0;

        motor_drive_stop();

//TX Only
#ifdef TX_ON_DEMAND
        {
            buttonedge = BUTTON_PIN_N;

            if ((saved_button_state != buttonedge)&& (!button_busy)&&(!deglitch_busy))
            {
                if(adc_vddio<115){
                    led_flash_dsiplay();
                }
                else
                {
                    button_busy = 1;
                    saved_button_state = buttonedge;

                    #ifdef EXT_TRX
                    rf_rx_clr();
                    delay_ms(10);
                    rf_tx_set();
                    #endif // EXT_TRX

                    #ifdef EXT_POWER_AMP
                        AX5043_PWRAMP=1;
                    #endif // EXT_POWER_AMP

                    lock_wheel=saved_button_state;
                    set_moto_drive_flag(MOTO_STOP);
                    moto_en_clr();

    #ifdef BUTTON_EDGE_ONE_SHT
                    if(buttonedge == BUTTON_MASK_EDGE){ // button press tx ,release rx
    #endif // BUTTON_EDGE_ONE_SHT
                        EA = 1;
                        if( !deglitch_busy )
                        {
                            deglitch_busy = 1;

                            change_radio_mode_trx(CHANGE_RX2TX);
                            i = axradio_set_mode(axradio_mode);
                            if (i != AXRADIO_ERR_NOERROR)
                                goto terminate_radio_error;

                            #if RADIO_MODE == AXRADIO_MODE_WOR_RECEIVE || RADIO_MODE == AXRADIO_MODE_WOR_ACK_RECEIVE
                                // LPOSC Calibrations needs full control of the radio, so it must run while the radio is idle otherwise
                                //calibrate_lposc();
                            #endif

                                delay_ms(300);
                                transmit_packet();
                                display_transmit_packet();
                        }
    #ifdef BUTTON_EDGE_ONE_SHT
                    }
    #endif // BUTTON_EDGE_ONE_SHT
                    rx_sv.ct=rx_sv.fg=0;
                    continue;

                }
            }
        }
        if((!button_busy)&&(!deglitch_busy))
        {
                IE_3 = 1;
                #ifdef EXT_TRX
                rf_tx_clr();
                delay_ms(10);

                switch(rx_sv.ct)
                {
                    case 0:
                        rf_rx_set();
                        break;
                    case rx_save_rate:
                        rf_rx_clr();
                        break;
                }
                if(rx_sv.ct<rx_save_full)
                    rx_sv.ct++;
                else
                    rx_sv.ct=0;

                //PORTC^=0x02;
                #endif // EXT_TRX
                #ifdef EXT_POWER_AMP
                    AX5043_PWRAMP=0;
                #endif // EXT_POWER_AMP

            if(vddio_st==2)
            {
                vddio_st=1;
                adc_vddio=adc_work(battery_ch);
                //display_writestr("adc_vddio=");display_writenum16(adc_vddio,4,1);com0_tx(0x0d);
            }
        }

#endif  // TX_ON_DEMAND
        {
            uint8_t flg = WTFLAG_CANSTANDBY;
#ifdef MCU_SLEEP
            if (axradio_cansleep()
#ifdef USE_DBGLINK
                && dbglink_txidle()
#endif // USE_DBGLINK
                && display_txidle())
                if(get_moto_drive_flag()==MOTO_STOP)
                    flg |= WTFLAG_CANSLEEP;
            //GPIOENABLE=1;
#endif // MCU_SLEEP

            wtimer_idle(flg);
        }
        //TX Only
        IE_3 = 0; // no ISR!
        EA = 1;
    }
    terminate_radio_error:
    display_radio_error(i);
#ifdef USE_DBGLINK
    dbglink_display_radio_error(i);
#endif // USE_DBGLINK
    terminate_error:

    for (;;) {
        wtimer_runcallbacks();
        {
            uint8_t flg = WTFLAG_CANSTANDBY;

#ifdef MCU_SLEEP && moto_drive_flag == MOTO_STOP
            if (axradio_cansleep()
#ifdef USE_DBGLINK
                && dbglink_txidle()
#endif // USE_DBGLINK
                && display_txidle())
                    flg |= WTFLAG_CANSLEEP;
#endif // MCU_SLEEP
            wtimer_idle(flg);
            reset_cpu();
        }
    }
    PROGRAMMING_MODE:
    display_writestr("PROGRAM MODE");com0_tx(0x0d);
    program_st=0;
    led_flash_dsiplay();
    timer2_init();

    for(;;){
        uint8_t checksum,addrtvl;
        uint8_t arrtemp[EEPROM_LEN];
        uint16_t addrtemp=0x0000;
        int8_t freqoffsetplus;

        WDTRESET=0xAE;

        if((uart0_rxcount()>0))
        {
            uart0_buff[0]=uart0_rx();

            for(i=1;i<uart0_buff[0]+1;i++){
                uart0_buff[i] = uart0_rx();
            }
            checksum=0;
            for(i=0;i<uart0_buff[0];i++)
                checksum+=uart0_buff[i];

            if((uart0_buff[1]==0x41)&&uart0_buff[(uart0_buff[0])]==checksum)
            {
                switch(uart0_buff[2])
                {
                    case 0: //reboot mcu
                        if((uart0_buff[3]==0xAA)&&(uart0_buff[4]==0x55)&&(uart0_buff[5]==0xFF))
                        {
                            display_writestr("MCU REBOOT OK");com0_tx(0x0d);
                            delay_ms(1000);
                            reset_cpu();
                        }
                        break;

                    case 0x20: //read page address
                        addrtemp=uart0_buff[3];
                        addrtemp<<=8;
                        addrtemp|=uart0_buff[4];
                        ep_pager(addrtemp,arrtemp,sizeof(arrtemp));
                        display_writestr("EEPROM READ ADDRESS OK");com0_tx(0x0d);
                        display_writestr("ADDRESS : ");
                        display_writehex16(addrtemp, 4, WRNUM_PADZERO);display_writestr(" ~ ");
                        display_writehex16(addrtemp+EEPROM_LEN, 4, WRNUM_PADZERO);com0_tx(0x0d);
                        display_writestr("VALUE : ");
                        for(i=0;i<sizeof(arrtemp);i++)
                            display_writehex16(arrtemp[i], 2, WRNUM_PADZERO);
                        com0_tx(0x0d);
                        break;

                     case 0x30: //read one address
                        addrtemp=uart0_buff[3];
                        addrtemp<<=8;
                        addrtemp|=uart0_buff[4];
                        addrtvl=read_subp(addrtemp);
                        display_writestr("EEPROM READ VALUE OK");com0_tx(0x0d);
                        display_writestr("ADDRESS : ");
                        display_writehex16(addrtemp, 4, WRNUM_PADZERO);com0_tx(0x0d);
                        display_writestr("VALUE : ");
                        display_writenum16(addrtvl, 3, WRNUM_SIGNED);com0_tx(0x0d);
                        break;

                    case 0x40: //write one address value
                        addrtemp=uart0_buff[3];
                        addrtemp<<=8;
                        addrtemp|=uart0_buff[4];
                        wri_subp(addrtemp,uart0_buff[5]);
                        delay_ms(5);
                        addrtvl=read_subp(addrtemp);
                        display_writestr("EEPROM WRITE VALUE OK");com0_tx(0x0d);
                        display_writestr("ADDRESS : ");
                        display_writehex16(addrtemp, 4, WRNUM_PADZERO);com0_tx(0x0d);
                        display_writestr("VALUE : ");
                        display_writenum16(addrtvl, 3, WRNUM_SIGNED);com0_tx(0x0d);
                        if((AX5043_PWRAMP)&&(addrtemp==EE_FQSET))
                        {
                            addrtvl=read_subp(EE_FQSET);
                            delay_ms(5);
                            freqoffsetplus=read_subp(EE_FQSETPM);

                            if(freqoffsetplus==1)
                                freqoffsetplus = -1 *addrtvl;
                            else if(freqoffsetplus==0)
                                freqoffsetplus=addrtvl;

                            axradio_set_freqoffset(freqoffsetplus);
                        }
                        if((addrtemp==EE_APC)&&(AX5043_PWRAMP))
                        {
                            addrtvl=read_subp(EE_APC);

                        }
                        break;
                    case 0x50: //write address
                        addrtemp=uart0_buff[3];
                        addrtemp<<=8;
                        addrtemp|=uart0_buff[4];
                        rotate_array(uart0_buff,uart0_buff[0],5);
                        ep_pagew(addrtemp,uart0_buff,EEPROM_LEN);
                        delay_ms(5);
                        ep_pager(addrtemp,arrtemp,sizeof(arrtemp));
                        display_writestr("EEPROM WRITE ADDRESS OK");com0_tx(0x0d);
                        display_writestr("ADDRESS : ");
                        display_writehex16(addrtemp, 4, WRNUM_PADZERO);display_writestr(" ~ ");
                        display_writehex16(addrtemp+EEPROM_LEN, 4, WRNUM_PADZERO);com0_tx(0x0d);
                        display_writestr("VALUE : ");
                        for(i=0;i<sizeof(arrtemp);i++)
                            display_writehex16(arrtemp[i], 2, WRNUM_PADZERO);
                        com0_tx(0x0d);
                        break;

                    case 0x60: //sw tx
                        if((uart0_buff[3]==0xFF)&&(uart0_buff[3]==0xFF)&&(uart0_buff[3]==0xFF))
                        {
                            addrtvl=read_subp(EE_FQSET);
                            delay_ms(5);
                            freqoffsetplus=read_subp(EE_FQSETPM);

                            if(freqoffsetplus==1)
                                freqoffsetplus = -1 *addrtvl;
                            else if(freqoffsetplus==0)
                                freqoffsetplus=addrtvl;

                            axradio_set_freqoffset(freqoffsetplus);
                            display_writestr("SW TX OK");com0_tx(0x0d);
                            #ifdef EXT_POWER_AMP
                                AX5043_DACCONFIG=0;
                                AX5043_DACVALUE0=0x55;
                                AX5043_PWRAMP=1;
                            #endif // EXT_POWER_AMP
                            rf_rx_clr();rf_tx_set();set_fm_tx();
                        }
                        break;
                    case 0x70: //sw rx
                        if((uart0_buff[3]==0xFF)&&(uart0_buff[3]==0xFF)&&(uart0_buff[3]==0xFF))
                        {
                            display_writestr("SW RX OK");com0_tx(0x0d);
                            #ifdef EXT_POWER_AMP
                                AX5043_PWRAMP=0;
                            #endif // EXT_POWER_AMP
                            rf_tx_clr();rf_rx_set();set_fm_rx();
                            delay_ms(30);
                            if(AX5043_RSSI>190)
                                trssi=255+64-AX5043_RSSI;
                            else
                                trssi=63-AX5043_RSSI;
                            display_writestr("AX5043_RSSI=");
                            display_writenum16(trssi, 3, WRNUM_TSDSEP);com0_tx(0x0d);
                        }
                        break;

                    case 0x80: //sw rf done
                        if((uart0_buff[3]==0xFF)&&(uart0_buff[3]==0xFF)&&(uart0_buff[3]==0xFF))
                        {
                            display_writestr("SW RF DONE OK");com0_tx(0x0d);
                            #ifdef EXT_POWER_AMP
                                AX5043_PWRAMP=0;
                            #endif // EXT_POWER_AMP
                            rf_tx_clr();rf_rx_clr();set_fm_done();
                        }
                        break;

                    case 0xF1: //test motor clockwise with timer2
                        if((uart0_buff[3]==0xFF)&&(uart0_buff[3]==0xFF)&&(uart0_buff[3]==0xFF))
                        {
                            display_writestr("MOTOR CLOCKWISE OK");com0_tx(0x0d);
                            moto_en_set();
                            set_timer2_onoff(SET);
                        }
                        break;
                    case 0xF2: //test motor anti-clockwise timer2
                        if((uart0_buff[3]==0xFF)&&(uart0_buff[3]==0xFF)&&(uart0_buff[3]==0xFF))
                        {
                            display_writestr("MOTOR ANTI-CLOCKWISE OK");com0_tx(0x0d);
                            moto_en_set();
                            set_timer2_onoff(CLR);
                        }
                        break;
                    case 0xF3: //test motor off timer2
                        if((uart0_buff[3]==0xFF)&&(uart0_buff[3]==0xFF)&&(uart0_buff[3]==0xFF))
                        {
                            display_writestr("MOTOR STOP OK");com0_tx(0x0d);
                            moto_en_clr();
                            set_timer2_onoff(CLR);
                        }
                        break;
                }
            }

            memset(uart0_buff,0,sizeof(uart0_buff));

        }
    }

}



