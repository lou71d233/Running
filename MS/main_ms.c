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

#include <ax8052f143.h>
#include <libmftypes.h>
#include <libmfradio.h>
#include <libmfflash.h>
#include <libmfwtimer.h>
#include<math.h>


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
#define BUTTON_MASK   0x10

#else
#include <libdvk2leds.h>

#define BUTTON_INTCHG INTCHGB
#define BUTTON_PIN    PINB
#define BUTTON_MASK   0x04

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


#ifdef TX_ON_DEMAND
#if defined __ICC8051__
bool deglitch_busy = 0;
#else
__bit deglitch_busy = 0;
#endif
#endif

struct wtimer_desc __xdata wakeup_desc;


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
    memcpy(demo_packet_, demo_packet, sizeof(demo_packet));
#endif
    if (framing_insert_counter) {
        demo_packet_[framing_counter_pos] = pkt_counter & 0xFF ;
        demo_packet_[framing_counter_pos+1] = (pkt_counter>>8) & 0xFF;
    }
    axradio_transmit(&remoteaddr, demo_packet_, sizeof(demo_packet));
}

static void display_transmit_packet(void)
{
    if (pkt_counter == 1) {
        display_setpos(0x40);
        display_writestr("TX    ");
#ifdef USE_DBGLINK
        if (DBGLNKSTAT & 0x10)
            dbglink_writestr("TX : \n");
#endif // USE_DBGLINK
    }
    display_setpos(0x4c);
    display_writehex16(pkt_counter, 4, WRNUM_PADZERO);
#ifdef USE_DBGLINK
    if (DBGLNKSTAT & 0x10) {
        dbglink_writehex16(pkt_counter, 4, WRNUM_PADZERO);
        dbglink_tx('\n');
    }
#endif // USE_DBGLINK

}


void axradio_statuschange(struct axradio_status __xdata *st)
{
    static uint16_t rssi_ratelimit;
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
#if RADIO_MODE == AXRADIO_MODE_SYNC_SLAVE || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_SLAVE
        switch (st->error) {
        case AXRADIO_ERR_TIMEOUT:
            led2_on();
            // fall through

        case AXRADIO_ERR_NOERROR:
        case AXRADIO_ERR_RESYNCTIMEOUT:
            led3_off();
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
            led2_off();
#ifdef USE_DISPLAY
            display_received_packet(st);
#endif // USE_DISPLAY
        }
#ifdef USE_DBGLINK
        switch (st->error) {
        case AXRADIO_ERR_RESYNCTIMEOUT:
            if (DBGLNKSTAT & 0x10)
                dbglink_writestr("RESYNC Timeout\n");
            break;

        case AXRADIO_ERR_RESYNC:
            if (DBGLNKSTAT & 0x10)
                dbglink_writestr("RESYNC\n");
            break;

        default:
            break;
        }
#endif // USE_DBGLINK
#else  // RADIO_MODE
        {
            uint8_t retran = (st->error != AXRADIO_ERR_NOERROR);
            ++pkts_received;
            led0_toggle();
#ifdef USE_DISPLAY
            if (st->error == AXRADIO_ERR_NOERROR)
                retran = display_received_packet(st) || retran;
#endif // USE_DISPLAY

            if (retran)
                led2_on();
            else
                led2_off();
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
        led3_on();
        
        led0_on();
        if (st->error == AXRADIO_ERR_RETRANSMISSION)
            led2_on();
#ifdef TX_ON_DEMAND
            if( st->error = AXRADIO_ERR_TIMEOUT )
                deglitch_busy = 0;
#endif
#if RADIO_MODE == AXRADIO_MODE_SYNC_MASTER || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_MASTER
        display_transmit_packet();
#endif

        break;

    case AXRADIO_STAT_TRANSMITEND:
        led0_off();
        if (st->error == AXRADIO_ERR_NOERROR) {
            led2_off();
#ifdef TX_ON_DEMAND
            deglitch_busy = 0;
#endif
#if RADIO_MODE == AXRADIO_MODE_ACK_TRANSMIT || RADIO_MODE == AXRADIO_MODE_WOR_ACK_TRANSMIT || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_MASTER
            display_setpos(0x0d);
            display_writestr(":-)");
#ifdef USE_DBGLINK
            if (DBGLNKSTAT & 0x10)
                dbglink_writestr(":-)\n");
#endif // USE_DBGLINK
#endif // RADIO_MODE
        } else if (st->error == AXRADIO_ERR_TIMEOUT) {
            led2_on();
#if RADIO_MODE == AXRADIO_MODE_ACK_TRANSMIT || RADIO_MODE == AXRADIO_MODE_WOR_ACK_TRANSMIT || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_MASTER
            display_setpos(0x0d);
            display_writestr(":-(");
#ifdef USE_DBGLINK
            if (DBGLNKSTAT & 0x10)
                dbglink_writestr(":-(\n");
#endif // USE_DBGLINK
#endif // RADIO_MODE
        }
        if (st->error == AXRADIO_ERR_BUSY)
            led3_on();
        else
            led3_off();
        break;

#if RADIO_MODE == AXRADIO_MODE_SYNC_MASTER || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_MASTER
    case AXRADIO_STAT_TRANSMITDATA:
        // in SYNC_MASTER mode, transmit data may be prepared between the call to TRANSMITEND until the call to TRANSMITSTART
        // TRANSMITDATA is called when the crystal oscillator is enabled, approximately 1ms before transmission
        transmit_packet();
        break;
#endif

    case AXRADIO_STAT_CHANNELSTATE:
        if (st->u.cs.busy)
            led3_on();
        else
            led3_off();
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
#else
    PORTA = 0xFF; //
    PORTB = 0xFD | (PINB & 0x02); //init LEDs to previous (frozen) state
    PORTC = 0xFF; //
    PORTR = 0x0B; //

    DIRA = 0x00; //
    DIRB = 0x0e; //  PB1 = LED; PB2 / PB3 are outputs (in case PWRAMP / ANSTSEL are used)
    DIRC = 0x00; //  PC4 = Switch
    DIRR = 0x15; //

#endif // else MINI_KIT

    axradio_setup_pincfg1();
    DPS = 0;
    IE = 0x40;
    EIE = 0x00;
    E2IE = 0x00;
    display_portinit();
    GPIOENABLE = 1; // unfreeze GPIO
#if defined(__ICC8051__)
    return coldstart;
#else
    return !coldstart; // coldstart -> return 0 -> var initialization; start from sleep -> return 1 -> no var initialization
#endif
}


void main(void)
{
		static uint8_t __data saved_button_state = 0xFF;
    uint8_t i;

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

    if (coldstart) {
        led0_off();
        led1_off();
        led2_off();
        led3_off();
        
        wakeup_desc.handler = wakeup_callback;
#ifdef TX_ON_DEMAND
        BUTTON_INTCHG |= BUTTON_MASK; //interrupt on button changed (button SW5 on DVK-2) for wake on button pressed
#endif // TX_ON_DEMAND

        display_init();
        display_setpos(0);
        i = axradio_init();
        if (i != AXRADIO_ERR_NOERROR) {
            if (i == AXRADIO_ERR_NOCHIP) {
                display_writestr("No AX5043 RF\nchip found");
#ifdef USE_DBGLINK
                if (DBGLNKSTAT & 0x10)
                    dbglink_writestr("No AX5043 RF chip found \n");
#endif // USE_DBGLINK
                goto terminate_error;
            }
            goto terminate_radio_error;
        }
        display_writestr("found AX5043\n");
#ifdef USE_DBGLINK
        if (DBGLNKSTAT & 0x10)
            dbglink_writestr("found AX5043\n");
#endif // USE_DBGLINK
        axradio_set_local_address(&localaddr);
        axradio_set_default_remote_address(&remoteaddr);


//RX Only
#if RADIO_MODE == AXRADIO_MODE_WOR_RECEIVE || RADIO_MODE == AXRADIO_MODE_WOR_ACK_RECEIVE
        // LPOSC Calibrations needs full control of the radio, so it must run while the radio is idle otherwise
        calibrate_lposc();
#endif


#if RADIO_MODE == MODE_SYNC_MASTER || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_MASTER || RADIO_MODE == AXRADIO_MODE_SYNC_SLAVE || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_SLAVE
        display_writestr("settle LPXOSC");
#ifdef USE_DBGLINK
        if (DBGLNKSTAT & 0x10)
            dbglink_writestr("settle LPXOSC\n");
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
            dbglink_writestr("\nMASTER & SLAVE\n");
        }
#endif // USE_DBGLINK

#if RADIO_MODE == AXRADIO_MODE_WOR_RECEIVE || RADIO_MODE == AXRADIO_MODE_WOR_ACK_RECEIVE
        display_writestr("SLAVE  RX WOR\n               ");
#ifdef USE_DBGLINK
        if (DBGLNKSTAT & 0x10)
            dbglink_writestr("SLAVE  RX WOR\n");
#endif // USE_DBGLINK

#elif RADIO_MODE == AXRADIO_MODE_SYNC_SLAVE || RADIO_MODE == AXRADIO_MODE_SYNC_ACK_SLAVE
        display_writestr("SLAVE  RX SYNC\n               ");
#ifdef USE_DBGLINK
            if (DBGLNKSTAT & 0x10)
        dbglink_writestr("SLAVE  RX SYNC\n");
#endif // USE_DBGLINK

#else  // RADIO_MODE
        display_writestr("SLAVE  RX CONT\n");
#ifdef USE_DBGLINK
        if (DBGLNKSTAT & 0x10)
            dbglink_writestr("SLAVE  RX CONT\n");
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


#ifdef TX_ON_DEMAND
    BUTTON_INTCHG |= BUTTON_MASK; //interrupt on button changed (button SW5 on DVK-2) for wake on button pressed
#endif // TX_ON_DEMAND

    for(;;) {
    		// warmstart
        wtimer_runcallbacks();
        EA = 0;
        
//TX Only
#ifdef TX_ON_DEMAND
        {
            uint8_t buttonedge;
            {
                uint8_t p;
                p = BUTTON_PIN;
                buttonedge = saved_button_state & ~p;
                saved_button_state = p;
            }
            if (buttonedge & BUTTON_MASK)
            {
                EA = 1;
                if( !deglitch_busy )
                {
                    deglitch_busy = 1;
                    transmit_packet();
                    display_transmit_packet();
                }
                continue;
            }
        }
        IE_3 = 1;
#endif  // TX_ON_DEMAND
        {
            uint8_t flg = WTFLAG_CANSTANDBY;


#ifdef MCU_SLEEP
            if (axradio_cansleep()
#ifdef USE_DBGLINK
                && dbglink_txidle()
#endif // USE_DBGLINK
                && display_txidle())
                    flg |= WTFLAG_CANSLEEP;
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
#ifdef MCU_SLEEP
            if (axradio_cansleep()
#ifdef USE_DBGLINK
                && dbglink_txidle()
#endif // USE_DBGLINK
                && display_txidle())
                    flg |= WTFLAG_CANSLEEP;
#endif // MCU_SLEEP
            wtimer_idle(flg);
        }
    }
}



