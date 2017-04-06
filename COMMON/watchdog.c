// Copyright (c) 2007,2008,2009,2010,2011,2012 AXSEM AG
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

#include "WATCHDOG.h"

#ifdef WATCH
volatile __bit gpioflag;
volatile __bit timer2flag;
volatile __bit watch_on = 0;
uint16_t timer2on;
uint16_t timer2off = 0x0007;
uint8_t cent = 0;
uint8_t sec = 0;
uint8_t minutes = 0;
uint8_t hours = 0;
#endif

void watch_init(void){
#ifdef WATCH
#ifdef LCD
    lcd_cleardisplay();
    lcd_waitlong();
    lcd_setpos(0);
    lcd_waitshort();
    lcd_writestr("Time  h  \'  \"   \n       :  :  :  ");
#endif
    timer2on = T2CLKSRC;
    watch_stop();
    watch_reset();
    IE_3 = 1;
    EIE_2 = 0;
#endif
}

uint8_t watch_work(void){
#ifdef WATCH
    if (gpioflag & watch_on)
    {
        watch_stop();
        gpioflag = 0;
        IE_3 = 1;
        return 1;
    }
    if (gpioflag & !watch_on)
    {
        watch_start();
        gpioflag = 0;
        IE_3 = 1;
        return 1;
    }
    if (timer2flag)
    {
        cent = cent + 1;
        if (cent > 99)
        {
            cent = 0;
            sec = sec + 1;
            if (sec > 59)
            {
                sec = 0;
                minutes = minutes + 1;
                if (minutes > 59)
                {
                    minutes = 0;
                    hours = hours + 1;
                }
            }
        }
//        if( DBGLNKSTAT & 0x10 )
//        {
//            dbglink_writenum16(hours,7,WRNUM_PADZERO);
//            dbglink_writestr(":");
//            dbglink_writenum16(minutes,2,WRNUM_PADZERO);
//            dbglink_writestr(":");
//            dbglink_writenum16(sec,2,WRNUM_PADZERO);
//            dbglink_writestr(":");
//            dbglink_writenum16(cent,2,WRNUM_PADZERO);
//            dbglink_writestr("\n");
//        }
#ifdef LCD
        lcd_writeu16(hours,7,0x46);
        lcd_writeu16(minutes,2,0x49);
        lcd_writeu16(sec,2,0x4c);
        lcd_writeu16(cent,2,0x4f);
#endif
        timer2flag = 0;
        EIE_2 = 1;
        return 1;
    }
#endif
    return 0;
}

#ifdef WATCH
void watch_start (void){
    EIE_2 = 1;
    T2CNT = cent;
    T2CLKSRC = timer2on;
    watch_on = 1;
}
void watch_stop (void){
    EIE_2 = 0;
    T2CLKSRC = timer2off;
    watch_on = 0;
}
void watch_reset (void){
    timer2flag = 0;
    cent = 0;
    sec = 0;
    minutes = 0;
    hours = 0;
#ifdef LCD
    lcd_writeu16(hours,7,0x46);
    lcd_writeu16(minutes,2,0x49);
    lcd_writeu16(sec,2,0x4c);
    lcd_writeu16(cent,2,0x4f);
#endif
}
#endif

#ifdef WATCH
void gpio_irq(void) __interrupt INT_GPIO{
	if(!PINB_2)
    {
        IE_3 = 0;
        gpioflag = 1;
    }
    else if(!PINB_3)
    {
        IE_3 = 0;
        watch_reset();
        IE_3 = 1;
    }
}
void timer2_irq(void) __interrupt INT_TIMER2{
    EIE_2 = 0;
    T2STATUS;
    timer2flag = 1;
}
#endif
