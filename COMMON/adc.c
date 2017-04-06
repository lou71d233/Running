#include "adc.h"
#include <stdio.h>

#define ADC

#ifdef ADC
//uint8_t dmaflag;
//uint16_t __code adcbuffer[ADC_BUFFER_SIZE];
//static __xdata struct DMA_descriptor dma_bd_adc;
//char screen[] = "Temp:     VDDIO: \n   C       .   V";
#endif

void adc_init(void)
{
    //ADC init
    ADCCLKSRC			= 0x30;//0x04;//0x3B;
    ADCCH0CONFIG		= 0xC2;
    ADCCH1CONFIG		= 0xC5;
    ADCCH2CONFIG		= 0xFF;
    ADCCH3CONFIG		= 0xFF;

    ADCTUNE0			= 0x01;
    ADCTUNE1			= 0x06;
    ADCTUNE2			= 0xFC;

    //ADCCONV				= 0x07;

}

uint8_t adc_work(uint8_t channel)
{
    uint8_t ie_save = IE;
    uint8_t eie_save = EIE;
    uint8_t e2ie_save = E2IE;

    uint32_t vddio_temp;
    uint32_t vddio,moto_AD1;
    uint8_t i;

    IE = 0;
    EIE = 0;
    E2IE = 0;

    //ADCCONV = 0x07;

    if(channel==battery_ch)
    {
        vddio = 0;

        for (i=0; i<ADC_BUFFER_SIZE; i++)
        {
            ADCCONV = 0x01;
            for (;;) {
                EA = 0;
                if (ADCCONV & 0x80)
                break;
                EIE_6 = 1;
                enter_standby();
                EIE_6 = 0;
                EA = 1;
            }
            vddio_temp=ADCCH0VAL1;
            vddio_temp<<=8;
            vddio_temp|=ADCCH0VAL0;
            vddio += vddio_temp;
            //uart0_writenum16(vddio,6,1);


        }
        vddio /= ADC_BUFFER_SIZE;
        //uart0_writenum16(vddio,6,1);
        vddio = vddio*1000/256-450;
        vddio /=100;
        vddio%=1000;
        //vddio = vddio/1000; //registor offset
        //uart0_writenum16(vddio,6,1);

        IE = ie_save;
        EIE = eie_save;
        E2IE = e2ie_save;

        return vddio;
    }

    if(channel==motor_ch)
    {
        moto_AD1 = 0;

/*
        for (i=0; i<ADC_BUFFER_SIZE; i++)
        {
            moto_AD1 += ADCCH1VAL1;
        }
        moto_AD1 /= ADC_BUFFER_SIZE;
        moto_AD1 = moto_AD1*1000/256-450;

        ADCCONV=0;
*/

        for (i=0; i<ADC_BUFFER_SIZE; i++)
        {
            ADCCONV = 0x01;
            for (;;) {
                EA = 0;
                if (ADCCONV & 0x80)
                break;
                EIE_6 = 1;
                enter_standby();
                EIE_6 = 0;
                EA = 1;
            }
            moto_AD1 += ADCCH1VAL1;
        }
        moto_AD1 /= ADC_BUFFER_SIZE;
        moto_AD1 = moto_AD1*1000/256-450;

        //moto_AD1 = ADCCH1VAL1;

        IE = ie_save;
        EIE = eie_save;
        E2IE = e2ie_save;

        return moto_AD1;
    }
    return 0;
}


