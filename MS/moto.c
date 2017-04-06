
#include<string.h>

#include "moto.h"

#ifdef USE_COM0
#include <libmfuart0.h>
#endif // USE_COM0


void rotate_array( uint8_t *A , int n ,int shift) {
    uint8_t i,j;
    for (j = 0 ; j < shift ; ++j ){
        for (i = 0 ; i < n ; ++i )
            A[ i ] = A[ ( i + 1 ) % n ] ;
    }
}

uint8_t moto_machine_drive(uint8_t *addr)
{
    static uint8_t shift_head=5;
    uint8_t strtemp[21+2];

    memcpy(strtemp,axradio_rxbuffer_ext,sizeof(strtemp));
    ////if(memcmp(strtemp,strat,sizeof(strat))==0){
    //ID CHECK
    //if((strtemp[1]==addr[8])&&(strtemp[2]==addr[9]))
    {
        rotate_array(strtemp,sizeof(strtemp),shift_head);
        if(memcmp(strtemp,strat,sizeof(strat))==0){
            rotate_array(strtemp,sizeof(strtemp),sizeof(strat));
            if(memcmp(strtemp,strset,sizeof(strset))==0){
                rotate_array(strtemp,sizeof(strtemp),sizeof(strset));

                if(memcmp(strtemp,strlock_wheel,sizeof(strlock_wheel))==0){
                    //lock wheel
                    #ifdef DEBUG_MSG
                    display_writestr("LOCK_WHEEL\n");
                    #endif // DEBUG_MSG
                    moto_en_clr();
                    delay_ms(150);
                    moto_r_set();
                    moto_en_set();
                    set_moto_drive_flag(MOTO_R);
                    return 1;

                }else if(memcmp(strtemp,strunlock_wheel,sizeof(strunlock_wheel))==0){
                    //unlock wheel
                    #ifdef DEBUG_MSG
                    display_writestr("UNLOCK_WHEEL\n");
                    #endif // DEBUG_MSG
                    moto_en_clr();
                    delay_ms(150);
                    moto_l_set();
                    moto_en_set();
                    set_moto_drive_flag(MOTO_L);
                    return 1;
                }
            }
        }
    }
    return 0;
}



void stop_moto_drive(void){
    moto_en_clr();
    set_moto_drive_flag(MOTO_STOP);
}

void set_moto_drive_flag(uint8_t flag)
{
    moto_drive_flag=flag;
}

uint8_t get_moto_drive_flag(void)
{
    return moto_drive_flag;
}
