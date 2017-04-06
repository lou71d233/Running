#ifndef MOTO_H_INCLUDED
#define MOTO_H_INCLUDED


#include "../AX_Radio_Lab_output/configms.h"
#include "../COMMON/display_com0.h"
#include "../COMMON/axradio.h"
#include "../COMMON/easyax5043.h"
#include "../COMMON/misc.h"

#define MOTO_TP10

#ifdef MOTO_TP10

#define MOTODIR DIRC
#define MOTOPORT PORTC

#define MOTO_R_PN PORTC_2
#define MOTO_L_PN PORTC_3

#define MOTO_R_PIN PINC_2
#define MOTO_L_PIN PINC_3

#define MOTO_R_N BIT2SET
#define MOTO_L_N BIT3SET


#define moto_en_set()	do {MOTOPORT |= (1<<2);} while (0)
#define moto_en_clr()	do {MOTOPORT &= (uint8_t)~(1<<2);} while (0)

#define moto_l_set()	do {MOTOPORT |= (1<<3);} while (0)
#define moto_r_set()	do {MOTOPORT &= (uint8_t)~(1<<3);} while (0)


#else
// sample test



#endif

#define MOTO_STOP 0
#define MOTO_R 1
#define MOTO_L 2
#define MOTO_RUNNING_R 3
#define MOTO_RUNNING_L 4

extern uint8_t moto_drive_flag = 0;
const uint8_t strat[3] = "AT+";
const uint8_t strset[4] = "SET@";

const uint8_t strlock_wheel[4] = "LOCK_WHEEL";
const uint8_t strunlock_wheel[4] = "UNLOCK_WHEEL";

extern uint8_t moto_machine_drive(uint8_t *addr);
//extern moto_machine_driv(void);
extern void set_moto_drive_flag(uint8_t flag);
extern uint8_t get_moto_drive_flag(void);

extern void stop_moto_drive(void);
extern void rotate_array( uint8_t *A , int n ,int shift);

#endif // MOTO_H_INCLUDED



