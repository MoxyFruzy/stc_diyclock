//
// STC15F204EA DIY LED Clock
// Copyright 2016, Jens Jensen
//

// silence: "src/main.c:672: warning 126: unreachable code"
#pragma disable_warning 126

#include "stc15.h"
#include <stdint.h>
#include <stdio.h>
#include "adc.h"
#include "ds1302.h"
#include "led.h"

#define FOSC    11059200

// clear wdt
#define WDT_CLEAR()    (WDT_CONTR |= 1 << 4)

// hardware configuration
#include "hwconfig.h"

// display mode states
enum keyboard_mode {
    K_NORMAL,
    K_SET_HOUR,
    K_SET_MINUTE,
    K_SET_HOUR_12_24,
};

// display mode states
enum display_mode {
    M_NORMAL,
    M_SET_HOUR_12_24,
    M_SEC_DISP,
};

/* ------------------------------------------------------------------------- */
/*
void _delay_ms(uint8_t ms)
{
    // delay function, tuned for 11.092 MHz clock
    // optimized to assembler
    ms; // keep compiler from complaining?
    __asm;
        ; dpl contains ms param value
    delay$:
        mov	b, #8   ; i
    outer$:
        mov	a, #243    ; j
    inner$:
        djnz acc, inner$
        djnz b, outer$
        djnz dpl, delay$
    __endasm;
}
*/

uint8_t  count;     // main loop counter
uint8_t temp;      // temperature sensor value
uint8_t  lightval;  // light sensor value

volatile uint8_t displaycounter;
volatile int8_t count_100;
volatile int16_t count_1000;
volatile int16_t count_5000;

volatile uint16_t count_timeout; // max 6.5536 sec
#define TIMEOUT_LONG 0xFFFF
volatile __bit blinker_slow;
volatile __bit blinker_fast;
volatile __bit loop_gate;

uint8_t dmode = M_NORMAL;     // display mode state
uint8_t kmode = K_NORMAL;

__bit  flash_01;
__bit  flash_23;

uint8_t rtc_hh_bcd;
uint8_t rtc_mm_bcd;
__bit rtc_pm;
__bit cfg_changed = 1;

volatile __bit S1_LONG;
volatile __bit S1_PRESSED;
volatile __bit S2_LONG;
volatile __bit S2_PRESSED;
#ifdef stc15w408as
volatile __bit S3_LONG;
volatile __bit S3_PRESSED;
#endif

volatile uint8_t debounce[NUM_SW];      // switch debounce buffer
volatile uint8_t switchcount[NUM_SW];
#define SW_CNTMAX 80

enum Event {
    EV_NONE,
    EV_S1_SHORT,
    EV_S1_LONG,
    EV_S2_SHORT,
    EV_S2_LONG,
    EV_S1S2_LONG,
#ifdef stc15w408as
    EV_S3_SHORT,
    EV_S3_LONG,
#endif
    EV_TIMEOUT,
};

volatile enum Event event;

void timer0_isr() __interrupt 1 __using 1
{
    uint8_t tmp;
    enum Event ev = EV_NONE;
    // display refresh ISR
    // cycle thru digits one at a time
    uint8_t digit = displaycounter % (uint8_t) 4;

    // turn off all digits, set high
    LED_DIGITS_OFF();

    // auto dimming, skip lighting for some cycles
    if (displaycounter % lightval < 4 ) {
        // fill digits
        LED_SEGMENT_PORT = dbuf[digit];
        // turn on selected digit, set low
        //LED_DIGIT_ON(digit);
        // issue #32, fix for newer sdcc versions which are using non-atomic port access
        tmp = ~((1<<LED_DIGITS_PORT_BASE) << digit);
        LED_DIGITS_PORT &= tmp;
    }
    displaycounter++;

    // 100/sec: 10 ms
    if (count_100 == 100) {
        count_100 = 0;
        // 10/sec: 100 ms
        if (count_1000 == 1000) {
            count_1000 = 0;
            blinker_fast = !blinker_fast;
            loop_gate = 1;
            // 2/sec: 500 ms
            if (count_5000 == 5000) {
                count_5000 = 0;
                blinker_slow = !blinker_slow;
            }
        }

#define MONITOR_S(n) \
        { \
            uint8_t s = n - 1; \
            /* read switch positions into sliding 8-bit window */ \
            debounce[s] = (debounce[s] << 1) | SW ## n ; \
            if (debounce[s] == 0) { \
                /* down for at least 8 ticks */ \
                S ## n ## _PRESSED = 1; \
                if (!S ## n ## _LONG) { \
                    switchcount[s]++; \
                } \
            } else { \
                /* released or bounced */ \
                if (S ## n ## _PRESSED) { \
                    if (!S ## n ## _LONG) { \
                        ev = EV_S ## n ## _SHORT; \
                    } \
                    S ## n ## _PRESSED = 0; \
                    S ## n ## _LONG = 0; \
                    switchcount[s] = 0; \
                } \
            } \
            if (switchcount[s] > SW_CNTMAX) { \
                S ## n ## _LONG = 1; \
                switchcount[s] = 0; \
                ev = EV_S ## n ## _LONG; \
            } \
        }

        MONITOR_S(1);
        MONITOR_S(2);
#ifdef stc15w408as
        MONITOR_S(3);
#endif

        if (ev == EV_S1_LONG && S2_PRESSED) {
            S2_LONG = 1;
            switchcount[1] = 0;
            ev = EV_S1S2_LONG;
        } else if (ev == EV_S2_LONG && S1_PRESSED) {
            S1_LONG = 1;
            switchcount[0] = 0;
            ev = EV_S1S2_LONG;
        }
        if (event == EV_NONE) {
            event = ev;
        }
    }
    count_100++;
    count_1000++;
    count_5000++;
}

/*
// macro expansion for MONITOR_S(1)
{
    uint8_t s = 1 - 1;
    debounce[s] = (debounce[s] << 1) | SW1 ;
    if (debounce[s] == 0) {
        S_PRESSED = 1;
        if (!S_LONG) {
            switchcount[s]++;
        }
    } else {
        if (S1_PRESSED) {
            if (!S1_LONG) {
                ev = EV_S1_SHORT;
            }
            S1_PRESSED = 0;
            S1_LONG = 0;
            switchcount[s] = 0;
        }
    }
    if (switchcount[s] > SW_CNTMAX) {
        S1_LONG = 1;
        switchcount[s] = 0;
        ev = EV_S1_LONG;
    }
}
*/

// Call timer0_isr() 10000/sec: 0.0001 sec
// Initialize the timer count so that it overflows after 0.0001 sec
// THTL = 0x10000 - FOSC / 12 / 10000 = 0x10000 - 92.16 = 65444 = 0xFFA4
void Timer0Init(void)		//100us @ 11.0592MHz
{
    // refer to section 7 of datasheet: STC15F2K60S2-en2.pdf
    // TMOD = 0;    // default: 16-bit auto-reload
    // AUXR = 0;    // default: traditional 8051 timer frequency of FOSC / 12
    // Initial values of TL0 and TH0 are stored in hidden reload registers: RL_TL0 and RL_TH0
	TL0 = 0xA4;		// Initial timer value
	TH0 = 0xFF;		// Initial timer value
    TF0 = 0;		// Clear overflow flag
    TR0 = 1;		// Timer0 start run
    ET0 = 1;        // Enable timer0 interrupt
    EA = 1;         // Enable global interrupt
}

// Formula was : 76-raw*64/637 - which makes use of integer mult/div routines
// Getting degF from degC using integer was not good as values were sometimes jumping by 2
// The floating point one is even worse in term of code size generated (>1024bytes...)
// Approximation for slope is 1/10 (64/637) - valid for a normal 20 degrees range
// & let's find some other trick (80 bytes - See also docs\Temp.ods file)
int8_t gettemp(uint16_t raw) {
    uint16_t val=raw;
    uint8_t temp;

    raw<<=2;
    if (CONF_C_F) raw<<=1;  // raw*5 (4+1) if Celcius, raw*9 (4*2+1) if Farenheit
    raw+=val;

    if (CONF_C_F) {val=6835; temp=32;}  // equiv. to temp=xxxx-(9/5)*raw/10 i.e. 9*raw/50
                                        // see next - same for degF
             else {val=5*757; temp=0;}  // equiv. to temp=xxxx-raw/10 or which is same 5*raw/50  
                                        // at 25degC, raw is 512, thus 24 is 522 and limit between 24 and 25 is 517
                                        // so between 0deg and 1deg, limit is 517+24*10 = 757 
                                        // (*5 due to previous adjustment of raw value)
    while (raw<val) {temp++; val-=50;}

    return temp + (cfg_table[CFG_TEMP_BYTE] & CFG_TEMP_MASK) - 4;
}

void dot3display(__bit pm)
{
    dotdisplay(3, pm);
}

/*********************************************/
int main()
{
    // SETUP
    // set photoresistor & ntc pins to open-drain output
    P1M1 |= (1<<ADC_LIGHT) | (1<<ADC_TEMP);
    P1M0 |= (1<<ADC_LIGHT) | (1<<ADC_TEMP);

    // init rtc
    ds_init();
    // init/read ram config
    ds_ram_config_init();

    // uncomment in order to reset minutes and hours to zero.. Should not need this.
    //ds_reset_clock();

    Timer0Init(); // display refresh & switch read

    // LOOP
    while (1)
    {
        enum Event ev;

        while (!loop_gate); // wait for open
        loop_gate = 0; // close gate

        ev = event;
        event = EV_NONE;

        // sample adc, run frequently
        if (count % (uint8_t) 4 == 0) {
            temp = gettemp(getADCResult(ADC_TEMP));
        }

        lightval = 4;

        // Read RTC
        ds_readburst();
        // parse RTC
        {
            rtc_hh_bcd = rtc_table[DS_ADDR_HOUR];
            if (H12_12) {
                rtc_hh_bcd &= DS_MASK_HOUR12;
            } else {
                rtc_hh_bcd &= DS_MASK_HOUR24;
            }
            rtc_pm = H12_12 && H12_PM;
            rtc_mm_bcd = rtc_table[DS_ADDR_MINUTES] & DS_MASK_MINUTES;
        }

        // keyboard decision tree
        switch (kmode) {

            case K_SET_HOUR:
                flash_01 = 1;
                if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast)) {
                    ds_hours_incr();
                }
                else if (ev == EV_S2_SHORT) {
                    kmode = K_SET_MINUTE;
                }
                break;

            case K_SET_MINUTE:
                flash_01 = 0;
                flash_23 = 1;
                if (ev == EV_S1_SHORT || (S1_LONG && blinker_fast)) {
                    ds_minutes_incr();
                }
                else if (ev == EV_S2_SHORT) {
                    kmode = K_SET_HOUR_12_24;
                }
                break;

            case K_SET_HOUR_12_24:
                dmode = M_SET_HOUR_12_24;
                if (ev == EV_S1_SHORT) {
                    ds_hours_12_24_toggle();
                    cfg_changed = 1;
                }
                else if (ev == EV_S2_SHORT) {
                    kmode = K_NORMAL;
                }
                break;

            case K_NORMAL:
            default:
                flash_01 = 0;
                flash_23 = 0;

                dmode = M_NORMAL;

                if (ev == EV_S2_LONG) {
                    kmode = K_SET_HOUR;
                }
#ifdef stc15w408as
                else if (ev == EV_S3_LONG) {
                    LED = !LED;
                }
#endif
        };

        // display execution tree

        clearTmpDisplay();

        switch (dmode) {
            case M_NORMAL:
            {
                uint8_t hh = rtc_hh_bcd;
                uint8_t mm = rtc_mm_bcd;
                __bit pm = rtc_pm;

                if (!flash_01 || blinker_fast || S1_LONG) {
                    uint8_t h0 = hh >> 4;
                    if (H12_12 && h0 == 0) {
                        h0 = LED_BLANK;
                    }
                    filldisplay(0, h0, 0);
                    filldisplay(1, hh & 0x0F, 0);
                }

                if (!flash_23 || blinker_fast || S1_LONG) {
                    filldisplay(2, mm >> 4, 0);
                    filldisplay(3, mm & 0x0F, 0);
                }

                if (blinker_slow || dmode != M_NORMAL) {
                    dotdisplay(1, 1);
                    dotdisplay(2, 1);
                }

                dot3display(pm);
                break;
            }
            case M_SET_HOUR_12_24:
                if (!H12_12) {
                    filldisplay(1, 2, 0);
                    filldisplay(2, 4, 0);
                } else {
                    filldisplay(1, 1, 0);
                    filldisplay(2, 2, 0);
                }
                filldisplay(3, LED_h, 0);
                break;

            case M_SEC_DISP:
                dotdisplay(0, 0);
                dotdisplay(1, blinker_slow);
                filldisplay(2,(rtc_table[DS_ADDR_SECONDS] >> 4) & (DS_MASK_SECONDS_TENS >> 4), blinker_slow);
                filldisplay(3, rtc_table[DS_ADDR_SECONDS] & DS_MASK_SECONDS_UNITS, 0);
                dot3display(0);
                break;
        }


        __critical {
            updateTmpDisplay();
        }

        count++;
        WDT_CLEAR();
    }
}
