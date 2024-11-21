/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
/*
/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#define DOT_TIME 500000   // 500 ms for dot
#define DASH_TIME 1500000 // 1500 ms for dash
#define CHAR_GAP 1500000  // 1500 ms gap between characters
#define WORD_GAP 3500000  // 3500 ms gap between words

typedef enum {
    IDLE,
    SOS_DOT,
    SOS_DASH,
    SOS_CHAR_GAP,
    SOS_WORD_GAP,
    OK_DOT,
    OK_DASH,
    OK_CHAR_GAP,
    OK_WORD_GAP
} State;

typedef enum {
    SOS,
    OK
} Message;

volatile State currentState = IDLE;
volatile Message currentMessage = SOS;
volatile uint_least8_t buttonPressed = 0;

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    static int sos_index = 0;
    static int ok_index = 0;
    static char *sos_pattern = "...---...";
    static char *ok_pattern = "---.-.-";

    if (buttonPressed && currentState == IDLE) {
        if (currentMessage == SOS) {
            currentMessage = OK;
        } else {
            currentMessage = SOS;
        }
        buttonPressed = 0;
    }

    switch (currentState) {
        case IDLE:
            if (currentMessage == SOS) {
                currentState = (sos_pattern[sos_index] == '.') ? SOS_DOT : SOS_DASH;
            } else {
                currentState = (ok_pattern[ok_index] == '.') ? OK_DOT : OK_DASH;
            }
            break;
        case SOS_DOT:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            Timer_setPeriod(myHandle, Timer_PERIOD_US, DOT_TIME);
            currentState = SOS_CHAR_GAP;
            break;
        case SOS_DASH:
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            Timer_setPeriod(myHandle, Timer_PERIOD_US, DASH_TIME);
            currentState = SOS_CHAR_GAP;
            break;
        case SOS_CHAR_GAP:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            Timer_setPeriod(myHandle, Timer_PERIOD_US, CHAR_GAP);
            sos_index++;
            if (sos_index >= 9) {
                sos_index = 0;
                currentState = SOS_WORD_GAP;
            } else {
                currentState = (sos_pattern[sos_index] == '.') ? SOS_DOT : SOS_DASH;
            }
            break;
        case SOS_WORD_GAP:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            Timer_setPeriod(myHandle, Timer_PERIOD_US, WORD_GAP);
            currentState = IDLE;
            break;
        case OK_DOT:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            Timer_setPeriod(myHandle, Timer_PERIOD_US, DOT_TIME);
            currentState = OK_CHAR_GAP;
            break;
        case OK_DASH:
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            Timer_setPeriod(myHandle, Timer_PERIOD_US, DASH_TIME);
            currentState = OK_CHAR_GAP;
            break;
        case OK_CHAR_GAP:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            Timer_setPeriod(myHandle, Timer_PERIOD_US, CHAR_GAP);
            ok_index++;
            if (ok_index >= 7) {
                ok_index = 0;
                currentState = OK_WORD_GAP;
            } else {
                currentState = (ok_pattern[ok_index] == '.') ? OK_DOT : OK_DASH;
            }
            break;
        case OK_WORD_GAP:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            Timer_setPeriod(myHandle, Timer_PERIOD_US, WORD_GAP);
            currentState = IDLE;
            break;
    }
}

/*
 *  ======== initTimer ========
 *  Initialize the timer.
 */
void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);
    params.period = DOT_TIME; // 500 ms
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialize timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Set the flag to indicate button press */
    buttonPressed = 1;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Toggle LED */
    GPIO_toggle(CONFIG_GPIO_LED_1);
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    /* Initialize the timer */
    initTimer();

    return (NULL);
}
