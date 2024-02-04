/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    led_and_button.c
 * @brief   On pressing button 1 Red and blue LED will glow and for button 2 green LED will glow
 *  Works only for MCU_APP30 targets
 */

#include <stdio.h>
#include <stdbool.h>
#include "coines.h"

/* Callback for button 1 interrupt */
static void button1CB(uint32_t param1, uint32_t param2);
/* Callback for button 2 interrupt */
static void button2CB(uint32_t param1, uint32_t param2);

int main(void)
{
    coines_open_comm_intf(COINES_COMM_INTF_USB, NULL); //Wait here till USB is connnected

    coines_set_pin_config(COINES_APP30_BUTTON_1, COINES_PIN_DIRECTION_IN, COINES_PIN_VALUE_HIGH);
    coines_attach_interrupt(COINES_APP30_BUTTON_1, button1CB, COINES_PIN_INTERRUPT_FALLING_EDGE);

    coines_set_pin_config(COINES_APP30_BUTTON_2, COINES_PIN_DIRECTION_IN, COINES_PIN_VALUE_HIGH);
    coines_attach_interrupt(COINES_APP30_BUTTON_2, button2CB, COINES_PIN_INTERRUPT_FALLING_EDGE);

    coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);

    return (0);
}

/*Callback for button 1 event */
void button1CB(uint32_t param1, uint32_t param2)
{
    (void)param1;
    (void)param2;

    coines_set_led(COINES_LED_RED, COINES_LED_STATE_ON);
    coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_OFF);
    coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_ON);
}

/*Callback for button 2 event */
void button2CB(uint32_t param1, uint32_t param2)
{
    (void)param1;
    (void)param2;
    
    coines_set_led(COINES_LED_RED, COINES_LED_STATE_OFF);
    coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_ON);
    coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_OFF);
}
