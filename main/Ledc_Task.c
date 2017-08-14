/**
 * @section License
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017, Thomas Barth, barth-dev.de
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "WebSocket_Task.h"
#include "ws2812.h"
#include "car_control.h"
#include "freertos/FreeRTOS.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#define LEDC_IO_0    (18)
#define LEDC_IO_1    (19)
#define LEDC_IO_2    (4)
#define LEDC_IO_3    (5)
#define GPIO_OUTPUT_IO_0    16
#define GPIO_OUTPUT_IO_1    17

#define GPIO_OUTPUT_IO_2    13
#define GPIO_INPUT_IO_0    14
#define GPIO_OUTPUT_IO_3    27
#define GPIO_INPUT_IO_1   34

//#define PINGuse_interrupt
rgbVal *pixels;
const int DATA_PIN = 18; // Avoid using any of the strapping pins on the ESP32
const uint16_t NUM_PIXELS = 7;  // How many pixels you want to drive
uint8_t MAX_COLOR_VAL = 32;
void initialise_gpio(void){
    gpio_pad_select_gpio(GPIO_OUTPUT_IO_0);
	/* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_OUTPUT_IO_0, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(GPIO_OUTPUT_IO_1);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_OUTPUT_IO_1, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(GPIO_OUTPUT_IO_2);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_OUTPUT_IO_2, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(GPIO_INPUT_IO_0);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_INPUT_IO_0, GPIO_MODE_INPUT);

    gpio_pad_select_gpio(GPIO_OUTPUT_IO_3);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_OUTPUT_IO_3, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(GPIO_INPUT_IO_1);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_INPUT_IO_1, GPIO_MODE_INPUT);
#ifdef PINGuse_interrupt
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    gpio_set_intr_type(GPIO_INPUT_IO_1, GPIO_INTR_ANYEDGE);

    gpio_install_isr_service(0);
     //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
     //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

#else

#endif

}
void initialise_ledc(void)
{
ledc_timer_config_t ledc_timer = {
    //set timer counter bit number
    .bit_num = LEDC_TIMER_13_BIT,
    //set frequency of pwm
    .freq_hz = 50,
    //timer mode,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    //timer index
    .timer_num = LEDC_TIMER_1
};
ledc_timer_config_t ledc_timer2 = {
    //set timer counter bit number
    .bit_num = LEDC_TIMER_13_BIT,
    //set frequency of pwm
    .freq_hz = 1000,
    //timer mode,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    //timer index
    .timer_num = LEDC_TIMER_0
};
ledc_timer_config(&ledc_timer);
ledc_timer_config(&ledc_timer2);

ledc_channel_config_t ledc_channel = {
    //set LEDC channel 0
    .channel = LEDC_CHANNEL_0,
    //set the duty for initialization.(duty range is 0 ~ ((2**bit_num)-1)
    .duty = 0,
    //GPIO number
    .gpio_num = LEDC_IO_0,
    //GPIO INTR TYPE, as an example, we enable fade_end interrupt here.
    .intr_type = LEDC_INTR_DISABLE,
    //set LEDC mode, from ledc_mode_t
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    //set LEDC timer source, if different channel use one timer,
    //the frequency and bit_num of these channels should be the same
    .timer_sel = LEDC_TIMER_1
};

//set the configuration
ledc_channel_config(&ledc_channel);

//config ledc channel1
ledc_channel.channel = LEDC_CHANNEL_1;
ledc_channel.gpio_num = LEDC_IO_1;
ledc_channel.timer_sel = LEDC_TIMER_1;
ledc_channel_config(&ledc_channel);
//config ledc channel2
ledc_channel.channel = LEDC_CHANNEL_2;
ledc_channel.gpio_num = LEDC_IO_2;
ledc_channel.timer_sel = LEDC_TIMER_0;
ledc_channel_config(&ledc_channel);
//config ledc channel3
ledc_channel.channel = LEDC_CHANNEL_3;
ledc_channel.gpio_num = LEDC_IO_3;
ledc_channel.timer_sel = LEDC_TIMER_0;
ledc_channel_config(&ledc_channel);
}
void ledc_write(ledc_channel_t  ledcChannel, uint32_t ledcDuty){
 ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledcChannel, ledcDuty);
 ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledcChannel);
}
void displayOff(void) {
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels[i] = makeRGBVal(0, 0, 0);
  }
 // ws2812_setColors(2, pixels);
  ws2812_setColors(NUM_PIXELS, pixels);
}
void fillColor(int r, int g, int b){
	for(uint16_t i=0; i<NUM_PIXELS; i++) {
		    pixels[i] = makeRGBVal(r, g, b);
		  }
	printf("\n[DEBUG]Filling color r:%i g:%i b:%i \n",r,g,b);
	 ws2812_setColors(NUM_PIXELS, pixels);
}

void initialise_ws2812b(void){
	ws2812_init(DATA_PIN, LED_WS2812B);
    pixels = (rgbVal*)malloc(sizeof(rgbVal) * NUM_PIXELS);
	displayOff();
	fillColor(255,0,0);
}
