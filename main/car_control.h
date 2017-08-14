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
#ifndef	_car_control_H_
#define _car_control_H_
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdlib.h>
void gpio_isr_handler(void *args);
void gpio_task(void* args);
void TrigPing(gpio_num_t TRIG, gpio_num_t ECHO);
double getDis(gpio_num_t TRIG, gpio_num_t ECHO);
double getDistance(gpio_num_t TRIG,gpio_num_t ECHO);
float doDistanceSensor(gpio_num_t GPIOTRIGOUTPUT, gpio_num_t GPIOECHOINPUT);
void readCellsVoltage(void);
void motorWrite(int leftduty,bool leftdirection,int rightduty,bool rightdirection);
void doCarUpdate(char* p_payload);
void safetyLoop(void);
void batterycheck(void);
#endif
