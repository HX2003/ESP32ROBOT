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
#include "ads1115.h"
#include "ws2812.h"
#include "initi2c.h"
#include "car_control.h"
#include "Ledc_Task.h"
#include "WebSocket_Task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_system.h>

//#define PINGuse_interrupt
/*#include "freertos/FreeRTOS.h"
#include "esp_heap_alloc_caps.h"
#include "hwcrypto/sha.h"
#include "esp_system.h"
#include "wpa2/utils/base64.h"
*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#define GPIO_OUTPUT_IO_0    16
#define GPIO_OUTPUT_IO_1    17

#define GPIO_OUTPUT_IO_2    13
#define GPIO_INPUT_IO_0    14
#define ECHO1 14
#define GPIO_OUTPUT_IO_3    27
#define GPIO_INPUT_IO_1   34
#define ECHO2 34
float dis1=0;
float dis2=0;
int number1=0;
int number2=0;
int duty1=0;
int duty2=0;
int speed=0;

int gpiout1=0;
int gpiout2=0;
int DistanceSet = 25;
int BatLowSet = 30;
int servoduty=0;
int servoduty2=0;
int counter2=0;
int check=0;
int keepUp=-1;
//Cell voltages
double cell3=0;
double cell2=0;
double cell1=0;
double cell0=0;
//ECHO TIME KEEPING
uint32_t ECHO1TIME = 0;
uint32_t ECHO2TIME = 0;

uint32_t ECHO1DUR = 0;
uint32_t ECHO2DUR = 0;
uint32_t gpio_num = 0;

bool ECHO1FINISH = 1;
bool ECHO2FINISH = 1;

uint16_t hello=0;
extern xQueueHandle gpio_evt_queue;
void gpio_isr_handler(void *args) {
    gpio_num = (uint32_t) args;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);

}
void gpio_task(void* args){
uint32_t io_num;
for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
  gpio_num = (uint16_t) io_num;
  //printf("gpio:%i",gpio_num);
    if(gpio_num==ECHO1){//printf("ECHO1:%i",ECHO1);

    	bool ECHO1LEVEL = gpio_get_level(ECHO1);
    	if(ECHO1LEVEL==1){
    		ECHO1TIME=system_get_time();

    	}
    	if(ECHO1LEVEL==0){
    		ECHO1DUR=system_get_time()-ECHO1TIME;
    	}
    }
    //second sensor
    if(gpio_num==ECHO2){//printf("ECHO2:%i",ECHO2);

    	bool ECHO2LEVEL = gpio_get_level(ECHO2);
    	if(ECHO2LEVEL==1){ //HIGH AND NEW CONNECTION
    		ECHO2TIME=system_get_time();
    	}
    	if(ECHO2LEVEL==0){
    		ECHO2DUR=system_get_time()-ECHO2TIME;
    	}
    }
        }
}
}
void TrigPing(gpio_num_t TRIG, gpio_num_t ECHO){

	 gpio_set_level(TRIG, 1);
	 ets_delay_us(100);
	 gpio_set_level(TRIG, 0);
}
double getDis(gpio_num_t TRIG, gpio_num_t ECHO){
	 uint32_t diff=0;
	if(ECHO==ECHO1){
		diff=ECHO1DUR;
	}else{
		diff=ECHO2DUR;
	}
	// printf("diff:%i", diff);
	double distance = 340.29 * diff / (1000 * 1000 * 2); // Distance in meters
	return distance;
}
double getDistance(gpio_num_t TRIG, gpio_num_t ECHO){
	TrigPing(TRIG,ECHO);
	 uint32_t startTime = system_get_time();
	 // Wait for echo to go high and THEN start the time
	 while (gpio_get_level(ECHO) == 0 &&
	 (system_get_time() - startTime) < 500 * 100) {
	 }
	 //now high or timeout
	 if((system_get_time() - startTime) < 500 * 100){
	 //within time
		 startTime = system_get_time();
		 while (gpio_get_level(ECHO) == 1 &&
		 (system_get_time() - startTime) < 500 * 100) {
		 // Do nothing;
		 }
		 //now low or timeout
		 if (gpio_get_level(ECHO) == 0 && (system_get_time() - startTime) < 500 * 100) {
			 uint32_t diff = system_get_time() - startTime; // Diff time in uSecs
			 // Distance is TimeEchoInSeconds * SpeedOfSound / 2
			 double distance = 340.29 * diff / (1000 * 1000 * 2); // Distance in meters
			 return distance;
			 } else {
			 return -1;
		 }

	 }else{
		 return -1;
	 }

}
float doDistanceSensor(gpio_num_t GPIOTRIGOUTPUT, gpio_num_t GPIOECHOINPUT){
   double distance1=getDistance(GPIOTRIGOUTPUT,GPIOECHOINPUT);

   return distance1*100;
}
void readCellsVoltage(void){
	uint16_t adc3,adc2,adc1,adc0;
	double adc3V,adc2V,adc1V,adc0V;
	//
	adc0=readADC_SingleEnded(0);
	adc1=readADC_SingleEnded(1);
	adc2=readADC_SingleEnded(2);
	adc3=readADC_SingleEnded(3);
	//printf("cells%i ,%i ,%i,%i",adc0, adc1, adc2, adc3);
	adc0V=conversionToVoltage(adc0,5.1);
	adc1V=conversionToVoltage(adc1,5.1);
	adc2V=conversionToVoltage(adc2,5.1);
	adc3V=conversionToVoltage(adc3,5.1);
	cell0=adc3V;
	cell1=adc2V-adc3V;
	cell2=adc1V-adc2V;
	cell3=adc0V-adc1V;


}
void batterycheck(void){
	while(1){
		vTaskDelay(2000/portTICK_RATE_MS);
		readCellsVoltage();
		printf("cells%lf mV,%lf mV,%lf mV,%lf mV\n",cell0,cell1,cell2,cell3);
		//scani2c();
		//vTaskDelay(2000/portTICK_RATE_MS);
	}
}

void motorWrite(int leftduty,bool leftdirection,int rightduty,bool rightdirection){
	gpio_set_level(GPIO_OUTPUT_IO_1, leftdirection);
	gpio_set_level(GPIO_OUTPUT_IO_0, rightdirection);
    ledc_write(LEDC_CHANNEL_3,leftduty);
    ledc_write(LEDC_CHANNEL_2,rightduty);
}
void safetyLoop(void){
	while(1){
counter2=counter2+1;
#ifdef PINGuse_interrupt
//TRIGGER SENSORS
TrigPing(GPIO_OUTPUT_IO_2,GPIO_INPUT_IO_0);
TrigPing(GPIO_OUTPUT_IO_3,GPIO_INPUT_IO_1);
//GET LAST VALUES
dis1=getDis(GPIO_OUTPUT_IO_2,GPIO_INPUT_IO_0);
dis2=getDis(GPIO_OUTPUT_IO_3,GPIO_INPUT_IO_1);
#else
//front sensor
dis1=doDistanceSensor(GPIO_OUTPUT_IO_2,GPIO_INPUT_IO_0);
//back sensor
dis2=doDistanceSensor(GPIO_OUTPUT_IO_3,GPIO_INPUT_IO_1);
#endif
if(DistanceSet!=0){
//front sensor detect wall && car moving forward stop car
if(dis1<DistanceSet&&gpiout1==1&&gpiout2==0){
	speed=0;
    motorWrite(0,gpiout1,0,gpiout2);
    printf("obstruction front");
}
//back sensor detect wall && car moving backwards stop car
if(dis2<DistanceSet&&gpiout1==0&&gpiout2==1){
	speed=0;
    motorWrite(0,gpiout1,0,gpiout2);
    printf("obstruction back");

}
}
if(isClientNotConnectedWifi()==1){
	speed=0;
	motorWrite(0,gpiout1,0,gpiout2);
}
if(websocketConnected==0){
	speed=0;
	motorWrite(0,gpiout1,0,gpiout2);
}
if(counter2%5==1){ //everytime there is a websocket data, keepup increments //every 4 execution of this
    if(keepUp<1){
    	speed=0;
    	motorWrite(0,gpiout1,0,gpiout2);

    }else{
       keepUp=-1;
    }
}
vTaskDelay(50/portTICK_RATE_MS);//allow for websocket
	}
}

void doCarUpdate(char* p_payload){
	//printf("content %.*s\n",p_payload);
	                    	   char* tokenpass;
	                    	   char *reserve;
	                    	   tokenpass = strtok_r(p_payload, ",",&reserve);
	                    	   int counts=0;
	                    	   while (tokenpass != NULL)
	                    	    {
	                    	      counts=counts+1;
	                    	      //printf("extracting... %s-%i\n",tokenpass,counts);
	                    	      if(counts==1){
	                    	    	  BatLowSet=atoi(tokenpass);
	                    	      }
	                    	      if(counts==2){
	                    	    	  DistanceSet=atoi(tokenpass);
	                    	      }
	                    	      if(counts==3){
	                    	    	  //printf ("Deg: %s\n",tokenpass);
	                    	    	  number1=atoi(tokenpass);
	                    	      }
	                    	      if(counts==4){
	                    	    	  //printf ("Speed: %s\n",tokenpass);
	                    	    	  number2=atoi(tokenpass);
	                    	      }
	                    	      if(counts==5){
	                    	          //printf ("Speed: %s\n",tokenpass);
	                    	          servoduty=atoi(tokenpass);
	                    	      }
	                    	      tokenpass = strtok_r (NULL, ",",&reserve);
	                    	    }
                               //keepup
	                    	   keepUp=keepUp+1;
	                    	   //front sensor
	                    	   printf ("BatLowSet: %i, DistanceSet: %i,Deg: %i,Speed: %i, Servo: %i\n",BatLowSet,DistanceSet,number1,number2,servoduty);
	                    	   printf ("Dis1: %f, Dis2: %f\n",dis1,dis2);
                               //find out speed
	                           if(number2==0){
	                           //speed 0
	                           speed=0;
	                           }else if(number2>0){
	                           //speed above 0
	                           if(number2>60){
	                           if(DistanceSet==0){
	                            //do not care about dis1
	                        	   speed=number2*32;
	                        	   gpiout1=1;
	                        	   gpiout2=0;
	                           }else if(dis1>DistanceSet){
	                           speed=number2*32;
	                           gpiout1=1;
	                           gpiout2=0;
	                            }else{
	                            speed=0;
	                            }
	                           }else{
	                           speed=0;
	                           }

	                           }else if(number2<0){
	                           //speed below 0
	                           if(number2<-60){
	                           if(DistanceSet==0){
	                        	//do not care about dis2
	                        	 speed=number2*-32;
	                        	 gpiout1=0;
	                        	 gpiout2=1;
	                          }else if(dis2>DistanceSet){
	                           speed=number2*-32;
	                           gpiout1=0;
	                           gpiout2=1;
	                           }else{
	                        	speed=0;
	                           }
	                           }else{
	                           speed=0;
	                           }
	                           }
	                           //find out degrees
	                           if(number1==0){
	                            duty1=speed;
	                            duty2=speed;
	                           }else if(number1>0){
	                        	duty1=speed;
	                        	duty2=speed-speed*(number1*2.22)/100;
	                           }else if(number1<0){
	                        	duty2=speed;
	                        	duty1=speed-speed*(number1*-2.22)/100;
	                           }
	                           printf("M1: %i M2: %i", duty1,duty2);
	                           motorWrite(duty1,gpiout1,duty2,gpiout2);
	                           //duty from 205 to 1024
	                           //range of 2^13/10*2.5=1024
	                           //205-1024 840/180=5.68
	                           //for -90  to 90
	                           /*if(servoduty<0){
	                        	   servoduty=(90-servoduty*-1)*4.66;
	                           }else if(servoduty>0){
	                        	   servoduty=servoduty*4.66+410;
	                           }*/
	                           servoduty=servoduty*2.33;
	                           servoduty=205+servoduty;
	                           servoduty2=servoduty;
	                           printf("servo duty: %i",servoduty2);
	                           ledc_write(LEDC_CHANNEL_1 ,servoduty2);
}
