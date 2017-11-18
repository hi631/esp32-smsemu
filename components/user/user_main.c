/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "../smsplus/shared.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "psxcontroller.h"
#include "driver/i2s.h"
#include "esp_partition.h"
#include <esp_spi_flash.h>
#include "spi_lcd.h"

#define  ZERO_LENGTH 0

static uint32_t start_f;
static uint32_t end_f;
static spi_flash_mmap_handle_t handle1;
char* romfptr;

QueueHandle_t vidQueue;

//Read an unaligned byte.
char unalChar(const char *adr) {
    //printf("%4x|",(int)adr-0x3ffb1110);
	//See if the byte is in memory that can be read unaligned anyway.
	if (!(((int)adr)&0x40000000)) return *adr;
	//Nope: grab a word and distill the byte.
	int *p=(int *)((int)adr&0xfffffffc);
	int v=*p;
	int w=((int)adr&3);
	if (w==0) return ((v>>0)&0xff);
	if (w==1) return ((v>>8)&0xff);
	if (w==2) return ((v>>16)&0xff);
	if (w==3) return ((v>>24)&0xff);
}

//These are the arrays we store in the app cpu IRAM. We don't use the app
//processor, so we can freely use its IRAM for other purposes.
typedef struct {
	unsigned char videodata[256*192];
	unsigned char sram[0x8000];
} AppIramData;

AppIramData *appIramData;

void system_load_sram(void) {
}

static void smsemu(void *arg) {
	int frameno, frameTgt;
	int lastTickCnt, tickCnt;
	int didFrame;
	int x;
	short sndleft[(SNDRATE/SMS_FPS)], sndright[(SNDRATE/SMS_FPS)];
	sms.use_fm=0;
	sms.country=TYPE_OVERSEAS;
    printf("Heap left %d\n", system_get_free_heap_size()); 
    appIramData = malloc(sizeof(AppIramData));
	sms.dummy=appIramData->videodata; //A normal cart shouldn't access this memory ever. Point it to vram just in case.
	sms.sram=appIramData->sram;
	bitmap.data=appIramData->videodata;
	bitmap.width=256;
	bitmap.height=192;
	bitmap.pitch=256;
	bitmap.depth=8;
	
	cart.pages=((512*1024)/0x4000);
	cart.rom=(char*)romfptr;
	cart.type=TYPE_SMS;

	emu_system_init(SNDRATE);
    init_sound();
	printf("Sound buffer: %d samples, enabled=%d.\n", snd.bufsize, snd.enabled);
	lastTickCnt=0;
    didFrame=0;
    
    system_reset(); ////
    
    printf("Heap left %d\n", system_get_free_heap_size());
    printf("Loop Start\n");
	while(1) {
		//tickCnt would be 100tick/sec, but because we're running at double the clock speed without informing
		//the RTOS, it's 1000tick/sec
		for (frameno=0; frameno<SMS_FPS; frameno++) {
			tickCnt=xTaskGetTickCount();
			if (tickCnt==lastTickCnt) tickCnt++;
			frameTgt=((tickCnt-lastTickCnt)*SMS_FPS)/Tickrate;
			frameTgt+=didFrame; //Try do diffuse frames a bit... otherwise, because of the low
				//granularity of the FreeRTOS tick variable, the drawn frames will be 'clumped together'.
			if (frameTgt<=frameno) {
				psxReadInput();     // PS Key Input
				sms_frame(0);
				//lcdWriteSMSFrame();
				didFrame=3;
				printf("1");
			} else {
				sms_frame(1);
				if (didFrame!=0) didFrame--;
				printf("0");
			}
           	xQueueSend(vidQueue, bitmap.data, 0); // LCD Display
            audio_frame_call();                   // Audio Out
			//for (x=0; x<snd.bufsize; x++) {
			//	i2sPushSample((snd.buffer[0][x]<<16)+snd.buffer[1][x]);
			//}
		}
		tickCnt=xTaskGetTickCount();
		if (tickCnt==lastTickCnt) tickCnt++;
		printf("\nfps=%d ", (SMS_FPS*Tickrate)/(tickCnt-lastTickCnt));
		lastTickCnt=tickCnt;
	}
}

//This runs on core 1.
static void videoTask(void *arg) {
	int x, y,tickCnt;

    ili9341_init();
    ili9341_write_frame(0, 0, 320, 240, NULL);

	x = (320-DEFAULT_WIDTH)/2;
    y = ((240-DEFAULT_HEIGHT)/2);
    while(1) {
		xQueueReceive(vidQueue, bitmap.data, portMAX_DELAY);
        tickCnt=xTaskGetTickCount();
        ili9341_write_frame(x, y, DEFAULT_WIDTH, DEFAULT_HEIGHT, bitmap.data);
        //printf("[%d]",(xTaskGetTickCount()-tickCnt)*1);
	}
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void emu_user_init(void)
{
	//Clock CPU at 160MHz. We kinda need the speed here...
	//SET_PERI_REG_MASK(CPU_PER_CONF_REG, PRODPORT_CPUPERIOD_SEL);

   	const esp_partition_t* part = esp_partition_find_first(0x40, 1, NULL);
	esp_err_t err=esp_partition_mmap(part, 0, 1*1024*1024, SPI_FLASH_MMAP_DATA, (const void**)&romfptr, &handle1);
    if (err==ESP_OK) printf("Mapping handle=%d ptr=%p\n", handle1, romfptr);
    
   	vidQueue=xQueueCreate(1, sizeof(bitmap.data));
	xTaskCreatePinnedToCore(&videoTask, "videoTask", 2048, NULL, 5, NULL, 1);

	//printf("SDK version:%s\n", system_get_sdk_version());
    printf("Heap left %d\n", system_get_free_heap_size()); 
	xTaskCreate(smsemu, "smsemu"  , 2048, NULL, 3, NULL);
	psxcontrollerInit();
	//i2sInit();
	//i2sSetRate(SNDRATE, 0);
}

