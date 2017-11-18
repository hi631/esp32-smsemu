// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "rom/ets_sys.h"
#include "rom/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/spi_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "spi_lcd.h"

#include "driver/spi_master.h"
#include "driver/i2s.h"
#include "../smsplus/shared.h"

#define PIN_NUM_MISO CONFIG_HW_LCD_MISO_GPIO
#define PIN_NUM_MOSI CONFIG_HW_LCD_MOSI_GPIO
#define PIN_NUM_CLK  CONFIG_HW_LCD_CLK_GPIO
#define PIN_NUM_CS   CONFIG_HW_LCD_CS_GPIO
#define PIN_NUM_DC   CONFIG_HW_LCD_DC_GPIO
#define PIN_NUM_RST  CONFIG_HW_LCD_RESET_GPIO
#define PIN_NUM_BCKL CONFIG_HW_LCD_BL_GPIO
#define LCD_SEL_CMD()   GPIO.out_w1tc = (1 << PIN_NUM_DC) // Low to send command 
#define LCD_SEL_DATA()  GPIO.out_w1ts = (1 << PIN_NUM_DC) // High to send data
#define LCD_RST_SET()   GPIO.out_w1ts = (1 << PIN_NUM_RST) 
#define LCD_RST_CLR()   GPIO.out_w1tc = (1 << PIN_NUM_RST)

#if CONFIG_HW_INV_BL
#define LCD_BKG_ON()    GPIO.out_w1tc = (1 << PIN_NUM_BCKL) // Backlight ON
#define LCD_BKG_OFF()   GPIO.out_w1ts = (1 << PIN_NUM_BCKL) //Backlight OFF
#else
#define LCD_BKG_ON()    GPIO.out_w1ts = (1 << PIN_NUM_BCKL) // Backlight ON
#define LCD_BKG_OFF()   GPIO.out_w1tc = (1 << PIN_NUM_BCKL) //Backlight OFF
#endif

#define SPI_NUM  0x3

#define LCD_TYPE_ILI 0
#define LCD_TYPE_ST 1


//
//  Audio
//

typedef struct sndinfo_s
{
   int sample_rate;
   int bps;
} sndinfo_t;

static void (*audio_callback)(void *buffer, int length) = NULL;
#if CONFIG_SOUND_ENA
QueueHandle_t queue;
static uint16_t *audio_frame;
#endif

static void do_audio_frame(void) {

#if CONFIG_SOUND_ENA
	//int left=SNDRATE/SMS_FPS;
    //while(left) {
		//int n=DEFAULT_FRAGSIZE;
		int n=snd.bufsize;
		//if (n>left) n=left;
		//audio_callback(audio_frame, n); //get more data
		//16 bit mono -> 32-bit (16 bit r+l)
		for (int i=n-1; i>=0; i--) {
		//for (int i=0; i<n; i++) {
			//audio_frame[i*2+1]=audio_frame[i];
			//audio_frame[i*2]=audio_frame[i];
            audio_frame[i*2+1]=snd.buffer[1][i]; // 音声
            audio_frame[i*2]  =snd.buffer[0][i];
		}
		i2s_write_bytes(0, audio_frame, 4*(n), portMAX_DELAY);
		//left-=n;
	//}
#endif
}

void audio_frame_call(void){
    do_audio_frame();
}

void osd_setsound(void (*playfunc)(void *buffer, int length))
{
   //Indicates we should call playfunc() to get more data.
   audio_callback = playfunc;
}

static void osd_stopsound(void)
{
   audio_callback = NULL;
}


static int osd_init_sound(void)
{
#if CONFIG_SOUND_ENA
	audio_frame=malloc(4*DEFAULT_FRAGSIZE);
	i2s_config_t cfg={
		.mode=I2S_MODE_DAC_BUILT_IN|I2S_MODE_TX|I2S_MODE_MASTER,
		.sample_rate=SNDRATE,
		.bits_per_sample=I2S_BITS_PER_SAMPLE_16BIT,
		.channel_format=I2S_CHANNEL_FMT_RIGHT_LEFT,
		.communication_format=I2S_COMM_FORMAT_I2S_MSB,
		.intr_alloc_flags=0,
		.dma_buf_count=4,
		.dma_buf_len=512
	};
	i2s_driver_install(0, &cfg, 4, &queue);
	i2s_set_pin(0, NULL);
	i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN); 

	//I2S enables *both* DAC channels; we only need DAC1.
	//ToDo: still needed now I2S supports set_dac_mode?
	CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_DAC_XPD_FORCE_M);
	CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_XPD_DAC_M);

#endif

	audio_callback = NULL;
    printf("Audio Init done.\n");

	return 0;
}

int init_sound(void){
    return osd_init_sound();
}

void osd_getsoundinfo(sndinfo_t *info)
{
   info->sample_rate = SNDRATE;
   info->bps = 16;
}

static void spi_test(){
    LCD_SEL_CMD();
}

static void spi_write_byte(const uint8_t data){
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 0x7, SPI_USR_MOSI_DBITLEN_S);
    WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), data);
    SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
    while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
}
static void LCD_WriteData4(const uint32_t data){
    LCD_SEL_DATA();
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 31, SPI_USR_MOSI_DBITLEN_S);
    WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), data);
    SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
    while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
}

static void LCD_WriteCommand(const uint8_t cmd)
{
    LCD_SEL_CMD();
    spi_write_byte(cmd);
}

static void LCD_WriteData(const uint8_t data)
{
    LCD_SEL_DATA();
    spi_write_byte(data);
}

int lcdDone=1;
int pal565[32];
char *bmData;

static void  ILI9341_INITIAL ()
{
    LCD_BKG_ON();
    //-Reset Sequence-//
    LCD_RST_SET();    ets_delay_us(100000);
    LCD_RST_CLR();    ets_delay_us(200000);
    LCD_RST_SET();    ets_delay_us(200000);

#if (CONFIG_HW_LCD_TYPE == LCD_TYPE_ILI)
    //************* Start Initial Sequence **********//
    LCD_WriteCommand(0xCF); LCD_WriteData(0x00); LCD_WriteData(0x83); LCD_WriteData(0X30);
    LCD_WriteCommand(0xED); LCD_WriteData(0x64); LCD_WriteData(0x03); LCD_WriteData(0X12); LCD_WriteData(0X81);
    LCD_WriteCommand(0xE8); LCD_WriteData(0x85); LCD_WriteData(0x01); LCD_WriteData(0x79); //i
    LCD_WriteCommand(0xCB); 
       LCD_WriteData(0x39); LCD_WriteData(0x2C); LCD_WriteData(0x00); LCD_WriteData(0x34); LCD_WriteData(0x02);
    LCD_WriteCommand(0xF7); LCD_WriteData(0x20);
    LCD_WriteCommand(0xEA); LCD_WriteData(0x00); LCD_WriteData(0x00);
    LCD_WriteCommand(0xC0); LCD_WriteData(0x26); //Power control //i  //VRH[5:0]
    LCD_WriteCommand(0xC1); LCD_WriteData(0x11); //Power control //i //SAP[2:0];BT[3:0]
    LCD_WriteCommand(0xC5); LCD_WriteData(0x35); LCD_WriteData(0x3E); //VCM control //i
    LCD_WriteCommand(0xC7); LCD_WriteData(0xBE);                      //VCM control2 //i   //≫oOs B1h
    LCD_WriteCommand(0x36); LCD_WriteData(0x28); // Memory Access Control //i //was 0x48
    LCD_WriteCommand(0x3A); LCD_WriteData(0x55);
    LCD_WriteCommand(0xB1); LCD_WriteData(0x00); LCD_WriteData(0x1B); //18
    LCD_WriteCommand(0xF2); LCD_WriteData(0x08); // 3Gamma Function Disable
    LCD_WriteCommand(0x26); LCD_WriteData(0x01); // Gamma curve selected
    LCD_WriteCommand(0xE0);    //Set Gamma
       LCD_WriteData(0x1F); LCD_WriteData(0x1A); LCD_WriteData(0x18); LCD_WriteData(0x0A); LCD_WriteData(0x0F);
       LCD_WriteData(0x06); LCD_WriteData(0x45); LCD_WriteData(0X87); LCD_WriteData(0x32); LCD_WriteData(0x0A);
       LCD_WriteData(0x07); LCD_WriteData(0x02); LCD_WriteData(0x07); LCD_WriteData(0x05); LCD_WriteData(0x00);
    LCD_WriteCommand(0XE1);    //Set Gamma
       LCD_WriteData(0x00); LCD_WriteData(0x25); LCD_WriteData(0x27); LCD_WriteData(0x05); LCD_WriteData(0x10);
       LCD_WriteData(0x09); LCD_WriteData(0x3A); LCD_WriteData(0x78); LCD_WriteData(0x4D); LCD_WriteData(0x05);
       LCD_WriteData(0x18); LCD_WriteData(0x0D); LCD_WriteData(0x38); LCD_WriteData(0x3A); LCD_WriteData(0x1F);
    LCD_WriteCommand(0x2A); LCD_WriteData(0x00); LCD_WriteData(0x00); LCD_WriteData(0x00); LCD_WriteData(0xEF);
    LCD_WriteCommand(0x2B); 
       LCD_WriteData(0x00); LCD_WriteData(0x00); LCD_WriteData(0x01); LCD_WriteData(0x3f); LCD_WriteCommand(0x2C);
    LCD_WriteCommand(0xB7); LCD_WriteData(0x07); 
    LCD_WriteCommand(0xB6);    // Display Function Control
       LCD_WriteData(0x0A); LCD_WriteData(0x82); LCD_WriteData(0x27); LCD_WriteData(0x00);
    //LCD_WriteCommand(0xF6); //LCD_WriteData(0x01); //LCD_WriteData(0x30); //not there

#endif
#if (CONFIG_HW_LCD_TYPE == LCD_TYPE_ST)

//212
//122
    LCD_WriteCommand(0x36); LCD_WriteData((1<<5)|(1<<6)); //MV 1, MX 1
    LCD_WriteCommand(0x3A); LCD_WriteData(0x55);
    LCD_WriteCommand(0xB2); 
       LCD_WriteData(0x0c); LCD_WriteData(0x0c); LCD_WriteData(0x00); LCD_WriteData(0x33); LCD_WriteData(0x33);
    LCD_WriteCommand(0xB7); LCD_WriteData(0x35);
    LCD_WriteCommand(0xBB); LCD_WriteData(0x2B);
    LCD_WriteCommand(0xC0); LCD_WriteData(0x2C);
    LCD_WriteCommand(0xC2); LCD_WriteData(0x01); LCD_WriteData(0xFF);
    LCD_WriteCommand(0xC3); LCD_WriteData(0x11);
    LCD_WriteCommand(0xC4); LCD_WriteData(0x20);
    LCD_WriteCommand(0xC6); LCD_WriteData(0x0f);
    LCD_WriteCommand(0xD0); LCD_WriteData(0xA4); LCD_WriteData(0xA1);
    LCD_WriteCommand(0xE0);
       LCD_WriteData(0xD0); LCD_WriteData(0x00); LCD_WriteData(0x05); LCD_WriteData(0x0E); LCD_WriteData(0x15);
       LCD_WriteData(0x0D); LCD_WriteData(0x37); LCD_WriteData(0x43); LCD_WriteData(0x47); LCD_WriteData(0x09);
       LCD_WriteData(0x15); LCD_WriteData(0x12); LCD_WriteData(0x16); LCD_WriteData(0x19);
    LCD_WriteCommand(0xE1);
       LCD_WriteData(0xD0); LCD_WriteData(0x00); LCD_WriteData(0x05); LCD_WriteData(0x0D); LCD_WriteData(0x0C);
       LCD_WriteData(0x06); LCD_WriteData(0x2D); LCD_WriteData(0x44); LCD_WriteData(0x40); LCD_WriteData(0x0E);
       LCD_WriteData(0x1C); LCD_WriteData(0x18); LCD_WriteData(0x16); LCD_WriteData(0x19);

#endif
    LCD_WriteCommand(0x11); ets_delay_us(100000);   //Exit Sleep
    LCD_WriteCommand(0x29); ets_delay_us(100000);   //Display on
}
//.............LCD API END----------

static void ili_gpio_init()
{
    gpio_set_direction(PIN_NUM_DC  , GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST , GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
}

static void spi_master_init()
{
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=20*1000*1000,           //Clock out at 20/10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        //.pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    
    spi_bus_initialize(VSPI_HOST, &buscfg, 1);    //Initialize the SPI bus
    spi_bus_add_device(VSPI_HOST, &devcfg, &spi); //Attach the LCD to the SPI bus

    SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_CS_SETUP | SPI_CS_HOLD | SPI_USR_MOSI);
    SET_PERI_REG_MASK(SPI_CTRL2_REG(SPI_NUM), ((0x4 & SPI_MISO_DELAY_NUM) << SPI_MISO_DELAY_NUM_S));
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_COMMAND);
    SET_PERI_REG_BITS(SPI_USER2_REG(SPI_NUM), SPI_USR_COMMAND_BITLEN, 0, SPI_USR_COMMAND_BITLEN_S);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_ADDR);
    SET_PERI_REG_BITS(SPI_USER1_REG(SPI_NUM), SPI_USR_ADDR_BITLEN, 0, SPI_USR_ADDR_BITLEN_S);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_MISO);
    SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_MOSI);
}

#define U16x2toU32(m,l) ((((uint32_t)(l>>8|(l&0xFF)<<8))<<16)|(m>>8|(m&0xFF)<<8))

void ili9341_write_frame(const uint16_t xs, const uint16_t ys, const uint16_t width, const uint16_t height, const uint8_t * data[]){
    int x, y;
    int i;
    uint16_t x1, y1;
    uint32_t xv, yv, dc;
    uint32_t temp[16];
    dc = (1 << PIN_NUM_DC);
    // sms Color Palette
   	for (x=0; x<32; x++) {
		pal565[x]=((bitmap.pal.color[x][0]>>3)<<11)|((bitmap.pal.color[x][1]>>2)<<5)|(bitmap.pal.color[x][2]>>3);
	}
	bmData=bitmap.data;
    // LCD Send Header
    x1 = xs+(width-1);       y1 = ys+0+(height-1);
    xv = U16x2toU32(xs,x1);  yv = U16x2toU32((ys+0),y1);
    while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
    LCD_WriteCommand(0x2A); LCD_WriteData4(xv);
    LCD_WriteCommand(0x2B); LCD_WriteData4(yv);
    LCD_WriteCommand(0x2C);
    GPIO.out_w1ts = dc;
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 511, SPI_USR_MOSI_DBITLEN_S);
    // LCD Send Data
    for (y=0; y<height; y++) {
        x = 0;
        while (x<width) {
            while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
            for (i=0; i<16; i++) {
                if(data == NULL){
                    if(height==239) { x1 = 0xf800; y1 = x1 ; } // Red
                    if(height==240) { x1 = 0x0007; y1 = x1 ; } // Blue
                } else {
                    x1 = pal565[(*bmData++)&31];
                    y1 = pal565[(*bmData++)&31];
                }
                x += 2;
                WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (i << 2)),U16x2toU32(x1,y1));
            }
            SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        }
    }
    while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
}

void ili9341_init()
{
    spi_master_init();
    ili_gpio_init();
    ILI9341_INITIAL ();

    //xt_set_interrupt_handler(XCHAL_TIMER_INTERRUPT(1), lcdPumpPixels, NULL);
    // //xt_ints_on(1<<XCHAL_TIMER_INTERRUPT(1));
	lcdDone=1;

	printf("LCD Init done.\n");
}

