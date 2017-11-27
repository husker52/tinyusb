#include <stdio.h>
#include <stdint.h>
#include "MKL25Z4.h"
#include "tusb.h"

#ifdef __cplusplus
#define CDECL extern "C" 
#else
#define CDECL 
#endif

struct gpio_map_st{
	FGPIO_Type pta;
	char unused_a[0x40-sizeof(FGPIO_Type)];
	FGPIO_Type ptb; 
	char unused_b[0x40-sizeof(FGPIO_Type)];
	FGPIO_Type ptc; 
	char unused_c[0x40-sizeof(FGPIO_Type)];
	FGPIO_Type ptd; 
	char unused_d[0x40-sizeof(FGPIO_Type)];
	FGPIO_Type pte; 
};

struct gpio_map_st __attribute__((section (".periph.gpio"))) gpio;

//struct aips_map_st __attribute__((section (".aips"))) aips_map;
SIM_Type __attribute__((section(".periph.sim"))) sim;
PORT_Type __attribute__((section(".periph.porta"))) porta;
PORT_Type __attribute__((section(".periph.portb"))) portb;
PORT_Type __attribute__((section(".periph.portc"))) portc;
PORT_Type __attribute__((section(".periph.portd"))) portd;
PORT_Type __attribute__((section(".periph.porte"))) porte;
I2C_Type __attribute__((section(".periph.i2c0"))) i2c0;
TPM_Type __attribute__((section(".periph.tpm0"))) tpm0;
TPM_Type __attribute__((section(".periph.tpm1"))) tpm1;
TPM_Type __attribute__((section(".periph.tpm2"))) tpm2;
MCG_Type __attribute__((section(".periph.mcg"))) mcg;
USB_Type __attribute__((section(".periph.usb"))) usb;


uint32_t mySysTick_Config(uint32_t ticks);
extern void put_char(char c);

volatile uint32_t msTicks = 0;
volatile uint32_t mytick = 0;

void toggleLED(){
	static uint8_t count = 0;
	
	count++;
	if(count > 2) count = 0;

	switch(count){
		case 0:
			FPTB->PTOR = (0x1 << 18);
			break;
		case 1:
			FPTB->PTOR = (0x1 << 19);
			break;
		case 2:
			FPTD->PTOR = (0x1 << 1);
			break;
	}
}

CDECL void SysTick_Handler(void) {
	    msTicks++;
	    if(msTicks == 1000){
		    msTicks = 0;
		    toggleLED();
	    }
	    mytick++;
}

uint32_t tusb_tick_get(void){
	return msTicks;
}

CDECL void init_ioports(){
	// configure GPIO ports
	FPTB->PDDR = (0x1 << 18) | (0x1 << 19);
	FPTD->PDDR = (0x1 << 1);
	// turn on clock for GPIO
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | \
		      SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK ;
	// set pin control for GPIO
	PORTB->PCR[18] |= (0x1 << 8);
	PORTB->PCR[19] |= (0x1 << 8);
	PORTD->PCR[1] |= (0x1 << 8);

	// turn on clock for I2C0
	SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK; // (0x1 << 6);
	// set pin control for I2C0
	PORTE->PCR[24] = (5 << 8);
	PORTE->PCR[25] = (5 << 8);
	// set pin control for TPM1 chan 0
	PORTE->PCR[20] = (3 << 8); // mux setting for tpm1
//	PORTE->PCR[20] = (1 << 8);  // mux setting for gpio
	FPTE->PDDR = (0x1 << 20);
	// set pin control for i2c1
	//PORTC->PCR[10] = (2 << 8);
	//PORTC->PCR[11] = (2 << 8);
}

#define PIXDATA 1

void gpio_toggle(uint16_t pin){
switch(pin){
	case PIXDATA:
		FPTE->PTOR = (0x1 << 20);
		break;
	default:
		break;
}

}


int main(){
	char test[100];
	int length,i;
	uint8_t data;
	uint32_t last_tick;
	uint16_t xval,yval,zval,status;

	SysTick_Config(SystemCoreClock/1000);
	init_ioports();
	tusb_init();

//	length = sprintf(test,"got here\n");

//	for(i=0;i<length;i++){
//		put_char(test[i]);
//	}
//	tpm_program();
	

//	while(1){
//		tpm_delayticks(tpm,30);
//		gpio_toggle(PIXDATA);
//
//	}


	last_tick = mytick + 1000;
	while(1){
		if(mytick == last_tick){
		}
	}
	return 0;
}

CDECL void HardFault_Handler(void) {
	while (1);

}
