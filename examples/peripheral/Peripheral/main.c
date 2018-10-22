/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 * 
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf.h"
#include "bsp.h"

////////////////////////////////////////////Define///////////////////////////
/*ADC definitions*/
#define ADC_PIN  1      //set the ADC pin {AIN}=2

/* Button definitions */
const uint8_t buttons_list[BUTTONS_NUMBER]={BUTTON_1,BUTTON_2,BUTTON_3,BUTTON_4};   //there are three buttons in use
const uint8_t leds_list[LEDS_NUMBER]={LED_1,LED_2,LED_3,LED_4}; // the LED in use

/*UART definitions*/
//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */
#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */

////////////////////////////////////End fo Define///////////////////////////

////////////////////////////////////////////UART//////////////////
/*Wedy add UART control*/
void uart_error_handle(app_uart_evt_t * p_event)
{
 if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
 {
 APP_ERROR_HANDLER(p_event->data.error_communication);
 }
 else if (p_event->evt_type == APP_UART_FIFO_ERROR)
 {
  APP_ERROR_HANDLER(p_event->data.error_code);
 }
}
////////////////////////////////////////////UART//////////////////

//////////////////VIBRATOR setting/////////////////////////////////////
#if Function_Vibrator
static void VIBRATOR_Init(void)    //set the pin control
{
 //simulate for vibrator power consumption
 nrf_gpio_cfg_output(Vibrator);   //set vibrator	
 nrf_gpio_pin_clear(Vibrator);
	
}
//////////////////VIBRATOR setting/////////////////////////////////////


//////////////////ADC content///////////////////////////
/*Wedy add ADC control*/
#elif Function_ADC
static void ADC_Start(void)    //ÆôADC control
{
 NRF_ADC->ENABLE =1;						
 NRF_ADC->TASKS_START = 1;     // enable
}

static void ADC_Stop(void)     //stop ADC
{
 NRF_ADC->ENABLE = 0;						//disable
 NRF_ADC->TASKS_STOP = 1;
}


static void ADC_Init(void)     //ADC intial
{
	nrf_gpio_cfg_output(LED_2);   //set LED_2	
	nrf_gpio_pin_clear(LED_2);
	
 //AIN[2],1.2V reference voltage,10bit=1024 //reference manual p165
 NRF_ADC->CONFIG= (ADC_CONFIG_EXTREFSEL_None<<ADC_CONFIG_EXTREFSEL_Pos) 
					 |(ADC_CONFIG_PSEL_AnalogInput2<<ADC_CONFIG_PSEL_Pos) 
	         |(ADC_CONFIG_REFSEL_VBG<<ADC_CONFIG_REFSEL_Pos) 
					 |(ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling<<ADC_CONFIG_INPSEL_Pos)
	         |(ADC_CONFIG_RES_10bit<<ADC_CONFIG_RES_Pos );
}
//////////////////ADC content///////////////////////////

////////////////////////////Button content///////////////////////////
/*Wedy add butoon control*/
#elif Function_Button
static void BUTTON_Init(void)                      //button intial
{
 uint8_t i;
 nrf_gpio_range_cfg_output(LED_START,LED_STOP);  //LED as output
	
 for(i=0;i<LEDS_NUMBER;i++)											//set the LED pin 
 {
 nrf_gpio_pin_set(leds_list[i]);
 }

 for(i=0;i<BUTTONS_NUMBER;i++)                  //it can pull up the LED when the button is pressed
 {
  NRF_GPIO->PIN_CNF[buttons_list[i]]=
	(GPIO_PIN_CNF_DIR_Input<<GPIO_PIN_CNF_DIR_Pos) | 
	(GPIO_PIN_CNF_INPUT_Connect<<GPIO_PIN_CNF_INPUT_Pos)|
	(GPIO_PIN_CNF_PULL_Pullup<<GPIO_PIN_CNF_PULL_Pos) |
	(GPIO_PIN_CNF_DRIVE_S0S1<<GPIO_PIN_CNF_DRIVE_Pos) |
	(GPIO_PIN_CNF_SENSE_Low<<GPIO_PIN_CNF_SENSE_Pos);
 }
 
 NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;  //interrupt
 NVIC_SetPriority(GPIOTE_IRQn,3);                  //set the priority with 3
 NVIC_EnableIRQ(GPIOTE_IRQn);                      //enable to interrupt
}

void GPIOTE_IRQHandler(void)                       //GPIOTE interrupt service                     
{
 if(NRF_GPIOTE->EVENTS_PORT !=0)                   
 {
    NRF_GPIOTE->EVENTS_PORT=0;                     //clear
	  for(uint8_t i=0;i<BUTTONS_NUMBER;i++)          //read the button situation
	  {
		 if(nrf_gpio_pin_read(buttons_list[i])==0)
		 {
		  nrf_delay_ms(10);                            //set the time to read
		  if(nrf_gpio_pin_read(buttons_list[i])==0)
		  {
			 nrf_gpio_pin_toggle(leds_list[i]);          // turns the LED
			}
		 }
		}
 }
}
#else
#error " please idenfify a function!"
#endif
////////////////////////////Button content///////////////////////////


/*******
*******
******
*****
***
**
 * @brief Function for main application entry.
 */
int main(void)                 
{
/*UART content */	
	uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_ENABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200
      };

		
    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);
/*UART content End*/			

#if Function_ADC		//Wedy ADC from GPIO pin_1			
		ADC_Init(); 	
		LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
    while (true)
    {
			float result;   //the type of the data     
		  ADC_Start();    //ÆôStart to ADC
      while(! NRF_ADC->EVENTS_END){};  
			result = (NRF_ADC->RESULT)*1.0;  //ADC= (Vin/3)*1024/1.2
			result=result*1.2/1024.0;         //V'= ADC*1.2/1024
			result *=3;												//Vin= V'*3
			printf("ADC VALUE %f\r\r",result);
			nrf_delay_ms(500);
			nrf_gpio_pin_toggle(LED_2);
    }
#elif Function_Vibrator  //Wedy vibrator power consumption test
		VIBRATOR_Init();
		for(int i=1;i<=100000;i++)
		{
			int tick=0;
			tick+=i;
			printf("%d\r\r", i);
			nrf_gpio_pin_toggle(Vibrator);
			nrf_delay_ms(500);
			//nrf_gpio_pin_write(Vibrator, 1);
		}
#elif Function_Button //Wedy button control to light up the LED
		BUTTON_Init();
		while(1);
#else
#error	"Function is not defined, please define the preprocessor symbol in the file! "	
#endif
	}


/** @} */
