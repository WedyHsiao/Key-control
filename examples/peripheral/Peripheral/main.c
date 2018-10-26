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
 
 /**wedy claim
 *@brief this development is for the multiple functions with below:
 *Function_BUTTON
 *Function_ADC
 *Function_VIBRATOR
 *Function_PWM
 *Function_EVENT
 *please use above symbol to define the function
 *each time re-define must rebuild the program then load
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h" // serial port display
#include "app_error.h" //for UART notify
#include "nrf_delay.h" //time delay
#include "nrf_gpio.h" //gpio control and setting
#include "nrf.h" //identify the software
#include "bsp.h" //button for functionally module
#include "app_timer.h" //add for the event control
#include "nrf_drv_gpiote.h" //gpiote control
#include "nrf_drv_clock.h" //add for the FCLK for event
#include "app_button.h" //event control 

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

/*PWM definitions*/
uint32_t Data_To_Timer[]={100000,900000,1000000};

/*Eevent definition*/
#define APP_TIMER_PRESCALER 0 /**< Value of the RTC1 PRESCALER register. */
#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)
#define APP_TIMER_OP_QUEUE_SIZE 4 /**< Size of timer operation queues. */


////////////////////////////////////End fo Define///////////////////////////

////////////////////////////////////////////UART//////////////////
/*Wedy add UART control*/
/*baudrate=115200, 8 component(view in the content)*/
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
/*vibrating counts*/
#if Function_VIBRATOR
static void VIBRATOR_Init(void)    //set the pin control
{
 //simulate for vibrator power consumption
 nrf_gpio_cfg_output(Vibrator);   //set vibrator	
 nrf_gpio_pin_clear(Vibrator);
	
}
//////////////////VIBRATOR setting/////////////////////////////////////

//////////////////PWM content/////////////////////////////////////
/*Wedy add PWM program trying
*Step 1. set the GPIO
*Setp 2. set the timer and value the CC[0](empty)&CC[1](cycle) then start the timer
*Step 3. enable and set to interrupt the CC[1], setting the priority
*Step 4. set the module of GPIOTE
*Step 5. set the PPI module
*Step 6. Do in the wihil(1)
********************************/
#elif Function_PWM
static void PWM_Init(void) // initial the pin for PWM experiment (GPIO intial)
{
	nrf_gpio_cfg_output(LED_3);                            //LED3 for PWM testing
 nrf_gpio_pin_set(LED_3);
}

/*There is no PWM module int the nRF5X series
*thus we use timer to achieve the PWM function*/ 
static void Timer_Init(void)				//timer intial
{
 NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;               //mode of counting
 NRF_TIMER0->PRESCALER = 4;                              //4/1MHZ
 NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;      //32bit for counting
 NRF_TIMER0->CC[0]=100000;                               //CC[0]=100000
 NRF_TIMER0->CC[1]=1000000;															//CC[1] (cycle) must > CC[0] to simulate a pulse
	
 NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE1_Msk ;  //compare to inturrupt
 NVIC_SetPriority(TIMER0_IRQn,3);                      //the priority for inturrupt with timer
 NVIC_EnableIRQ(TIMER0_IRQn);                          //enable to inturrupt the timer
 NRF_TIMER0->TASKS_START =1;                           //start the timer
}

static void PPI_Init(void)                             //PPI module intial (Programmable Peripheral Interconnect)
{
	//enhance the response time
 NRF_PPI->CH[0].EEP = (uint32_t)(&NRF_TIMER0->EVENTS_COMPARE[0]);  
 NRF_PPI->CH[0].TEP = (uint32_t)(&NRF_GPIOTE->TASKS_OUT[0]);
 NRF_PPI->CH[1].EEP = (uint32_t)(&NRF_TIMER0->EVENTS_COMPARE[1]);
 NRF_PPI->CH[1].TEP = (uint32_t)(&NRF_GPIOTE->TASKS_OUT[0]);
 //use channel 0 and channel 1 to control the mode
 NRF_PPI->CHEN = (PPI_CHEN_CH0_Enabled<<PPI_CHEN_CH0_Pos)|(PPI_CHEN_CH1_Enabled<<PPI_CHEN_CH1_Pos); 
}

static void GPIOTE_Init(void)                         //GPIOTE intial
{
 //TASK to turns the LED3 which means to transver the status H/L
 NRF_GPIOTE->CONFIG[0]=(GPIOTE_CONFIG_MODE_Task<<GPIOTE_CONFIG_MODE_Pos) |
 (GPIOTE_CONFIG_OUTINIT_High<<GPIOTE_CONFIG_OUTINIT_Pos)|
	(GPIOTE_CONFIG_POLARITY_Toggle<<GPIOTE_CONFIG_POLARITY_Pos)|
	(LED_3<<GPIOTE_CONFIG_PSEL_Pos);
}


void TIMER0_IRQHandler(void)                       //service for interrupt TIMER                      
{
 static uint8_t int_counter=0;
 int_counter++;
 if(NRF_TIMER0->EVENTS_COMPARE[1] != 0)                   
 {
    NRF_TIMER0->EVENTS_COMPARE[1]=0;              //interrupt to clear
	  NRF_TIMER0->TASKS_CLEAR= 1;                   //clear the timer ->0
 }
  if(int_counter==3)
	{
	 int_counter=0;
	}
  NRF_TIMER0->CC[0]=Data_To_Timer[int_counter];   //CC[0] get the data again
}
//////////////////PWM content/////////////////////////////////////

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
#elif Function_BUTTON
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
////////////////////////////Button content///////////////////////////

////////////////////////////EVENT event control content///////////////////////////
#elif Function_EVENT
/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
 
void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
				
				case BSP_EVENT_KEY_0:
					printf("Hellow_I'm button_1");
        break;
				
				case BSP_EVENT_KEY_1:
					printf("Hi_This is button_2");
        break;
				
				case BSP_EVENT_KEY_2:
					printf("is long press");
        break;

				case BSP_EVENT_KEY_3:
					printf("release again");
        break;
				
        default:
            break;
    }
}

static void buttons_init(void)
{

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 BUTTON_DETECTION_DELAY, 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

}					


////////////////////////////EVENT event control content///////////////////////////
#else
#error " please idenfify a function!"
#endif


/*UART content */
static void UART_Init(void)
{	
	uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_ENABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200 //the baudrate is set
      };

		
    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);			
}
/*UART content End*/

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
	UART_Init();
	uint32_t err_code;
	
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
		
#elif Function_VIBRATOR  //Wedy vibrator power consumption test
		VIBRATOR_Init();
		for(int i=1;i<=100000;i++)
		{
			int tick=0;
			tick+=i;
			printf("%d\r\r", i);
			nrf_gpio_pin_toggle(Vibrator);
			//nrf_gpio_pin_write(Vibrator, 0);
			nrf_delay_ms(500);
			//nrf_gpio_pin_write(Vibrator, 1);
		}
		
#elif Function_PWM //Wedy do the PWM test
NRF_CLOCK ->EVENTS_HFCLKSTARTED = 0;             //start the main timer
 NRF_CLOCK ->TASKS_HFCLKSTART = 1;
 while(NRF_CLOCK ->EVENTS_HFCLKSTARTED ==0)
 {} 
 PWM_Init();                                     //PWM GPIO intial
 Timer_Init();                                    //TIMER0 intial
 GPIOTE_Init();                                   //GPIOTE intial
 PPI_Init();                                      //PPI intial
 while(1);
	 
#elif Function_BUTTON //Wedy button control to light up the LED
		BUTTON_Init();
		while(1);
		
#elif Function_EVENT
		
		printf("\r\n Event Start!\r\n");
		
		//app_button uses app_timer, if this is not initialize, then initialize it here
		APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
		
		err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
       
		bsp_buttons_enable();
		
		buttons_init();
		bsp_event_to_button_action_assign(0, BSP_BUTTON_ACTION_PUSH, BSP_EVENT_KEY_0);
		bsp_event_to_button_action_assign(1, BSP_BUTTON_ACTION_PUSH, BSP_EVENT_KEY_1);
		bsp_event_to_button_action_assign(2, BSP_BUTTON_ACTION_LONG_PUSH, BSP_EVENT_KEY_2);
		bsp_event_to_button_action_assign(3, BSP_BUTTON_ACTION_RELEASE, BSP_EVENT_KEY_3);
		
		
		
		for(;;)
		{
		}

		
#else
#error	"Function is not defined, please define the preprocessor symbol in the file! "	
#endif
	}


/** @} */
