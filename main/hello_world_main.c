/* MCPWM basic config example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * This example will show you how to use each submodule of MCPWM unit.
 * The example can't be used without modifying the code first.
 * Edit the macros at the top of mcpwm_example_basic_config.c to enable/disable the submodules which are used in the example.
 */

#include <stdio.h>
#include <unistd.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "soc/rtc.h"
#include "esp_log.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "soc/mcpwm_periph.h"
#include <math.h>
#include <time.h>

#define MCPWM_EN_CARRIER 0   //Make this 1 to test carrier submodule of mcpwm, set high frequency carrier parameters
#define MCPWM_EN_DEADTIME 1  //Make this 1 to test deadtime submodule of mcpwm, set deadtime value and deadtime mode
#define MCPWM_EN_FAULT 0     //Make this 1 to test fault submodule of mcpwm, set action on MCPWM signal on fault occurence like overcurrent, overvoltage, etc
#define MCPWM_EN_SYNC 1      //Make this 1 to test sync submodule of mcpwm, sync timer signals
#define MCPWM_EN_CAPTURE 0   //Make this 1 to test capture submodule of mcpwm, measure time between rising/falling edge of captured signal
#define MCPWM_GPIO_INIT 0    //select which function to use to initialize gpio signals
#define CAP_SIG_NUM 3   //Three capture signals
#define BLDC_SPEED_UPDATE_PERIOD_US    200  // 1segundo
#define BLDC_SPEED_UPDATE_PERIOD_US_RAMPA    1000  // 1segundo
#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28)  //Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29)  //Capture 2 interrupt bit
#define TIMER_DIVIDER   (1)

#define GPIO_PWM0A_OUT 23   //Set GPIO 19 as PWM0A
#define GPIO_PWM0B_OUT 22   //Set GPIO 18 as PWM0B

#define GPIO_PWM1A_OUT 21   //Set GPIO 17 as PWM1A
#define GPIO_PWM1B_OUT 19   //Set GPIO 16 as PWM1B

#define GPIO_PWM2A_OUT 32   //Set GPIO 15 as PWM2A
#define GPIO_PWM2B_OUT 33   //Set GPIO 14 as PWM2B

#define GPIO_CAP0_IN   36   //Set GPIO 23 as  CAP0
#define GPIO_CAP1_IN   25   //Set GPIO 25 as  CAP1
#define GPIO_CAP2_IN   26   //Set GPIO 26 as  CAP2
#define GPIO_SYNC0_IN   2   //Set GPIO 02 as SYNC0
#define GPIO_SYNC1_IN   4   //Set GPIO 04 as SYNC1
#define GPIO_SYNC2_IN   5   //Set GPIO 05 as SYNC2
#define GPIO_FAULT0_IN 32   //Set GPIO 32 as FAULT0
#define GPIO_FAULT1_IN 34   //Set GPIO 34 as FAULT1
#define GPIO_FAULT2_IN 34   //Set GPIO 34 as FAULT2




#define SPWM			1
#define SVPWM			2
#define THIPWM_FOUR		3
#define THIPWM_SIX		4
#define DPWMMAX			5
#define DPWMMIN			6
#define DPWM0			7
#define DPWM1			8
#define DPWM2			9
#define DPWM3			10

#define INDEX_FREQ 		1
#define INDEX_AMPL		2
#define INDEX_MODL		3
#define INDEX_RAMP		4
#define INDEX_TMRA		5

#define	DIVISIONS_RAMPA 20

#define arg_angle		0
#define arg_amplitude	1
#define arg_modulation	2

#define delta_fr_rmp        0
#define freq_final_rmp 		1
#define delta_ampl_rmp 		2
#define ampl_final_rmp 		3
#define ramp_index			4



#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)


static const int RX_BUF_SIZE = 1024;
void gen_SPWM(int angle, float amplitude);
void gen_SVPWM(int angle, float amplitude);
void gen_THIPWM(int angle, float amplitude, float amplitude_armonic);
void gen_DPWMMAX(int angle, float amplitude);
void gen_DPWMMIN(int angle, float amplitude);
void gen_DPWMM(int angle, float amplitude, int type_modulation);
float gen_triangle(int angle, int amplitude);
float find_time_couter_ramap(float time_rampa);
void config_comand_esp(float freq, float ampl, int modul, int ramp, float time_ramp);
void update_rampa(float * rampa_args);

int args[3];
float rampa_args[5];
float delta_freq = 0.0;

esp_timer_handle_t periodic_timer_rampa = NULL;
const esp_timer_create_args_t periodic_timer_args_rampa = {
        .callback = update_rampa,
        .arg = rampa_args,
    };


typedef struct {
    uint32_t capture_signal;
    mcpwm_capture_signal_t sel_cap_signal;
} capture;

uint32_t *current_cap_value = NULL;
uint32_t *previous_cap_value = NULL;

/////////////////////////////////////////// GLOBAL VARIABLES ///////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////////////

//xQueueHandle cap_queue;
#if MCPWM_EN_CAPTURE
static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};
#endif

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
#if MCPWM_GPIO_INIT
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, GPIO_PWM2A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, GPIO_PWM2B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP0_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, GPIO_CAP1_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_2, GPIO_CAP2_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_SYNC_0, GPIO_SYNC0_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_SYNC_1, GPIO_SYNC1_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_SYNC_2, GPIO_SYNC2_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_FAULT_0, GPIO_FAULT0_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_FAULT_1, GPIO_FAULT1_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_FAULT_2, GPIO_FAULT2_IN);
#else
    mcpwm_pin_config_t pin_config = {
        .mcpwm0a_out_num = GPIO_PWM0A_OUT,
        .mcpwm0b_out_num = GPIO_PWM0B_OUT,
        .mcpwm1a_out_num = GPIO_PWM1A_OUT,
        .mcpwm1b_out_num = GPIO_PWM1B_OUT,
        .mcpwm2a_out_num = GPIO_PWM2A_OUT,
        .mcpwm2b_out_num = GPIO_PWM2B_OUT,
        .mcpwm_sync0_in_num  = GPIO_SYNC0_IN,
        .mcpwm_sync1_in_num  = GPIO_SYNC1_IN,
        .mcpwm_sync2_in_num  = GPIO_SYNC2_IN,
        .mcpwm_fault0_in_num = GPIO_FAULT0_IN,
        .mcpwm_fault1_in_num = GPIO_FAULT1_IN,
        .mcpwm_fault2_in_num = GPIO_FAULT2_IN,
        .mcpwm_cap0_in_num   = GPIO_CAP0_IN,
        .mcpwm_cap1_in_num   = GPIO_CAP1_IN,
        .mcpwm_cap2_in_num   = GPIO_CAP2_IN
    };
    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);
#endif
    gpio_pulldown_en(GPIO_CAP0_IN);    //Enable pull down on CAP0   signal
    gpio_pulldown_en(GPIO_CAP1_IN);    //Enable pull down on CAP1   signal
    gpio_pulldown_en(GPIO_CAP2_IN);    //Enable pull down on CAP2   signal
    gpio_pulldown_en(GPIO_SYNC0_IN);   //Enable pull down on SYNC0  signal
    gpio_pulldown_en(GPIO_SYNC1_IN);   //Enable pull down on SYNC1  signal
    gpio_pulldown_en(GPIO_SYNC2_IN);   //Enable pull down on SYNC2  signal
    gpio_pulldown_en(GPIO_FAULT0_IN);  //Enable pull down on FAULT0 signal
    gpio_pulldown_en(GPIO_FAULT1_IN);  //Enable pull down on FAULT1 signal
    gpio_pulldown_en(GPIO_FAULT2_IN);  //Enable pull down on FAULT2 signal
}

/**
 * @brief Set gpio 12 as our test signal that generates high-low waveform continuously, connect this gpio to capture pin.
 */
static void gpio_test_signal(void *arg)
{
    printf("intializing test signal...\n");
//    gpio_config_t gp;
//    gp.intr_type = GPIO_INTR_DISABLE;
//    gp.mode = GPIO_MODE_OUTPUT;
    //gp.pin_bit_mask = GPIO_SEL_12;

//    gpio_config(&gp);
    while (1) {
        //here the period of test signal is 20ms
//        gpio_set_level(GPIO_NUM_12, 1); //Set high
        vTaskDelay(10);             //delay of 10ms
//        gpio_set_level(GPIO_NUM_12, 0); //Set low
        vTaskDelay(10);         //delay of 10ms
    }
}

/**
 * @brief When interrupt occurs, we receive the counter value and display the time between two rising edge
 */
static void disp_captured_signal(void *arg)
{
    capture evt;
    while (1) {
    	printf("10");
    	sleep(10);
    }
}

#if MCPWM_EN_CAPTURE
/**
 * @brief this is ISR handler function, here we check for interrupt that triggers rising edge on CAP0 signal and according take action
 */
static void IRAM_ATTR isr_handler(void)
{
    uint32_t mcpwm_intr_status;
    capture evt;
    mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val; //Read interrupt status
    //calculate the interval in the ISR,
    //so that the interval will be always correct even when cap_queue is not handled in time and overflow.
    if (mcpwm_intr_status & CAP0_INT_EN) { //Check for interrupt on rising edge on CAP0 signal
        current_cap_value[0] = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0); //get capture signal counter value
        evt.capture_signal = (current_cap_value[0] - previous_cap_value[0]) / (rtc_clk_apb_freq_get() / 1000000);
        previous_cap_value[0] = current_cap_value[0];
        evt.sel_cap_signal = MCPWM_SELECT_CAP0;
        xQueueSendFromISR(cap_queue, &evt, NULL);
    }
    if (mcpwm_intr_status & CAP1_INT_EN) { //Check for interrupt on rising edge on CAP0 signal
        current_cap_value[1] = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP1); //get capture signal counter value
        evt.capture_signal = (current_cap_value[1] - previous_cap_value[1]) / (rtc_clk_apb_freq_get() / 1000000);
        previous_cap_value[1] = current_cap_value[1];
        evt.sel_cap_signal = MCPWM_SELECT_CAP1;
        xQueueSendFromISR(cap_queue, &evt, NULL);
    }
    if (mcpwm_intr_status & CAP2_INT_EN) { //Check for interrupt on rising edge on CAP0 signal
        current_cap_value[2] = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP2); //get capture signal counter value
        evt.capture_signal = (current_cap_value[2] - previous_cap_value[2]) / (rtc_clk_apb_freq_get() / 1000000);
        previous_cap_value[2] = current_cap_value[2];
        evt.sel_cap_signal = MCPWM_SELECT_CAP2;
        xQueueSendFromISR(cap_queue, &evt, NULL);
    }
    MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status;
}
#endif

/**
 * @brief Configure whole MCPWM module
 */
static void mcpwm_example_config(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initialize mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000;    //frequency = 1000Hz
    pwm_config.cmpr_a = 50.0;       //duty cycle of PWMxA = 60.0%
    pwm_config.cmpr_b = 50.0;       //duty cycle of PWMxb = 50.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
    pwm_config.frequency = 20000;     //frequency = 500Hz
    pwm_config.cmpr_a = 50.0;       //duty cycle of PWMxA = 45.9%
    pwm_config.cmpr_b = 50;    //duty cycle of PWMxb = 07.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);   //Configure PWM1A & PWM1B with above settings
    pwm_config.frequency = 20000;     //frequency = 400Hz
    pwm_config.cmpr_a = 50;       //duty cycle of PWMxA = 23.2%
    pwm_config.cmpr_b = 50;       //duty cycle of PWMxb = 97.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER; //frequency is half when up down count mode is set i.e. SYMMETRIC PWM
    pwm_config.duty_mode = MCPWM_DUTY_MODE_1;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);   //Configure PWM2A & PWM2B with above settings

#if MCPWM_EN_CARRIER
    //3. carrier configuration
    //comment if you don't want to use carrier mode
    //in carrier mode very high frequency carrier signal is generated at mcpwm high level signal
    mcpwm_carrier_config_t chop_config;
    chop_config.carrier_period = 6;         //carrier period = (6 + 1)*800ns
    chop_config.carrier_duty = 3;           //carrier duty = (3)*12.5%
    chop_config.carrier_os_mode = MCPWM_ONESHOT_MODE_EN; //If one shot mode is enabled then set pulse width, if disabled no need to set pulse width
    chop_config.pulse_width_in_os = 3;      //first pulse width = (3 + 1)*carrier_period
    chop_config.carrier_ivt_mode = MCPWM_CARRIER_OUT_IVT_EN; //output signal inversion enable
    mcpwm_carrier_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &chop_config);  //Enable carrier on PWM2A and PWM2B with above settings
    //use mcpwm_carrier_disable function to disable carrier on mcpwm timer on which it was enabled
#endif

#if MCPWM_EN_DEADTIME
    //4. deadtime configuration
    //comment if you don't want to use deadtime submodule
    //add rising edge delay or falling edge delay. There are 8 different types, each explained in mcpwm_deadtime_type_t in mcpwm.h
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 10, 10);   //Enable deadtime on PWM2A and PWM2B with red = (1000)*100ns on PWM2A
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 10, 10);        //Enable deadtime on PWM1A and PWM1B with fed = (2000)*100ns on PWM1B
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 10, 10);  //Enable deadtime on PWM0A and PWM0B with red = (656)*100ns & fed = (67)*100ns on PWM0A and PWM0B generated from PWM0A
    //use mcpwm_deadtime_disable function to disable deadtime on mcpwm timer on which it was enabled
#endif

#if MCPWM_EN_FAULT
    //5. enable fault condition
    //comment if you don't want to use fault submodule, also u can comment the fault gpio signals
    //whenever fault occurs you can configure mcpwm signal to either force low, force high or toggle.
    //in cycmode, as soon as fault condition is over, the mcpwm signal is resumed, whereas in oneshot mode you need to reset.
    mcpwm_fault_init(MCPWM_UNIT_0, MCPWM_HIGH_LEVEL_TGR, MCPWM_SELECT_F0); //Enable FAULT, when high level occurs on FAULT0 signal
    mcpwm_fault_init(MCPWM_UNIT_0, MCPWM_HIGH_LEVEL_TGR, MCPWM_SELECT_F1); //Enable FAULT, when high level occurs on FAULT1 signal
    mcpwm_fault_init(MCPWM_UNIT_0, MCPWM_HIGH_LEVEL_TGR, MCPWM_SELECT_F2); //Enable FAULT, when high level occurs on FAULT2 signal
    mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_F0, MCPWM_FORCE_MCPWMXA_HIGH, MCPWM_FORCE_MCPWMXB_LOW); //Action taken on PWM1A and PWM1B, when FAULT0 occurs
    mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_F1, MCPWM_FORCE_MCPWMXA_LOW, MCPWM_FORCE_MCPWMXB_HIGH); //Action taken on PWM1A and PWM1B, when FAULT1 occurs
    mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_F2, MCPWM_FORCE_MCPWMXA_HIGH, MCPWM_FORCE_MCPWMXB_LOW); //Action taken on PWM0A and PWM0B, when FAULT2 occurs
    mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_F1, MCPWM_FORCE_MCPWMXA_LOW, MCPWM_FORCE_MCPWMXB_HIGH); //Action taken on PWM0A and PWM0B, when FAULT1 occurs
#endif

#if MCPWM_EN_SYNC
    //6. Syncronization configuration
    //comment if you don't want to use sync submodule, also u can comment the sync gpio signals
    //here synchronization occurs on PWM1A and PWM1B
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_SYNC0, 200);    //Load counter value with 20% of period counter of mcpwm timer 1 when sync 0 occurs
#endif

#if MCPWM_EN_CAPTURE
    //7. Capture configuration
    //comment if you don't want to use capture submodule, also u can comment the capture gpio signals
    //configure CAP0, CAP1 and CAP2 signal to start capture counter on rising edge
    //we generate a gpio_test_signal of 20ms on GPIO 12 and connect it to one of the capture signal, the disp_captured_function displays the time between rising edge
    //In general practice you can connect Capture  to external signal, measure time between rising edge or falling edge and take action accordingly
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    //enable interrupt, so each this a rising edge occurs interrupt is triggered
    MCPWM[MCPWM_UNIT_0]->int_ena.val = CAP0_INT_EN | CAP1_INT_EN | CAP2_INT_EN;  //Enable interrupt on  CAP0, CAP1 and CAP2 signal
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
#endif

}

void init_uart(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
	uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

float FREQUENCIA_GLOBAL = 60;
float delta_omega_t(void){
	float w_t = (FREQUENCIA_GLOBAL * BLDC_SPEED_UPDATE_PERIOD_US * 0.000001 * 360);
	return w_t;
}


int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
    //ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

int string_comands_esp(char * input_str){
	char * token;
	token = strtok(input_str, ";");
	int index=1;
	float freq = 60.0;
	float ampl = 50.0;
	int modul = 1;
	int ramp = 1;
	float time_ramp = 1.0;
	while (token != NULL) {
		switch(index){
			case (INDEX_FREQ):
				freq = atof(token);
				break;
			case(INDEX_AMPL):
				ampl = atof(token);
				break;
			case(INDEX_MODL):
				modul = atoi(token);
				break;
			case(INDEX_RAMP):
				ramp = atoi(token);
			break;
			case(INDEX_TMRA):
				time_ramp = atoi(token);
				break;
		}
	    token = strtok(NULL, ";");
	    index++;
	  }
	config_comand_esp(freq, ampl, modul, ramp, time_ramp);
	return 0;

}

void config_comand_esp(float freq, float ampl, int modul, int ramp, float time_ramp){
	float actual_freq = FREQUENCIA_GLOBAL;
	float delta_freq = (freq - actual_freq) / (float) DIVISIONS_RAMPA;
	float actual_ampl = args[arg_amplitude];
	float delta_ampl_aux = (ampl -  actual_ampl) / (float) DIVISIONS_RAMPA;
	rampa_args[delta_fr_rmp] = delta_freq;
	rampa_args[delta_ampl_rmp] = delta_ampl_aux;
	rampa_args[ampl_final_rmp] = ampl;
	rampa_args[freq_final_rmp] = freq;
    args[arg_modulation] = modul;
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer_rampa, find_time_couter_ramap(time_ramp)));
//	args[arg_modulation] = modul;
//	FREQUENCIA_GLOBAL = freq;
//	delta_freq = delta_omega_t();
//	args[arg_amplitude] = ampl;

}

void update_rampa(float * rampa_args){


	FREQUENCIA_GLOBAL = FREQUENCIA_GLOBAL + rampa_args[delta_fr_rmp];
	delta_freq = delta_omega_t();
	args[arg_amplitude] = args[arg_amplitude] + rampa_args[delta_ampl_rmp];
	if(rampa_args[ramp_index] >= DIVISIONS_RAMPA){

		FREQUENCIA_GLOBAL = rampa_args[freq_final_rmp];
		args[arg_amplitude] = rampa_args[ampl_final_rmp];
		ESP_ERROR_CHECK(esp_timer_stop(periodic_timer_rampa));
		rampa_args[ramp_index] = 0;
		printf("%d\n",(int)rampa_args[ramp_index]);

	}
	rampa_args[ramp_index] = rampa_args[ramp_index] + 1;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
    	printf("Hello");
        sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2500 / portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    char string[RX_BUF_SIZE+1];
    int rxBytes=1;

    while (1) {
    	vTaskDelay(1000/ portTICK_PERIOD_MS);
        rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 10/ portTICK_PERIOD_MS);
        if (rxBytes > 0) {

        	vTaskDelay(100 / portTICK_PERIOD_MS);
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

//            for(int i = 0;i<rxBytes;i++){
//            	string[i] = data[i];
//            }
            vTaskDelay(1000/ portTICK_PERIOD_MS);
            string_comands_esp((char *)data);
            printf("\n%s\n",data);

        }
    }
    free(data);
}

void update_vel_motor(int *args){
	float *angle = &args[arg_angle];
	int *amplitude = &args[arg_amplitude];
	int *type_modulation = &args[arg_modulation];
	if(*angle >= 360){
		*angle = 0;
	}

	switch(*type_modulation){
		case SPWM:
			gen_SPWM(*angle, *amplitude);
			break;
		case SVPWM:
			gen_SVPWM(*angle, *amplitude);
			break;
		case THIPWM_FOUR:
			gen_THIPWM(*angle, *amplitude,4);
			break;
		case THIPWM_SIX:
			gen_THIPWM(*angle, *amplitude,6);
			break;
		case DPWMMAX:
			gen_DPWMMAX(*angle, *amplitude);
			break;
		case DPWMMIN:
			gen_DPWMMIN(*angle, *amplitude);
			break;
		case DPWM0:
			gen_DPWMM(*angle, *amplitude, 0);
			break;
		case DPWM1:
			gen_DPWMM(*angle, *amplitude, 1);
			break;
		case DPWM2:
			gen_DPWMM(*angle, *amplitude, 2);
			break;
		case DPWM3:
			gen_DPWMM(*angle, *amplitude, 3);
			break;
		default:
			gen_SPWM(*angle, *amplitude);
			break;
	}
	*angle += delta_freq;

}

void gen_SPWM(int angle, float amplitude){
	float rad_angle = angle * M_PI / 180;
	float phase_angle =  (120) * M_PI / 180;
	float duty_cycle_sinal_phase_a = amplitude * sin(rad_angle) * 0.5 + amplitude * 0.5;
	float duty_cycle_sinal_phase_b = amplitude * sin(rad_angle + phase_angle) * 0.5 + amplitude * 0.5;
	float duty_cycle_sinal_phase_c = amplitude * sin(rad_angle - phase_angle) * 0.5 + amplitude * 0.5;


    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, 0, duty_cycle_sinal_phase_a);   //Configure PWM0A & PWM0B with above settings
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, 0, duty_cycle_sinal_phase_b);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, 0, duty_cycle_sinal_phase_c);
}

void gen_SVPWM(int angle, float amplitude){
	float rad_angle = angle * M_PI / 180;
	float phase_angle =  (120) * M_PI / 180;
	float sin_a = sin(rad_angle);
	float sin_b = sin(rad_angle + phase_angle);
	float sin_c = sin(rad_angle - phase_angle);
	float voff = ((fmax(fmax(sin_a, sin_b), sin_c)) + (fmin(fmin(sin_a, sin_b), sin_c))) / 2;
	float duty_cycle_sinal_phase_a = amplitude * sin_a * 0.5 + 0.5 * amplitude - voff * amplitude * 0.5;
	float duty_cycle_sinal_phase_b = amplitude * sin_b  * 0.5 + 0.5 * amplitude - voff * amplitude * 0.5;
	float duty_cycle_sinal_phase_c = amplitude * sin_b * 0.5 + 0.5 * amplitude -  voff * amplitude * 0.5;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, 0, duty_cycle_sinal_phase_a);   //Configure PWM0A & PWM0B with above settings
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, 0, duty_cycle_sinal_phase_b);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, 0, duty_cycle_sinal_phase_c);

}

void gen_THIPWM(int angle, float amplitude, float amplitude_harmonic){
	float rad_angle = angle * M_PI / 180;
	float phase_angle =  (120) * M_PI / 180;
	float duty_cycle_sinal_phase_a = 0;
	float duty_cycle_sinal_phase_b = 0;
	float duty_cycle_sinal_phase_c = 0;
	if (amplitude_harmonic == 4){
		if(amplitude > 90){
			amplitude = 90;
		}
		duty_cycle_sinal_phase_a = (amplitude * sin(rad_angle) * 0.5 + amplitude * 0.5 + (amplitude * sin(3*rad_angle)) / amplitude_harmonic) / 1.1;
		duty_cycle_sinal_phase_b = (amplitude * sin(rad_angle + phase_angle) * 0.5 + amplitude * 0.5 + (amplitude * sin(3*rad_angle + phase_angle)) / amplitude_harmonic) / 1.1;
		duty_cycle_sinal_phase_c = (amplitude * sin(rad_angle - phase_angle) * 0.5 + amplitude * 0.5 + (amplitude * sin(3*rad_angle - phase_angle)) / amplitude_harmonic) / 1.1;
	}else{
		duty_cycle_sinal_phase_a = amplitude * sin(rad_angle) * 0.5 + amplitude * 0.5 + amplitude * sin(3*rad_angle) / amplitude_harmonic;
		duty_cycle_sinal_phase_b = amplitude * sin(rad_angle + phase_angle) * 0.5 + amplitude * 0.5 + amplitude * sin(3*rad_angle + phase_angle) / amplitude_harmonic;
		duty_cycle_sinal_phase_c = amplitude * sin(rad_angle - phase_angle) * 0.5 + amplitude * 0.5 + amplitude * sin(3*rad_angle - phase_angle ) / amplitude_harmonic;
	}
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, 0, duty_cycle_sinal_phase_a);   //Configure PWM0A & PWM0B with above settings
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, 0, duty_cycle_sinal_phase_b);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, 0, duty_cycle_sinal_phase_c);

}

void gen_DPWMMAX(int angle, float amplitude){
	float rad_angle = angle * M_PI / 180;
	float phase_angle =  (120) * M_PI / 180;
	float sin_a = sin(rad_angle);
	float sin_b = sin(rad_angle + phase_angle);
	float sin_c = sin(rad_angle - phase_angle);
	float max_sin = 1 - fmax(fmax(sin_a, sin_b), sin_c);
	float duty_cycle_sinal_phase_a = amplitude * sin_a * 0.5 + amplitude * 0.5 + max_sin * 0.50 * amplitude;
	float duty_cycle_sinal_phase_b = amplitude * sin_b * 0.5 + amplitude * 0.5 + max_sin * 0.50 * amplitude;
	float duty_cycle_sinal_phase_c = amplitude * sin_c * 0.5 + amplitude * 0.5 + max_sin * 0.50 * amplitude;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, 0, duty_cycle_sinal_phase_a);   //Configure PWM0A & PWM0B with above settings
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, 0, duty_cycle_sinal_phase_b);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, 0, duty_cycle_sinal_phase_c);
}


void gen_DPWMMIN(int angle, float amplitude){
	float rad_angle = angle * M_PI / 180;
	float phase_angle =  (120) * M_PI / 180;
	float sin_a = sin(rad_angle);
	float sin_b = sin(rad_angle + phase_angle);
	float sin_c = sin(rad_angle - phase_angle);
	float min_sin = 1 + fmin(fmin(sin_a, sin_b), sin_c);
	float duty_cycle_sinal_phase_a = amplitude * sin_a * 0.5 + amplitude * 0.5 - min_sin * 0.5 * amplitude;
	float duty_cycle_sinal_phase_b = amplitude * sin_b * 0.5 + amplitude * 0.5 - min_sin * 0.5 * amplitude;
	float duty_cycle_sinal_phase_c = amplitude * sin_c * 0.5 + amplitude * 0.5 - min_sin * 0.5 * amplitude;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, 0, duty_cycle_sinal_phase_a);   //Configure PWM0A & PWM0B with above settings
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, 0, duty_cycle_sinal_phase_b);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, 0, duty_cycle_sinal_phase_c);
}

void gen_DPWMM(int angle, float amplitude, int type_modulation){
	float rad_angle = angle * M_PI / 180;
	float phase_angle =  (120) * M_PI / 180;
	float sin_a = sin(rad_angle);
	float sin_b = sin(rad_angle + phase_angle);
	float sin_c = sin(rad_angle - phase_angle);
	float sin_delta = 0;
	float delta_angle = 0;
	float mi_modulation = 0;
	if(type_modulation == 0){
		delta_angle = 30;
	}else if(type_modulation == 1){
		delta_angle = 60;
	}else if(type_modulation == 2){
		delta_angle = 90;
	}
	else{
		delta_angle = 120;
	}
	if(angle >= (0 + delta_angle) && angle <= (60 + delta_angle)){
		sin_delta = (1 - mi_modulation) - sin_a ;
	}else if(angle > (60 + delta_angle) && angle <= (120 + delta_angle)){
		sin_delta = (1 - mi_modulation) + sin_b ;
	}else if(angle > (120 + delta_angle) && angle <= (180 + delta_angle)){
		sin_delta = (1 - mi_modulation) - sin_c ;
	}else if(angle > (180 + delta_angle) && angle <= (240 + delta_angle)){
		sin_delta = (1 - mi_modulation) + sin_a ;
	}else if(angle > (240 + delta_angle) && angle <= (300 + delta_angle)){
		sin_delta = (1 - mi_modulation) - sin_b ;
	}else if (angle > (300 + delta_angle) && angle <= (360 + delta_angle)){
		sin_delta = (1 - mi_modulation) + sin_c ;
	}

	float duty_cycle_sinal_phase_a = amplitude * sin_a * 0.5 + amplitude * 0.5 + sin_delta * 0.5 * amplitude;
	float duty_cycle_sinal_phase_b = amplitude * sin_b * 0.5 + amplitude * 0.5 + sin_delta * 0.5 * amplitude;
	float duty_cycle_sinal_phase_c = amplitude * sin_c * 0.5 + amplitude * 0.5 + sin_delta * 0.5 * amplitude;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, 0, duty_cycle_sinal_phase_a);   //Configure PWM0A & PWM0B with above settings
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, 0, duty_cycle_sinal_phase_b);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, 0, duty_cycle_sinal_phase_c);

}


float find_time_couter_ramap(float time_rampa){
	float value_counter = BLDC_SPEED_UPDATE_PERIOD_US_RAMPA * (time_rampa / (BLDC_SPEED_UPDATE_PERIOD_US_RAMPA * DIVISIONS_RAMPA * 0.000001));
	return value_counter;

}


void app_main(void)
{
    init_uart();
	delta_freq = delta_omega_t();
    printf("Testing MCPWM...\n");
    printf("Testing MCPWM...\n");
    int *arg;
    mcpwm_example_config(&arg);
    printf("After config...\n");
    int valor = 0;
    FREQUENCIA_GLOBAL = 0;
    float freq_inicial = 60;
    float ampl_inicial = 50;
    float time_rampa_incial = 5;
    args[arg_angle] = 0.0;
    args[arg_amplitude] = 0;
    args[arg_modulation] = SPWM;

    rampa_args[delta_fr_rmp] = freq_inicial / DIVISIONS_RAMPA; // valor calculado partindo de zero até 60 com 20 divisões
	rampa_args[freq_final_rmp] = freq_inicial;
	rampa_args[delta_ampl_rmp] = ampl_inicial / DIVISIONS_RAMPA; // valor calculado partindo de zero até 50% com 20 divisões
	rampa_args[ampl_final_rmp] = ampl_inicial;
	rampa_args[ramp_index] = 0;
    //rampa_args
    esp_timer_handle_t periodic_timer = NULL;
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = update_vel_motor,
        .arg = args,
    };


    printf("before creation timers...\n");
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args_rampa, &periodic_timer_rampa));

    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer_rampa, find_time_couter_ramap(time_rampa_incial)));
    //vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, BLDC_SPEED_UPDATE_PERIOD_US));


    //cap_queue = xQueueCreate(1, sizeof(capture)); //comment if you don't want to use capture module
    current_cap_value = (uint32_t *)malloc(CAP_SIG_NUM*sizeof(uint32_t)); //comment if you don't want to use capture module
    previous_cap_value = (uint32_t *)malloc(CAP_SIG_NUM*sizeof(uint32_t));  //comment if you don't want to use capture module

    printf("after creation timers...\n");
    //xTaskCreate(disp_captured_signal, "mcpwm_config", 4096, NULL, 5, NULL);  //comment if you don't want to use capture module

    //xTaskCreate(gpio_test_signal, "gpio_test_signal", 4096, NULL, 5, NULL); //comment if you don't want to use capture module

    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, 5, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, 4, NULL);
    //xTaskCreate(mcpwm_example_config, "mcpwm_example_config", 4096, NULL, 5, NULL);

    //timer_config_t config = {
    //    .divider = TIMER_DIVIDER,
    //    .counter_dir = TIMER_COUNT_UP,
    //    .counter_en = TIMER_PAUSE,
    ///    .alarm_en = TIMER_ALARM_EN,
    //    .auto_reload = TIMER_AUTORELOAD_EN
    //};

    //timer_init(TIMER_GROUP_0, TIMER_0, &config);
    //timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    //timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 100);
    //timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    //timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, update_vel_motor, &angle, 0);
    //timer_start(TIMER_GROUP_0, TIMER_0);
}
