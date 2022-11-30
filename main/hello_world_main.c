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
#include "soc/rtc.h"
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
#define BLDC_SPEED_UPDATE_PERIOD_US    1000   // 1segundo
#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28)  //Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29)  //Capture 2 interrupt bit
#define TIMER_DIVIDER   (1)

#define GPIO_PWM0A_OUT 23   //Set GPIO 19 as PWM0A
#define GPIO_PWM0B_OUT 22   //Set GPIO 18 as PWM0B

#define GPIO_PWM1A_OUT 21   //Set GPIO 17 as PWM1A
#define GPIO_PWM1B_OUT 19   //Set GPIO 16 as PWM1B

#define GPIO_PWM2A_OUT 16   //Set GPIO 15 as PWM2A
#define GPIO_PWM2B_OUT 17   //Set GPIO 14 as PWM2B

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


#define arg_angle		0
#define arg_amplitude	1
#define arg_modulation	2

void gen_SPWM(int angle, float amplitude);
void gen_SVPWM(int angle, float amplitude);
void gen_THIPWM(int angle, float amplitude, float amplitude_armonic);
void gen_DPWMMAX(int angle, float amplitude);
void gen_DPWMMIN(int angle, float amplitude);
float gen_triangle(int angle, int amplitude);

typedef struct {
    uint32_t capture_signal;
    mcpwm_capture_signal_t sel_cap_signal;
} capture;

uint32_t *current_cap_value = NULL;
uint32_t *previous_cap_value = NULL;

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
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 100, 100);   //Enable deadtime on PWM2A and PWM2B with red = (1000)*100ns on PWM2A
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 100, 100);        //Enable deadtime on PWM1A and PWM1B with fed = (2000)*100ns on PWM1B
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 100, 100);  //Enable deadtime on PWM0A and PWM0B with red = (656)*100ns & fed = (67)*100ns on PWM0A and PWM0B generated from PWM0A
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

void update_vel_motor(int *args){
	int *angle = &args[arg_angle];
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
		default:
			gen_SPWM(*angle, *amplitude);
			break;

	}
	*angle += 8;

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
	float duty_cycle_sinal_phase_a = amplitude * sin(rad_angle) * 0.5 + 0.5* amplitude + gen_triangle(angle, amplitude) + 0.5* amplitude;
	float duty_cycle_sinal_phase_b = amplitude * sin(rad_angle + phase_angle) * 0.5 + 0.5* amplitude + gen_triangle(angle, amplitude);
	float duty_cycle_sinal_phase_c = amplitude * sin(rad_angle - phase_angle) * 0.5 + 0.5* amplitude + gen_triangle(angle, amplitude);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, 0, duty_cycle_sinal_phase_a);   //Configure PWM0A & PWM0B with above settings
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, 0, duty_cycle_sinal_phase_b);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, 0, duty_cycle_sinal_phase_c);

}

void gen_THIPWM(int angle, float amplitude, float amplitude_armonic){
	float rad_angle = angle * M_PI / 180;
	float phase_angle =  (120) * M_PI / 180;
	float duty_cycle_sinal_phase_a = amplitude * sin(rad_angle) * 0.5 + amplitude * 0.5 + amplitude * sin(3*rad_angle) / amplitude_armonic;
	float duty_cycle_sinal_phase_b = amplitude * sin(rad_angle + phase_angle) * 0.5 + amplitude * 0.5 + amplitude * sin(3*rad_angle + phase_angle) / amplitude_armonic;
	float duty_cycle_sinal_phase_c = amplitude * sin(rad_angle - phase_angle) * 0.5 + amplitude * 0.5 + amplitude * sin(3*rad_angle - phase_angle) / amplitude_armonic;
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
	float max_sin = fmax(fmax(sin_a, sin_b), sin_c);
	float duty_cycle_sinal_phase_a = amplitude * sin_a * 0.5 + amplitude * 0.5 - max_sin * 0.25 * amplitude;
	float duty_cycle_sinal_phase_b = amplitude * sin_b * 0.5 + amplitude * 0.5 - max_sin * 0.25 * amplitude;
	float duty_cycle_sinal_phase_c = amplitude * sin_c * 0.5 + amplitude * 0.5 - max_sin * 0.25 * amplitude;
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
	float min_sin = fmax(fmin(sin_a, sin_b), sin_c);
	float duty_cycle_sinal_phase_a = amplitude * sin_a * 0.5 + amplitude * 0.5 - min_sin * 0.5 * amplitude;
	float duty_cycle_sinal_phase_b = amplitude * sin_b * 0.5 + amplitude * 0.5 - min_sin * 0.5 * amplitude;
	float duty_cycle_sinal_phase_c = amplitude * sin_c * 0.5 + amplitude * 0.5 - min_sin * 0.5 * amplitude;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, 0, duty_cycle_sinal_phase_a);   //Configure PWM0A & PWM0B with above settings
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, 0, duty_cycle_sinal_phase_b);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, 0, duty_cycle_sinal_phase_c);
}


float gen_triangle(int angle, int amplitude){

	int relative_angle = angle;
	float amplitude_triangle;
	if(angle <= 30){
		relative_angle = angle;
	}
	else if(angle > 30 && angle <= 90){
		relative_angle = 60 - angle;
	}
	else if(angle > 90 && angle <= 150){
		relative_angle = -120 + angle;
	}
	else if(angle > 150 && angle <= 210){
		relative_angle = 180 - angle;
	}
	else if(angle > 210 && angle <= 270){
		relative_angle = -240 + angle;
	}
	else if(angle > 270 && angle <= 330){
		relative_angle = 300 - angle;
	}
	else{
		relative_angle = -360 + angle;
	}

	amplitude_triangle = (((float)relative_angle) / 30) * amplitude * 0.25;

	return amplitude_triangle;

}

int args[3];

void app_main(void)
{
    printf("Testing MCPWM...\n");
    printf("Testing MCPWM...\n");
    int *arg;
    mcpwm_example_config(&arg);
    printf("After config...\n");
    int valor = 0;

    args[arg_angle] = 0;
    args[arg_amplitude] = 50;
    args[arg_modulation] = SVPWM;
    esp_timer_handle_t periodic_timer = NULL;
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = update_vel_motor,
        .arg = args,
    };


    printf("before creation timers...\n");
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, BLDC_SPEED_UPDATE_PERIOD_US));
    //cap_queue = xQueueCreate(1, sizeof(capture)); //comment if you don't want to use capture module
    current_cap_value = (uint32_t *)malloc(CAP_SIG_NUM*sizeof(uint32_t)); //comment if you don't want to use capture module
    previous_cap_value = (uint32_t *)malloc(CAP_SIG_NUM*sizeof(uint32_t));  //comment if you don't want to use capture module
    printf("after creation timers...\n");
    //xTaskCreate(disp_captured_signal, "mcpwm_config", 4096, NULL, 5, NULL);  //comment if you don't want to use capture module
    xTaskCreate(gpio_test_signal, "gpio_test_signal", 4096, NULL, 5, NULL); //comment if you don't want to use capture module
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
