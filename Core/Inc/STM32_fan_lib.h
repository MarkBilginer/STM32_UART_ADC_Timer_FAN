/*
 * STM32_fan_lib.h
 *
 *  Created on: May 28, 2021
 *      Author: mark
 */

#ifndef INC_STM32_FAN_LIB_H_
#define INC_STM32_FAN_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void init_hal(void);
void config_system_clock(void);
void init_adc1(void);
void init_tim1(void);
void init_tim2(uint32_t prescaler, uint32_t cnt_mode, uint32_t period, uint32_t clk_div, uint32_t auto_re_pre);
void init_usart2_uart(uint32_t baudrate);
void init_gpio(void);
void tim1_complement_pwm_start();
void tim1_complement_pwm_stop();
void tim2_base_start_interrupt();
void receive_input_fan_control();
void print_message(char msg[]);
void print_fan_control_result();
void receive_input_fan_speed();
char *dec(uint8_t x, char *s);
void convert_digits_to_number(char* number,char* duty_cycle);
uint8_t is_fan_speed_received();
void reset_fan_speed_received();
void reset_fan_control_received();
uint8_t is_fan_control_received();
void print_fan_speed_result();
uint8_t calculate_pulse_width_fan_control(uint8_t duty_percent);
void tim2_base_stop_interrupt();
void adjust_tim1_pulse_width(uint32_t pulse_width);
void tim2_base_stop_interrupt();
void set_current_mode(uint8_t val);
uint8_t get_current_mode();
void set_previous_mode(uint8_t val);
uint8_t get_previous_mode();
uint8_t is_mode_changed();
void set_is_exit(uint8_t val);
uint8_t get_is_exit();
char get_fan_control();
uint8_t get_fan_speed_int();
void tim2_options_init(uint8_t option);

/* USER CODE BEGIN EFP */


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif


#endif /* INC_STM32_FAN_LIB_H_ */
