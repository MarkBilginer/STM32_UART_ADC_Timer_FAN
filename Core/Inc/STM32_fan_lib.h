/** @file STM32_fan_lib.h
 *  @brief Library layer function declarations for the STM32L4.
 *
 *  Provides a layer of abstraction. Aim is to make the application portable,
 *  and separating the vendor dependence. The library layer wraps around the
 *  vendor specific driver layer.
 *
 *  @author Mark Bilginer (GitHub: MarkBilginer)
 *  @bug No known bugs.
 */

#ifndef INC_STM32_FAN_LIB_H_
#define INC_STM32_FAN_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

/* -- Includes -- */

/* -- Hardware Abstraction Layer -- */
#include "stm32l4xx_hal.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* -- Exported functions prototypes -- */
void set_is_exit(uint8_t val);
uint8_t get_is_exit();

/** @brief Calculates from duty percent the corresponding pulse width.
 *  @param duty percent
 *  @return pulse width
 */
uint8_t calculate_pulse_width(uint8_t duty_percent);

/** @brief Changes pulse variable of tim1.
 *  @param pulse_width
 *  @return None
 */
void adjust_tim1_pulse_width(uint32_t pulse_width);

/** @brief information prompt for setting fan speed.
 *  @param None
 *  @return None
 */
void print_fan_speed_result();

/** @brief Converts fan speed string to integer and returns it.
 *  @param None
 *  @return fan speed, in other words, duty cycle.
 */
uint8_t get_fan_speed_int();

/** @brief Receive from user duty cycle/ fan speed percentage.
 *
 *  Checks also whether user input is valid.
 *
 *  @param None
 *  @return None
 */
void receive_input_fan_speed();

/** @brief Displays user choice of input mode using UART.
 *  @param None
 *  @return None
 */
void print_fan_control_result();

/** @brief Receive from user input mode.
 *
 *  Checks also whether user input is valid.
 *  Can also terminate the app when user input equals 0.
 *
 *  @param None
 *  @return None
 */
void receive_input_fan_control();

/** @brief Initializes TIM2.
 *
 * Helper function of tim2_options_init.
 * Represents the option terminal.
 *
 *  @param None
 *  @return None
 */
void init_tim2_terminal();

/** @brief Initializes TIM2.
 *
 * Helper function of tim2_options_init.
 * Represents the option potentiometer.
 *
 *  @param None
 *  @return None
 */
void init_tim2_potentiometer();

/** @brief Initializes TIM2 according to user input.
 *  @param option The users option selection.
 *  @return None
 */
void tim2_options_init(uint8_t option);

/** @brief Sets users mode input globally.
 *  @param val represents users mode input.
 *  @return None
 */
void set_current_mode(uint8_t val);

/** @brief Gets users mode input.
 *  @param None
 *  @return current mode
 */
uint8_t get_current_mode();

/** @brief Convenience method to print a message to console using UART.
 *  @param msg The string to be printed.
 *  @return None
 */
void print_message(char msg[]);

/** @brief Initializes Hardware Access Layer.
 *
 * Wrapper function. Wraps around the vendor specific function.
 *
 *  @param None
 *  @return None
 */
void init_hal(void);

/** @brief System Clock Configuration.
  * @return None
  */
void config_system_clock(void);

/** @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void init_adc1(void);

/** @brief TIM1 Initialization Function
 *  @param None
 *  @retval None
 */
void init_tim1(void);

/** @brief TIM2 Initialization Function
 *  @param prescaler
 *  @param cnt_mode
 *  @param period
 *  @param clk_div
 *  @param auto_re_pre
 *  @return None
 */
void init_tim2(uint32_t prescaler, uint32_t cnt_mode, uint32_t period,
		uint32_t clk_div, uint32_t auto_re_pre);

/** @brief USART2's UART Initialization Function
  * @param baudrate Rate of information transmission.
  * @return None
  */
void init_usart2_uart(uint32_t baudrate);

/** @brief GPIO Initialization Function.
  * @param None
  * @return None
  */
void init_gpio(void);

/** @brief Starts TIM1 PWM output.
 *
 * Since the output of TIM1 is inverted, we need to call
 * the complement function of PWM start.
 *
 * @param None
 * @return None
 */
void tim1_complement_pwm_start();

/** @brief Stops the TIM! complement PWM output.
 *  @param None
 *  @return None
 */
void tim1_complement_pwm_stop();

/** @brief Starts interrupt mode of TIM2.
 *  @param None
 *  @return None
 */
void tim2_base_start_interrupt();

/** @brief Stops interrupt mode of TIM2.
 *  @param None
 *  @return None
 */
void tim2_base_stop_interrupt();

/** @brief Starts interrupt mode of TIM1.
 *  @param None
 *  @return None
 */
void tim_base_start_interrupt();

/** @brief Stops interrupt mode of TIM1.
 *  @param None
 *  @return None
 */
void tim_base_stop_interrupt();

/** @brief Executed in case of error occurrence.
  * @return None
  */
void Error_Handler(void);

int power(int x, unsigned int y);
unsigned int strlength(char *p);
int atoi(char* str);
char* itoa(int num, char* buffer, int base);

/* -- Private defines -- */
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

#ifdef __cplusplus
}
#endif


#endif /* INC_STM32_FAN_LIB_H_ */
