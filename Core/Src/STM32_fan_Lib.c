/*
 * STM32_fan_Lib.c
 *
 *  Created on: May 28, 2021
 *      Author: mark
 */
#include "STM32_fan_lib.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

uint8_t fan_control_received = 0;
uint8_t fan_speed_received = 0;
uint8_t is_exit = 0;


// 1 denotes terminal and 2 potentiometer
char fan_control[sizeof(char)] = {0};
char fan_speed_str[3*sizeof(char) + 1] = "100";

uint8_t mode_change = 0;
uint8_t current_mode = 0;
uint8_t previous_mode = 0;

void set_is_exit(uint8_t val){
	is_exit = val;
}

uint8_t get_is_exit(){
	return is_exit;
}

// Reverses a string 'str' of length 'len'
void reverse(char* str, int len){
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}


// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d){
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint){
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

char *dec(uint8_t x, char *s)
{
    *--s = 0;
    if (!x) *--s = '0';
    for (; x; x/=10) *--s = '0'+x%10;
    return s;
}


/**
  * @brief System Clock Configuration
  * @retval None
  */

uint8_t convert_digits_to_number1(uint8_t duty_cycle){
	uint8_t number = duty_cycle;
	return number;
}
void convert_digits_to_number(char* number, char* duty_cycle){
	sprintf(number, "%s%s", duty_cycle[0], duty_cycle[1]);
}

uint8_t calculate_pulse_width_fan_control(uint8_t duty_percent){

	float pulse_width = (duty_percent * 200) / ((float) 100);

	return pulse_width;
}

uint32_t calculate_duty_cycle(uint32_t duty_percent){

	float fraction_percent = duty_percent/ ((float) 100);

	return (fraction_percent * 200);
}


uint32_t calculate_pulse_width(uint8_t *fan_speed){
	return calculate_duty_cycle(convert_digits_to_number1(*fan_speed));
}

void adjust_tim1_pulse_width(uint32_t pulse_width){
	htim1.Instance->CCR3 = pulse_width;
}

void print_fan_speed_result(){
	char *fan_speed_str_l = &fan_speed_str[0];
	char str1[] = "Setting fan speed to ";
	char str2[] = "% power.\r\n";

	print_message(str1);
	print_message(fan_speed_str_l);
	print_message(str2);
}

uint8_t get_fan_speed_int(){
	char *fan_speed_str_l = &fan_speed_str[0];
	return atoi(fan_speed_str_l);
}

char get_fan_control(){
	return fan_control[0];
}

uint8_t is_fan_speed_received(){
	return fan_speed_received;
}
void reset_fan_speed_received(){
	fan_speed_received = 0;
}
void receive_input_fan_speed(){
	//pointer might need to be incremented to show whole result
	char *fan_speed_str_l = &fan_speed_str[0];

	if(current_mode == 1){
		HAL_UART_Receive(&huart2,(uint8_t *) fan_speed_str_l, strlen(fan_speed_str_l), HAL_MAX_DELAY);
		uint8_t fan_speed = get_fan_speed_int();
		if(fan_speed >= 0 && fan_speed <= 100){
			print_fan_speed_result();
			adjust_tim1_pulse_width(calculate_pulse_width_fan_control(fan_speed));
			tim2_base_stop_interrupt();
			print_message("Continue? Enter a mode:\r\n");
		}
		else{
			print_message("Invalid fan_speed has been entered. Try again:\r\n");
			receive_input_fan_speed();
		}
	}
}

void print_fan_control_result(){
	uint8_t *fan_control_l = &fan_control[0];
	if(*fan_control_l == '1'){
		print_message("You have selected \“fan control via terminal\”.\r\n");
	}
	else if(*fan_control_l == '2'){
		print_message("You have selected \“fan control via potentiometer\”.\r\n");
	}
	else{
		//print_message("You have entered an invalid fan control option. Try again...\r\n");
	}
}

uint8_t is_fan_control_received(){
	return fan_control_received;
}
void reset_fan_control_received(){
	fan_control_received = 0;
}

void receive_input_fan_control(){

	uint8_t *fan_control_l = &fan_control[0];

	//print_message("Input 1 for terminal input and 2 for potentiometer control:\r\n");
	if(HAL_UART_Receive(&huart2, fan_control_l, sizeof(*fan_control_l), HAL_MAX_DELAY) == HAL_OK){

		uint8_t input_mode = *fan_control_l - '0';
		if(input_mode == 1 || input_mode == 2){
			print_fan_control_result();
			set_current_mode(input_mode);
			tim1_complement_pwm_start();
			tim2_options_init(get_current_mode());
			tim2_base_start_interrupt();
		}
		else if(input_mode == 0){
			//exit program
			print_message("Exiting_program...\r\n");
			//deinitialization code comes here
			tim1_complement_pwm_stop(); // doesnt work start function gets called in app.
			tim2_base_stop_interrupt();// doesnt work start function gets called in app.
			set_is_exit(1);
			//probably a flag needs to be used. to exit application and main function.
			print_message("Exit accomplished.\r\n");
		}
		else{
			print_message("You have entered an invalid fan control option. Try again...\r\n");
			receive_input_fan_control();
		}
	}

}

void init_tim2_terminal(){
	//for a 5sec interval
	init_tim2(999, TIM_COUNTERMODE_UP, 159999, TIM_CLOCKDIVISION_DIV1, TIM_AUTORELOAD_PRELOAD_ENABLE);
}

void init_tim2_potentiometer(){
	//for a 1 sec interval
	init_tim2(999, TIM_COUNTERMODE_UP, 31999, TIM_CLOCKDIVISION_DIV1, TIM_AUTORELOAD_PRELOAD_ENABLE);
}

void tim2_options_init(uint8_t option){
	if(option == 1){
		init_tim2_terminal();
	}
	else if(option == 2){
		init_tim2_potentiometer();
	}
	else{

	}
}

void print_message(char msg[]){
	  HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlen(msg), 100);
}

void init_hal(void){
	HAL_Init();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void config_system_clock(void)
{
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	  /** Configure LSE Drive Capability
	  */
	  HAL_PWR_EnableBkUpAccess();
	  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
	  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	  RCC_OscInitStruct.MSICalibrationValue = 0;
	  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	  RCC_OscInitStruct.PLL.PLLM = 1;
	  RCC_OscInitStruct.PLL.PLLN = 16;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
	  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
	  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure the main internal regulator output voltage
	  */
	  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Enable MSI Auto calibration
	  */
	  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void init_adc1(void){

	  /* USER CODE BEGIN ADC1_Init 0 */

	  /* USER CODE END ADC1_Init 0 */

	  ADC_ChannelConfTypeDef sConfig = {0};

	  /* USER CODE BEGIN ADC1_Init 1 */

	  /* USER CODE END ADC1_Init 1 */
	  /** Common config
	  */
	  hadc1.Instance = ADC1;
	  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	  hadc1.Init.LowPowerAutoWait = DISABLE;
	  hadc1.Init.ContinuousConvMode = DISABLE;
	  hadc1.Init.NbrOfConversion = 1;
	  hadc1.Init.DiscontinuousConvMode = DISABLE;
	  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc1.Init.DMAContinuousRequests = DISABLE;
	  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	  hadc1.Init.OversamplingMode = DISABLE;
	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_5;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  sConfig.Offset = 0;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN ADC1_Init 2 */

	  /* USER CODE END ADC1_Init 2 */
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
void init_tim1(void)
{

	  /* USER CODE BEGIN TIM1_Init 0 */

	  /* USER CODE END TIM1_Init 0 */

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};
	  TIM_OC_InitTypeDef sConfigOC = {0};
	  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	  /* USER CODE BEGIN TIM1_Init 1 */

	  /* USER CODE END TIM1_Init 1 */
	  htim1.Instance = TIM1;
	  htim1.Init.Prescaler = 999;
	  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim1.Init.Period = 199;
	  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim1.Init.RepetitionCounter = 0;
	  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = 100;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	  sBreakDeadTimeConfig.DeadTime = 0;
	  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	  sBreakDeadTimeConfig.BreakFilter = 0;
	  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	  sBreakDeadTimeConfig.Break2Filter = 0;
	  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN TIM1_Init 2 */

	  /* USER CODE END TIM1_Init 2 */
	  HAL_TIM_MspPostInit(&htim1);
}

void set_current_mode(uint8_t val){
	current_mode = val;
}

uint8_t get_current_mode(){
	return current_mode;
}

void set_previous_mode(uint8_t val){
	previous_mode = val;
}

uint8_t get_previous_mode(){
	return previous_mode;
}

uint8_t is_mode_changed(){
	if(previous_mode != current_mode){
		return (uint8_t) 1;
	}
	else{
		return (uint8_t) 0;
	}
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void init_tim2(uint32_t prescaler, uint32_t cnt_mode, uint32_t period, uint32_t clk_div, uint32_t auto_re_pre)
{

	  /* USER CODE BEGIN TIM2_Init 0 */

	  /* USER CODE END TIM2_Init 0 */

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  /* USER CODE BEGIN TIM2_Init 1 */

	  /* USER CODE END TIM2_Init 1 */
	  htim2.Instance = TIM2;
	  htim2.Init.Prescaler = 999;
	  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim2.Init.Period = 31999;
	  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN TIM2_Init 2 */

	  /* USER CODE END TIM2_Init 2 */

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){

	if(htim->Instance == TIM2){
		if(current_mode == 1){
			print_message("Please enter speed in %.\r\n");
			print_message("The format is as follows:\r\n");
			print_message("		005		-		5%\r\n");
			print_message("		010		-		10%\r\n");
			print_message("		100		-		100%\r\n");
			print_message("Enter: \r\n\r\n");
		}
		else if(current_mode == 2){
			float adc_reading;
			uint32_t fan_speed_as_pulse;
			uint8_t fan_speed_as_percent;
			uint8_t max_percent = 100;
			uint8_t base_ten = 10;
			char fan_speed_as_percent_str[3*sizeof(char) + 1] = "100";
			float resolution_adc = 4096;
			float pulse_max = 200.0f;
			char str1[] = "Potentiometer set to ";
			char str2[] = "% power. (Continue? Enter a mode: )\r\n";

			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			adc_reading = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);

			fan_speed_as_pulse = (adc_reading * pulse_max)/resolution_adc;
			fan_speed_as_percent = (adc_reading * max_percent) / resolution_adc;

			print_message(str1);
			print_message(itoa(fan_speed_as_percent, fan_speed_as_percent_str, base_ten));
			print_message(str2);

			adjust_tim1_pulse_width(fan_speed_as_pulse);
		}
		//else if(current_mode == 0){
			//tim2_base_stop_interrupt();
		//}
	}
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void init_usart2_uart(uint32_t baudrate)
{

	  /* USER CODE BEGIN USART2_Init 0 */

	  /* USER CODE END USART2_Init 0 */

	  /* USER CODE BEGIN USART2_Init 1 */

	  /* USER CODE END USART2_Init 1 */
	  huart2.Instance = USART2;
	  huart2.Init.BaudRate = baudrate;
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;
	  huart2.Init.StopBits = UART_STOPBITS_1;
	  huart2.Init.Parity = UART_PARITY_NONE;
	  huart2.Init.Mode = UART_MODE_TX_RX;
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	  if (HAL_UART_Init(&huart2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN USART2_Init 2 */

	  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void init_gpio(void){

	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin : LD3_Pin */
	  GPIO_InitStruct.Pin = LD3_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

}

void tim1_complement_pwm_start(){
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void tim1_complement_pwm_stop(){
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

void tim2_base_start_interrupt(){
	HAL_TIM_Base_Start_IT(&htim2);
}
void tim2_base_stop_interrupt(){
	HAL_TIM_Base_Stop_IT(&htim2);
}


void tim_base_start_interrupt(){
	HAL_TIM_Base_Start_IT(&htim1);
}
void tim_base_stop_interrupt(){
	HAL_TIM_Base_Stop_IT(&htim1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
