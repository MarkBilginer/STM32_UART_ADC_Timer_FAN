/** @file main.c
 *  @brief Application run is contained in this file.
 *
 *  This file contains the applications' main() function.
 *
 *  This is the entry point for the application.
 *  Application name is STM32_UART_ADC_Timer_Fan.
 *  What the Application does? Lets user choose
 *  between terminal or potentiometer control of an
 *  external fan.
 *
 *  @author Mark Bilginer (GitHub: MarkBilginer)
 *  @bug something like deinit system might need to be implemented.
 */

#include "main.h"

/** @brief  The application entry point.
 *  @return Should not return.
 */
int main(void)
{
	init_system();
	print_header();
	while (1) {
		application();
	}
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
