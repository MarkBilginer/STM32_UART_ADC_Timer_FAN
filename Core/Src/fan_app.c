/** @file fan_app.c
 *  @brief Application layer helper and wrapper functions implementation.
 *
 *  Aims to make the code more readable. Application layer,
 *  library layer and driver layer concept was in mind.
 *
 *  @author Mark Bilginer (GitHub: MarkBilginer)
 *  @bug No known bugs.
 */

/* -- Includes -- */
/* -- Application abstraction layer -- */
#include "fan_app.h"

void init_system()
{
	init_hal();
	config_system_clock();
	init_gpio();
	init_usart2_uart(115200);
	init_tim1();
	init_adc1();
}

void print_header()
{
	print_message("\r\n\r\n");
	print_message("************************************************"
			"******************************************************\r\n");
	print_message("*********************************** Final Exam -"
			" <Mark> <Bilginer> ***********************************\r\n");
	print_message("********** Enter 1 for \“fan control via terminal\” "
			"or 2 for \“fan control via potentiometer\” ***********\r\n");
	print_message("***************************************** Enter 0 "
			"to \"exit\" ******************************************\r\n");
	print_message("******** During run, change the mode by typing 1"
			" or 2 or 0. Turns on the corresponding mode **********\r\n");
	print_message("************************************************"
			"******************************************************\r\n");
	print_message("\r\n\r\n");
	print_message("Input:\r\n");
}

void application()
{
	if(!(get_is_exit() == 1)){

		receive_input_fan_control();

		if(get_current_mode() == 1){
			receive_input_fan_speed();
		}

		//set_previous_mode(get_current_mode());
	}
}
