/** @file fan_app.h
 *  @brief Application layer helper and wrapper function declarations.
 *  Aims to make the code more readable. Application layer,
 *  library layer and driver layer concept was in mind.
 *
 *  @author Mark Bilginer (GitHub: MarkBilginer)
 *  @bug No known bugs.
 */

#ifndef INC_FAN_APP_H_
#define INC_FAN_APP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* -- Includes -- */

/* -- Library abstraction layer -- */
#include "STM32_fan_lib.h"

/* -- Exported functions prototypes -- */

/** @brief The application logic flow is here.
 *  @param None
 *  @return None
 */
void application(void);

/** @brief The application header.
 *
 *  Includes credentials and user manual.
 *
 *  @param None
 *  @return None
 */
void print_header(void);

/** @brief Itializes system components.
 *  @param None
 *  @return None
 */
void init_system(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_FAN_APP_H_ */
