/**
 * @file DR16.hpp template
 * @author - GUO, Zilin
 *         - Your Name 
 * @brief  This is the DR16 template file directly copy from the PA2
 * @date 2023-10-18
 *
 * @copyright This file is only for HKUST Enterprize RM2024 internal
 * competition. All Rights Reserved.
 */
#pragma once // In case of multiple inclusion
#include "AppConfig.h"
#if USE_DR16


/*Inlcude the main.h files, where you could access the huart1 handle*/
#include "main.h"

namespace DR16
{

/**
 * @brief the DR16 remote controller structure
 * @brief This structure is should be based on the DR16 user manual
 * @note  Now you have to refer to the DR16 User Manual and define your own data structure
 */
typedef struct
{
    /*Your own implementation*/

} RcData;

/**
 * @brief Access the decoded remote controller datat by this API
 * @remark You are recommended to:
 *         - Return a constant pointer of the decoded remote controller data
 * inthis fucntion
 */
const RcData *getRcData();

/*===========================================================================*/
/*You can declare your own function here, supposing you would like to desgin a
 * more complicated DR16 module*/



/*===========================================================================*/

/**
 * @brief The initialization of your assingment DR16 Module
 * @remark You are recommended to:
 *         - Register your callback in the function
 *         - Start the first round of the UART data reception
 * @remark If you wish to, please implement the function body in the DR16.cpp
 * file
 */
void init();

}  // namespace DR16
#endif  // DR16_UART