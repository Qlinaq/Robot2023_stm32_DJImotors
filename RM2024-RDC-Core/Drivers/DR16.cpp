/**
 * @file DR16.cpp
 * @brief Implement the function definition that is declared in DR16.hpp file
 */

#include "DR16.hpp"
#if USE_DR16
namespace DR16
{

/**
 * @brief Define a singleton RcData structure instance here.
 * @remark If you wish to, please decode the DR16 data from the buffer to here
 * @remark Refer to the definition of the structure in the "DR16.hpp" files
 */
static RcData rcData;

/*Return the constant pointer of the current decoded data*/
const RcData *getRcData() { return &rcData; }

/*================================================================================*/
/*You are free to declare your buffer, or implement your own function(callback, decoding) here*/






/*================================================================================*/
void init()
{
    /*If you would like to, please implement your function definition here*/
}

}  // namespace DR16

#endif