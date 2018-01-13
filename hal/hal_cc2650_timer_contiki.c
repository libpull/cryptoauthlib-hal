#include "atca_hal.h"
#include "rtimer.h"
#include <stdio.h>

/** \defgroup hal_ Hardware abstraction layer (hal_)
 *
 * \brief
 * These methods define the hardware abstraction layer for communicating with a CryptoAuth device
 *
   @{ */

/** \brief This function delays for a number of microseconds.
 *
 * \param[in] delay number of 0.001 milliseconds to delay
 */
void atca_delay_us(uint32_t delay)
{
    rtimer_clock_t end_time = RTIMER_NOW() + (delay*RTIMER_SECOND/1000000);
    while(RTIMER_CLOCK_LT(RTIMER_NOW(), end_time));
}

/** \brief This function delays for a number of tens of microseconds.
 *
 * \param[in] delay number of 0.01 milliseconds to delay
 */
void atca_delay_10us(uint32_t delay)
{
   atca_delay_us(delay*10); 
}

/** \brief This function delays for a number of milliseconds.
 *
 *         You can override this function if you like to do
 *         something else in your system while delaying.
 * \param[in] delay number of milliseconds to delay
 */
void atca_delay_ms(uint32_t delay)
{
    atca_delay_us(delay*1000);
}

/** @} */
