/**************************************************************************************************
  Filename:       BladeMonitor.h
  Revised:        $Date: 2016-01-19 $
  Revision:       $Revision: 1 $

  Description:    This file contains the Battery Manager application
                  definitions and prototypes.

**************************************************************************************************/

#ifndef BLADEMONITOR_H
#define BLADEMONITOR_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */
/* Delay */
#ifdef TI_DRIVERS_I2C_INCLUDED
#define delay_ms(i) Task_sleep( ((i) * 1000) / Clock_tickPeriod )
#define MS_2_TICKS(ms) ( ((ms) * 1000) / Clock_tickPeriod )
#else
#define delay_ms(i) ( CPUdelay(8000*(i)) )
#endif
/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the Simple BLE Peripheral.
 */
extern void BladeMonitor_createTask(void);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* BladeMonitor_H */
