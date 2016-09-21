/**********************************************************************************************
 * Filename:       BladeMonitorService.h
 *
 * Description:    This file contains the BladeMonitorService service definitions and
 *                 prototypes.
 *
 *************************************************************************************************/


#ifndef _BLADEMONITORSERVICE_H_
#define _BLADEMONITORSERVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
* CONSTANTS
*/
// Service UUID
#define BLADEMONITORSERVICE_SERV_UUID 0xBBFF

//  Characteristic defines
#define BLADEMONITORSERVICE_OFFSETANGLE      0
#define BLADEMONITORSERVICE_OFFSETANGLE_UUID 0xCA00
#define BLADEMONITORSERVICE_OFFSETANGLE_LEN  4				//float, r, n

//  Characteristic defines
#define BLADEMONITORSERVICE_BATTERYVOLTAGE      1
#define BLADEMONITORSERVICE_BATTERYVOLTAGE_UUID 0xCA01
#define BLADEMONITORSERVICE_BATTERYVOLTAGE_LEN  4				//float, r, n

//  Characteristic defines
#define BLADEMONITORSERVICE_BATTERYLIFE      2
#define BLADEMONITORSERVICE_BATTERYLIFE_UUID 0xCA02
#define BLADEMONITORSERVICE_BATTERYLIFE_LEN  4				//float, r, n

//  Characteristic defines
#define BLADEMONITORSERVICE_BLADELIFE      3
#define BLADEMONITORSERVICE_BLADELIFE_UUID 0xCA03
#define BLADEMONITORSERVICE_BLADELIFE_LEN  4				//float, r, n

//  Characteristic defines
#define BLADEMONITORSERVICE_TEMPERATURE     4
#define BLADEMONITORSERVICE_TEMPERATURE_UUID 0xCA04
#define BLADEMONITORSERVICE_TEMPERATURE_LEN  4				//float, r, n
//  Characteristic defines
#define BLADEMONITORSERVICE_STARTDATE      5
#define BLADEMONITORSERVICE_STARTDATE_UUID 0xCA05
#define BLADEMONITORSERVICE_STARTDATE_LEN  4				//uint32_t, UNIX date/time  in seconds, r, w

//  Characteristic defines
#define BLADEMONITORSERVICE_BLADETYPE      6
#define BLADEMONITORSERVICE_BLADETYPE_UUID 0xCA06
#define BLADEMONITORSERVICE_BLADETYPE_LEN  20				////char array max of 20, r, w

//  Characteristic defines
#define BLADEMONITORSERVICE_BLADEWIDTH      	7
#define BLADEMONITORSERVICE_BLADEWIDTH_UUID 0xCA07
#define BLADEMONITORSERVICE_BLADEWIDTH_LEN  4				//float (mm), r, w

//  Characteristic defines
#define BLADEMONITORSERVICE_CUSTOMERSITE      	8
#define BLADEMONITORSERVICE_CUSTOMERSITE_UUID 0xCA08
#define BLADEMONITORSERVICE_CUSTOMERSITE_LEN  20				//char array max of 20, r, w

//  Characteristic defines
#define BLADEMONITORSERVICE_CONVEYORNAME      	9
#define BLADEMONITORSERVICE_CONVEYORNAME_UUID 0xCA09
#define BLADEMONITORSERVICE_CONVEYORNAME_LEN  20				//char array max of 20, r, w

//todo systemTime
//  Characteristic defines
#define BLADEMONITORSERVICE_SYSTEMTIME      10
#define BLADEMONITORSERVICE_SYSTEMTIME_UUID 0xCA10
#define BLADEMONITORSERVICE_SYSTEMTIME_LEN  4				//uint32_t, UNIX date/time  in seconds, r, w, n

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*BladeMonitorServiceChange_t)( uint8 paramID );

typedef struct
{
  BladeMonitorServiceChange_t        pfnChangeCb;  // Called when characteristic value changes
} BladeMonitorServiceCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * BladeMonitorService_AddService- Initializes the BladeMonitorService service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t BladeMonitorService_AddService( void );

/*
 * BladeMonitorService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t BladeMonitorService_RegisterAppCBs( BladeMonitorServiceCBs_t *appCallbacks );

/*
 * BladeMonitorService_SetParameter - Set a BladeMonitorService parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t BladeMonitorService_SetParameter( uint8 param, uint8 len, void *value );

/*
 * BladeMonitorService_GetParameter - Get a BladeMonitorService parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t BladeMonitorService_GetParameter( uint8 param, void *value );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _BLADEMONITORSERVICE_H_ */
