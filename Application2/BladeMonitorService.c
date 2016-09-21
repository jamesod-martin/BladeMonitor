/**********************************************************************************************
 * Filename:       BladeMonitorService.c
 *
 * Description:    This file contains the implementation of the service.
 *
 *************************************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "BladeMonitorService.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// BladeMonitorService Service UUID
CONST uint8_t BladeMonitorServiceUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLADEMONITORSERVICE_SERV_UUID)
};

// offsetAngle UUID
CONST uint8_t BladeMonitorService_offsetAngleUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLADEMONITORSERVICE_OFFSETANGLE_UUID)
};
// batteryVoltage UUID
CONST uint8_t BladeMonitorService_batteryVoltageUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLADEMONITORSERVICE_BATTERYVOLTAGE_UUID)
};
//batteryLife
CONST uint8_t BladeMonitorService_batteryLifeUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLADEMONITORSERVICE_BATTERYLIFE_UUID)
};
// bladeLife UUID
CONST uint8_t BladeMonitorService_bladeLifeUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLADEMONITORSERVICE_BLADELIFE_UUID)
};
// Temperature UUID
CONST uint8_t BladeMonitorService_TemperatureUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLADEMONITORSERVICE_TEMPERATURE_UUID)
};
// startDate UUID
CONST uint8_t BladeMonitorService_startDateUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLADEMONITORSERVICE_STARTDATE_UUID)
};
// bladeType UUID
CONST uint8_t BladeMonitorService_bladeTypeUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLADEMONITORSERVICE_BLADETYPE_UUID)
};
// bladeWidth UUID
CONST uint8_t BladeMonitorService_bladeWidthUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLADEMONITORSERVICE_BLADEWIDTH_UUID)
};
// customerSite UUID
CONST uint8_t BladeMonitorService_customerSiteUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLADEMONITORSERVICE_CUSTOMERSITE_UUID)
};
// conveyorName UUID
CONST uint8_t BladeMonitorService_conveyorNameUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLADEMONITORSERVICE_CONVEYORNAME_UUID)
};
// systemTime UUID
CONST uint8_t BladeMonitorService_systemTimeUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(BLADEMONITORSERVICE_SYSTEMTIME_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static BladeMonitorServiceCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t BladeMonitorServiceDecl = { ATT_UUID_SIZE, BladeMonitorServiceUUID };

// Characteristic "offsetAngle" Properties (for declaration)
static uint8_t BladeMonitorService_offsetAngleProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "offsetAngle" Value variable
static uint8_t BladeMonitorService_offsetAngleVal[BLADEMONITORSERVICE_OFFSETANGLE_LEN] = {0};

// Characteristic "offsetAngle" CCCD
static gattCharCfg_t *BladeMonitorService_offsetAngleConfig;

// Characteristic "batteryVoltage" Properties (for declaration)
static uint8_t BladeMonitorService_batteryVoltageProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "batteryVoltage" Value variable
static uint8_t BladeMonitorService_batteryVoltageVal[BLADEMONITORSERVICE_BATTERYVOLTAGE_LEN] = {0};

// Characteristic "batteryVoltage" CCCD
static gattCharCfg_t *BladeMonitorService_batteryVoltageConfig;

// Characteristic "batteryLife" Properties (for declaration)
static uint8_t BladeMonitorService_batteryLifeProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "batteryLife" Value variable
static uint8_t BladeMonitorService_batteryLifeVal[BLADEMONITORSERVICE_BATTERYLIFE_LEN] = {0};

// Characteristic "batteryLife" CCCD
static gattCharCfg_t *BladeMonitorService_batteryLifeConfig;

// Characteristic "bladeLife" Properties (for declaration)
static uint8_t BladeMonitorService_bladeLifeProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "bladeLife" Value variable
static uint8_t BladeMonitorService_bladeLifeVal[BLADEMONITORSERVICE_BLADELIFE_LEN] = {0};

// Characteristic "bladeLife" CCCD
static gattCharCfg_t *BladeMonitorService_bladeLifeConfig;

// Characteristic "Temperature" Properties (for declaration)
static uint8_t BladeMonitorService_TemperatureProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "Temperature" Value variable
static uint8_t BladeMonitorService_TemperatureVal[BLADEMONITORSERVICE_TEMPERATURE_LEN] = {0};

// Characteristic "Temperature" CCCD
static gattCharCfg_t *BladeMonitorService_TemperatureConfig;

// Characteristic "startDate" Properties (for declaration)
static uint8_t BladeMonitorService_startDateProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic "startDate" Value variable
static uint8_t BladeMonitorService_startDateVal[BLADEMONITORSERVICE_STARTDATE_LEN] = {0};

// Characteristic "bladeType" Properties (for declaration)
static uint8_t BladeMonitorService_bladeTypeProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic "bladeType" Value variable
static uint8_t BladeMonitorService_bladeTypeVal[BLADEMONITORSERVICE_BLADETYPE_LEN] = {0};

// Characteristic "bladeWidth" Properties (for declaration)
static uint8_t BladeMonitorService_bladeWidthProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic "bladeWidth" Value variable
static uint8_t BladeMonitorService_bladeWidthVal[BLADEMONITORSERVICE_BLADEWIDTH_LEN] = {0};

// Characteristic "customerSite" Properties (for declaration)
static uint8_t BladeMonitorService_customerSiteProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic "customerSite" Value variable
static uint8_t BladeMonitorService_customerSiteVal[BLADEMONITORSERVICE_BLADEWIDTH_LEN] = {0};


// Characteristic "conveyorName" Properties (for declaration)
static uint8_t BladeMonitorService_conveyorNameProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic "conveyorName" Value variable
static uint8_t BladeMonitorService_conveyorNameVal[BLADEMONITORSERVICE_BLADEWIDTH_LEN] = {0};

// Characteristic "systemTime" Properties (for declaration)
static uint8_t BladeMonitorService_systemTimeProps = GATT_PROP_READ | GATT_PROP_NOTIFY | GATT_PROP_WRITE;

// Characteristic "systemTime" Value variable
static uint8_t BladeMonitorService_systemTimeVal[BLADEMONITORSERVICE_SYSTEMTIME_LEN] = {0};

// Characteristic "systemTime" CCCD
static gattCharCfg_t *BladeMonitorService_systemTimeConfig;

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t BladeMonitorServiceAttrTbl[] =
{
  // BladeMonitorService Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&BladeMonitorServiceDecl
  },
    // offsetAngle Characteristic Declaration !!! the declaration uses the BladeMonitorService_offsetAngleProps to describe read/write/notify
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &BladeMonitorService_offsetAngleProps
    },
      // offsetAngle Characteristic Value
      {
        { ATT_UUID_SIZE, BladeMonitorService_offsetAngleUUID },
        GATT_PERMIT_READ,
        0,
        BladeMonitorService_offsetAngleVal
      },
      // offsetAngle CCCD		//Client characteristic configuration descriptor. Used for signing up for notifications needs to be writeable for notify to work
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&BladeMonitorService_offsetAngleConfig
      },
    // batteryVoltage Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &BladeMonitorService_batteryVoltageProps
    },
      // batteryVoltage Characteristic Value
      {
        { ATT_UUID_SIZE, BladeMonitorService_batteryVoltageUUID },
        GATT_PERMIT_READ,
        0,
        BladeMonitorService_batteryVoltageVal
      },
      // batteryVoltage CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&BladeMonitorService_batteryVoltageConfig
      },
    // batteryLife Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &BladeMonitorService_batteryLifeProps
    },
      // batteryLife Characteristic Value
      {
        { ATT_UUID_SIZE, BladeMonitorService_batteryLifeUUID },
        GATT_PERMIT_READ,
        0,
        BladeMonitorService_batteryLifeVal
      },
      // batteryLife CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&BladeMonitorService_batteryLifeConfig
      },
    // bladeLife Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &BladeMonitorService_bladeLifeProps
    },
      // bladeLife Characteristic Value
      {
        { ATT_UUID_SIZE, BladeMonitorService_bladeLifeUUID },
        GATT_PERMIT_READ,
        0,
        BladeMonitorService_bladeLifeVal
      },
      // bladeLife CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&BladeMonitorService_bladeLifeConfig
      },
    // Temperature Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &BladeMonitorService_TemperatureProps
    },
      // Temperature Characteristic Value
      {
        { ATT_UUID_SIZE, BladeMonitorService_TemperatureUUID },
        GATT_PERMIT_READ,
        0,
        BladeMonitorService_TemperatureVal
      },
      // Temperature CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&BladeMonitorService_TemperatureConfig
      },
    // startDate Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &BladeMonitorService_startDateProps
    },
      // startDate Characteristic Value
      {
        { ATT_UUID_SIZE, BladeMonitorService_startDateUUID },
        GATT_PERMIT_READ,
        0,
        BladeMonitorService_startDateVal
      },
    // bladeType Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &BladeMonitorService_bladeTypeProps
    },
      // bladeType Characteristic Value
      {
        { ATT_UUID_SIZE, BladeMonitorService_bladeTypeUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        BladeMonitorService_bladeTypeVal
      },
	// bladeWidth Characteristic Declaration
	{
	  { ATT_BT_UUID_SIZE, characterUUID },
	  GATT_PERMIT_READ,
	  0,
	  &BladeMonitorService_bladeWidthProps
	},
	  // bladeWidth Characteristic Value
	  {
		{ ATT_UUID_SIZE, BladeMonitorService_bladeWidthUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		0,
		BladeMonitorService_bladeWidthVal
	  },
	// customerSite Characteristic Declaration
	{
	  { ATT_BT_UUID_SIZE, characterUUID },
	  GATT_PERMIT_READ ,
	  0,
	  &BladeMonitorService_customerSiteProps
	},
	  // customerSite Characteristic Value
	  {
		{ ATT_UUID_SIZE, BladeMonitorService_customerSiteUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		0,
		BladeMonitorService_customerSiteVal
	  },
	  // conveyorName Characteristic Declaration
	  	{
	  	  { ATT_BT_UUID_SIZE, characterUUID },
	  	  GATT_PERMIT_READ,
	  	  0,
	  	  &BladeMonitorService_conveyorNameProps
	  	},
	  	  // conveyorName Characteristic Value
	  	  {
	  		{ ATT_UUID_SIZE, BladeMonitorService_conveyorNameUUID },
	  		GATT_PERMIT_READ | GATT_PERMIT_WRITE,
	  		0,
	  		BladeMonitorService_customerSiteVal
	  	  },
		  // systemTime Characteristic Declaration
		  	{
		  	  { ATT_BT_UUID_SIZE, characterUUID },
		  	  GATT_PERMIT_READ ,
		  	  0,
		  	  &BladeMonitorService_systemTimeProps
		  	},
		  // systemTime Characteristic Value
			{
			  { ATT_UUID_SIZE, BladeMonitorService_systemTimeUUID },
			  GATT_PERMIT_READ,
			  0,
			  BladeMonitorService_systemTimeVal
			},
			// systemTime CCCD
			{
			  { ATT_BT_UUID_SIZE, clientCharCfgUUID },
			  GATT_PERMIT_READ | GATT_PERMIT_WRITE,
			  0,
			  (uint8 *)&BladeMonitorService_systemTimeConfig
			},
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t BladeMonitorService_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 *pLen, uint16 offset,
                                           uint16 maxLen, uint8 method );
static bStatus_t BladeMonitorService_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint16 len, uint16 offset,
                                            uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t BladeMonitorServiceCBs =
{
  BladeMonitorService_ReadAttrCB,  // Read callback function pointer
  BladeMonitorService_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * BladeMonitorService_AddService- Initializes the BladeMonitorService service by registering
 *          GATT attributes with the GATT server.
 *
 */
bStatus_t BladeMonitorService_AddService( void )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  BladeMonitorService_offsetAngleConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( BladeMonitorService_offsetAngleConfig == NULL )
  {
    return ( bleMemAllocError );
  }
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, BladeMonitorService_offsetAngleConfig );

  // Allocate Client Characteristic Configuration table
  BladeMonitorService_batteryVoltageConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( BladeMonitorService_batteryVoltageConfig == NULL )
  {
    return ( bleMemAllocError );
  }
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, BladeMonitorService_batteryVoltageConfig );

  // Allocate Client Characteristic Configuration table
  BladeMonitorService_batteryLifeConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( BladeMonitorService_batteryLifeConfig == NULL )
  {
    return ( bleMemAllocError );
  }
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, BladeMonitorService_batteryLifeConfig );

  // Allocate Client Characteristic Configuration table
  BladeMonitorService_bladeLifeConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( BladeMonitorService_bladeLifeConfig == NULL )
  {
    return ( bleMemAllocError );
  }
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, BladeMonitorService_bladeLifeConfig );

  // Allocate Client Characteristic Configuration table
  BladeMonitorService_TemperatureConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( BladeMonitorService_TemperatureConfig == NULL )
  {
    return ( bleMemAllocError );
  }
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, BladeMonitorService_TemperatureConfig );

	// Allocate Client Characteristic Configuration table
	  BladeMonitorService_systemTimeConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
	  if ( BladeMonitorService_systemTimeConfig == NULL )
	  {
		return ( bleMemAllocError );
	  }

	  // Initialize Client Characteristic Configuration attributes
	  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, BladeMonitorService_systemTimeConfig );

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( BladeMonitorServiceAttrTbl,
                                        GATT_NUM_ATTRS( BladeMonitorServiceAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &BladeMonitorServiceCBs );

  return ( status );
}

/*
 * BladeMonitorService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t BladeMonitorService_RegisterAppCBs( BladeMonitorServiceCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

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
bStatus_t BladeMonitorService_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case BLADEMONITORSERVICE_OFFSETANGLE:
      if ( len == BLADEMONITORSERVICE_OFFSETANGLE_LEN )
      {
        memcpy(BladeMonitorService_offsetAngleVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( BladeMonitorService_offsetAngleConfig, (uint8_t *)&BladeMonitorService_offsetAngleVal, FALSE,
                                    BladeMonitorServiceAttrTbl, GATT_NUM_ATTRS( BladeMonitorServiceAttrTbl ),
                                    INVALID_TASK_ID,  BladeMonitorService_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case BLADEMONITORSERVICE_BATTERYVOLTAGE:
      if ( len == BLADEMONITORSERVICE_BATTERYVOLTAGE_LEN )
      {
        memcpy(BladeMonitorService_batteryVoltageVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( BladeMonitorService_batteryVoltageConfig, (uint8_t *)&BladeMonitorService_batteryVoltageVal, FALSE,
                                    BladeMonitorServiceAttrTbl, GATT_NUM_ATTRS( BladeMonitorServiceAttrTbl ),
                                    INVALID_TASK_ID,  BladeMonitorService_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case BLADEMONITORSERVICE_BATTERYLIFE:
      if ( len == BLADEMONITORSERVICE_BATTERYLIFE_LEN )
      {
        memcpy(BladeMonitorService_batteryLifeVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( BladeMonitorService_batteryLifeConfig, (uint8_t *)&BladeMonitorService_batteryLifeVal, FALSE,
                                    BladeMonitorServiceAttrTbl, GATT_NUM_ATTRS( BladeMonitorServiceAttrTbl ),
                                    INVALID_TASK_ID,  BladeMonitorService_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case BLADEMONITORSERVICE_BLADELIFE:
      if ( len == BLADEMONITORSERVICE_BLADELIFE_LEN )
      {
        memcpy(BladeMonitorService_bladeLifeVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( BladeMonitorService_bladeLifeConfig, (uint8_t *)&BladeMonitorService_bladeLifeVal, FALSE,
                                    BladeMonitorServiceAttrTbl, GATT_NUM_ATTRS( BladeMonitorServiceAttrTbl ),
                                    INVALID_TASK_ID,  BladeMonitorService_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case BLADEMONITORSERVICE_TEMPERATURE:
      if ( len == BLADEMONITORSERVICE_TEMPERATURE_LEN )
      {
        memcpy(BladeMonitorService_TemperatureVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( BladeMonitorService_TemperatureConfig, (uint8_t *)&BladeMonitorService_TemperatureVal, FALSE,
                                    BladeMonitorServiceAttrTbl, GATT_NUM_ATTRS( BladeMonitorServiceAttrTbl ),
                                    INVALID_TASK_ID,  BladeMonitorService_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case BLADEMONITORSERVICE_STARTDATE:
      if ( len == BLADEMONITORSERVICE_STARTDATE_LEN )
      {
        memcpy(BladeMonitorService_startDateVal, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case BLADEMONITORSERVICE_BLADETYPE:
      if ( len == BLADEMONITORSERVICE_BLADETYPE_LEN )
      {
        memcpy(BladeMonitorService_bladeTypeVal, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case BLADEMONITORSERVICE_BLADEWIDTH:
      if ( len == BLADEMONITORSERVICE_BLADEWIDTH_LEN )
      {
        memcpy(BladeMonitorService_bladeWidthVal, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    case BLADEMONITORSERVICE_CUSTOMERSITE:
      if ( len == BLADEMONITORSERVICE_CUSTOMERSITE_LEN )
      {
        memcpy(BladeMonitorService_customerSiteVal, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    case BLADEMONITORSERVICE_CONVEYORNAME:
      if ( len == BLADEMONITORSERVICE_CONVEYORNAME_LEN )
      {
        memcpy(BladeMonitorService_conveyorNameVal, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    case BLADEMONITORSERVICE_SYSTEMTIME:
      if ( len == BLADEMONITORSERVICE_SYSTEMTIME_LEN )
      {
        memcpy(BladeMonitorService_systemTimeVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( BladeMonitorService_systemTimeConfig, (uint8_t *)&BladeMonitorService_systemTimeVal, FALSE,
                                    BladeMonitorServiceAttrTbl, GATT_NUM_ATTRS( BladeMonitorServiceAttrTbl ),
                                    INVALID_TASK_ID,  BladeMonitorService_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*
 * BladeMonitorService_GetParameter - Get a BladeMonitorService parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t BladeMonitorService_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
	  case BLADEMONITORSERVICE_STARTDATE:
		memcpy(value, BladeMonitorService_startDateVal, BLADEMONITORSERVICE_STARTDATE_LEN);
		break;
	  case BLADEMONITORSERVICE_BLADETYPE:
		memcpy(value, BladeMonitorService_bladeTypeVal, BLADEMONITORSERVICE_BLADETYPE_LEN);
		break;
	  case BLADEMONITORSERVICE_BLADEWIDTH:
		memcpy(value, BladeMonitorService_bladeWidthVal, BLADEMONITORSERVICE_BLADEWIDTH_LEN);
		break;
	  case BLADEMONITORSERVICE_CUSTOMERSITE:
		memcpy(value, BladeMonitorService_customerSiteVal, BLADEMONITORSERVICE_CUSTOMERSITE_LEN);
		break;
	  case BLADEMONITORSERVICE_CONVEYORNAME:
		memcpy(value, BladeMonitorService_conveyorNameVal, BLADEMONITORSERVICE_CONVEYORNAME_LEN);
		break;
	  case BLADEMONITORSERVICE_SYSTEMTIME:
		memcpy(value, BladeMonitorService_systemTimeVal, BLADEMONITORSERVICE_SYSTEMTIME_LEN);
		break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          BladeMonitorService_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t BladeMonitorService_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                       uint8 *pValue, uint16 *pLen, uint16 offset,
                                       uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the offsetAngle Characteristic Value
if ( ! memcmp(pAttr->type.uuid, BladeMonitorService_offsetAngleUUID, pAttr->type.len) )
  {
    if ( offset > BLADEMONITORSERVICE_OFFSETANGLE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, BLADEMONITORSERVICE_OFFSETANGLE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the batteryVoltage Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, BladeMonitorService_batteryVoltageUUID, pAttr->type.len) )
  {
    if ( offset > BLADEMONITORSERVICE_BATTERYVOLTAGE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, BLADEMONITORSERVICE_BATTERYVOLTAGE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the batteryLife Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, BladeMonitorService_batteryLifeUUID, pAttr->type.len) )
  {
    if ( offset > BLADEMONITORSERVICE_BATTERYLIFE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, BLADEMONITORSERVICE_BATTERYLIFE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the bladeLife Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, BladeMonitorService_bladeLifeUUID, pAttr->type.len) )
  {
    if ( offset > BLADEMONITORSERVICE_BLADELIFE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, BLADEMONITORSERVICE_BLADELIFE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Temperature Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, BladeMonitorService_TemperatureUUID, pAttr->type.len) )
  {
    if ( offset > BLADEMONITORSERVICE_TEMPERATURE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, BLADEMONITORSERVICE_TEMPERATURE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the startDate Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, BladeMonitorService_startDateUUID, pAttr->type.len) )
  {
    if ( offset > BLADEMONITORSERVICE_STARTDATE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, BLADEMONITORSERVICE_STARTDATE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the bladeType Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, BladeMonitorService_bladeTypeUUID, pAttr->type.len) )
  {
    if ( offset > BLADEMONITORSERVICE_BLADETYPE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, BLADEMONITORSERVICE_BLADETYPE_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
// See if request is regarding the bladeWidth Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, BladeMonitorService_bladeWidthUUID, pAttr->type.len) )
{
  if ( offset > BLADEMONITORSERVICE_BLADEWIDTH_LEN )  // Prevent malicious ATT ReadBlob offsets.
  {
    status = ATT_ERR_INVALID_OFFSET;
  }
  else
  {
    *pLen = MIN(maxLen, BLADEMONITORSERVICE_BLADEWIDTH_LEN - offset);  // Transmit as much as possible
    memcpy(pValue, pAttr->pValue + offset, *pLen);
  }
}
// See if request is regarding the customerSite Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, BladeMonitorService_customerSiteUUID, pAttr->type.len) )
{
  if ( offset > BLADEMONITORSERVICE_CUSTOMERSITE_LEN )  // Prevent malicious ATT ReadBlob offsets.
  {
    status = ATT_ERR_INVALID_OFFSET;
  }
  else
  {
    *pLen = MIN(maxLen, BLADEMONITORSERVICE_CUSTOMERSITE_LEN - offset);  // Transmit as much as possible
    memcpy(pValue, pAttr->pValue + offset, *pLen);
  }
}
// See if request is regarding the conveyorName Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, BladeMonitorService_conveyorNameUUID, pAttr->type.len) )
{
  if ( offset > BLADEMONITORSERVICE_CONVEYORNAME_LEN )  // Prevent malicious ATT ReadBlob offsets.
  {
    status = ATT_ERR_INVALID_OFFSET;
  }
  else
  {
    *pLen = MIN(maxLen, BLADEMONITORSERVICE_CONVEYORNAME_LEN - offset);  // Transmit as much as possible
    memcpy(pValue, pAttr->pValue + offset, *pLen);
  }
}
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}


/*********************************************************************
 * @fn      BladeMonitorService_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t BladeMonitorService_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                        uint8 *pValue, uint16 len, uint16 offset,
                                        uint8 method )
{
  bStatus_t status  = SUCCESS;
  /*uint8_t   notifyApp = 0xFF;					//This is used to notify the application of which val changed. if it is important. Only used
  	  	  	  	  	  	  	  	  	  	  	  	//here for change in start time which should initiate a change in starting angle
  //old way***************************************************

  // If attribute permissions require authorization to write, return error
    if ( gattPermitAuthorWrite( pAttr->permissions ) )
    {
      // Insufficient authorization
      return ( ATT_ERR_INSUFFICIENT_AUTHOR );
    }

    if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {
      // 16-bit UUID
      uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
      switch ( uuid )		//this will write the to the appropriate value based on the uuid
      {
        case BLADEMONITORSERVICE_STARTDATE_UUID:		//if the start date was changed....
        	notifyApp = BLADEMONITORSERVICE_STARTDATE;
        case BLADEMONITORSERVICE_SYSTEMTIME_UUID:
        	//Validate the value
			if ( offset == 0 )
			{
			if ( len != 20 )
			{
			  status = ATT_ERR_INVALID_VALUE_SIZE;
			}
			}
			else
			{
			status = ATT_ERR_ATTR_NOT_LONG;
			}
			//Write the value
			if ( status == SUCCESS )
			{
			  uint8 *pCurValue = (uint8 *)pAttr->pValue;
			  *pCurValue = pValue[0];
			}
			break;

        case BLADEMONITORSERVICE_BLADETYPE_UUID:
        case BLADEMONITORSERVICE_BLADEWIDTH_UUID:
        case BLADEMONITORSERVICE_CUSTOMERSITE_UUID:
        case BLADEMONITORSERVICE_CONVEYORNAME_UUID:
          //Validate the value
          if ( offset == 0 )
          {
            if ( len != 20 )
            {
              status = ATT_ERR_INVALID_VALUE_SIZE;
            }
          }
          else
          {
            status = ATT_ERR_ATTR_NOT_LONG;
          }

          //Write the value
          if ( status == SUCCESS )
          {
            uint8 *pCurValue = (uint8 *)pAttr->pValue;
            *pCurValue = pValue[0];
          }
          break;

        case GATT_CLIENT_CHAR_CFG_UUID:
          status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                   offset, GATT_CLIENT_CFG_NOTIFY );
          break;

        default:
          // Should never get here! (characteristics 2 and 4 do not have write permissions)
          status = ATT_ERR_ATTR_NOT_FOUND;
          break;
      }
    }
    else
    {
      // 128-bit UUID
      status = ATT_ERR_INVALID_HANDLE;
    }

    // If a characteristic value changed then callback function to notify application of change
    if ( (notifyApp != 0xFF ) && pAppCBs && pAppCBs->pfnChangeCb )
    {
      pAppCBs->pfnChangeCb( notifyApp );
    }

    return ( status );
	*/
  uint8_t   paramID = 0xFF;					//This is used to notify the application of which val changed. if it is important. Only used
    	  	  	  	  	  	  	  	  	  	  	  	//here for change in start time which should initiate a change in starting angle

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  // See if request is regarding the startDate Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, BladeMonitorService_startDateUUID, pAttr->type.len) )
  {
    if ( offset + len > BLADEMONITORSERVICE_STARTDATE_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == BLADEMONITORSERVICE_STARTDATE_LEN)
        paramID = BLADEMONITORSERVICE_STARTDATE;
    }
  }
  // See if request is regarding the bladeType Characteristic Value
    else if ( ! memcmp(pAttr->type.uuid, BladeMonitorService_bladeTypeUUID, pAttr->type.len) )
    {
      if ( offset + len > BLADEMONITORSERVICE_BLADETYPE_LEN )
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // Copy pValue into the variable we point to from the attribute table.
        memcpy(pAttr->pValue + offset, pValue, len);

        // Only notify application if entire expected value is written
        if ( offset + len == BLADEMONITORSERVICE_BLADETYPE_LEN)
          paramID = BLADEMONITORSERVICE_BLADETYPE;
      }
    }
  // See if request is regarding the bladeWidth Characteristic Value
    else if ( ! memcmp(pAttr->type.uuid, BladeMonitorService_bladeWidthUUID, pAttr->type.len) )
    {
      if ( offset + len > BLADEMONITORSERVICE_BLADEWIDTH_LEN )
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // Copy pValue into the variable we point to from the attribute table.
        memcpy(pAttr->pValue + offset, pValue, len);

        // Only notify application if entire expected value is written
        if ( offset + len == BLADEMONITORSERVICE_BLADEWIDTH_LEN)
          paramID = BLADEMONITORSERVICE_BLADEWIDTH;
      }
    }
  // See if request is regarding the customerSite Characteristic Value
    else if ( ! memcmp(pAttr->type.uuid, BladeMonitorService_customerSiteUUID, pAttr->type.len) )
    {
      if ( offset + len > BLADEMONITORSERVICE_CUSTOMERSITE_LEN )
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // Copy pValue into the variable we point to from the attribute table.
        memcpy(pAttr->pValue + offset, pValue, len);

        // Only notify application if entire expected value is written
        if ( offset + len == BLADEMONITORSERVICE_CUSTOMERSITE_LEN)
          paramID = BLADEMONITORSERVICE_CUSTOMERSITE;
      }
    }
  // See if request is regarding the conveyorName Characteristic Value
    else if ( ! memcmp(pAttr->type.uuid, BladeMonitorService_conveyorNameUUID, pAttr->type.len) )
    {
      if ( offset + len > BLADEMONITORSERVICE_CONVEYORNAME_LEN )
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // Copy pValue into the variable we point to from the attribute table.
        memcpy(pAttr->pValue + offset, pValue, len);

        // Only notify application if entire expected value is written
        if ( offset + len == BLADEMONITORSERVICE_CONVEYORNAME_LEN)
          paramID = BLADEMONITORSERVICE_CONVEYORNAME;
      }
    }
  // See if request is regarding the systemTime Characteristic Value
    else if ( ! memcmp(pAttr->type.uuid, BladeMonitorService_systemTimeUUID, pAttr->type.len) )
    {
      if ( offset + len > BLADEMONITORSERVICE_SYSTEMTIME_LEN )
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // Copy pValue into the variable we point to from the attribute table.
        memcpy(pAttr->pValue + offset, pValue, len);

        // Only notify application if entire expected value is written
        if ( offset + len == BLADEMONITORSERVICE_SYSTEMTIME_LEN)
          paramID = BLADEMONITORSERVICE_SYSTEMTIME;
      }
    }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (paramID != 0xFF)
    if ( pAppCBs && pAppCBs->pfnChangeCb )
      pAppCBs->pfnChangeCb( paramID ); // Call app function from stack task context.

  return status;
}
