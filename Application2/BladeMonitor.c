/**************************************************************************************************
  Filename:       BladeMonitor.c
  Revised:        $Date: 2016-07-26 11:43:11  $
  Revision:       $Revision: 2 $

  Description:    This file contains the Martin Blade Monitor program


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/BIOS.h>

#include <time.h>					/********** For RTC ****************/
#include <ti/sysbios/hal/Seconds.h>

#include <driverlib/aon_batmon.h>	/********** For Battery Voltage Monitoring ******************/

#include <ti/drivers/PIN/PINCC26XX.h>
#include <ti/drivers/UART.h>

#include <driverlib/aux_adc.h>
#include <driverlib/aux_wuc.h>
#include <inc/hw_aux_evctl.h>

#include "hci_tl.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"

#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/I2C.h>
#include "bsp_i2c.h"
#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral.h"
#include "gapbondmgr.h"
#include "math.h"

#include "osal_snv.h"
#include "ICallBleAPIMSG.h"


#include "util.h"
#include "Board.h"

#include "MPU6500.h"
#include "DataStorage.h"

#include "BladeMonitor.h"
#include "BladeMonitorService.h"


/*********************************************************************
 * CONSTANTS
 */
// Advertising interval when device is discoverable (units of 625us, 160=100ms, 1600 = 1s)
#define DEFAULT_ADVERTISING_INTERVAL          16000

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     1600
#else
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8
#endif // FEATURE_OAD

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               1000

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define SBP_TASK_PRIORITY                     1


#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT                  0x0001
#define BM_VALUE_CHANGE_EVT                   0x0002
#define SBP_PERIODIC_EVT                      0x0004
#define SBP_CONN_EVT_END_EVT                  0x0008

//ADC Sampling constants
#define SAMPLECOUNT 4			//number of samples to average
#define SAMPLETYPE uint16_t
#define SAMPLESIZE sizeof(SAMPLETYPE)

SAMPLETYPE adcSamples[SAMPLECOUNT];
SAMPLETYPE singleSample;

/************** MEASUREMENT VARIABLES **********************************/
sampleStruct currentSample = {0};	//stores time of last sample, offsetAngle, batteryVoltage, and temperature
float 	batteryLife = 1000, //Calculated esitmate of battery life in days.
		startingAngle = 0,  //angle
		bladeLife = 300;  //estimated blade Life in days

time_t startingAngleSetTime = 0; //time variable from RTC to record when the new angle was set.
uint32_t systemTime = 0;					//for storing seconds in UNIX time
time_t currentTime;
uint32_t startDate = 1473447902; //change this for setting current time.

/**************** USER SET VARIABLES ********************************/

char bladeType[20] = {'B','l','a','d','e',' ','T','y','p','e'};
float bladeWidth = 0;  //bladeWidth in mm
char customerSite[20] = {0};
char conveyorName[20] = {0};

uint16_t totalNumSamples = 0;
uint16_t madeIt = 0, testSampleTotal = 0;
uint32_t averageSample = 0;

PIN_Handle pinHandle;
PIN_State  pinState;
PIN_Status pinStatus = PIN_NO_ACCESS;

uint8_t I2CSelect = 0;
uint8_t writeDone = 0;

/**************************** global debug variables ******************************/
float xAcc, yAcc, zAcc;			//holds the floats for acceleration values
uint8_t breakWire = 1, active = 0, breakTest = 0, breakMake = 0;
uint32_t numMeasurements = 0;
float tempTemp; //holds temporary temperature value for debugging

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} sbpEvt_t;


/*********************************************************************
 * LOCAL VARIABLES
 */
// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

//Semaphore for analog sampling
Semaphore_Struct sem2;
Semaphore_Handle hSem;
Hwi_Struct hwi;

// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x12,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'B',  'l',  'a',  'd',  'e',  'M',  'o',  'n',  'i',  't',  'o',  'r',  '0',  '0', '0', '0', '2',
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID)
#else
  LO_UINT16(BLADEMONITORSERVICE_SERV_UUID),
  HI_UINT16(BLADEMONITORSERVICE_SERV_UUID)
#endif //!FEATURE_OAD
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "BladeMonitor00001";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
/***************RTOS and comms related functions **************************/
static void BladeMonitor_init( void );
static void BladeMonitor_taskFxn(UArg a0, UArg a1);
static uint8_t BladeMonitor_processStackMsg(ICall_Hdr *pMsg);
static uint8_t BladeMonitor_processGATTMsg(gattMsgEvent_t *pMsg);
static void BladeMonitor_processAppMsg(sbpEvt_t *pMsg);
static void BladeMonitor_processStateChangeEvt(gaprole_States_t newState);
static void BladeMonitor_processValueChangeCB(uint8_t paramID);
static void BladeMonitor_performPeriodicTask(void);
static void BladeMonitor_sendAttRsp(void);
static void BladeMonitor_freeAttRsp(uint8_t status);
static void BladeMonitor_stateChangeCB(gaprole_States_t newState);
#ifndef FEATURE_OAD
static void BladeMonitor_charValueChangeCB(uint8_t paramID);
#endif //!FEATURE_OAD
static void BladeMonitor_enqueueMsg(uint8_t event, uint8_t state);

/*****************Application functions **************************************/
uint32_t GetADCValue(uint8_t analogAddress);
float ConvertVoltage(uint32_t intVoltage);
float ConvertTemperature(uint32_t a);
float MeasureBatteryVoltage();
float GetNewOffsetAngle();
float GetNewTemperature();
uint8_t CheckBreakWire();
void SetStartingAngle();
float CalculateAngleFromGround(float yAcc, float zAcc);
float GetCurrentAngleFromGround();

#ifdef FEATURE_OAD
void BladeMonitor_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD

static void BladeMonitor_clockHandler(UArg arg);
void adcIsr(UArg a0);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t BladeMonitor_gapRoleCBs =
{
  BladeMonitor_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t BladeMonitor_BondMgrCBs =
{
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
#ifndef FEATURE_OAD
//static simpleProfileCBs_t BladeMonitor_simpleProfileCBs =
static BladeMonitorServiceCBs_t BladeMonitor_BladeMonitorServiceCBs =
{
  BladeMonitor_charValueChangeCB // Characteristic value change callback
};
#endif //!FEATURE_OAD

#ifdef FEATURE_OAD
static oadTargetCBs_t BladeMonitor_oadCBs =
{
  BladeMonitor_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      BladeMonitor_createTask
 *
 * @brief   Task creation function for the BladeMonitor.
 *
 */
void BladeMonitor_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;

  Task_construct(&sbpTask, BladeMonitor_taskFxn, &taskParams, NULL);

  // Construct semaphore used for pending in task
	Semaphore_Params sParams;
	Semaphore_Params_init(&sParams);
	sParams.mode = Semaphore_Mode_BINARY;

	Semaphore_construct(&sem2, 0, &sParams);
	hSem = Semaphore_handle(&sem2);
}

/*********************************************************************
 * @fn      BladeMonitor_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void BladeMonitor_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);
  madeIt++; //1

  Seconds_set(startDate); 		//sets the RTC to an initial value.
  // Hard code the BD Address till CC2650 board gets its own IEEE address
  //uint8 bdAddress[B_ADDR_LEN] = { 0xAD, 0xD0, 0x0A, 0xAD, 0xD0, 0x0A };
  //HCI_EXT_SetBDADDRCmd(bdAddress);
  currentTime = time(NULL);
  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(40);
  AONBatMonEnable();   // enable battery monitor
  // Setup I2C
  bspI2cInit();
  madeIt++;//2

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, BladeMonitor_clockHandler,
                      SBP_PERIODIC_EVT_PERIOD, 0, false, SBP_PERIODIC_EVT);

  //Create hardware interrupt for ADC sampling
  Hwi_Params hwiParams;
  Hwi_Params_init(&hwiParams);
  hwiParams.enableInt = true;

  Hwi_construct(&hwi, INT_AUX_ADC, adcIsr, &hwiParams, NULL);
  // Set up pins
  pinHandle = PIN_open(&pinState, BoardGpioInitTable);
  madeIt++;//3

  //set initial outputs
  PIN_setOutputValue(pinHandle, Board_TEMP_SENSE_ON, 0);   //TempSense is low until right before a reading.
  PIN_setOutputValue(pinHandle, Board_WIRE_OUT, 0);	  // Wireout is low until right before a reading.
  PIN_setOutputValue(pinHandle, Board_MPU_POWER, 0);  // Mpu_power is low until right before a reading.

  madeIt++;//4
  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);  //Minimum time upon connection establishment before the peripheral
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	 // starts a connection update procedure. In seconds (default 6 seconds)
  madeIt++;//5
  // Setup the GAP Peripheral Role Profile
  {
	//GAPRole parameters
    uint8_t initialAdvertEnable = TRUE;//Enable or disable advertising. Default is TRUE = enabled. For all hardware platforms,
    									//device starts advertising upon initialization
    uint16_t advertOffTime = 0; // How long to remain off after advertising stops before starting again. Default is 30 s.
    							//If set to 0, advertising will not start again.
    uint8_t enableUpdateRequest = 1;  //sends a request to update parameters.
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;//Minimum connection interval to allow (n × 125 ms). Range: 7.5 ms to 4s. Default is 7.5 ms.
    																//Also used for param update.
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;//Maximum connection interval to allow (n × 125 ms). Range: 7.5 ms to 4s.
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;  //Slave latency to use for a param update. Range: 0 – 499. Default is 0.
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;  //Current supervision time-out

    // Setting the parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t), &advertOffTime);
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);//Scan Response data. This is set in array scanRspData above
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData); //Advertisement data. Set in above array. Contains the service uuid
    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t), &enableUpdateRequest);//Set this to true to send a param update request.
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t), &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t), &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t), &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t), &desiredConnTimeout);
  }
  madeIt++;//6
  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);  //Minimum advertising interval in limited discovery mode (n × 0.625 ms)
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);  //Maximum advertising interval in limited discovery mode (n × 0.625 ms)
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);  //Minimum advertising interval in general discovery mode (n × 0.625 ms)
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);  //Maximum advertising interval in general discovery mode (n × 0.625 ms)
  }

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = 0; // passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t), &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }
  madeIt++;//7
   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  madeIt++;//8
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  madeIt++;//9
  DevInfo_AddService();                        // Device Information Service
  madeIt++;//10
  BladeMonitorService_AddService();			//Custom BladeMonitorService
  madeIt++;//11
/*#ifndef FEATURE_OAD
 // SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
#endif //!FEATURE_OAD*/

#ifdef FEATURE_OAD
  VOID OAD_addService();                 // OAD Profile
  OAD_register((oadTargetCBs_t *)&BladeMonitor_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
#endif

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE


#ifndef FEATURE_OAD

  // Register callback with SimpleGATTprofile
  BladeMonitorService_RegisterAppCBs(&BladeMonitor_BladeMonitorServiceCBs);
#endif //!FEATURE_OAD
  madeIt++;//12
  // Start the Device
  VOID GAPRole_StartDevice(&BladeMonitor_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&BladeMonitor_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  //Initialize readable characteristics
  BladeMonitorService_SetParameter(BLADEMONITORSERVICE_OFFSETANGLE, BLADEMONITORSERVICE_OFFSETANGLE_LEN, &currentSample.offsetAngle);
  BladeMonitorService_SetParameter(BLADEMONITORSERVICE_BATTERYVOLTAGE, BLADEMONITORSERVICE_BATTERYVOLTAGE_LEN, &currentSample.batteryVoltage);
  BladeMonitorService_SetParameter(BLADEMONITORSERVICE_BATTERYLIFE, BLADEMONITORSERVICE_BATTERYLIFE_LEN, &batteryLife);
  BladeMonitorService_SetParameter(BLADEMONITORSERVICE_BLADELIFE, BLADEMONITORSERVICE_BLADELIFE_LEN, &bladeLife);
  BladeMonitorService_SetParameter(BLADEMONITORSERVICE_TEMPERATURE, BLADEMONITORSERVICE_TEMPERATURE_LEN, &currentSample.temperature);
  BladeMonitorService_SetParameter(BLADEMONITORSERVICE_STARTDATE, BLADEMONITORSERVICE_STARTDATE_LEN, &startDate);
  BladeMonitorService_SetParameter(BLADEMONITORSERVICE_BLADETYPE, BLADEMONITORSERVICE_BLADETYPE_LEN, &bladeType);
  BladeMonitorService_SetParameter(BLADEMONITORSERVICE_BLADEWIDTH, BLADEMONITORSERVICE_BLADEWIDTH_LEN, &bladeWidth);
  BladeMonitorService_SetParameter(BLADEMONITORSERVICE_CUSTOMERSITE, BLADEMONITORSERVICE_CUSTOMERSITE_LEN, &customerSite);
  BladeMonitorService_SetParameter(BLADEMONITORSERVICE_CONVEYORNAME, BLADEMONITORSERVICE_CONVEYORNAME_LEN, &conveyorName);
  BladeMonitorService_SetParameter(BLADEMONITORSERVICE_SYSTEMTIME, BLADEMONITORSERVICE_SYSTEMTIME_LEN, &systemTime);

  	//start periodic Clock on initialization.
  	Util_startClock(&periodicClock);
  	madeIt++;//13
}

/*********************************************************************
 * @fn      BladeMonitor_taskFxn
 *
 * @brief   Application task entry point for the Battery Manager.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void BladeMonitor_taskFxn(UArg a0, UArg a1)
{

  // Initialize application
  BladeMonitor_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Event *pEvt = (ICall_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & SBP_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              BladeMonitor_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = BladeMonitor_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          BladeMonitor_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }

    if (events & SBP_PERIODIC_EVT)
    {
      events &= ~SBP_PERIODIC_EVT;

      Util_startClock(&periodicClock); //This restarts the clock...

      // Perform periodic application task
      BladeMonitor_performPeriodicTask();
    }

#ifdef FEATURE_OAD
    while (!Queue_empty(hOadQ))
    {
      oadTargetWrite_t *oadWriteEvt = Queue_dequeue(hOadQ);

      // Identify new image.
      if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
      {
        OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }
      // Write a next block request.
      else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
      {
        OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }

      // Free buffer.
      ICall_free(oadWriteEvt);
    }
#endif //FEATURE_OAD
  }
}

/*********************************************************************
 * @fn      BladeMonitor_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t BladeMonitor_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = BladeMonitor_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            break;

          default:
            break;
        }
      }
      break;

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      BladeMonitor_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t BladeMonitor_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      BladeMonitor_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
   /* LCD_WRITE_STRING_VALUE("FC Violated:", pMsg->msg.flowCtrlEvt.opcode,
                           10, LCD_PAGE5);*/
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    //LCD_WRITE_STRING_VALUE("MTU Size:", pMsg->msg.mtuEvt.MTU, 10, LCD_PAGE5);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      BladeMonitor_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void BladeMonitor_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      BladeMonitor_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      //LCD_WRITE_STRING_VALUE("Rsp send retry:", rspTxRetry, 10, LCD_PAGE5);
    }
  }
}

/*********************************************************************
 * @fn      BladeMonitor_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void BladeMonitor_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      //LCD_WRITE_STRING_VALUE("Rsp sent, retry:", rspTxRetry, 10, LCD_PAGE5);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      //LCD_WRITE_STRING_VALUE("Rsp retry failed:", rspTxRetry, 10, LCD_PAGE5);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      BladeMonitor_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void BladeMonitor_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBP_STATE_CHANGE_EVT:
      BladeMonitor_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

    case BM_VALUE_CHANGE_EVT:
      BladeMonitor_processValueChangeCB(pMsg->hdr.state);
      break;

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      BladeMonitor_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void BladeMonitor_stateChangeCB(gaprole_States_t newState)
{
  BladeMonitor_enqueueMsg(SBP_STATE_CHANGE_EVT, newState);
}

/*********************************************************************
 * @fn      BladeMonitor_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void BladeMonitor_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

      }
      break;

    case GAPROLE_ADVERTISING:
      break;

#ifdef PLUS_BROADCASTER
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of
     * state to the application.  These are then disabled here so that sending
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;

        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);

        advertEnabled = TRUE;

        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);

        // Reset flag for next connection.
        firstConnFlag = false;

        BladeMonitor_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        //Util_startClock(&periodicClock);

        //LCD_WRITE_STRING("Connected", LCD_PAGE2);
        //LCD_WRITE_STRING(Util_convertBdAddr2Str(peerAddress), LCD_PAGE3);

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);

            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      //LCD_WRITE_STRING("Connected Advertising", LCD_PAGE2);
      break;

    case GAPROLE_WAITING:
      //Util_stopClock(&periodicClock);
      BladeMonitor_freeAttRsp(bleNotConnected);

      //LCD_WRITE_STRING("Disconnected", LCD_PAGE2);

      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      BladeMonitor_freeAttRsp(bleNotConnected);

      //LCD_WRITE_STRING("Timed Out", LCD_PAGE2);


      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif //#ifdef (PLUS_BROADCASTER)
      break;

    case GAPROLE_ERROR:
      //LCD_WRITE_STRING("Error", LCD_PAGE2);
      break;

    default:
      //LCD_WRITE_STRING("", LCD_PAGE2);
      break;
  }

  // Update the state
  //gapProfileState = newState;
}

#ifndef FEATURE_OAD
/*********************************************************************
 * @fn      BladeMonitor_charValueChangeCB
 *
 * @brief   Callback from BladeMonitorService
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void BladeMonitor_charValueChangeCB(uint8_t paramID)
{
  BladeMonitor_enqueueMsg(BM_VALUE_CHANGE_EVT, paramID);
}
#endif //!FEATURE_OAD

/*********************************************************************
 * @fn      BladeMonitor_processValueChangeCB
 *
 * @brief   Process a pending Battery Manager characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void BladeMonitor_processValueChangeCB(uint8_t paramID)
{
#ifndef FEATURE_OAD

  switch(paramID)
  {
	  case BLADEMONITORSERVICE_STARTDATE:
		  SetStartingAngle();
		  break;
	  case BLADEMONITORSERVICE_SYSTEMTIME:
		  BladeMonitorService_GetParameter(BLADEMONITORSERVICE_SYSTEMTIME, &systemTime);
		  Seconds_set(systemTime);
		  break;
    default:
      // should not reach here!
      break;
  }
#endif //!FEATURE_OAD
}

/*********************************************************************
 * @fn      BladeMonitor_performPeriodicTask
 *		The board will wake up periodically to run this task.
 */
static void BladeMonitor_performPeriodicTask(void)
{
	if( active == 0){
		if (breakWire){// If the wire has not been broken, continue to check if the wire is broken until it is
			breakWire = CheckBreakWire();
		}else if (!breakWire){
		//Add startup delay in here after testing is complete
		SetStartingAngle();
		active = 1;// sets starting angle and active bit so the board stops checking the break wire.
		}
	}
	madeIt++; //14
	if (active){
		currentTime = time(NULL);
		currentSample.timeStamp = Seconds_get();
		madeIt++; //15
		numMeasurements++;
		currentSample.offsetAngle = GetNewOffsetAngle();
		BladeMonitorService_SetParameter(BLADEMONITORSERVICE_OFFSETANGLE, BLADEMONITORSERVICE_OFFSETANGLE_LEN, &currentSample.offsetAngle);
		madeIt++; //16
		currentSample.temperature = GetNewTemperature();
		BladeMonitorService_SetParameter(BLADEMONITORSERVICE_TEMPERATURE, BLADEMONITORSERVICE_TEMPERATURE_LEN, &currentSample.temperature);
		madeIt++;
		currentSample.batteryVoltage = MeasureBatteryVoltage();
		BladeMonitorService_SetParameter(BLADEMONITORSERVICE_BATTERYVOLTAGE, BLADEMONITORSERVICE_BATTERYVOLTAGE_LEN, &currentSample.batteryVoltage);
		madeIt++; //17
		if (numMeasurements > 10) {
			batteryLife = EstimateBatteryLife(currentSample.batteryVoltage);
			bladeLife = EstimateBladeLife(currentSample.offsetAngle);
		}
		BladeMonitorService_SetParameter(BLADEMONITORSERVICE_BATTERYLIFE, BLADEMONITORSERVICE_BATTERYLIFE_LEN, &batteryLife);
		BladeMonitorService_SetParameter(BLADEMONITORSERVICE_BLADELIFE, BLADEMONITORSERVICE_BLADELIFE_LEN, &bladeLife);
		madeIt++; //18
		StoreData(currentSample);
	}
}

#if defined(FEATURE_OAD)
/*********************************************************************
 * @fn      BladeMonitor_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
void BladeMonitor_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);

  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;

    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_enqueue(hOadQ, (Queue_Elem *)oadWriteEvt);

    // Post the application's semaphore.
    Semaphore_post(sem);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      BladeMonitor_clockHandler
 *
 * @brief   Handler function for clock timeouts. Stores the event argument so that it is recognized as a clock interrupt and
 * posts a semaphore to wake up the main program.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void BladeMonitor_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      BladeMonitor_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
static void BladeMonitor_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}
/*******************************************************************/
/*******************************************************************/
/**********  End of RTOS/BLE API functions   ***********************/
/*******************************************************************/
/*******************************************************************/
/*******************************************************************/
void adcIsr(UArg a0) {

  // Pop sample from FIFO to allow clearing ADC_IRQ event
  singleSample = AUXADCReadFifo();
  // Clear ADC_IRQ flag. Note: Missing driver for this.
  HWREGBITW(AUX_EVCTL_BASE + AUX_EVCTL_O_EVTOMCUFLAGSCLR, AUX_EVCTL_EVTOMCUFLAGSCLR_ADC_IRQ_BITN) = 1;

  // Post semaphore to wakeup task
  Semaphore_post(hSem);

}

/**********************************************************
 * Gets the Value from an analog input given the analogAddress
 * *********/
uint32_t GetADCValue(uint8_t analogAddress){
	// Enable clock for ADC digital and analog interface (not currently enabled in driver)
	AUXWUCClockEnable(AUX_WUC_MODCLKEN0_ANAIF_M|AUX_WUC_MODCLKEN0_AUX_ADI4_M);

	// Connect to analogAddress as analog input.
	AUXADCSelectInput(analogAddress);

	// Set up ADC
	AUXADCEnableSync(AUXADC_REF_FIXED, AUXADC_SAMPLE_TIME_10P9_MS, AUXADC_TRIGGER_MANUAL);

	// Disallow STANDBY mode while using the ADC.
	Power_setConstraint(Power_SB_DISALLOW);

	uint8_t currentSample = 0;
	uint32_t averageSample = 0;
	while(currentSample < SAMPLECOUNT) {
		//Sleep 1ms in IDLE mode
		Task_sleep(1000 / Clock_tickPeriod);
		madeIt = 2;
		// Trigger ADC sampling
		AUXADCGenManualTrigger();
		totalNumSamples++;
		// Wait in IDLE until done
		Semaphore_pend(hSem, BIOS_WAIT_FOREVER );
		adcSamples[currentSample++] = singleSample;
		averageSample += singleSample;
	}
	averageSample = averageSample / SAMPLECOUNT;
	// Disable ADC
	AUXADCDisable();
	// Allow STANDBY mode again
	Power_releaseConstraint(Power_SB_DISALLOW);
	return  averageSample;
}

/**********************************************************
 * Returns a float value for a voltage integer input given Vref=4.3, R1 = 1M, R2 = 44.2K, with ADC Input impedance = 1M, R2effective=42.329K
 * gives rDivider = 0.0406, but this had to be adjusted to rDivider = 0.0422 during calibration.
 * *********/
float ConvertVoltage(uint32_t  a){
	float b, floatVoltage, maxADC = 4095, rDivider = 0.0422;
	b = (float) a;
	floatVoltage = ((b  * 4.3)/ maxADC) / (rDivider);

	return floatVoltage;
}

/**********************************************************
 * Returns a float value for a temperature integer input from thermistor/resistor divider
 */
float ConvertTemperature(uint32_t a){
	float b = 0.0;
	int i = 0;
	b = (((float) a) * 4.3)/ 4095;
	tempTemp = b;
	while(tempLookup[i][0] && b > tempLookup[i][0]){
		i++;
	}
	if (i >= sizeof(tempLookup)){
		return 999.99;
	}
	if((tempLookup[i+1][0]-b) < (b - tempLookup[i][0])){
		b = tempLookup[i+1][1];
	}else{
		b = tempLookup[i][1];
	}
	return b;
}
/**********************************************************
 * Returns a float value for the offset Angle
 */
float GetNewOffsetAngle(){
	float newAngle = GetCurrentAngleFromGround();
	//subtract new position from old position
	//return new position
	return (startingAngle - newAngle);
}
/**********************************************************
 * Gets the current angle from ground using the average of 3 accelerometer readings.
 */
float GetCurrentAngleFromGround(){
	int16_t tempData[3] = {0};
	float accData[3] = {0};
	PIN_setOutputValue(pinHandle, Board_MPU_POWER, 1);//power up mpu-6500
	delay_ms(40);		//wait for mpu-6500 to power up
	MPU6500Init();
	//wait 20ms or until interrupt pin high, the take 3 readings and average them
	delay_ms(20);
	GetAccelerometerData(tempData);
	accData[0] += ((float) tempData[0]) / 16384;
	accData[1] += ((float) tempData[1]) / 16384;
	accData[2] += ((float) tempData[2]) / 16384;
	delay_ms(5);
	GetAccelerometerData(tempData);
	accData[0] += ((float) tempData[0]) / 16384;
	accData[1] += ((float) tempData[1]) / 16384;
	accData[2] += ((float) tempData[2]) / 16384;
	delay_ms(5);
	GetAccelerometerData(tempData);
	accData[0] += ((float) tempData[0]) / 16384;
	accData[1] += ((float) tempData[1]) / 16384;
	accData[2] += ((float) tempData[2]) / 16384;
	accData[0] /= 3;
	accData[1] /= 3;
	accData[2] /= 3;
	PIN_setOutputValue(pinHandle, Board_MPU_POWER, 0);//power down mpu-6500
	xAcc = accData[0];
	yAcc = accData[1];
	zAcc = accData[2];

	//calculate new position relative to ground.
	float newAngle;
	newAngle = CalculateAngleFromGround(yAcc, zAcc);
	return newAngle;
}

/**********************************************************
 * Takes in the y and z accelerometer values and calculates the board angle relative to the ground
 */

float CalculateAngleFromGround(float yAcc, float zAcc){
	float angle = 0.0;
	if (zAcc < 0 && yAcc > 0){
		angle = -atan(yAcc / zAcc) * 57.296;  //57.296 = 180/pi
	}else if (zAcc > 0 && yAcc > 0){
		angle = atan(zAcc / yAcc) * 57.296 + 90;
	}else if (zAcc > 0 && yAcc < 0){
		angle = -atan(yAcc / zAcc) * 57.296 + 180;
	}else if (zAcc < 0 && yAcc < 0){
		angle = atan(zAcc / yAcc) * 57.926 + 270;
	}
	return angle;
}

/**********************************************************
 * Returns a float value for the temperature from the thermistor
 */
float GetNewTemperature(){
	float temp = 0.0;
	PIN_setOutputValue(pinHandle, Board_TEMP_SENSE_ON, 1); //power on the thermistor
	delay_ms(1);
	temp = ConvertTemperature(GetADCValue(Board_A_TEMP_SENSE));//read the thermistor input and convert to float
	PIN_setOutputValue(pinHandle, Board_TEMP_SENSE_ON, 0); //power off the thermistor
	return temp;

}

/***************************************************************
 *	Uses batmon.h to get the current battery level in volts
 */
float MeasureBatteryVoltage(){
	uint32_t intVolts;
	intVolts = AONBatMonBatteryVoltageGet(); //Get battery voltage as integer
	intVolts = (intVolts * 125) >> 5;	//converts to millivolts
	return (float)intVolts;
}
/***************************************************************
 *	Checks for continuity of the Break Wire returns 1 if break wire is detected, 0 if not
 */

uint8_t CheckBreakWire(){
	uint8_t temp = 0;
	PIN_setOutputValue(pinHandle, Board_WIRE_OUT, 1);//apply signal to break wire.
	delay_ms(5); //wait 5 mS
	temp = PIN_getInputValue(Board_WIRE_IN);
	PIN_setOutputValue(pinHandle, Board_WIRE_OUT, 0);
	breakMake++;
	breakTest = temp;
	return temp;
}

/***************************************************************
 *	Sets the StartingAngle.
 */
void SetStartingAngle(){
	startingAngleSetTime = time(NULL);
	//use current angle as starting angle...
	startingAngle = GetCurrentAngleFromGround();
}


