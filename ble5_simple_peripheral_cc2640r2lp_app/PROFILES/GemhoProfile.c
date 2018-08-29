/**********************************************************************************************
 * Filename:       GemhoProfile.c
 *
 * Description:    This file contains the implementation of the service.
 *
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *************************************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/System.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "GemhoProfile.h"
#include <ti/drivers/UART.h>
#include "Board.h"
#include <stdio.h>
#include <ti/drivers/ADC.h>
#include <math.h>

#define GEMNOTIFY_TASK_STACK_SIZE                   2048
#define DELAY_MS(i)      (Task_sleep(((i) * 1000) / Clock_tickPeriod))
#define DELAY_US(i)      (Task_sleep((i) / Clock_tickPeriod))

// GemhoProfile Service UUID
CONST uint8_t GemhoProfileUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GEMHOPROFILE_SERV_UUID), HI_UINT16(GEMHOPROFILE_SERV_UUID)
};

// gemho_service UUID
CONST uint8_t GemhoProfile_Gemho_serviceUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(GEMHOPROFILE_GEMHO_SERVICE_UUID), HI_UINT16(GEMHOPROFILE_GEMHO_SERVICE_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static Task_Struct notifyTask;
static Char notifyTaskStack[GEMNOTIFY_TASK_STACK_SIZE];

static GemhoProfileCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t GemhoProfileDecl = { ATT_BT_UUID_SIZE, GemhoProfileUUID };

// Characteristic "Gemho_service" Properties (for declaration)
static uint8_t GemhoProfile_Gemho_serviceProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// Characteristic "Gemho_service" Value variable
static uint8_t GemhoProfile_Gemho_serviceVal[GEMHOPROFILE_GEMHO_SERVICE_LEN] = {0};

// Characteristic "Gemho_service" CCCD
static gattCharCfg_t *GemhoProfile_Gemho_serviceConfig;

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t GemhoProfileAttrTbl[] =
{
  // GemhoProfile Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&GemhoProfileDecl
  },
    // Gemho_service Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &GemhoProfile_Gemho_serviceProps
    },
      // Gemho_service Characteristic Value
      {
        { ATT_BT_UUID_SIZE, GemhoProfile_Gemho_serviceUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        GemhoProfile_Gemho_serviceVal
      },
      // Gemho_service CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&GemhoProfile_Gemho_serviceConfig
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t GemhoProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 *pLen, uint16 offset,
                                           uint16 maxLen, uint8 method );
static bStatus_t GemhoProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint16 len, uint16 offset,
                                            uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t GemhoProfileCBs =
{
  GemhoProfile_ReadAttrCB,  // Read callback function pointer
  GemhoProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

bStatus_t GemhoProfile_Notification(gattCharCfg_t *charCfgTbl, uint8 *pValue,
                                     uint8 authenticated,
                                     gattAttribute_t *attrTbl, uint16 numAttrs,
                                     uint8 *pData, uint16 dataLen)
{
    uint8 i;
    bStatus_t status = SUCCESS;

    // Verify input parameters
    if ((charCfgTbl == NULL) || (pValue == NULL) || (attrTbl == NULL))
    {
        return ( INVALIDPARAMETER);
    }

    for (i = 0; i < linkDBNumConns; i++)
    {
        gattCharCfg_t *pItem = &(charCfgTbl[i]);

        if ((pItem->connHandle != INVALID_CONNHANDLE) && (pItem->value != GATT_CFG_NO_OPERATION))
        {
            gattAttribute_t *pAttr;

            // Find the characteristic value attribute
            pAttr = GATTServApp_FindAttr(attrTbl, numAttrs, pValue);
            if (pAttr != NULL)
            {
                if (pItem->value & GATT_CLIENT_CFG_NOTIFY)
                {
                    attHandleValueNoti_t noti;
                    uint16 len;

                    // If the attribute value is longer than (ATT_MTU - 3) octets, then
                    // only the first (ATT_MTU - 3) octets of this attributes value can
                    // be sent in a notification.
                    noti.pValue = (uint8 *) GATT_bm_alloc(pItem->connHandle, ATT_HANDLE_VALUE_NOTI, dataLen, &len);
                    if (noti.pValue != NULL && dataLen <= len)
                    {
                        noti.handle = pAttr->handle;
                        noti.len = dataLen;
                        memcpy( noti.pValue, pData, dataLen);

                        status = GATT_Notification(pItem->connHandle, &noti, authenticated);
                    }

                    if (status != SUCCESS)
                    {
                        GATT_bm_free((gattMsg_t *) &noti, ATT_HANDLE_VALUE_NOTI);
                    }
                }
            }
        }
    } // for

    return (status);
}

#if 0
static void SimpleNotify_taskFxn(UArg a0, UArg a1)
{

//    ADC_Handle   adc_12, adc_11, adc_8;
//    ADC_Params   params;
//    int_fast16_t res12=0, res11=0, res8=0;
//    uint16_t adcValue12, adcValue11, adcValue8;
//
//    ADC_init();
//    ADC_Params_init(&params);
//    adc_12 = ADC_open(CC2640R2DK_5XD_ADC5, &params);
//    adc_11 = ADC_open(CC2640R2DK_5XD_ADC4, &params);
//    adc_8 = ADC_open(CC2640R2DK_5XD_ADC1, &params);
//
//    while(1)
//    {
//        if(linkDB_NumActive() > 0)
//        {
//            double adcValueMicroVolt_12, adcValueMicroVolt_11, adcValueMicroVolt_8;
//            char buf[128] = "";
//
//            res12 = ADC_convert(adc_12, &adcValue12);
//            res11 = ADC_convert(adc_11, &adcValue11);
//            res8 = ADC_convert(adc_8, &adcValue8);
//
//            if (res12 == ADC_STATUS_SUCCESS && res11 == ADC_STATUS_SUCCESS && res8 == ADC_STATUS_SUCCESS)
//            {
//                adcValueMicroVolt_12 = ADC_convertToMicroVolts(adc_12, adcValue12)/1000.0;
//                adcValueMicroVolt_11 = ADC_convertToMicroVolts(adc_11, adcValue11)/1000.0;
//                adcValueMicroVolt_8 = ADC_convertToMicroVolts(adc_8, adcValue8)/1000.0;
//                sprintf(buf, "%.f %.f %.f", adcValueMicroVolt_12, adcValueMicroVolt_11, adcValueMicroVolt_8);
//                GemhoProfile_Notification(GemhoProfile_Gemho_serviceConfig, (uint8_t *)&GemhoProfile_Gemho_serviceVal, FALSE,
//                                          GemhoProfileAttrTbl, GATT_NUM_ATTRS( GemhoProfileAttrTbl ),
//                                          (uint8 *)buf, strlen(buf));
//            }
//
//            DELAY_MS(200);
//        }
//        else
//        {
//            DELAY_MS(1000);
//        }
//    }

    ADC_Handle   adc_13, adc_12, adc_8;
    ADC_Params   params;
    int_fast16_t res13=0, res12=0, res8=0;
    uint16_t adcValue13, adcValue12, adcValue8;

    ADC_init();
    ADC_Params_init(&params);
    adc_13 = ADC_open(CC2640R2DK_5XD_ADC6, &params);
    adc_12 = ADC_open(CC2640R2DK_5XD_ADC5, &params);
    adc_8 = ADC_open(CC2640R2DK_5XD_ADC1, &params);

    while(1)
    {
        if(linkDB_NumActive() > 0)
        {
            double adcValueMicroVolt_13, adcValueMicroVolt_12, adcValueMicroVolt_8;
            char buf[128] = "";

            res13 = ADC_convert(adc_13, &adcValue13);
            res12 = ADC_convert(adc_12, &adcValue12);
            res8 = ADC_convert(adc_8, &adcValue8);

            if (res13 == ADC_STATUS_SUCCESS && res12 == ADC_STATUS_SUCCESS && res8 == ADC_STATUS_SUCCESS)
            {
                adcValueMicroVolt_13 = ADC_convertToMicroVolts(adc_13, adcValue13)/1000.0;
                adcValueMicroVolt_12 = ADC_convertToMicroVolts(adc_12, adcValue12)/1000.0;
                adcValueMicroVolt_8 = ADC_convertToMicroVolts(adc_8, adcValue8)/1000.0;
                sprintf(buf, "%.f %.f %.f", adcValueMicroVolt_13, adcValueMicroVolt_12, adcValueMicroVolt_8);
                GemhoProfile_Notification(GemhoProfile_Gemho_serviceConfig, (uint8_t *)&GemhoProfile_Gemho_serviceVal, FALSE,
                                          GemhoProfileAttrTbl, GATT_NUM_ATTRS( GemhoProfileAttrTbl ),
                                          (uint8 *)buf, strlen(buf));
            }

            DELAY_MS(200);
        }
        else
        {
            DELAY_MS(1000);
        }
    }


//    ADC_Handle   adc_13, adc_12, adc_11, adc_8;
//    ADC_Params   params;
//    int_fast16_t res13=0, res12=0, res11=0, res8=0;
//    uint16_t adcValue13, adcValue12, adcValue11, adcValue8;
//
//    ADC_init();
//    ADC_Params_init(&params);
//    adc_13 = ADC_open(CC2640R2DK_5XD_ADC6, &params);
//    adc_12 = ADC_open(CC2640R2DK_5XD_ADC5, &params);
//    adc_11 = ADC_open(CC2640R2DK_5XD_ADC4, &params);
//    adc_8 = ADC_open(CC2640R2DK_5XD_ADC1, &params);
//
//    while(1)
//    {
//        if(linkDB_NumActive() > 0)
//        {
//            double adcValueMicroVolt_13, adcValueMicroVolt_12, adcValueMicroVolt_11, adcValueMicroVolt_8;
//            char buf[128] = "";
//
//            res13 = ADC_convert(adc_13, &adcValue13);
//            res12 = ADC_convert(adc_12, &adcValue12);
//            res11 = ADC_convert(adc_11, &adcValue11);
//            res8 = ADC_convert(adc_8, &adcValue8);
//
//            if (res13 == ADC_STATUS_SUCCESS && res12 == ADC_STATUS_SUCCESS && res11 == ADC_STATUS_SUCCESS && res8 == ADC_STATUS_SUCCESS)
//            {
//                adcValueMicroVolt_13 = ADC_convertToMicroVolts(adc_13, adcValue13)/1000.0;
//                adcValueMicroVolt_12 = ADC_convertToMicroVolts(adc_12, adcValue12)/1000.0;
//                adcValueMicroVolt_11 = ADC_convertToMicroVolts(adc_11, adcValue11)/1000.0;
//                adcValueMicroVolt_8 = ADC_convertToMicroVolts(adc_8, adcValue8)/1000.0;
//                sprintf(buf, "%.f %.f %.f %.f", adcValueMicroVolt_13, adcValueMicroVolt_12,
//                        adcValueMicroVolt_11, adcValueMicroVolt_8);
//                GemhoProfile_Notification(GemhoProfile_Gemho_serviceConfig, (uint8_t *)&GemhoProfile_Gemho_serviceVal, FALSE,
//                                          GemhoProfileAttrTbl, GATT_NUM_ATTRS( GemhoProfileAttrTbl ),
//                                          (uint8 *)buf, strlen(buf));
//            }
//
//            DELAY_MS(200);
//        }
//        else
//        {
//            DELAY_MS(1000);
//        }
//    }
}
#else
static void SimpleNotify_taskFxn(UArg a0, UArg a1)
{
    ADC_Handle   adc, vdds;
    ADC_Params   params;
    int_fast16_t res0=0, res1=0;
    uint16_t adcValue0, adcValue1;

    ADC_init();
    ADC_Params_init(&params);
    adc = ADC_open(CC2640R2DK_5XD_ADC6, &params);
    vdds = ADC_open(CC2640R2DK_5XD_ADC1, &params);

    if (adc == NULL || vdds == NULL)
    {
        while (1)
            DELAY_MS(1000);
    }

    while(1)
    {
        if(linkDB_NumActive() > 0)
        {
            double adcValue0MicroVolt, adcVDDSMicroVolt;
            uint32 loopCount = 1000*5;
            double temperature = 0;
            double RT = 0;
            const double B=3949;
            const double TN=273.15+37;//常温
            const double RN=30.218;//常温对应的阻值，注意单位是千欧
            char buf[128] = "";
            uint32 tick1, tick2;

            tick1 = Clock_getTicks();

            adcValue0MicroVolt = 0;
            adcVDDSMicroVolt = 0;
            for(int i=0; i<loopCount; i++)
            {
                res0 = ADC_convert(adc, &adcValue0);
                res1 = ADC_convert(vdds, &adcValue1);

                if (res0 == ADC_STATUS_SUCCESS && res1 == ADC_STATUS_SUCCESS)
                {
                    adcValue0MicroVolt += ADC_convertToMicroVolts(adc, adcValue0)/1000.0;
                    adcVDDSMicroVolt += ADC_convertToMicroVolts(vdds, adcValue1)/1000.0;
                }
            }

            RT = 30.0*adcValue0MicroVolt/(adcVDDSMicroVolt-adcValue0MicroVolt);
            temperature = 1/(1/TN + log(RT/RN)/B)-273.15;

            tick2 = Clock_getTicks();

            sprintf(buf, "%.4f %.4f %d", RT, temperature, (tick2-tick1)*Clock_tickPeriod/1000);
            sprintf((char *)GemhoProfile_Gemho_serviceVal, "%.4f %.4f", RT, temperature);
//            sprintf(buf, "%.4f %.4f %.4f %.4f %d %d", adcValue0MicroVolt/1000000.0, adcVDDSMicroVolt/1000000.0, RT, temperature,
//                    adcValue0, adcValue1);
            GemhoProfile_Notification(GemhoProfile_Gemho_serviceConfig, (uint8_t *)&GemhoProfile_Gemho_serviceVal, FALSE,
                                      GemhoProfileAttrTbl, GATT_NUM_ATTRS( GemhoProfileAttrTbl ),
                                      (uint8 *)buf, strlen(buf));

            DELAY_MS(1000);
        }
        else
        {
            DELAY_MS(1000);
        }
    }

}
#endif


bStatus_t GemhoProfile_AddService( void )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  GemhoProfile_Gemho_serviceConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( GemhoProfile_Gemho_serviceConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, GemhoProfile_Gemho_serviceConfig );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( GemhoProfileAttrTbl,
                                        GATT_NUM_ATTRS( GemhoProfileAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &GemhoProfileCBs );

  Task_Params taskParams;
  Task_Params_init(&taskParams);
  taskParams.stack = notifyTaskStack;
  taskParams.stackSize = GEMNOTIFY_TASK_STACK_SIZE;
  taskParams.priority = 1;
  Task_construct(&notifyTask, SimpleNotify_taskFxn, &taskParams, NULL);


  return ( status );
}

/*
 * GemhoProfile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t GemhoProfile_RegisterAppCBs( GemhoProfileCBs_t *appCallbacks )
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
 * GemhoProfile_SetParameter - Set a GemhoProfile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t GemhoProfile_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case GEMHOPROFILE_GEMHO_SERVICE:
      if ( len == GEMHOPROFILE_GEMHO_SERVICE_LEN )
      {
        memcpy(GemhoProfile_Gemho_serviceVal, value, len);

        // Try to send notification.
//        GATTServApp_ProcessCharCfg( GemhoProfile_Gemho_serviceConfig, (uint8_t *)&GemhoProfile_Gemho_serviceVal, FALSE,
//                                    GemhoProfileAttrTbl, GATT_NUM_ATTRS( GemhoProfileAttrTbl ),
//                                    INVALID_TASK_ID,  GemhoProfile_ReadAttrCB);
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
 * GemhoProfile_GetParameter - Get a GemhoProfile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t GemhoProfile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case GEMHOPROFILE_GEMHO_SERVICE:
      memcpy(value, GemhoProfile_Gemho_serviceVal, GEMHOPROFILE_GEMHO_SERVICE_LEN);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          GemhoProfile_ReadAttrCB
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
static bStatus_t GemhoProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                       uint8 *pValue, uint16 *pLen, uint16 offset,
                                       uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the Gemho_service Characteristic Value
if ( ! memcmp(pAttr->type.uuid, GemhoProfile_Gemho_serviceUUID, pAttr->type.len) )
  {
    if ( offset > GEMHOPROFILE_GEMHO_SERVICE_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, GEMHOPROFILE_GEMHO_SERVICE_LEN - offset);  // Transmit as much as possible
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
 * @fn      GemhoProfile_WriteAttrCB
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
static bStatus_t GemhoProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                        uint8 *pValue, uint16 len, uint16 offset,
                                        uint8 method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  // See if request is regarding the Gemho_service Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, GemhoProfile_Gemho_serviceUUID, pAttr->type.len) )
  {
    if ( offset + len > GEMHOPROFILE_GEMHO_SERVICE_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == GEMHOPROFILE_GEMHO_SERVICE_LEN)
        paramID = GEMHOPROFILE_GEMHO_SERVICE;

//      uint8 testChar[] = "123456789";
//
//      GemhoProfile_Notification( GemhoProfile_Gemho_serviceConfig, (uint8_t *)&GemhoProfile_Gemho_serviceVal, FALSE,
//                                  GemhoProfileAttrTbl, GATT_NUM_ATTRS( GemhoProfileAttrTbl ),
//                                  testChar, sizeof(testChar));

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


