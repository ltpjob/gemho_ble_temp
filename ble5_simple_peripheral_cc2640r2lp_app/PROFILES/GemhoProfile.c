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
#include "ThinkGearStreamParser.h"
#include <stdio.h>

#define GEMNOTIFY_TASK_STACK_SIZE                   2048
#define DELAY_MS(i)      (Task_sleep(((i) * 1000) / Clock_tickPeriod))

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

void handleDataValueFunc(unsigned char extendedCodeLevel,
                                unsigned char code, unsigned char numBytes,
                                const unsigned char *value, void *customData)
{
    static uint8 poorSignal = 0;
    static uint8 heartRate = 0;
//    static int16 rawWave = 0;
    static uint8 count = 0;

    if (extendedCodeLevel == 0)
    {
        if(code == 0x80)
        {
            static uint8 buffer[20] = "";

//            rawWave = (value[0]<<8) | value[1];
            buffer[count*2] = value[0];
            buffer[count*2+1] = value[1];
            count++;

            if(count >= 9)
            {
                buffer[count*2] = poorSignal;
                buffer[count*2+1] = heartRate;
                count = 0;
                GemhoProfile_Notification( GemhoProfile_Gemho_serviceConfig, (uint8_t *)&GemhoProfile_Gemho_serviceVal, FALSE,
                                            GemhoProfileAttrTbl, GATT_NUM_ATTRS( GemhoProfileAttrTbl ),
                                            (uint8 *)buffer, sizeof(buffer));
            }


//            snprintf(buffer, sizeof(buffer), "%u,%d,%u", poorSignal, rawWave, heartRate);
//            GemhoProfile_Notification( GemhoProfile_Gemho_serviceConfig, (uint8_t *)&GemhoProfile_Gemho_serviceVal, FALSE,
//                                        GemhoProfileAttrTbl, GATT_NUM_ATTRS( GemhoProfileAttrTbl ),
//                                        (uint8 *)buffer, strlen(buffer));


        }
        else if(code == 0x02)
        {
            poorSignal = value[0];
        }
        else if(code == 0x03)
        {
            heartRate = value[0];
        }
    }

}

static void SimpleNotify_taskFxn(UArg a0, UArg a1)
{
    char input[128];
    UART_Handle uart;
    UART_Params uartParams;
    ThinkGearStreamParser parser;

    UART_init();

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 57600;

    uart = UART_open(Board_UART0, &uartParams);

    if(uart == NULL)
    {
        while (1);
    }

    THINKGEAR_initParser (&parser, PARSER_TYPE_PACKETS, handleDataValueFunc, NULL);

    while(1)
    {
        if(linkDB_NumActive() > 0)
        {
            int len = UART_read(uart, input, sizeof(input));
            for(int i=0; i<len; i++)
            {
                THINKGEAR_parseByte (&parser, input[i]);
            }
        }
        else
        {
            DELAY_MS(100);
        }
    }

}


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
        GATTServApp_ProcessCharCfg( GemhoProfile_Gemho_serviceConfig, (uint8_t *)&GemhoProfile_Gemho_serviceVal, FALSE,
                                    GemhoProfileAttrTbl, GATT_NUM_ATTRS( GemhoProfileAttrTbl ),
                                    INVALID_TASK_ID,  GemhoProfile_ReadAttrCB);
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


