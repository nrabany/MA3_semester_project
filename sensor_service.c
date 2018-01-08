/**
  ******************************************************************************
  * @file    sensor_service.c
  * @author  Central LAB
  * @version V1.0.0
  * @date    21-Nov-2016
  * @brief   Add bluetooth services using vendor specific profiles.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include <stdio.h>
#include "TargetFeatures.h"
#include "main.h"
#include "sensor_service.h"
#include "console.h"
#include "bluenrg_utils.h"
#include "bluenrg_l2cap_aci.h"
#include "uuid_ble_service.h"
#include "bluenrg_gap_aci.h"

/* Exported variables ---------------------------------------------------------*/
int connected = FALSE;
uint8_t set_connectable = TRUE;
int temperature_threshold;
int humidity_threshold;
/* Imported Variables -------------------------------------------------------------*/
extern uint32_t ConnectionBleStatus;

extern TIM_HandleTypeDef    TimCCHandle;

extern uint8_t bdaddr[6];

/* Private variables ------------------------------------------------------------*/
static uint16_t HWServW2STHandle;
static uint16_t EnvironmentalCharHandle;
static uint16_t LedCharHandle;

static uint16_t ConfigServW2STHandle;
static uint16_t ConfigCharHandle;

static uint16_t ConsoleW2STHandle;
static uint16_t TermCharHandle;
static uint16_t StdErrCharHandle;

static uint8_t LastStderrBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastStderrLen;
static uint8_t LastTermBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastTermLen;

static uint8_t  EnvironmentalCharSize=2; /* Size for Environmental BLE characteristic */

static uint16_t connection_handle = 0;

/* Private functions ------------------------------------------------------------*/
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
static void GAP_DisconnectionComplete_CB(void);
static void ConfigCommandParsing(uint8_t * att_data, uint8_t data_length);

/* Private define ------------------------------------------------------------*/

#ifdef ACC_BLUENRG_CONGESTION
#define ACI_GATT_UPDATE_CHAR_VALUE safe_aci_gatt_update_char_value
static int32_t breath;

PeripheralDevice perDevs;

/* @brief  Update the value of a characteristic avoiding (for a short time) to
 *         send the next updates if an error in the previous sending has
 *         occurred.
 * @param  servHandle The handle of the service
 * @param  charHandle The handle of the characteristic
 * @param  charValOffset The offset of the characteristic
 * @param  charValueLen The length of the characteristic
 * @param  charValue The pointer to the characteristic
 * @retval tBleStatus Status
 */


tBleStatus safe_aci_gatt_update_char_value(uint16_t servHandle, 
				      uint16_t charHandle,
				      uint8_t charValOffset,
				      uint8_t charValueLen,   
				      const uint8_t *charValue)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
  
  if (breath > 0) {
    breath--;
  } else {
    ret = aci_gatt_update_char_value(servHandle,charHandle,charValOffset,charValueLen,charValue);
    
    if (ret != BLE_STATUS_SUCCESS){
      breath = ACC_BLUENRG_CONGESTION_SKIP;
    }
  }
  
  return (ret);
}

#else /* ACC_BLUENRG_CONGESTION */
#define ACI_GATT_UPDATE_CHAR_VALUE aci_gatt_update_char_value
#endif /* ACC_BLUENRG_CONGESTION */


/**
 * @brief  Add the Config service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConfigW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_CONFIG_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3,&ConfigServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS)
    goto fail;

  COPY_CONFIG_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, uuid, 20 /* Max Dimension */,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &ConfigCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  //STLBLE_PRINTF("Error while adding Configuration service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update Stderr characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Stderr_Update(uint8_t *data,uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages*/
  for(Offset =0; Offset<length; Offset +=W2ST_CONSOLE_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastStderrBuffer,data+Offset,DataToSend);
    LastStderrLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
      return BLE_STATUS_ERROR;
    }
    HAL_Delay(10);
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Term_Update(uint8_t *data,uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages */
  for(Offset =0; Offset<length; Offset +=W2ST_CONSOLE_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastTermBuffer,data+Offset,DataToSend);
    LastTermLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
        STLBLE_PRINTF("Error Updating Stdout Char\r\n");
      return BLE_STATUS_ERROR;
    }
    HAL_Delay(20);
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Stderr characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Stderr_Update_AfterRead(void)
{
  tBleStatus ret;

  ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, LastStderrLen , LastStderrBuffer);
  if (ret != BLE_STATUS_SUCCESS) {
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Term_Update_AfterRead(void)
{
  tBleStatus ret;

  ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, LastTermLen , LastTermBuffer);
  if (ret != BLE_STATUS_SUCCESS) {
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Stdout Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      STLBLE_PRINTF("Error Updating Stdout Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}
/* @brief  Send a notification for answering to a configuration command for Accelerometer events
 * @param  uint32_t Feature Feature calibrated
 * @param  uint8_t Command Replay to this Command
 * @param  uint8_t data result to send back
 * @retval tBleStatus Status
 */
tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t data)
{
  tBleStatus ret;
  uint8_t buff[2+4+1+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_BE_32(buff+2,Feature);
  buff[6] = Command;
  buff[7] = data;

  ret = aci_gatt_update_char_value (ConfigServW2STHandle, ConfigCharHandle, 0, 8,buff);
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Configuration Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      STLBLE_PRINTF("Error Updating Configuration Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Add the HW Features service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_HWServW2ST_Service(void)
{
  tBleStatus ret;
  int32_t NumberChars = 4;

  uint8_t uuid[16];

  COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE,
                          1+3*NumberChars,
                          &HWServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  /* Fill the Environmental BLE Characteristc */
  COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid);
    if(TargetBoardFeatures.NumTempSensors==2) {
      uuid[14] |= 0x05; /* Two Temperature values*/
      EnvironmentalCharSize+=2*2;
    } else if(TargetBoardFeatures.NumTempSensors==1) {
      uuid[14] |= 0x04; /* One Temperature value*/
      EnvironmentalCharSize+=2;
    }

    if(TargetBoardFeatures.HandleHumSensor) {
     uuid[14] |= 0x08; /* Humidity */
     EnvironmentalCharSize+=2;
    }

    if(TargetBoardFeatures.HandlePressSensor) {
      uuid[14] |= 0x10; /* Pressure value*/
      EnvironmentalCharSize+=4;
    }

  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, EnvironmentalCharSize,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &EnvironmentalCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_LED_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &LedCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  //STLBLE_PRINTF("Error while adding HW's Characteristcs service.\n");
  return BLE_STATUS_ERROR;
}


/**
 * @brief  Update LEDs characteristic value
 * @param  uint8_t LedStatus LEDs status 0/1 (off/on)
 * @retval tBleStatus   Status
 */
tBleStatus LED_Update(uint8_t LedStatus)
{
  tBleStatus ret;

  uint8_t buff[2+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = LedStatus;

  ret = aci_gatt_update_char_value(HWServW2STHandle, LedCharHandle, 0, 2+1,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating LED Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      STLBLE_PRINTF("Error Updating Temp Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}


/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t addr[6] Address of peer device
 * @param  uint16_t handle Connection handle
 * @retval None
 */
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{  
  connected = TRUE;
  connection_handle = handle;

#ifdef ENABLE_USB_DEBUG_CONNECTION
  STLBLE_PRINTF(">>>>>>CONNECTED %x:%x:%x:%x:%x:%x\r\n",addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]);
#endif /* ENABLE_USB_DEBUG_CONNECTION */

  ConnectionBleStatus=0;
  
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
static void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;

#ifdef ENABLE_USB_DEBUG_CONNECTION  
  STLBLE_PRINTF("<<<<<<DISCONNECTED\r\n");
#endif /* ENABLE_USB_DEBUG_CONNECTION */  

  /* Make the device connectable again. */
  set_connectable = TRUE;

  ConnectionBleStatus=0;

  if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
    /* Stopping Error */
    Error_Handler();
  }
}

/**
 * @brief  This function is called when there is a change on the gatt attribute
 * With this function it's possible to understand if one application 
 * is subscribed or not to the one service
 * @param uint16_t att_handle Handle of the attribute
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length)
{
  if(attr_handle == ConfigCharHandle + 2) {
    ;/* do nothing... only for removing the message "Notification UNKNOW handle" */
  }
  else if(attr_handle == EnvironmentalCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_ENV);

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }

      /* Set the new Capture compare value */
      {
        uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
        /* Set the Capture Compare Register value */
        __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
      }
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_ENV);

      /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }
    }
#ifdef ENABLE_USB_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Env=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV) ? "ON" : "OFF");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      STLBLE_PRINTF("--->Env=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV) ? "ON" : "OFF");
#endif /* ENABLE_USB_DEBUG_CONNECTION */
  }
  else if(attr_handle == StdErrCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_STD_ERR);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_ERR);
    }
  } else if(attr_handle == TermCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_STD_TERM);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_TERM);
    }
  } 
  else if(attr_handle == LedCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_LED);
      /* Update the LED feature */
      LED_Update(TargetBoardFeatures.LedStatus);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_LED);
    }
#ifdef ENABLE_USB_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Led=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_LED) ? "ON" : "OFF");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      STLBLE_PRINTF("--->Led=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_LED) ? "ON" : "OFF");
#endif /* ENABLE_USB_DEBUG_CONNECTION */
  } else if (attr_handle == ConfigCharHandle + 1) {
    /* Received one write command from Client on Configuration characteristc */
    ConfigCommandParsing(att_data, data_length);    
  } else {
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Notification UNKNOW handle\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      STLBLE_PRINTF("Notification UNKNOW handle\r\n");
    }
  }
}

/**
 * @brief  This function makes the parsing of the Configuration Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
static void ConfigCommandParsing(uint8_t * att_data, uint8_t data_length)
{
  uint32_t FeatureMask = (att_data[3]) | (att_data[2]<<8) | (att_data[1]<<16) | (att_data[0]<<24);
  uint8_t Command = att_data[4];
  uint8_t Data    = att_data[5];

  switch (FeatureMask) {
    case FEATURE_MASK_LED:
      /* Led events */
#ifdef ENABLE_USB_DEBUG_CONNECTION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
        BytesToWrite = sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%2x\n\r",FeatureMask,Command);
        Term_Update(BufferToWrite,BytesToWrite);
      } else {
        STLBLE_PRINTF("Conf Sig F=%lx C=%2x\r\n",FeatureMask,Command);
      }
#endif /* ENABLE_USB_DEBUG_CONNECTION */
     switch(Command) {
      case 1:
        TargetBoardFeatures.LedStatus=1;
        LedOnTargetPlatform();
        Config_Notify(FEATURE_MASK_LED,Command,Data);
        break;
      case 0:
        TargetBoardFeatures.LedStatus=0;
        LedOffTargetPlatform();
        Config_Notify(FEATURE_MASK_LED,Command,Data);
        break;
     }
     /* Update the LED feature */
     if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_LED)) {
       LED_Update(TargetBoardFeatures.LedStatus);
     }
    break;
  }
}

/**
 * @brief  This function is called whenever there is an ACI event to be processed.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void *pckt Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
  if(hci_pckt->type != HCI_EVENT_PKT) {
    return;
  }
  
  switch(event_pckt->evt){
    
  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;
  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;
      
      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
          /* Connection parameters:
          cc->status: connection status (0x00: Connection successfully
          completed);
          cc->handle: connection handle to be used for the communication during
          the connection;
          cc->role: BLE device role (0x01: master; 0x02: slave);
          cc->peer_bdaddr_type: connected device address type (0x00: public;
          0x01: random);
          cc->peer_bdaddr: connected device address;
          cc->interval: connection interval;
          cc->latency: connection latency;
          cc->supervision_timeout: connection supervision timeout;
          cc->master_clock_accuracy: master clock accuracy;*/
          /* Add user code for handling connection event based on application
          scenarios */
          connection_handle = cc->handle;
        }
        break;
      case EVT_LE_ADVERTISING_REPORT: /* BlueNRG-MS stack */
       {
    	  //printf("EVT_LE_ADVERTISING_REPORT\n");
		  tBleStatus ret;
		  le_advertising_info *pr = (void *)(evt->data+1); /* evt->data[0] is
		  number of reports (On BlueNRG-MS is always 1) */
		  /* le_advertising_info parameters:
		  pr->evt_type: event type (advertising packets types);
		  pr->bdaddr_type: type of the peer address (PUBLIC_ADDR,RANDOM_ADDR);
		  pr->bdaddr: address of the peer device found during scanning;
		  pr->length: length of advertising or scan response data;
		  pr->data_RSSI[]: length advertising or scan response data + RSSI.
		  RSSI is last octect (signed integer).*/
		  /* Add user code for decoding the le_advertising_info event data based
		  on the specific pr->evt_type (ADV_IND, SCAN_RSP, ..)*/
		  tBDAddr GAP_Peripheral_addr = {0x4d,0x56,0x30,0x47,0x6e,0xc0};
		  uint8_t correct_addr = 1;
		  for(int i=0;i<6;i++)
		  {
			  if(pr->bdaddr[i]!=GAP_Peripheral_addr[i])
				  correct_addr = 0;
		  }
		  if(correct_addr == 1){
			   ret = aci_gap_create_connection(0x4000, 0x4000, pr->bdaddr_type, pr->bdaddr, PUBLIC_ADDR, 40, 40, 0, 60, 2000 , 2000);
			   if (ret != BLE_STATUS_SUCCESS)
			   {
				   PRINTF("Failure.\n")
			   }
       	   }
		  else
		  {
			  aci_gap_terminate_gap_procedure(0x02); //Otherwise, if discovery procedure not terminated --> can't discover anymore
		  }
		   //Print adresse of device found
		   //printf("bdaddr1=[%02x %02x %02x %02x %02x %02x]\n", pr->bdaddr[5], pr->bdaddr[4], pr->bdaddr[3], pr->bdaddr[2], pr->bdaddr[1], pr->bdaddr[0]);
		   //printf("bdaddr2=[%02x %02x %02x %02x %02x %02x]\n", pr->bdaddr[11], pr->bdaddr[10], pr->bdaddr[9], pr->bdaddr[8], pr->bdaddr[7], pr->bdaddr[6]);

       }/* EVT_LE_ADVERTISING_REPORT */
       break;

      }
    }
    break;
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode){
      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          evt_gatt_read_permit_req *pr = (void*)blue_evt->data; 
        }
        break;
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        if(TargetBoardFeatures.bnrg_expansion_board==IDB05A1) {
              evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
              Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
            } else {
              evt_gatt_attr_modified_IDB04A1 *evt = (evt_gatt_attr_modified_IDB04A1*)blue_evt->data;
              Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
            }
        break;
      case EVT_BLUE_ATT_READ_RESP:
      {
    	  //printf("evt in EVT_BLUE_ATT_READ_RESP\n");
		  evt_att_read_resp *evt = (void*)blue_evt->data;
		  Char_Read_Value_CB(evt->conn_handle, evt->event_data_length, evt->attribute_value);
		}
      break;
      case EVT_BLUE_GATT_NOTIFICATION:
	   {
		   //printf("evt in EVT_BLUE_GATT_NOTIFICATION\n");
		   evt_gatt_attr_notification *evt = (evt_gatt_attr_notification*)blue_evt->data;
		   /*evt_gatt_attr_notification data:
		   evt->conn_handle: connection handle;
		   evt->event_data_length: length of attribute value + handle (2 bytes);
		   evt->attr_handle: handle of the notified characteristic;
		   evt->attr_value[]: characteristic value.
		   Add user code for handling the received notification based on the
		   application scenario.
		   */
		}
	   break;
      case EVT_BLUE_GAP_PROCEDURE_COMPLETE:
      {
      /* When the general discovery procedure is terminated
       EVT_BLUE_GAP_PROCEDURE_COMPLETE event is returned with the procedure code set to GAP_GENERAL_DISCOVERY_PROC (0x02).*/
		evt_gap_procedure_complete *pr = (void*)blue_evt->data;
		/* evt_gap_procedure_complete parameters:
		pr->procedure_code: terminated procedure code;
		pr->status: BLE_STATUS_SUCCESS, BLE_STATUS_FAILED or ERR_AUTH_FAILURE;
		pr->data[VARIABLE_SIZE]: procedure specific data, if applicable
		*/
		/* If needed, add user code for handling the event data */
      }/* EVT_BLUE_GAP_PROCEDURE_COMPLETE */
      break;
     }/* switch(blue_evt->ecode) */
    }/* EVT_VENDOR */
	break;
  }
}

//------------------------------------------------------------------
//PeripheralDevices_t perDevs;
void Connection_Process(void)
{
	tBleStatus ret;
	/* Start the general discovery procedure (active scan) using the
	following parameters:
	Scan_Interval: 0x4000;
	Scan_Window: 0x4000;
	Own_address_type: 0x00 (public device address);
	filterDuplicates: 0x00 (duplicate filtering disabled);*/
	ret = aci_gap_start_general_discovery_proc(0x4000, 0x4000,0x00,0x00);
	//printf("error discovery = %d\n",ret);
	if (ret != BLE_STATUS_SUCCESS)
	{
		PRINTF("Failure.\n")

				/*printf for errors*/
				/*printf("discovery process: ");
				if(ret == BLE_STATUS_INVALID_PARAMETER)
					printf("BLE_STATUS_INVALID_PARAMETER\n");
				else if(ret == ERR_COMMAND_DISALLOWED)
					printf("ERR_COMMAND_DISALLOWED\n");
				else
					printf("unknown\n");*/

	}
	//else printf("discovering...\n");
	HAL_Delay(200);
}

void Reading_Process(void)
{
	tBleStatus ret;
	ret = aci_gatt_read_charac_val(connection_handle, EnvironmentalCharHandle + 1);
	if (ret != BLE_STATUS_SUCCESS)
	{
		PRINTF("Failure.\n")
						//Error while reading
						/*printf("reading process: ");
						if(ret == BLE_STATUS_INVALID_PARAMETER)
							printf("BLE_STATUS_INVALID_PARAMETER\n");
						else if(ret == ERR_COMMAND_DISALLOWED)
							printf("ERR_COMMAND_DISALLOWED\n");
						else
							printf("unknown\n");*/
	}
	//else printf("reading..\n");
}

void Char_Read_Value_CB(uint16_t handle, uint8_t data_length, uint8_t* attr_value)
{
	 uint16_t temp_thresh = temperature_threshold;
	 perDevs.pressure = (attr_value[5]<<24) | (attr_value[4]<<16) | (attr_value[3]<<8) | attr_value[2];
	 perDevs.humidity = (attr_value[7]<<8) | attr_value[6];
	 perDevs.temperature = (attr_value[9]<<8) | attr_value[8];

	 /*
	 printf("Press: %ld.%ld  -  ", perDevs.pressure/100, perDevs.pressure%100);
	 printf("Hum: %ld.%ld  -  ", perDevs.humidity/10, perDevs.humidity%10);
	 printf("Temp: %ld.%ld\n", perDevs.temperature/10, perDevs.temperature%10);
	 */

	 //Switch the led on when temperature is over the threshold temp_tresh
	 if(perDevs.temperature/10>=temp_thresh && !TargetBoardFeatures.LedStatus)
	 {
		 LedOnTargetPlatform();
		 TargetBoardFeatures.LedStatus =1;
	 }
	 else if(perDevs.temperature/10<temp_thresh && TargetBoardFeatures.LedStatus)
	 {
		 LedOffTargetPlatform();
		 TargetBoardFeatures.LedStatus =0;
	 }
}

void setConnectable(void)
{
	tBleStatus ret;
	char local_name[8] = {AD_TYPE_COMPLETE_LOCAL_NAME,NAME_STLBLE};
	//hci_le_set_scan_resp_data(0,NULL);
	ret = aci_gap_set_discoverable(ADV_IND, 0, 0,	// MODIFIED -> aci_gap_set_discoverable(ADV_IND, 0, 0,   //ADV_IND, ADV_SCAN_IND, or ADV_NONCONN_IND
	#ifndef STATIC_BLE_MAC
	                           STATIC_RANDOM_ADDR,
	#else /* STATIC_BLE_MAC */
	                           PUBLIC_ADDR,
	#endif /* STATIC_BLE_MAC */
	                           NO_WHITE_LIST_USE,
	                           sizeof(local_name), local_name, 0, NULL, 0, 0);
	if (ret != BLE_STATUS_SUCCESS)
	{
		PRINTF("Failure.\n")
						//Error while reading
						//printf("error status: 0x%02x. \n",ret);
	}
	else
	{
		//printf("set discoverable!\n");
		set_connectable=FALSE;
	}
}



/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
