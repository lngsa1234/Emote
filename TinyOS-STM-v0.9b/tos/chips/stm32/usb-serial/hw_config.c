/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : hw_config.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

#include <stm32f10x_rcc.h>
#include <misc.h>
#include "hw_config.h"
#include "usb_type.h"
#include "usb_conf.h"
#include "usb_mem.h"
#include "usb_regs.h"
#include "usb_core.h"
#include "usb_pwr.h"
#include "usb_desc.h"

uint8_t  USART_Rx_Buffer [USART_RX_DATA_SIZE]; 
uint32_t USART_Rx_ptr_in = 0;
uint32_t USART_Rx_ptr_out = 0;
uint32_t USART_Rx_length  = 0;

uint8_t  USB_Tx_State = 0;
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
/* Extern variables ----------------------------------------------------------*/

//extern LINE_CODING linecoding;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  
  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
// void USB_Cable_Config (FunctionalState NewState)
// {
  // if (NewState != DISABLE)
  // {
    // GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  // }
  // else
  // {
    // GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  // }
// }

/*******************************************************************************
* Function Name  :  USART_Config_Default.
* Description    :  configure the EVAL_COM1 with default values.
* Input          :  None.
* Return         :  None.
*******************************************************************************/

// void USART_Config_Default(void)
// {
  // /* EVAL_COM1 default configuration */
  // /* EVAL_COM1 configured as follow:
        // - BaudRate = 9600 baud  
        // - Word Length = 8 Bits
        // - One Stop Bit
        // - Parity Odd
        // - Hardware flow control disabled
        // - Receive and transmit enabled
  // */
  // USART_InitStructure.USART_BaudRate = 9600;
  // USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  // USART_InitStructure.USART_StopBits = USART_StopBits_1;
  // USART_InitStructure.USART_Parity = USART_Parity_Odd;
  // USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  // USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  // /* Configure and enable the USART */
  // STM_EVAL_COMInit(COM1, &USART_InitStructure);

  // /* Enable the USART Receive interrupt */
  // USART_ITConfig(EVAL_COM1, USART_IT_RXNE, ENABLE);
// }

/*******************************************************************************
* Function Name  :  USART_Config.
* Description    :  Configure the EVAL_COM1 according to the line coding structure.
* Input          :  None.
* Return         :  Configuration status
                    TRUE : configuration done with success
                    FALSE : configuration aborted.
*******************************************************************************/
// bool USART_Config(void)
// {

  // /* set the Stop bit*/
  // switch (linecoding.format)
  // {
    // case 0:
      // USART_InitStructure.USART_StopBits = USART_StopBits_1;
      // break;
    // case 1:
      // USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
      // break;
    // case 2:
      // USART_InitStructure.USART_StopBits = USART_StopBits_2;
      // break;
    // default :
    // {
      // USART_Config_Default();
      // return (FALSE);
    // }
  // }

  // /* set the parity bit*/
  // switch (linecoding.paritytype)
  // {
    // case 0:
      // USART_InitStructure.USART_Parity = USART_Parity_No;
      // break;
    // case 1:
      // USART_InitStructure.USART_Parity = USART_Parity_Even;
      // break;
    // case 2:
      // USART_InitStructure.USART_Parity = USART_Parity_Odd;
      // break;
    // default :
    // {
      // USART_Config_Default();
      // return (FALSE);
    // }
  // }

  // /*set the data type : only 8bits and 9bits is supported */
  // switch (linecoding.datatype)
  // {
    // case 0x07:
      // /* With this configuration a parity (Even or Odd) should be set */
      // USART_InitStructure.USART_WordLength = USART_WordLength_8b;
      // break;
    // case 0x08:
      // if (USART_InitStructure.USART_Parity == USART_Parity_No)
      // {
        // USART_InitStructure.USART_WordLength = USART_WordLength_8b;
      // }
      // else 
      // {
        // USART_InitStructure.USART_WordLength = USART_WordLength_9b;
      // }
      
      // break;
    // default :
    // {
      // USART_Config_Default();
      // return (FALSE);
    // }
  // }

  // USART_InitStructure.USART_BaudRate = linecoding.bitrate;
  // USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  // USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
  // /* Configure and enable the USART */
  // STM_EVAL_COMInit(COM1, &USART_InitStructure);

  // return (TRUE);
//}

/*******************************************************************************
* Function Name  : USB_To_USART_Send_Data.
* Description    : send the received data from USB to the UART 0.
* Input          : data_buffer: data address.
                   Nb_bytes: number of bytes to send.
* Return         : none.
*******************************************************************************/
extern void UsbSerialByte(uint8_t b);
extern void UsbSerialRxDone( uint8_t* buf, uint16_t len );
void USB_To_USART_Send_Data(uint8_t* data_buffer, uint8_t Nb_bytes)
{
  
  uint32_t i;
  
  for (i = 0; i < Nb_bytes; i++)
  {
	UsbSerialByte(*(data_buffer + i));
    //USART_SendData(EVAL_COM1, *(data_buffer + i));
    //while(USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TXE) == RESET); 
  }
  UsbSerialRxDone(data_buffer, Nb_bytes);
}

/*******************************************************************************
* Function Name  : Handle_USBAsynchXfer.
* Description    : send data to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void Handle_USBAsynchXfer (void)
{
  uint16_t USB_Tx_ptr;
  uint16_t USB_Tx_length;
  
  if(USB_Tx_State != 1)
  {
    if (USART_Rx_ptr_out == USART_RX_DATA_SIZE)
    {
      USART_Rx_ptr_out = 0;
    }
    
    if(USART_Rx_ptr_out == USART_Rx_ptr_in) 
    {
      USB_Tx_State = 0; 
      return;
    }
    
    if(USART_Rx_ptr_out > USART_Rx_ptr_in) /* rollback */
    { 
      USART_Rx_length = USART_RX_DATA_SIZE - USART_Rx_ptr_out;
    }
    else 
    {
      USART_Rx_length = USART_Rx_ptr_in - USART_Rx_ptr_out;
    }
    
    if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE)
    {
      USB_Tx_ptr = USART_Rx_ptr_out;
      USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;
      
      USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;	
      USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;	
    }
    else
    {
      USB_Tx_ptr = USART_Rx_ptr_out;
      USB_Tx_length = USART_Rx_length;
      
      USART_Rx_ptr_out += USART_Rx_length;
      USART_Rx_length = 0;
    }
    USB_Tx_State = 1; 
      
    UserToPMABufferCopy(&USART_Rx_Buffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
    SetEPTxCount(ENDP1, USB_Tx_length);
    SetEPTxValid(ENDP1);
  }  
  
}
/*******************************************************************************
* Function Name  : UART_To_USB_Send_Data.
* Description    : send the received data from UART 0 to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void USART_To_USB_Send_Data(uint8_t b)
{
  
  USART_Rx_Buffer[USART_Rx_ptr_in] = b;

  USART_Rx_ptr_in++;
  
  /* To avoid buffer overflow */
  if(USART_Rx_ptr_in == USART_RX_DATA_SIZE)
  {
    USART_Rx_ptr_in = 0;
  }
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
  Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
  Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
