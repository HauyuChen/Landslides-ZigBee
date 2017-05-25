/**************************************************************************************************
  Filename:       DemoCollector.c

  Description:    Collector application for the Sensor Demo utilizing Simple API.

                  The collector node can be set in a state where it accepts 
                  incoming reports from the sensor nodes, and can send the reports
                  via the UART to a PC tool. The collector node in this state
                  functions as a gateway. The collector nodes that are not in the
                  gateway node function as routers in the network.  


  Copyright 2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/******************************************************************************
 * INCLUDES
 */

#include "ZComDef.h"
#include "OSAL.h"
#include "OSAL_Nv.h"
#include "sapi.h"
#include "hal_key.h"
#include "hal_led.h"
#include "hal_lcd.h"
#include "hal_uart.h"
#include "string.h"


#include "DemoApp.h"

/******************************************************************************
 * CONSTANTS
 */

#define REPORT_FAILURE_LIMIT                4
#define ACK_REQ_INTERVAL                    5 // each 5th packet is sent with ACK request

// General UART frame offsets
#define FRAME_SOF_OFFSET                    0
#define FRAME_LENGTH_OFFSET                 1 
#define FRAME_CMD0_OFFSET                   2
#define FRAME_CMD1_OFFSET                   3
#define FRAME_DATA_OFFSET                   4

// ZB_RECEIVE_DATA_INDICATION offsets
#define ZB_RECV_SRC_OFFSET                  0
#define ZB_RECV_CMD_OFFSET                  2
#define ZB_RECV_LEN_OFFSET                  4
#define ZB_RECV_DATA_OFFSET                 6
#define ZB_RECV_FCS_OFFSET                  8

// ZB_RECEIVE_DATA_INDICATION frame length
#define ZB_RECV_LENGTH                      15

// PING response frame length and offset
#define SYS_PING_RSP_LENGTH                 7 
#define SYS_PING_CMD_OFFSET                 1

// Stack Profile
#define ZIGBEE_2007                         0x0040
#define ZIGBEE_PRO_2007                     0x0041

#ifdef ZIGBEEPRO
#define STACK_PROFILE                       ZIGBEE_PRO_2007             
#else 
#define STACK_PROFILE                       ZIGBEE_2007
#endif

#define CPT_SOP                             0xFE
#define SYS_PING_REQUEST                    0x0021
#define SYS_PING_RESPONSE                   0x0161
#define ZB_RECEIVE_DATA_INDICATION          0x8746

// Application States
#define APP_INIT                            0
#define APP_START                           2
#define APP_BINDED                          3    

// Application osal event identifiers
#define MY_START_EVT                        0x0001
#define MY_REPORT_EVT                       0x0002
#define MY_FIND_COLLECTOR_EVT               0x0004
#define MY_ANGLE_EVT                        0x0008


//扩展控制指令
#define  control      P1_3
#define  control_IN   P1DIR &= ~0x08
#define  control_OUT  P1DIR |=  0x08
//定义绿灯引脚，定义输入输出
#define  green_led      P1_4
#define  green_led_IN   P1DIR &= ~0x10
#define  green_led_OUT  P1DIR |=  0x10
/******************************************************************************
 * TYPEDEFS
 */
typedef struct
{
  uint16              accsign;
  uint16              source;
  uint16              parent;
  uint16              ACCEL_X;
  uint16              ACCEL_Y;
  uint16              ACCEL_Z;
  uint16              GYRO_X;
  uint16              GYRO_Y;
  uint16              GYRO_Z;
} gtwData_t;     //定义数据的结构体，用于存放X，Y，Z轴的信息

/**************************定义AT指令***************************************/
#define INIT_APN               "at+cgdcont=1,\"ip\",\"cmnet\""//配置apn
#define ENTRY_TCPIP            "at%etcpip"                    //进入tcpip 功能
#define CHECK_NET              "at%etcpip?"                   //查看本地网络状况
#define OPEN_TCP               "AT%IPOPEN=\"TCP\",\"119.146.68.41\",5000"//打开一条tcp 链接
#define ASCII_MODE             "at%iomode=0,1,1"              //ascii mode 
#define HEX_MODE               "AT%IOMODE=1,1,0"              //hex mode
#define TEST                    "AT%IPSEND=\"303030303030\""  //密码，与服务器连接用到
uint8 END[2]={0x0d,0x0a};//双引号
uint8 COMM = 0;

uint16 hex_2_ascii(uint8 *data, uint8 *buffer, uint16 len);//定义HEX转化为ASCII码的函数

/******************************************************************************
 * LOCAL VARIABLES
 */

static uint8 appState =             APP_INIT;
static uint8 reportState =          FALSE;
static uint8 myStartRetryDelay =    10;          // milliseconds
static uint8 isGateWay =            FALSE;
static uint16 myBindRetryDelay =    2000;        // milliseconds
static uint16 myReportPeriod =      1000;        // milliseconds

static uint8 reportFailureNr =      0;
static uint16 parentShortAddr;
static gtwData_t gtwData;           //定义一个结构体对象，用于存放X，Y，Z轴的信息

/******************************************************************************
 * LOCAL FUNCTIONS
 */

static uint8 calcFCS(uint8 *pBuf, uint8 len);
static void sysPingReqRcvd(void);
static void sysPingRsp(void);
static void sendGtwReport(gtwData_t *gtwData);
static void sendDummyReport(void);
/******************************************************************************
 * GLOBAL VARIABLES
 */

// Inputs and Outputs for Collector device
#define NUM_OUT_CMD_COLLECTOR                2
#define NUM_IN_CMD_COLLECTOR                 2

// List of output and input commands for Collector device
const cId_t zb_InCmdList[NUM_IN_CMD_COLLECTOR] =
{
  SENSOR_REPORT_CMD_ID,
  DUMMY_REPORT_CMD_ID
};

const cId_t zb_OutCmdList[NUM_IN_CMD_COLLECTOR] =
{
  SENSOR_REPORT_CMD_ID,
  DUMMY_REPORT_CMD_ID
};

// Define SimpleDescriptor for Collector device
const SimpleDescriptionFormat_t zb_SimpleDesc =
{
  MY_ENDPOINT_ID,             //  Endpoint
  MY_PROFILE_ID,              //  Profile ID
  DEV_ID_COLLECTOR,           //  Device ID
  DEVICE_VERSION_COLLECTOR,   //  Device Version
  0,                          //  Reserved
  NUM_IN_CMD_COLLECTOR,       //  Number of Input Commands
  (cId_t *) zb_InCmdList,     //  Input Command List
  NUM_OUT_CMD_COLLECTOR,      //  Number of Output Commands
  (cId_t *) zb_OutCmdList     //  Output Command List
};

/******************************************************************************
 * FUNCTIONS
 */

/******************************************************************************
 * @fn          zb_HandleOsalEvent
 *
 * @brief       The zb_HandleOsalEvent function is called by the operating
 *              system when a task event is set
 *
 * @param       event - Bitmask containing the events that have been set
 *
 * @return      none
 */
void zb_HandleOsalEvent( uint16 event )
{
  uint8 logicalType;
  uint8 txOptions;
  static uint8 reportNr=0; 
  
  if(event & SYS_EVENT_MSG)
  {
    
  }
  
  if( event & ZB_ENTRY_EVENT )
  {  

      /****************************修改部分************************************************************/
  
    
     // blind LED 1 to indicate starting/joining a network
    HalLedBlink ( HAL_LED_1, 0, 50, 500 );
    HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
    
    if ( appState == APP_INIT )
    {
    #ifdef MY_TYPE_COLLECTOR
    logicalType = ZG_DEVICETYPE_COORDINATOR;
    zb_WriteConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType);
    initUart(uartRxCB);
    #else
    logicalType = ZG_DEVICETYPE_ROUTER;
    zb_WriteConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType);
    #endif
    }
    // Start the device 
    zb_StartRequest();
  
    /***********************************************************************************************/
  }
  
  if ( event & MY_START_EVT )
  {
    zb_StartRequest();
  }
  
  if ( event & MY_REPORT_EVT )
  {
    if (isGateWay) 
    {
      osal_start_timerEx( sapi_TaskID, MY_REPORT_EVT, myReportPeriod );
    }
    else if (appState == APP_BINDED) 
    {
      sendDummyReport();
      osal_start_timerEx( sapi_TaskID, MY_REPORT_EVT, myReportPeriod );
    }
  }
  if ( event & MY_FIND_COLLECTOR_EVT )
  { 
    // Find and bind to a gateway device (if this node is not gateway)
    if (!isGateWay) 
    {
      zb_BindDevice( TRUE, DUMMY_REPORT_CMD_ID, (uint8 *)NULL );
    }
  }
  
  if(event & MY_ANGLE_EVT)//??????????????????????
  {
    
    switch(COMM)
    {
      case 0:
        HalUARTWrite(HAL_UART_PORT_0,INIT_APN,sizeof(INIT_APN));//初始化连接命令
        HalUARTWrite(HAL_UART_PORT_0,END,sizeof(END));
        COMM++;
        osal_start_timerEx( sapi_TaskID, MY_ANGLE_EVT, 3*myReportPeriod );//设置0.5秒执行下一条指令
        //HalUARTWrite(HAL_UART_PORT_0,END,sizeof(END));
        break;
      case 1:
        HalUARTWrite(HAL_UART_PORT_0,ENTRY_TCPIP,sizeof(ENTRY_TCPIP));//进入TCP/IP命令
        HalUARTWrite(HAL_UART_PORT_0,END,sizeof(END));
        COMM++;
        osal_start_timerEx( sapi_TaskID, MY_ANGLE_EVT, 3*myReportPeriod );//设置0.5秒执行下一条指令
       
        break;
      case 2:
          HalUARTWrite(HAL_UART_PORT_0,OPEN_TCP, sizeof(OPEN_TCP));//打开连接命令
          HalUARTWrite(HAL_UART_PORT_0,END,sizeof(END));
          COMM=COMM+2;
          osal_start_timerEx( sapi_TaskID, MY_ANGLE_EVT, 3*myReportPeriod );//设置0.5秒执行下一条指令
        break;
      case 3:
          HalUARTWrite(HAL_UART_PORT_0,HEX_MODE, sizeof(HEX_MODE));//设置发送字符为hex mode
          HalUARTWrite(HAL_UART_PORT_0,END,sizeof(END));
          //COMM++;
        //osal_start_timerEx( sapi_TaskID, MY_ANGLE_EVT, 3*myReportPeriod );//设置0.5秒执行下一条指令
         break;
      case 4:
          HalUARTWrite(HAL_UART_PORT_0,TEST,sizeof(TEST));
          HalUARTWrite(HAL_UART_PORT_0,END,sizeof(END));
          COMM=COMM+2;
         
        //HalUARTWrite(HAL_UART_PORT_0,ASCII_MODE, sizeof(ASCII_MODE));//设置发送字符为ascii mode
       
        osal_start_timerEx( sapi_TaskID, MY_ANGLE_EVT, 3*myReportPeriod );//设置0.5秒执行下一条指令
         break;
      case 5: 
        HalUARTWrite(HAL_UART_PORT_0,CHECK_NET, sizeof(CHECK_NET));//查询网络命令
        //COMM++;
        //osal_start_timerEx( sapi_TaskID, MY_ANGLE_EVT, 3*myReportPeriod );//设置0.5秒执行下一条指令;
         break;
    default :
      osal_start_timerEx( sapi_TaskID, MY_REPORT_EVT,5* myReportPeriod );//设置5秒自动广播
      break;
    }
  }
  
  if ( event & MY_ANGLE_EVT )
  {
    if ( ++reportNr<ACK_REQ_INTERVAL && reportFailureNr==0 ) 
     {
      txOptions = AF_TX_OPTIONS_NONE;
      }
      else 
     {
       txOptions = AF_MSG_ACK_REQUEST;
         reportNr = 0;
     }
    zb_SendDataRequest( 0xFFFF, SENSOR_REPORT_CMD_ID, 1, "1", 0, txOptions, 0 );//广播**********************************************
    if(COMM>4)
    {
    osal_start_timerEx( sapi_TaskID, MY_ANGLE_EVT, 1*myReportPeriod );//設置每1秒请求数据*******************************
    }
  }
}

/******************************************************************************
 * @fn      zb_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 EVAL_SW4
 *                 EVAL_SW3
 *                 EVAL_SW2
 *                 EVAL_SW1
 *
 * @return  none
 */
void zb_HandleKeys( uint8 shift, uint8 keys )
{
  static uint8 allowBind=FALSE;

  uint8 logicalType;

  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
      if ( appState == APP_INIT  )
      {
        // Key 1 starts device as a coordinator
        logicalType = ZG_DEVICETYPE_COORDINATOR;
        zb_WriteConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType);
                
        // Reset the device with new configuration
        zb_SystemReset();
      }
    }
    if ( keys & HAL_KEY_SW_2 )
    {
      allowBind ^= 1;
      if (allowBind) 
      {
        // Turn ON Allow Bind mode infinitly
        zb_AllowBind( 0xFF );
        HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
        //This node is the gateway node
        isGateWay = TRUE;
        
        // Update the display
        #if defined ( LCD_SUPPORTED )
        HalLcdWriteString( "Gateway Mode", HAL_LCD_LINE_2 );
        #endif
      }
      else
      {
        // Turn OFF Allow Bind mode infinitly
        zb_AllowBind( 0x00 );
        HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
        isGateWay = FALSE;
        
        // Update the display
        #if defined ( LCD_SUPPORTED )
        HalLcdWriteString( "Collector", HAL_LCD_LINE_2 );
        #endif
      }
    }
    if ( keys & HAL_KEY_SW_3 )
    {
      // Start reporting
      osal_set_event( sapi_TaskID, MY_REPORT_EVT );
    }
    if ( keys & HAL_KEY_SW_4 )
    {
      
    }
    }
  }


/******************************************************************************
 * @fn          zb_StartConfirm
 *
 * @brief       The zb_StartConfirm callback is called by the ZigBee stack
 *              after a start request operation completes
 *
 * @param       status - The status of the start operation.  Status of
 *                       ZB_SUCCESS indicates the start operation completed
 *                       successfully.  Else the status is an error code.
 *
 * @return      none
 */
void zb_StartConfirm( uint8 status )
{ 
  // If the device sucessfully started, change state to running
  if ( status == ZB_SUCCESS )   
  {
    // Set LED 1 to indicate that node is operational on the network
    HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
    
    // Update the display
    #if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "SensorDemo", HAL_LCD_LINE_1 );
    HalLcdWriteString( "Collector", HAL_LCD_LINE_2 );
    #endif
    
    // Change application state
    appState = APP_START;
    
    // Set event to bind to a collector
    osal_set_event( sapi_TaskID, MY_FIND_COLLECTOR_EVT );
       
     // Store parent short address
    zb_GetDeviceInfo(ZB_INFO_PARENT_SHORT_ADDR, &parentShortAddr);
    
    //警报初始化，将I/O设置为输出，输出低电平
    control_OUT;
    control=0;
    //绿灯初始化,将I/O设置为输出，输出高电平
    green_led_OUT;
    green_led=1;
    
    osal_start_timerEx( sapi_TaskID, MY_ANGLE_EVT, 5*myReportPeriod );//設置每2秒请求数据*******************************
  }
  else
  {
    // Try again later with a delay
    osal_start_timerEx( sapi_TaskID, MY_START_EVT, myStartRetryDelay );
  }
}

/******************************************************************************
 * @fn          zb_SendDataConfirm
 *
 * @brief       The zb_SendDataConfirm callback function is called by the
 *              ZigBee stack after a send data operation completes
 *
 * @param       handle - The handle identifying the data transmission.
 *              status - The status of the operation.
 *
 * @return      none
 */
void zb_SendDataConfirm( uint8 handle, uint8 status )
{
  if ( status != ZB_SUCCESS && !isGateWay ) 
  {
    if ( ++reportFailureNr>=REPORT_FAILURE_LIMIT ) 
    {   
       // Stop reporting
       osal_stop_timerEx( sapi_TaskID, MY_REPORT_EVT );
       
       // After failure reporting start automatically when the device
       // is binded to a new gateway
       reportState=TRUE;
       
       // Delete previous binding
       zb_BindDevice( FALSE, DUMMY_REPORT_CMD_ID, (uint8 *)NULL );
       
       // Try binding to a new gateway
       osal_set_event( sapi_TaskID, MY_FIND_COLLECTOR_EVT );
       reportFailureNr=0;
    }
  }
  else if ( !isGateWay ) 
  {
    reportFailureNr=0;
  }
}

/******************************************************************************
 * @fn          zb_BindConfirm
 *
 * @brief       The zb_BindConfirm callback is called by the ZigBee stack
 *              after a bind operation completes.
 *
 * @param       commandId - The command ID of the binding being confirmed.
 *              status - The status of the bind operation.
 *
 * @return      none
 */
void zb_BindConfirm( uint16 commandId, uint8 status )
{
  if( status == ZB_SUCCESS )
  {
    appState = APP_BINDED;
    // Set LED2 to indicate binding successful
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
    
    // After failure reporting start automatically when the device
    // is binded to a new gateway
   /* if ( reportState ) 
    {
      // Start reporting
      osal_set_event( sapi_TaskID, MY_REPORT_EVT );
    }*/
  }
  else
  {
    osal_start_timerEx( sapi_TaskID, MY_FIND_COLLECTOR_EVT, myBindRetryDelay );
  }
}

/******************************************************************************
 * @fn          zb_AllowBindConfirm
 *
 * @brief       Indicates when another device attempted to bind to this device
 *
 * @param
 *
 * @return      none
 */
void zb_AllowBindConfirm( uint16 source )
{
  
}

/******************************************************************************
 * @fn          zb_FindDeviceConfirm
 *
 * @brief       The zb_FindDeviceConfirm callback function is called by the
 *              ZigBee stack when a find device operation completes.
 *
 * @param       searchType - The type of search that was performed.
 *              searchKey - Value that the search was executed on.
 *              result - The result of the search.
 *
 * @return      none
 */
void zb_FindDeviceConfirm( uint8 searchType, uint8 *searchKey, uint8 *result )
{
}

/******************************************************************************
 * @fn          zb_ReceiveDataIndication
 *
 * @brief       The zb_ReceiveDataIndication callback function is called
 *              asynchronously by the ZigBee stack to notify the application
 *              when data is received from a peer device.
 *
 * @param       source - The short address of the peer device that sent the data
 *              command - The commandId associated with the data
 *              len - The number of bytes in the pData parameter
 *              pData - The data sent by the peer device
 *
 * @return      none
 */
void zb_ReceiveDataIndication( uint16 source, uint16 command, uint16 len, uint8 *pData)
{ 
  gtwData.accsign=BUILD_UINT16(pData[sign+1], pData[sign]);//将节点发来的数据存入结构体
  gtwData.parent = BUILD_UINT16(pData[SENSOR_PARENT_OFFSET+ 1], pData[SENSOR_PARENT_OFFSET]);//将节点发来的数据存入结构体
  gtwData.source=source;//将节点发来的数据存入结构体
  gtwData.ACCEL_X = BUILD_UINT16(pData[SENSOR_ACCEL_XOUT_H_OFFSET+ 1], pData[SENSOR_ACCEL_XOUT_H_OFFSET]);//将节点发来的数据存入结构体
  gtwData.ACCEL_Y = BUILD_UINT16(pData[SENSOR_ACCEL_YOUT_H_OFFSET+ 1], pData[SENSOR_ACCEL_YOUT_H_OFFSET]);//将节点发来的数据存入结构体
  gtwData.ACCEL_Z = BUILD_UINT16(pData[SENSOR_ACCEL_ZOUT_H_OFFSET+ 1], pData[SENSOR_ACCEL_ZOUT_H_OFFSET]);//将节点发来的数据存入结构体
  gtwData.GYRO_X = BUILD_UINT16(pData[SENSOR_GYRO_XOUT_H_OFFSET+ 1], pData[SENSOR_GYRO_XOUT_H_OFFSET]);//将节点发来的数据存入结构体
  gtwData.GYRO_Y = BUILD_UINT16(pData[SENSOR_GYRO_YOUT_H_OFFSET+ 1], pData[SENSOR_GYRO_YOUT_H_OFFSET]);//将节点发来的数据存入结构体
  gtwData.GYRO_Z = BUILD_UINT16(pData[SENSOR_GYRO_ZOUT_H_OFFSET+ 1], pData[SENSOR_GYRO_ZOUT_H_OFFSET]);//将节点发来的数据存入结构体
  
  // Update the display
  #if defined ( LCD_SUPPORTED )
  HalLcdWriteScreen( "Report", "rcvd" );
  #endif
  
  // Send gateway report
  sendGtwReport(&gtwData);
}

/******************************************************************************
 * @fn          uartRxCB
 *
 * @brief       Callback function for UART 
 *
 * @param       port - UART port
 *              event - UART event that caused callback 
 *
 * @return      none
 */
void uartRxCB( uint8 port, uint8 event )
{
  uint8 pBuf[RX_BUF_LEN];
  uint16 cmd;
  uint16 len;
  
  if ( event != HAL_UART_TX_EMPTY ) 
  {
    //HalLcdWriteString(pBuf+1,3);
    // Read from UART
    len = HalUARTRead( HAL_UART_PORT_0, pBuf, RX_BUF_LEN );
    
    if ( len>0 ) 
    {
      cmd = BUILD_UINT16(pBuf[SYS_PING_CMD_OFFSET+ 1], pBuf[SYS_PING_CMD_OFFSET]);
  
      if( (pBuf[FRAME_SOF_OFFSET] == CPT_SOP) && (cmd == SYS_PING_REQUEST) ) //如果是系统请求
      {
        sysPingReqRcvd();
      }
      else
      {       
              
        
       //判断是否控制信息
          if (pBuf[11]=='4' && pBuf[12]=='1' && pBuf[13]=='4' && pBuf[14]=='1' )//判断接收到的语句中，是否包含“4141”，即AA的ASCII码
          {
            //control_OUT;
            control=1;  //P1_3输入高电平
            green_led=0;//关闭绿灯
          }
          
          if (pBuf[11]=='4' && pBuf[12]=='2' && pBuf[13]=='4' && pBuf[14]=='2' )//判断接收到的语句中，是否包含“4242”，即BB的ASCII码
          {
            //control_OUT;
            control=0;  //P1_3输入低电平
            green_led=1;//开启绿灯
          }
    }
  }
  }
}

/******************************************************************************
 * @fn          sysPingReqRcvd
 *
 * @brief       Ping request received 
 *
 * @param       none
 *              
 * @return      none
 */
static void sysPingReqRcvd(void)
{
   sysPingRsp();
}

/******************************************************************************
 * @fn          sysPingRsp
 *
 * @brief       Build and send Ping response
 *
 * @param       none
 *              
 * @return      none
 */
static void sysPingRsp(void)
{
  uint8 pBuf[SYS_PING_RSP_LENGTH];
  
  // Start of Frame Delimiter
  pBuf[FRAME_SOF_OFFSET] = CPT_SOP;
  
  // Length
  pBuf[FRAME_LENGTH_OFFSET] = 2; 
  
  // Command type
  pBuf[FRAME_CMD0_OFFSET] = LO_UINT16(SYS_PING_RESPONSE); 
  pBuf[FRAME_CMD1_OFFSET] = HI_UINT16(SYS_PING_RESPONSE);
  
  // Stack profile
  pBuf[FRAME_DATA_OFFSET] = LO_UINT16(STACK_PROFILE);
  pBuf[FRAME_DATA_OFFSET+ 1] = HI_UINT16(STACK_PROFILE);
  
  // Frame Check Sequence
  pBuf[SYS_PING_RSP_LENGTH - 1] = calcFCS(&pBuf[FRAME_LENGTH_OFFSET], (SYS_PING_RSP_LENGTH - 2));
  
  // Write frame to UART
  HalUARTWrite(HAL_UART_PORT_0,pBuf, SYS_PING_RSP_LENGTH);
}

/******************************************************************************
 * @fn          sendGtwReport
 *
 * @brief       Build and send gateway report
 *
 * @param       none
 *              
 * @return      none
 */
static void sendGtwReport(gtwData_t *gtwData)
{
  uint8 temparr[42];//定义要发送的总长度：11位AT指令头，4位标志位，24位数据，1位单引号，2位结束符）
  
  if(COMM<5)//检测是否建立TCP连接,并发送“00”标志位
  {
    return;//如果还没建立，则结束
  }
        uint8 pFrame[ZB_RECV_LENGTH];//定义无符号8位的对象数组来提取结构体中高8位，低8位的信息
        //gtwData.sign=pData[sign];
        pFrame[sign]=  HI_UINT16(gtwData->accsign);
        pFrame[sign+1]= LO_UINT16(gtwData->accsign);
        
        pFrame[SENSOR_ACCEL_XOUT_H_OFFSET]=  HI_UINT16(gtwData->ACCEL_X);
        pFrame[SENSOR_ACCEL_XOUT_H_OFFSET+1]=  LO_UINT16(gtwData->ACCEL_X);
        
        pFrame[SENSOR_ACCEL_YOUT_H_OFFSET]=  HI_UINT16(gtwData->ACCEL_Y);
        pFrame[SENSOR_ACCEL_YOUT_H_OFFSET+1] =  LO_UINT16(gtwData->ACCEL_Y);

        pFrame[SENSOR_ACCEL_ZOUT_H_OFFSET]=  HI_UINT16(gtwData->ACCEL_Z);
        pFrame[SENSOR_ACCEL_ZOUT_H_OFFSET+1] =  LO_UINT16(gtwData->ACCEL_Z);

        pFrame[SENSOR_GYRO_XOUT_H_OFFSET]=  HI_UINT16(gtwData->GYRO_X);
        pFrame[SENSOR_GYRO_XOUT_H_OFFSET+1] =  LO_UINT16(gtwData->GYRO_X);

        pFrame[SENSOR_GYRO_YOUT_H_OFFSET]=  HI_UINT16(gtwData->GYRO_Y);
        pFrame[SENSOR_GYRO_YOUT_H_OFFSET+1] =  LO_UINT16(gtwData->GYRO_Y);

        pFrame[SENSOR_GYRO_ZOUT_H_OFFSET]=  HI_UINT16(gtwData->GYRO_Z);
        pFrame[SENSOR_GYRO_ZOUT_H_OFFSET+1] =  LO_UINT16(gtwData->GYRO_Z);
  
        pFrame[SENSOR_PARENT_OFFSET]     = LO_UINT16(gtwData->parent); 
        pFrame[SENSOR_PARENT_OFFSET+1]   = HI_UINT16(gtwData->parent);
  
  // Frame Check Sequence
  pFrame[ZB_RECV_LENGTH - 2] = calcFCS(&pFrame[FRAME_LENGTH_OFFSET], (ZB_RECV_LENGTH - 2) );
  
  strcpy(temparr,"AT%IPSEND=\"");//为要传送的对象加上AT指令
  
  hex_2_ascii(pFrame,temparr+11,14);
  temparr[39]=0X22;//为要传送的对象加上单引号
  temparr[40]=0X0d;
  temparr[41]=0X0a;
  // Write report to UART
  HalUARTWrite(HAL_UART_PORT_0,temparr,42);//将数据通过串口输出
  //HalUARTWrite(HAL_UART_PORT_0,END,sizeof(END));//输出AT指令结尾
}

/******************************************************************************
 * @fn          sendDummyReport
 *
 * @brief       Send dummy report (used to visualize collector nodes on PC GUI)
 *
 * @param       none
 *              
 * @return      none
 */
static void sendDummyReport(void)
{
//  uint8 pData[SENSOR_REPORT_LENGTH];
//  static uint8 reportNr=0;
//  uint8 txOptions;
//  
//  // dummy report data
//  pData[SENSOR_TEMP_OFFSET] =  0xFF;
//  pData[SENSOR_VOLTAGE_OFFSET] = 0xFF; 
//    
//  pData[SENSOR_PARENT_OFFSET] =  HI_UINT16(parentShortAddr);
//  pData[SENSOR_PARENT_OFFSET+ 1] =  LO_UINT16(parentShortAddr);
//  
//  // Set ACK request on each ACK_INTERVAL report
//  // If a report failed, set ACK request on next report
//  if ( ++reportNr<ACK_REQ_INTERVAL && reportFailureNr==0 ) 
//  {
//    txOptions = AF_TX_OPTIONS_NONE;
//  }
//  else 
//  {
//    txOptions = AF_MSG_ACK_REQUEST;
//    reportNr = 0;
//  }
//  
//  // Destination address 0xFFFE: Destination address is sent to previously
//  // established binding for the commandId.
//  zb_SendDataRequest( 0xFFFE, DUMMY_REPORT_CMD_ID, SENSOR_REPORT_LENGTH, pData, 0, txOptions, 0 );
}

/******************************************************************************
 * @fn          calcFCS
 *
 * @brief       This function calculates the FCS checksum for the serial message 
 *
 * @param       pBuf - Pointer to the end of a buffer to calculate the FCS.
 *              len - Length of the pBuf.
 *
 * @return      The calculated FCS.
 ******************************************************************************
 */
static uint8 calcFCS(uint8 *pBuf, uint8 len)
{
  uint8 rtrn = 0;

  while (len--)
  {
    rtrn ^= *pBuf++;
  }

  return rtrn;
}

/*******************************************
      HEX 到 ASCII 的转换函数
      入口参数： data: 转换数据的入口指针
      buffer: 转换后数据入口指针
      len : 需要转换的长度
      返回参数：转换后数据长度
*******************************************/

uint16 hex_2_ascii(uint8 *data, uint8 *buffer, uint16 len)
{
    const uint8 ascTable[17] = {"0123456789ABCDEF"};
    uint8 *tmp_p = buffer;
    uint16 i, pos;
    pos = 0;
    for(i = 0; i < len; i++)
      {
        tmp_p[pos++] = ascTable[data[i] >> 4];
        tmp_p[pos++] = ascTable[data[i] & 0x0f];
      }
    tmp_p[pos] = '\0';
    return pos;
}