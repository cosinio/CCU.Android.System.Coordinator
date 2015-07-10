
/*********************************************************************
 * INCLUDES
 */
#include "OSAL_Nv.h"
#include "hal_flash.h"
#include "string.h"
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"

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

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;
uint8 SampleApp_TransID;  // This is the unique message ID (counter)
afAddrType_t SampleApp_Periodic_DstAddr;
afAddrType_t SampleApp_Flash_DstAddr;
afAddrType_t Point_To_Point_DstAddr;//  点对点通信定义
aps_Group_t SampleApp_Group;
uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;
bool msg = true;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void StrConvert2Hex(const unsigned char *str, uint8 *arr ,int length);
unsigned char Char2Hex(char c);
void SampleApp_BroadcastIPandMAC( void );
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendPointToPointMessage(void); // 点对点通讯定义
void SampleApp_SendFlashMessage( uint16 flashTime );
void initUart1(halUARTCBack_t UART_callback);
void UART_callback ( uint8 port, uint8 event );
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_Init
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 * @return  none
 */
void SampleApp_Init( uint8 task_id )
{
  
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
  //initial uart1 and uart0
  initUart1(UART_callback);
  MT_UartInit();//初始化
  
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Broadcast to everyone
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;
  
  //  点对点通讯定义
  Point_To_Point_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //点播
  Point_To_Point_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  Point_To_Point_DstAddr.addr.shortAddr = 0x0000;//发给协调器
  
  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );

  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 * @return  none
 */
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    //Databuf = osal_mem_alloc(MSGpkt->cmd.DataLength);
    while ( MSGpkt )
    {
        switch ( MSGpkt->hdr.event )
        {        
          // Received when a messages is received (OTA) for this endpoint
          case AF_INCOMING_MSG_CMD:
            SampleApp_MessageMSGCB( MSGpkt );
            break;  
          default:
            break;
        }    
        
      // Release the memory
      osal_msg_deallocate((uint8 *)MSGpkt );
     
      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }
  
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in SampleApp_Init()).
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
     
     SampleApp_SendPeriodicMessage();  //发送数据函数
     
    // Setup to send message again in normal period (+ a little jitter)
   
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 * @param   none
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{  
  uint8 asc_16[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
  uint16 flashTime,temp;
  uint8 exaddr[8];
  uint8 shaddr[2];
  uint8 i=0;
  

  uint8 tmp;
  uint8 *TmpBuf = osal_mem_alloc(8);
  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_POINT_TO_POINT_CLUSTERID:  
      
      if(msg){
        P1_1 = 1;
        msg = false;
      }else{
        P1_1 = 0;
        msg = true;
      }
      
      for(i=0;i<pkt->cmd.DataLength;i++){
        if(i<11 && i>2){
          exaddr [i-3] = pkt->cmd.Data[i];
        }
      }		
     
      HalUARTWrite(0,&pkt->cmd.Data[0],1);
      HalUARTWrite(0,&pkt->cmd.Data[1],1);
      HalUARTWrite(0,&pkt->cmd.Data[2],1);  
      HalUARTWrite(0,&asc_16[exaddr[0]%256/16],1);
      HalUARTWrite(0,&asc_16[exaddr[0]%16],1);
      HalUARTWrite(0,&asc_16[exaddr[1]%256/16],1);
      HalUARTWrite(0,&asc_16[exaddr[1]%16],1);
      HalUARTWrite(0,&asc_16[exaddr[2]%256/16],1);
      HalUARTWrite(0,&asc_16[exaddr[2]%16],1);  
      HalUARTWrite(0,&asc_16[exaddr[3]%256/16],1);
      HalUARTWrite(0,&asc_16[exaddr[3]%16],1);   
      HalUARTWrite(0,&asc_16[exaddr[4]%256/16],1);
      HalUARTWrite(0,&asc_16[exaddr[4]%16],1); 
      HalUARTWrite(0,&asc_16[exaddr[5]%256/16],1);
      HalUARTWrite(0,&asc_16[exaddr[5]%16],1);      
      HalUARTWrite(0,&asc_16[exaddr[6]%256/16],1);
      HalUARTWrite(0,&asc_16[exaddr[6]%16],1);  
      HalUARTWrite(0,&asc_16[exaddr[7]%256/16],1);
      HalUARTWrite(0,&asc_16[exaddr[7]%16],1);            
      HalUARTWrite(0,pkt->cmd.Data+11,pkt->cmd.DataLength-11);
      HalUARTWrite(0,"\n",2);
      
      break;

  case Device_annce:
     
    /*---------------------EP 第一次入网，上传自己的mac，ip，和其他的信息；*/
    for(i=0;i<pkt->cmd.DataLength;i++){
      if(i<2){
        shaddr[i] = pkt->cmd.Data[i];
      }else if(i<10){
        exaddr [i-2] = pkt->cmd.Data[i];
      }
    }			
    /* /C|MAC|IP|Cluster_id|Group_id|endPoint| */
    HalUARTWrite(0,"/I|",3);  //串口显示MAC address   
    HalUARTWrite(0,&asc_16[exaddr[0]%256/16],1);
    HalUARTWrite(0,&asc_16[exaddr[0]%16],1);
    HalUARTWrite(0,&asc_16[exaddr[1]%256/16],1);
    HalUARTWrite(0,&asc_16[exaddr[1]%16],1);
    HalUARTWrite(0,&asc_16[exaddr[2]%256/16],1);
    HalUARTWrite(0,&asc_16[exaddr[2]%16],1);  
    HalUARTWrite(0,&asc_16[exaddr[3]%256/16],1);
    HalUARTWrite(0,&asc_16[exaddr[3]%16],1);   
    HalUARTWrite(0,&asc_16[exaddr[4]%256/16],1);
    HalUARTWrite(0,&asc_16[exaddr[4]%16],1); 
    HalUARTWrite(0,&asc_16[exaddr[5]%256/16],1);
    HalUARTWrite(0,&asc_16[exaddr[5]%16],1);      
    HalUARTWrite(0,&asc_16[exaddr[6]%256/16],1);
    HalUARTWrite(0,&asc_16[exaddr[6]%16],1);  
    HalUARTWrite(0,&asc_16[exaddr[7]%256/16],1);
    HalUARTWrite(0,&asc_16[exaddr[7]%16],1);
    HalUARTWrite(0,"|",1);     //network IP address;
    HalUARTWrite(0,&asc_16[shaddr[0]%256/16],1);
    HalUARTWrite(0,&asc_16[shaddr[0]%16],1);
    HalUARTWrite(0,&asc_16[shaddr[1]%256/16],1);
    HalUARTWrite(0,&asc_16[shaddr[1]%16],1);
    HalUARTWrite(0,"|",1);  //串口显示IEEE address     00 12 4B 00 03 3A 64 F9
    HalUARTWrite(0,pkt->cmd.Data+10,pkt->cmd.DataLength-10);//上传携带的属性名称
    HalUARTWrite(0,"|",1);    
    temp=pkt->clusterId; //读出数据包的Cluster_ID	
    TmpBuf[0] = LO_UINT16( temp );
    TmpBuf[1] = HI_UINT16( temp );                    
    HalUARTWrite(0,&asc_16[TmpBuf[0]%256/16],1);
    HalUARTWrite(0,&asc_16[TmpBuf[0]%16],1);
    HalUARTWrite(0,&asc_16[TmpBuf[1]%256/16],1);
    HalUARTWrite(0,&asc_16[TmpBuf[1]%16],1); 
    HalUARTWrite(0,"|",1);
    temp=pkt->groupId; //读出数据包的Group_ID	  
    TmpBuf[2] = LO_UINT16( temp );
    TmpBuf[3] = HI_UINT16( temp );   
    HalUARTWrite(0,&asc_16[TmpBuf[2]%256/16],1);
    HalUARTWrite(0,&asc_16[TmpBuf[2]%16],1);
    HalUARTWrite(0,&asc_16[TmpBuf[3]%256/16],1);
    HalUARTWrite(0,&asc_16[TmpBuf[3]%16],1);
    HalUARTWrite(0,"|",1);
    tmp = pkt ->endPoint;
    HalUARTWrite(0,&asc_16[tmp%256/16],1);
    HalUARTWrite(0,&asc_16[tmp%16],1);    
    HalUARTWrite(0,"|",1);
    HalUARTWrite(0,"*",1);
    HalUARTWrite(0,"\n",2); 			  // 回车换行
    
  	
    break;
    //暂时未用到
    case SAMPLEAPP_FLASH_CLUSTERID:
      flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
      HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
      break;
  }
  
  osal_mem_free(TmpBuf);
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 * @brief   Send the periodic message.
 * @param   none
 * @return  none
 */
void SampleApp_SendPeriodicMessage( void )
{
  uint8 data[10]={178,201,45,56,46,58,77,32,88,19};
  if ( AF_DataRequest(  &Point_To_Point_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_POINT_TO_POINT_CLUSTERID,
                       10,                //一共10个数据
                       data,              //装载要发送的数据
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {}
}
/*********************************************************************
 * @fn      SampleApp_SendFlashMessage
 * @brief   Send the flash message to group 1.
 * @param   flashTime - in milliseconds
 * @return  none
 */
void SampleApp_SendFlashMessage( uint16 flashTime )
{
  uint8 buffer[3];
  buffer[0] = (uint8)(SampleAppFlashCounter++);
  buffer[1] = LO_UINT16( flashTime );
  buffer[2] = HI_UINT16( flashTime );

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}
/*********************************************************************
 * @fn          initUart1
 * @brief       initial uart1.
 * @param       UART_callback - uart callback. 
 * @return      null.
 */
void initUart1(halUARTCBack_t UART_callback)
{
  halUARTCfg_t uartConfig;
  uartConfig.configured           = TRUE;              
  uartConfig.baudRate             = HAL_UART_BR_115200;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 48;
  uartConfig.rx.maxBufSize        = 128;
  uartConfig.tx.maxBufSize        = 128;
  uartConfig.idleTimeout          = 6;   
  uartConfig.intEnable            = TRUE;              
  uartConfig.callBackFunc         = UART_callback;
  HalUARTOpen (HAL_UART_PORT_1, &uartConfig);
}
/*********************************************************************
 * @fn          UART_callback
 * @brief       Uart callback function.
 * @param       port - Com Port 0 or com port 1. 
 * @param       event - UART Events.
 * @return      null.
 */
void UART_callback ( uint8 port, uint8 event )
{ 
  uint16 uart1_len=0;
  if(( event & ( HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT ) ) ) 
  {   
    if(port == HAL_UART_PORT_0)//不用来接收数据，仅仅用来上传数据
    {
      //(HAL_UART_PORT_0, i8_uart_buf, len);   //读取串口数据到buf指向的内存，处添加数据解析函数    
    }
    else//port == HAL_UART_PORT_1
    {
      uart1_len = Hal_UART_RxBufLen(HAL_UART_PORT_1); //取出本次接收到的字符长度 
      uint8 *uart1_buf = osal_mem_alloc(uart1_len);
      HalUARTRead(HAL_UART_PORT_1, uart1_buf, uart1_len);
    
      //数据逻辑操作
      afAddrType_t SampleApp_Send_DstAddr;//目标地址
      unsigned int len;
      //获取目标网络地址
      uint8 arr[2];
      uint16 nwkAddr;
      StrConvert2Hex(uart1_buf+3, arr,4);//转换网络地址
      nwkAddr = BUILD_UINT16(arr[0],arr[1]); 
      
      SampleApp_Send_DstAddr.addr.shortAddr = nwkAddr;
      SampleApp_Send_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
      SampleApp_Send_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
      len = strlen(uart1_buf);
      
      //HCCU发送控制命令，则将该指令转发给EP
      if(uart1_buf[1] == 'C' ||uart1_buf[1] == 'c'){
        
        //协调器 /c|IP|Cluster_id|Group_id|endPoint----|T,V|
       if( AF_DataRequest( &SampleApp_Send_DstAddr, 
                       &SampleApp_epDesc,
                       SAMPLEAPP_COM_CLUSTERID,
                       len,// 数据长度         
                       uart1_buf,//数据内容
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
       {
         HalUARTWrite(1,"success\n",9);       
       }else{
         HalUARTWrite(1,"failed\n",8);
       }
      }
      
      //发送自身的MAC地址，作为HCCU的唯一标识码；
      if(uart1_buf[1] == 'A' || uart1_buf[1] == 'a'){
         uint8 *TmpBuf = osal_mem_alloc(8);
         uint8 asc_16[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
         uint8 *IEEEAddr;
         //获取长地址
         IEEEAddr = NLME_GetExtAddr();
         osal_cpyExtAddr( TmpBuf,IEEEAddr);
         HalUARTWrite(1,"/H|",3);
         HalUARTWrite(1,&asc_16[TmpBuf[0]%256/16],1);
         HalUARTWrite(1,&asc_16[TmpBuf[0]%16],1);
         HalUARTWrite(1,&asc_16[TmpBuf[1]%256/16],1);
         HalUARTWrite(1,&asc_16[TmpBuf[1]%16],1);
         HalUARTWrite(1,&asc_16[TmpBuf[2]%256/16],1);
         HalUARTWrite(1,&asc_16[TmpBuf[2]%16],1);  
         HalUARTWrite(1,&asc_16[TmpBuf[3]%256/16],1);
         HalUARTWrite(1,&asc_16[TmpBuf[3]%16],1);   
         HalUARTWrite(1,&asc_16[TmpBuf[4]%256/16],1);
         HalUARTWrite(1,&asc_16[TmpBuf[4]%16],1); 
         HalUARTWrite(1,&asc_16[TmpBuf[5]%256/16],1);
         HalUARTWrite(1,&asc_16[TmpBuf[5]%16],1);      
         HalUARTWrite(1,&asc_16[TmpBuf[6]%256/16],1);
         HalUARTWrite(1,&asc_16[TmpBuf[6]%16],1);  
         HalUARTWrite(1,&asc_16[TmpBuf[7]%256/16],1);
         HalUARTWrite(1,&asc_16[TmpBuf[7]%16],1);
         HalUARTWrite(1,"|*\r\n",6);
         osal_mem_free(TmpBuf);
      }
      osal_mem_free(uart1_buf);
    }
  } 
}
/*********************************************************************
 * @fn          Char2Hex
 * @brief       To change a char type into hex .
 * @param       c - A char which will be converted to the corresponding hex. 
 * @return      unsigned char , its range is between 0~F.
 */
unsigned char Char2Hex(char c)
{
  if('A' <= c && c <='Z')
    return c- 'A' +10;
  if('a' <= c && c <= 'z')
    return c-'a' +10;
  return c - '0';
}
/*********************************************************************
 * @fn          StrConvert2Hex
 * @brief       To change a string type into hex .
 * @param       str - A string which will be converted to the corresponding hex. 
 * @param       arr - An array which will save the hex. 
 * @param       length - length of the string.
 * @return      null.
 */
void StrConvert2Hex (const unsigned char *str, uint8 *arr ,int length)
{
  for(int i =0;i<length;++i)
  {
    if(i%2 ==0){
      arr[i/2]=Char2Hex(str[i]);
      arr[i/2] <<=4;
    }
    else
      arr[i/2] |= Char2Hex(str[i]);
  }
}
/*********************************************************************
*********************************************************************/
