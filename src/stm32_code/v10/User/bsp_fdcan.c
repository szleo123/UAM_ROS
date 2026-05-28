#include "bsp_fdcan.h"
/**
************************************************************************
* @brief:      	bsp_can_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能
************************************************************************
**/
void bsp_can_init(void)
{
	
	can1_filter_init();
  can2_filter_init();
  can3_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
//	HAL_FDCAN_Start(&hfdcan2);
//	HAL_FDCAN_Start(&hfdcan3);
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_WATERMARK, 0);
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan1,
                                       0 | FDCAN_IT_RX_FIFO0_WATERMARK | FDCAN_IT_RX_FIFO0_WATERMARK
                                           | FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF
                                           | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR
                                           | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING,
                                       0x00000F00);
    HAL_FDCAN_Start(&hfdcan2);                               //开启FDCAN
//	HAL_FDCAN_Start(&hfdcan2);
//	HAL_FDCAN_Start(&hfdcan3);
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_WATERMARK, 0);
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2,
                                       0 | FDCAN_IT_RX_FIFO0_WATERMARK | FDCAN_IT_RX_FIFO0_WATERMARK
                                           | FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF
                                           | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR
                                           | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING,
                                       0x00000F00);
//    HAL_FDCAN_Start(&hfdcan3);                               //开启FDCAN
////	HAL_FDCAN_Start(&hfdcan2);
////	HAL_FDCAN_Start(&hfdcan3);
////	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
////	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_WATERMARK, 0);
////	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);
//	HAL_FDCAN_ActivateNotification(&hfdcan3,
//                                       0 | FDCAN_IT_RX_FIFO0_WATERMARK | FDCAN_IT_RX_FIFO0_WATERMARK
//                                           | FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF
//                                           | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR
//                                           | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING,
//                                       0x00000F00);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
//void can1_filter_init(void)
//{
//	FDCAN_FilterTypeDef fdcan_filter;
//	
//	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
//	fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
//	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
//	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
//	fdcan_filter.FilterID1 = 0x00;                               
//	fdcan_filter.FilterID2 = 0x00;

//	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter); 		 				  //接收ID2
//	//拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
//	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
//	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
//    
////	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1);
////	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
//}

void can1_filter_init(void)
{
    FDCAN_FilterTypeDef fdcan_filter;

    // ==========================================================
    // 1. 配置标准帧过滤器 (为了你的旧电机)
    // ==========================================================
    fdcan_filter.IdType = FDCAN_STANDARD_ID;      // 类型：标准帧
    fdcan_filter.FilterIndex = 0;                 // 标准帧过滤器索引0
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x00;                // 掩码0，接收所有标准帧
    fdcan_filter.FilterID2 = 0x00;
    
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter); // 配置进去

    // ==========================================================
    // 2. 配置扩展帧过滤器 (为了你的AK电机)
    // ==========================================================
    // 注意：不用担心覆盖，HAL库会根据 IdType 写到不同的寄存器区域
    fdcan_filter.IdType = FDCAN_EXTENDED_ID;      // 类型：扩展帧
    fdcan_filter.FilterIndex = 0;                 // 扩展帧过滤器索引0 (与标准帧索引独立)
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x00;                // 掩码0，接收所有扩展帧
    fdcan_filter.FilterID2 = 0x00;

    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter); // 配置进去

    // ==========================================================
    // 3. 全局配置
    // ==========================================================
    // 拒绝不匹配的帧 (Reject)，但因为上面两个过滤器都是Mask 0x00 (全通)，
    // 所以实际上标准帧和扩展帧都会通过过滤器进入FIFO0。
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
}

/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
//void can2_filter_init(void)
//{
//	FDCAN_FilterTypeDef fdcan_filter;
//	
//	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
//	fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
//	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
//	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
//	fdcan_filter.FilterID1 = 0x00;                               
//	fdcan_filter.FilterID2 = 0x00;

//	HAL_FDCAN_ConfigFilter(&hfdcan2,&fdcan_filter); 		 				  //接收ID2
//	//拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
//	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
//	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
//    
////	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1);
////	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
//}

void can2_filter_init(void)
{
    FDCAN_FilterTypeDef fdcan_filter;
    
    // ==========================================================
    // 1. 配置标准帧过滤器 (为了接收达妙电机等标准帧设备)
    // ==========================================================
    fdcan_filter.IdType = FDCAN_STANDARD_ID;                       // 标准ID
    fdcan_filter.FilterIndex = 0;                                  // 标准帧滤波器索引0
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   // 掩码模式
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           // 关联到 FIFO0  
    fdcan_filter.FilterID1 = 0x00;                                 // 掩码0，接收所有标准帧
    fdcan_filter.FilterID2 = 0x00;

    HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);               // 写入标准帧配置

    // ==========================================================
    // 2. 配置扩展帧过滤器 (为了接收CubeMars等扩展帧设备)
    // ==========================================================
    fdcan_filter.IdType = FDCAN_EXTENDED_ID;                       // 扩展ID
    fdcan_filter.FilterIndex = 0;                                  // 扩展帧滤波器索引0 (与标准帧独立，不会冲突)
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   // 掩码模式
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           // 同样关联到 FIFO0
    fdcan_filter.FilterID1 = 0x00;                                 // 掩码0，接收所有扩展帧
    fdcan_filter.FilterID2 = 0x00;

    HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);               // 写入扩展帧配置

    // ==========================================================
    // 3. 全局配置
    // ==========================================================
    // 拒绝接收匹配不成功的标准ID和扩展ID, 不接收远程帧
    // (因为上面两个Filter设置了 Mask 0x00 全通，所以所有正常的数据帧都会顺利进入 FIFO0)
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    
    // 配置 FIFO0 水位线，收到 1 帧就触发中断
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
}

/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can3_filter_init(void)
{
//	FDCAN_FilterTypeDef fdcan_filter;
//	
//	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
//	fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
//	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
//	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
//	fdcan_filter.FilterID1 = 0x00;                               
//	fdcan_filter.FilterID2 = 0x00;

//	HAL_FDCAN_ConfigFilter(&hfdcan3,&fdcan_filter); 		 				  //接收ID2
//	//拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
//	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
//	HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
    
//	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1);
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
}

void bsp_fdcan_set_baud(hcan_t *hfdcan, uint8_t mode, uint8_t baud)
{
	uint32_t nom_brp=0, nom_seg1=0, nom_seg2=0, nom_sjw=0;
	uint32_t dat_brp=0, dat_seg1=0, dat_seg2=0, dat_sjw=0;
	
	/*	nominal_baud = 80M/brp/(1+seg1+seg2)
		sample point = (1+seg1)/(1+seg1+sjw)  */
	if(mode == CAN_CLASS)
	{
		switch (baud)
		{
			case CAN_BR_125K: 	nom_brp=4 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; // sample point 87.5%
			case CAN_BR_200K: 	nom_brp=2 ; nom_seg1=174; nom_seg2=25; nom_sjw=25; break; // sample point 87.5%
			case CAN_BR_250K: 	nom_brp=2 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; // sample point 87.5%
			case CAN_BR_500K: 	nom_brp=1 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; // sample point 87.5%
			case CAN_BR_1M:		nom_brp=1 ; nom_seg1=59 ; nom_seg2=20; nom_sjw=20; break; // sample point 75%
		}
		dat_brp=1 ; dat_seg1=29; dat_seg2=10; dat_sjw=10;	// 仲裁域默认1M
		hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	}
	/*	data_baud	 = 80M/brp/(1+seg1+seg2)
		sample point = (1+seg1)/(1+seg1+sjw)  */
	if(mode == CAN_FD_BRS)
	{
		switch (baud)
		{
			case CAN_BR_2M: 	dat_brp=1 ; dat_seg1=29; dat_seg2=10; dat_sjw=10; break;	// sample point 75%
			case CAN_BR_2M5: 	dat_brp=1 ; dat_seg1=25; dat_seg2=6 ; dat_sjw=6 ; break;	// sample point 81.25%
			case CAN_BR_3M2: 	dat_brp=1 ; dat_seg1=19; dat_seg2=5 ; dat_sjw=5 ; break;	// sample point 80%
			case CAN_BR_4M: 	dat_brp=1 ; dat_seg1=14; dat_seg2=5 ; dat_sjw=5 ; break;	// sample point 75%
			case CAN_BR_5M:		dat_brp=1 ; dat_seg1=13; dat_seg2=2 ; dat_sjw=2 ; break;	// sample point 87.5%
		}
		nom_brp=1 ; nom_seg1=59 ; nom_seg2=20; nom_sjw=20; // 数据域默认1M
		hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
	}
	
	HAL_FDCAN_DeInit(hfdcan);
	
	hfdcan->Init.NominalPrescaler = nom_brp;
	hfdcan->Init.NominalTimeSeg1  = nom_seg1;
	hfdcan->Init.NominalTimeSeg2  = nom_seg2;
	hfdcan->Init.NominalSyncJumpWidth = nom_sjw;
	
	hfdcan->Init.DataPrescaler = dat_brp;
	hfdcan->Init.DataTimeSeg1  = dat_seg1;
	hfdcan->Init.DataTimeSeg2  = dat_seg2;
	hfdcan->Init.DataSyncJumpWidth = dat_sjw;
	
	HAL_FDCAN_Init(hfdcan);
}


/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
	
	if(len<=8)
		pTxHeader.DataLength = len;
	else if(len==12)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
	else if(len==16)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
	else if(len==20)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
	else if(len==24)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
	else if(len==32)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
	else if(len==48)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
	else if(len==64)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
	
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_ON;
    pTxHeader.FDFormat=FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
 
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data)!=HAL_OK) 
		return 1;//发送
	return 0;	
}


/**
 * @brief  通用发送函数 V2 (兼容标准帧和扩展帧)
 * @param  hfdcan: FDCAN句柄
 * @param  id: CAN ID (如果是扩展帧，可以传29位ID)
 * @param  id_type: FDCAN_STANDARD_ID (旧电机) 或 FDCAN_EXTENDED_ID (AK电机)
 * @param  data: 数据指针
 * @param  len: 数据长度
 */
uint8_t fdcanx_send_data_v2(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint32_t id_type, uint8_t *data, uint32_t len)
{   
    FDCAN_TxHeaderTypeDef pTxHeader;

    // 1. ID 配置
    pTxHeader.Identifier = id;
    pTxHeader.IdType = id_type; // AK电机使用 FDCAN_EXTENDED_ID

    // 2. 帧格式配置 (重要！AK电机必须用经典模式)
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
    pTxHeader.FDFormat = FDCAN_FRAME_CLASSIC; // 强制使用经典 CAN (兼容性最好)
    pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;  // 经典 CAN 不需要波特率切换
    
    // 3. 长度配置 (修复了这里！使用标准宏定义)
    // STM32H7 必须使用 FDCAN_DLC_BYTES_x 宏，不能直接赋值 len
    switch(len)
    {
        case 0: pTxHeader.DataLength = FDCAN_DLC_BYTES_0; break;
        case 1: pTxHeader.DataLength = FDCAN_DLC_BYTES_1; break;
        case 2: pTxHeader.DataLength = FDCAN_DLC_BYTES_2; break;
        case 3: pTxHeader.DataLength = FDCAN_DLC_BYTES_3; break;
        case 4: pTxHeader.DataLength = FDCAN_DLC_BYTES_4; break;
        case 5: pTxHeader.DataLength = FDCAN_DLC_BYTES_5; break;
        case 6: pTxHeader.DataLength = FDCAN_DLC_BYTES_6; break;
        case 7: pTxHeader.DataLength = FDCAN_DLC_BYTES_7; break;
        case 8: pTxHeader.DataLength = FDCAN_DLC_BYTES_8; break; // 关键修复
        // 以下是 FD 模式才会用到的，但在 Classic 模式下写了也无妨，只要 len 没传错
        case 12: pTxHeader.DataLength = FDCAN_DLC_BYTES_12; break;
        case 16: pTxHeader.DataLength = FDCAN_DLC_BYTES_16; break;
        case 20: pTxHeader.DataLength = FDCAN_DLC_BYTES_20; break;
        case 24: pTxHeader.DataLength = FDCAN_DLC_BYTES_24; break;
        case 32: pTxHeader.DataLength = FDCAN_DLC_BYTES_32; break;
        case 48: pTxHeader.DataLength = FDCAN_DLC_BYTES_48; break;
        case 64: pTxHeader.DataLength = FDCAN_DLC_BYTES_64; break;
        default: 
            // 如果长度不匹配标准值，默认按 8 字节处理 (或直接返回错误)
            pTxHeader.DataLength = FDCAN_DLC_BYTES_8; 
            break;
    }

    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker = 0;

    // 发送
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK) 
        return 1;
    
    return 0;   
}


/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{	
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len;
	
	if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &pRxHeader, buf)==HAL_OK)
	{
		*rec_id = pRxHeader.Identifier;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8)
			len = pRxHeader.DataLength;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_12)
			len = 12;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_16)
			len = 16;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_20)
			len = 20;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_24)
			len = 24;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_32)
			len = 32;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_48)
			len = 48;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_64)
			len = 64;
		
		return len;//接收数据
	}
	return 0;	
}



__weak void fdcan1_rx_callback(void)
{

}


__weak void fdcan2_rx_callback(void)
{

}

__weak void fdcan3_rx_callback(void)
{

}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan == &hfdcan1)
    {
		fdcan1_rx_callback();
    }
    
    if (hfdcan == &hfdcan2)
    {
		fdcan2_rx_callback();
    }
//    if (hfdcan == &hfdcan3)
//    {
//		fdcan3_rx_callback();
//    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{


}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	if(ErrorStatusITs & FDCAN_IR_BO)
	{
		CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
	}
	if(ErrorStatusITs & FDCAN_IR_EP)
	{
//		MX_FDCAN1_Init();
//		bsp_can_init();
	}
}









