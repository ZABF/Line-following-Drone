//#include "system.h"
#include "vl53lxx_i2c.h"
#include "vl53lxx.h"
#include "vl53l1_api.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly	
 * vl53lxx应用代码, 包括vl53l0x和vl53l1x
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

//TaskHandle_t vl53l0xTaskHandle = NULL;
//TaskHandle_t vl53l1xTaskHandle = NULL;

u16 vl53lxxId = 0;	/*vl53芯片ID*/
bool isEnableVl53lxx = true;		/*是否使能激光*/

static bool isInitvl53l0x = false;	/*初始化vl53l0x*/
static bool isInitvl53l1x = false;	/*初始化vl53l1x*/
static bool reInitvl53l0x = false;	/*再次初始化vl53l0x*/
static bool reInitvl53l1x = false;	/*再次初始化vl53l1x*/

static u8 count = 0;
static u8 validCnt = 0;
static u8 inValidCnt = 0;

static u16 range_last = 0;
float quality = 1.0f;

zRange_t vl53lxx;


void vl53l0xTask(void* arg);
void vl53l1xTask(void* arg);
	
void vl53lxxInit(void)
{
	vl53IICInit();	
	delay_ms(10);
	
	/*vl53l1x 初始化*/
	VL53L1_RdWord(&dev, 0x010F, &vl53lxxId);
	if(vl53lxxId == VL53L1X_ID){	
		vl53l1xSetParam();
		return;
	}	
	
	vl53lxxId = 0;
}

void read_vl53l1x(void)
{
	int status;
	u8 isDataReady = 0;
	static VL53L1_RangingMeasurementData_t rangingData;

	status = VL53L1_GetMeasurementDataReady(&dev, &isDataReady);
				
	if(isDataReady){
		status = VL53L1_GetRangingMeasurementData(&dev, &rangingData);
		if(status==0){
			range_last = rangingData.RangeMilliMeter * 0.1f;	/*单位cm*/				
		}
		status = VL53L1_ClearInterruptAndStartMeasurement(&dev);
	}	
	
	if(range_last < VL53L1X_MAX_RANGE)			
		validCnt++;			
	else 			
		inValidCnt++;			
	
	if(inValidCnt + validCnt == 10){
		quality += (validCnt/10.f - quality) * 0.1f;	/*低通*/
		validCnt = 0;
		inValidCnt = 0;
	}	
	if (range_last != 0 && range_last < VL53L1X_MAX_RANGE) {			
		vl53lxx.distance = 	(float)range_last;
	}
}
//void vl53l1xTask(void* arg)
//{
//	int status;
//	u8 isDataReady = 0;
////	TickType_t xLastWakeTime = xTaskGetTickCount();;
//	static VL53L1_RangingMeasurementData_t rangingData;

//	vl53l1xSetParam();	/*设置vl53l1x 参数*/
//	
//	while(1) 
//	{
//		if(reInitvl53l1x == true)
//		{
//			count = 0;
//			reInitvl53l1x = false;			
//			vl53l1xSetParam();	/*设置vl53l1x 参数*/
////			xLastWakeTime = xTaskGetTickCount();
//		}else
//		{	
//			status = VL53L1_GetMeasurementDataReady(&dev, &isDataReady);
//						
//			if(isDataReady)
//			{
//				status = VL53L1_GetRangingMeasurementData(&dev, &rangingData);
//				if(status==0)
//				{
//					range_last = rangingData.RangeMilliMeter * 0.1f;	/*单位cm*/				
//				}
//				status = VL53L1_ClearInterruptAndStartMeasurement(&dev);
//			}	
//			
//			if(range_last < VL53L1X_MAX_RANGE)			
//				validCnt++;			
//			else 			
//				inValidCnt++;			
//			
//			if(inValidCnt + validCnt == 10)
//			{
//				quality += (validCnt/10.f - quality) * 0.1f;	/*低通*/
//				validCnt = 0;
//				inValidCnt = 0;
//			}
//			
////			if(getModuleID() != OPTICAL_FLOW)
////			{
////				if(++count > 10)
////				{
////					count = 0;
////					VL53L1_StopMeasurement(&dev);
////					vTaskSuspend(vl53l1xTaskHandle);	/*挂起激光测距任务*/					
////				}				
////			}else count = 0;
//						
////			vTaskDelayUntil(&xLastWakeTime, 50);
//		}		
//	}
//}

//bool vl53lxxReadRange(zRange_t* zrange)
//{
//	if(vl53lxxId == VL53L1X_ID) {
//		zrange->quality = quality;		//可信度
//		vl53lxx.quality = quality;
//		
//		if (range_last != 0 && range_last < VL53L1X_MAX_RANGE) {			
//			zrange->distance = (float)range_last;	//单位[cm]	
//			vl53lxx.distance = 	zrange->distance;
//			return true;
//		}
//	}	
//	return false;
//}

/*使能激光*/
void setVl53lxxState(u8 enable)
{
	isEnableVl53lxx = enable;
}


