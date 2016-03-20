#include "I2C.h"
#include "globals.h"
#include "Daten_Filter.h"
#include "PID.h"
#include "main.h"
#include <stdio.h>
#define SLAVE_ADDR (MPU6050_ADDRESS_AD0_LOW<<1)
#define TIMEOUT 2000

static I2C_HandleTypeDef hnd;
static DMA_HandleTypeDef hndDMArx;
static DMA_HandleTypeDef hndDMAtx;
static uint8_t tmpBuffer[14];

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	//Enable the GPIO-clock
	__GPIOB_CLK_ENABLE();

	//Enable the I2C-Clock
	__I2C1_CLK_ENABLE();

	//Enable the DMA-Clock
	__DMA1_CLK_ENABLE();

	//Configure GPIOB Pin_6 as SCL for I2C
	GPIO_InitStruct.Pin       = GPIO_PIN_6;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	//init GPIOB PIN6 (SCL-Pin)
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	//Configure GPIOB Pin_7 as SDA for USART
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	//init GPIOB PIN7 (SDA-Pin)
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Alternate = 0;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = 0;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	hndDMArx.Instance = DMA1_Stream5;
	hndDMArx.Init.Channel = DMA_CHANNEL_1;
	hndDMArx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hndDMArx.Init.PeriphInc = DMA_PINC_DISABLE;
	hndDMArx.Init.MemInc = DMA_MINC_ENABLE;
	hndDMArx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hndDMArx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hndDMArx.Init.Mode = DMA_NORMAL;
	hndDMArx.Init.Priority = DMA_PRIORITY_MEDIUM;
	hndDMArx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hndDMArx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hndDMArx.Init.PeriphBurst = DMA_PBURST_SINGLE;
	hndDMArx.Init.MemBurst = DMA_MBURST_SINGLE;


	HAL_DMA_Init(&hndDMArx);

	__HAL_LINKDMA(hi2c, hdmarx, hndDMArx);

	hndDMAtx.Instance = DMA1_Stream6;
	hndDMAtx.Init.Channel = DMA_CHANNEL_1;
	hndDMAtx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hndDMAtx.Init.PeriphInc = DMA_PINC_DISABLE;
	hndDMAtx.Init.MemInc = DMA_MINC_ENABLE;
	hndDMAtx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hndDMAtx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hndDMAtx.Init.Mode = DMA_NORMAL;
	hndDMAtx.Init.Priority = DMA_PRIORITY_MEDIUM;
	hndDMAtx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hndDMAtx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hndDMAtx.Init.PeriphBurst = DMA_PBURST_SINGLE;
	hndDMAtx.Init.MemBurst = DMA_MBURST_SINGLE;

	HAL_DMA_Init(&hndDMAtx);

	__HAL_LINKDMA(hi2c, hdmatx, hndDMAtx);

	//Setup the Interruptlevel for I2C
	HAL_NVIC_SetPriority(I2C1_EV_IRQn, 1, 1);
	HAL_NVIC_SetPriority(I2C1_ER_IRQn, 1, 1);
	//enable the interrupt for I2C1
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
	HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
	//Setup the Interruptlevel for DMA
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 1);
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 1);
	//enable the interrupt for DMA1
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	//Setup the Interruptlevel for INT-Pin GPIOB 3
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0x0F , 4);
}




int initMPU(void){
	int initOkay = -1;
	HAL_I2C_StateTypeDef state;
	hnd.Instance = I2C1;

	hnd.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hnd.Init.ClockSpeed	= 400000;	//DEF MODE
	hnd.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hnd.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hnd.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hnd.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	hnd.Init.OwnAddress1 = 1;
	hnd.Init.OwnAddress2 = 2;
	
	HAL_I2C_Init(&hnd);

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
	
	HAL_Delay(50);
	
	state = HAL_I2C_GetState(&hnd); 
	
	if(state == HAL_I2C_STATE_READY){
		initOkay = 0;
	}
	
	printf("WHO IS MPU: %x\n",SCCB_Read(MPU6050_RA_WHO_AM_I));
	
	Initial_MPU6050();
	//Enable IRQ for Interupt Pin GPIOB 3
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	return initOkay;

}

void Initial_MPU6050(void)
{
	HAL_Delay(50); // for stability
	//    Reset to defalt
	MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, ENABLE);
	//	  SetClockSource(MPU6050_CLOCK_PLL_XGYRO)
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
	//    SetFullScaleGyroRange(MPU6050_GYRO_FS_250)
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_250);
	//    SetFullScaleAccelRange(MPU6050_ACCEL_FS_2)
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_2);
	//    interupt(Enable)
	MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, ENABLE);
	//    SetSleepModeStatus(DISABLE)
	MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, DISABLE);
	//			SetDLPF(MPU6050_DLPF_BW_5)
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_98);

	HAL_Delay(50); // for stability

	MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, DISABLE);

	HAL_Delay(50); // for stability
}


void MPU6050_GetRawAccelGyro(int16_t* AccelGyro)
{
	int32_t i;
	HAL_StatusTypeDef error;
	//	  if(HAL_I2C_Mem_Read(&hnd,MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_XOUT_H,1,tmpBuffer,14,3)!=HAL_OK){
	//			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
	//		}
	if(HAL_I2C_GetState(&hnd) == HAL_I2C_STATE_READY){
		error = HAL_I2C_Mem_Read_DMA(&hnd,MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_XOUT_H,1,tmpBuffer,14);
		if(error!=HAL_OK){
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
			 /* Generate Stop */
			 HAL_Delay10u(10);
			 hnd.Instance->SR2;
			 hnd.Instance->CR1 |= I2C_CR1_STOP;
			 hnd.Instance->SR2;
			 HAL_Delay10u(10);
			 hnd.Instance->CR1 |= I2C_CR1_START;
			 hnd.Instance->SR2;
			 /* Generate Stop */
			 HAL_Delay10u(10);
			 hnd.Instance->CR1 |= I2C_CR1_STOP;
			 hnd.Instance->SR2;

		}else{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
		}
	}
	//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
	/* Get acceleration */
	for (i = 0; i < 3; i++)
		AccelGyro[i] = ((int16_t) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);
	/* Get Temp */
	AccelGyro[3] = ((int16_t) ((uint16_t) tmpBuffer[2 * 3] << 8) + tmpBuffer[2 * 3 + 1]);
	/* Get Angular rate */
	for (i = 4; i < 7; i++)
		AccelGyro[i] = ((int16_t) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);
}

uint8_t SCCB_Write(uint8_t addr, uint8_t data)
{
    uint8_t ret=0;
    uint8_t buf[2] = {0, 0};
		buf[0] = addr;
		buf[1] = data;
    //__disable_irq();
    if (HAL_I2C_Master_Transmit(&hnd, SLAVE_ADDR, buf, 2, TIMEOUT) != HAL_OK) {
        ret=0xFF;
    }
    //__enable_irq();
    return ret;
}

uint8_t SCCB_Read(uint8_t addr)
{
    uint8_t data=0;
		uint8_t res =  HAL_I2C_Master_Transmit(&hnd, SLAVE_ADDR, &addr, 1, TIMEOUT);
	printf("res: %u",res);
    //__disable_irq();
    if (res != HAL_OK) {
        data = 0xFF;
        goto error_w;
    }
    if (HAL_I2C_Master_Receive(&hnd, SLAVE_ADDR, &data, 1, TIMEOUT) != HAL_OK) {
        data = 0xFF;
    }
error_w:
    //__enable_irq();
    return data;
}


void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t tmp;
	uint8_t mask;
	HAL_I2C_Mem_Read(&hnd,slaveAddr,regAddr,1,&tmp,1,1);
    mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data; // combine data with existing byte
    HAL_I2C_Mem_Write(&hnd,slaveAddr,regAddr,1,&tmp,1,3);
}

void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t tmp;
	HAL_I2C_Mem_Read(&hnd,slaveAddr,regAddr,1,&tmp,1,3);
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    HAL_I2C_Mem_Write(&hnd,slaveAddr,regAddr,1,&tmp,1,1);
}

void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t tmp;
	uint8_t mask;
	HAL_I2C_Mem_Read(&hnd,slaveAddr,regAddr,1,&tmp,1,3);
    mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmp &= mask;
    tmp >>= (bitStart - length + 1);
    *data = tmp;
}

void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t tmp;
	HAL_I2C_Mem_Read(&hnd,slaveAddr,regAddr,1,&tmp,1,3);
    *data = tmp & (1 << bitNum);
}

void DMA1_Stream5_IRQHandler(void) {
	HAL_DMA_IRQHandler(hnd.hdmarx);
}

void DMA1_Stream6_IRQHandler(void) {
	HAL_DMA_IRQHandler(hnd.hdmatx);
}

void I2C1_EV_IRQHandler(void) {
	HAL_I2C_EV_IRQHandler(&hnd);  //Eventhandler - Dieser kümmert sich automatisch um das erzeugen von Startconditions, ACKs, NACKs, Stopbits und Stopcondition
}

void I2C1_ER_IRQHandler(void) {
	HAL_I2C_ER_IRQHandler(&hnd);  //Errorhandler
}

void EXTI3_IRQHandler(void){
	callback_MPU6050();
}
