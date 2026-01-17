#include "BH1750.h"

/* ============================================================
   I2C2 INIT (PB10 = SCL, PB11 = SDA)
   ============================================================ */
void I2C2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;

    /* Clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* PB10 - SCL, PB11 - SDA */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C config */
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStruct.I2C_ClockSpeed = 100000;
    I2C_Init(I2C2, &I2C_InitStruct);

    I2C_Cmd(I2C2, ENABLE);
}

/* ============================================================
   I2C WRITE 1 BYTE
   ============================================================ */
void I2C2_WriteByte(uint8_t addr, uint8_t data)
{
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));

    I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C2, addr << 1, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(I2C2, data);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTOP(I2C2, ENABLE);
}

/* ============================================================
   I2C READ 2 BYTES (BH1750 OUTPUT)
   ============================================================ */
uint16_t I2C2_Read2Bytes(uint8_t addr)
{
    uint16_t value;

    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));

    I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C2, addr << 1, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    /* Read MSB */
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    value = (I2C_ReceiveData(I2C2) << 8);

    /* Read LSB */
    I2C_AcknowledgeConfig(I2C2, DISABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    value |= I2C_ReceiveData(I2C2);

    I2C_GenerateSTOP(I2C2, ENABLE);
    I2C_AcknowledgeConfig(I2C2, ENABLE);

    return value;
}

/* ============================================================
   BH1750 INIT
   ============================================================ */
void BH1750_Init(void)
{
    I2C2_WriteByte(BH1750_ADDR_LOW, 0x01);  // POWER ON
    I2C2_WriteByte(BH1750_ADDR_LOW, 0x10);  // CONTINUOUS H-RES MODE (1 lux)
}

/* ============================================================
   READ LUX
   ============================================================ */
float BH1750_ReadLux(void)
{
    uint16_t raw = I2C2_Read2Bytes(BH1750_ADDR_LOW);
    return (float)raw / 1.2f;
}
