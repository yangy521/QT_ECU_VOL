/******************** (C) COPYRIGHT  XI`AN QEXPAND *******************************
* �ļ���  ��DrvSpi.c
* ����    ��Spi�����ļ�       
* ʵ��ƽ̨��ECU
* ��汾  ��V1.0.0
* ����    ��
* �޸�ʱ��: 2023-05-16
**********************************************************************************/	
#include "Device.h"

void vDrvSpiInit(void)
{
	/*******************************************************************************
	* SPI
	*�ܽ�51	PC10 SPI2_SCK	CLK;	�ܽ�52	PC11 SPI2_MISO	DO;
	*�ܽ�53	PC12 SPI2_MOSI DI;		�ܽ�54	PD0 CS;
	*******************************************************************************/
	rcu_periph_clock_enable(RCU_SPI2);
	/* SPI0 GPIO config:SCK/PA5, MISO/PA6, MOSI/PA7 */
	gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
	gpio_init(GPIOD, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);	
	gpio_pin_remap_config(GPIO_SPI2_REMAP, ENABLE);

	spi_parameter_struct spi_init_struct;
	/* SPI0 parameter config */
	spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
	spi_init_struct.device_mode          = SPI_MASTER;
	spi_init_struct.frame_size           = SPI_FRAMESIZE_16BIT;
	spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
	spi_init_struct.nss                  = SPI_NSS_SOFT;
	spi_init_struct.prescale             = SPI_PSC_32;
	spi_init_struct.endian               = SPI_ENDIAN_MSB;
	spi_init(SPI2, &spi_init_struct);

	spi_enable(SPI2);	//ʹ��SPI
	gpio_bit_write(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
}

// SPIд�뺯��
void spiWrite(uint16_t data) 
{
    while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
    spi_i2s_data_transmit(SPI2, data);
}

// SPI��ȡ����
uint16_t spiRead(void)
{
    while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
    return spi_i2s_data_receive(SPI2);
}

