#include "i2c.h"
#include "stm32wbxx_ll_cortex.h"
#include "stm32wbxx_ll_i2c.h"
#include "stm32wbxx_ll_gpio.h"
#include "stm32wbxx_ll_rcc.h"
#include "stm32wbxx_ll_bus.h"

#define I2C_MP2664		I2C3

#if 0
void i2c_init(void)
{
	LL_I2C_InitTypeDef I2C_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	LL_RCC_SetI2CClockSource(LL_RCC_I2C3_CLKSOURCE_PCLK1);

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	/**I2C3 GPIO Configuration
	PA7   ------> I2C3_SCL
	PB4   ------> I2C3_SDA
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3);

	/** I2C Initialization
	*/
	I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
	I2C_InitStruct.Timing = 0x10707DBC;
	I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
	I2C_InitStruct.DigitalFilter = 0;
	I2C_InitStruct.OwnAddress1 = 0;
	I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
	I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
	LL_I2C_Init(I2C3, &I2C_InitStruct);
	LL_I2C_EnableAutoEndMode(I2C3);
	LL_I2C_SetOwnAddress2(I2C3, 0, LL_I2C_OWNADDRESS2_NOMASK);
	LL_I2C_DisableOwnAddress2(I2C3);
	LL_I2C_DisableGeneralCall(I2C3);
	LL_I2C_EnableClockStretching(I2C3);
}

uint8_t i2c_check_address(uint8_t i2c_addr)
{
    LL_I2C_HandleTransfer(I2C_MP2664, i2c_addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    LL_I2C_TransmitData8(I2C_MP2664, 0x00);

    while( LL_I2C_IsActiveFlag_TXE(I2C_MP2664) == RESET ) {
        if(LL_SYSTICK_IsActiveCounterFlag() && LL_I2C_IsActiveFlag_NACK(I2C_MP2664)) {
            return I2C_ADDR_NACK;
        }
    }

    LL_I2C_ClearFlag_STOP(I2C_MP2664);

    return I2C_OK;
}

#if 0
void i2c_detect(void)
{
	uint8_t devices = 0u;
	extern I2C_HandleTypeDef hi2c1;

	printf("Searching for I2C devices on the bus...\n");
	/* Values outside 0x03 and 0x77 are invalid. */
	for (uint8_t i = 0x03u; i < 0x78u; i++)
	{
		uint8_t address = i << 1u ;
		/* In case there is a positive feedback, print it out. */
		if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c1, address, 3u, 10u))
		{
			printf("Device found: 0x%02X\n", address);
			devices++;
		}
	}
	/* Feedback of the total number of devices. */
	if (0u == devices)
	{
	printf("No device found.\n");
	}
	else
	{
	printf("Total found devices: %d\n", devices);
	}
}
#endif

void i2c_read_register(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data)
{
    LL_I2C_HandleTransfer(I2C_MP2664, reg_addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    while( LL_I2C_IsActiveFlag_ADDR(I2C_MP2664) != RESET );

    LL_I2C_TransmitData8(I2C_MP2664, reg_addr);
    while( LL_I2C_IsActiveFlag_TXE(I2C_MP2664) == RESET );

    LL_I2C_ClearFlag_STOP(I2C_MP2664);

    LL_I2C_HandleTransfer(I2C_MP2664, reg_addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

    while( LL_I2C_IsActiveFlag_RXNE(I2C_MP2664) == RESET );
    *data = LL_I2C_ReceiveData8(I2C_MP2664);

    LL_I2C_ClearFlag_STOP(I2C_MP2664);
}



void i2c_write_register(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data)
{
    LL_I2C_HandleTransfer(I2C_MP2664, i2c_addr, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    while( LL_I2C_IsActiveFlag_ADDR(I2C_MP2664) != RESET );

    LL_I2C_TransmitData8(I2C_MP2664, reg_addr);
    while( LL_I2C_IsActiveFlag_TXE(I2C_MP2664) == RESET );

    LL_I2C_TransmitData8(I2C_MP2664, data);
    while( LL_I2C_IsActiveFlag_TXE(I2C_MP2664) == RESET );

    LL_I2C_ClearFlag_STOP(I2C_MP2664);

    //Delay_ms(5);
}

void i2c_read_registers(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t size)
{
    LL_I2C_ClearFlag_STOP(I2C_MP2664);

    LL_I2C_HandleTransfer(I2C_MP2664, i2c_addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    while( LL_I2C_IsActiveFlag_ADDR(I2C_MP2664) != RESET );

    LL_I2C_TransmitData8(I2C_MP2664, reg_addr);
    while( LL_I2C_IsActiveFlag_TXE(I2C_MP2664) == RESET );

    LL_I2C_ClearFlag_STOP(I2C_MP2664);

    LL_I2C_HandleTransfer(I2C_MP2664, i2c_addr, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

    do {
        while( LL_I2C_IsActiveFlag_RXNE(I2C_MP2664) == RESET );
        *buffer = LL_I2C_ReceiveData8(I2C_MP2664);
        buffer++;
    } while(--size > 0);

    LL_I2C_ClearFlag_STOP(I2C_MP2664);
}

void i2c_write_registers(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t size)
{
    uint8_t wr_cnt = 0, tmp = 0;

    LL_I2C_ClearFlag_STOP(I2C_MP2664);

    do{

        wr_cnt = (size > 254) ? 254 : size;

        tmp = wr_cnt;

        LL_I2C_HandleTransfer(I2C_MP2664, i2c_addr, LL_I2C_ADDRSLAVE_7BIT, wr_cnt+1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
        while( LL_I2C_IsActiveFlag_ADDR(I2C_MP2664) != RESET );

        LL_I2C_TransmitData8(I2C_MP2664, reg_addr);
        while( LL_I2C_IsActiveFlag_TXE(I2C_MP2664) == RESET );

        do{
            LL_I2C_TransmitData8(I2C_MP2664, *buffer);
            while( LL_I2C_IsActiveFlag_TXE(I2C_MP2664) == RESET );
            buffer++;
        }while(--wr_cnt > 0);

        size -= tmp;
        reg_addr += tmp;

        LL_I2C_ClearFlag_STOP(I2C_MP2664);

        //Delay_ms(5);

    }while(size > 0);
}
#endif
