/**
 * Copyright (c) 2021 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
  * ----------------------------------------------------------------------------------------------------
  * Includes
  * ----------------------------------------------------------------------------------------------------
  */
#include <stdio.h>

#include "port_common.h"

#include "wizchip_conf.h"
#include "w6x00_spi.h"

/**
  * ----------------------------------------------------------------------------------------------------
  * Variables
  * ----------------------------------------------------------------------------------------------------
  */

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;

/**
  * ----------------------------------------------------------------------------------------------------
  * Functions
  * ----------------------------------------------------------------------------------------------------
  */
static inline void wizchip_select(void)
{
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
}

static inline void wizchip_deselect(void)
{
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void wizchip_reset()
{

}

static uint8_t wizchip_read(void)
{
    uint8_t rx_data = 0;
    uint8_t tx_data = 0xFF;

		while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
    HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, 1, 10);

    return rx_data;
}

static void wizchip_write(uint8_t tx_data)
{
		uint8_t rtnByte;

    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
    HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rtnByte, 1, 10);
}

#ifndef USE_SPI_DMA
static void wizchip_read_buf(uint8_t* rx_data, datasize_t len)
{
    uint8_t *tx_data;
    tx_data = (uint8_t *)malloc((unsigned int)len);
    memset(tx_data, 0xFF, len);
  
		while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, len, 10);
  
    free(tx_data);
}

static void wizchip_write_buf(uint8_t* tx_data, datasize_t len)
{
		uint8_t *rx_data;
  
    rx_data = (uint8_t *)malloc((unsigned int)len);  
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, len, 10);
  
    free(rx_data);
}

#else

static void wizchip_read_burst(uint8_t *pBuf, uint16_t len)
{
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY &&
		  HAL_DMA_GetState(hspi1.hdmarx) != HAL_DMA_STATE_READY && HAL_DMA_GetState(hspi1.hdmatx) != HAL_DMA_STATE_READY);

	HAL_SPI_Receive_DMA(&hspi1, pBuf, len);

	while (HAL_DMA_GetState(hspi1.hdmarx) == HAL_DMA_STATE_BUSY|| HAL_DMA_GetState(hspi1.hdmarx) == HAL_DMA_STATE_RESET);
	while (HAL_DMA_GetState(hspi1.hdmatx) == HAL_DMA_STATE_BUSY|| HAL_DMA_GetState(hspi1.hdmatx) == HAL_DMA_STATE_RESET);
	return;
}

static void wizchip_write_burst(uint8_t *pBuf, uint16_t len)
{
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY &&
		  HAL_DMA_GetState(hspi1.hdmarx) != HAL_DMA_STATE_READY && HAL_DMA_GetState(hspi1.hdmatx) != HAL_DMA_STATE_READY);

	HAL_SPI_Transmit_DMA(&hspi1, pBuf, len);

	while (HAL_DMA_GetState(hspi1.hdmarx) == HAL_DMA_STATE_BUSY|| HAL_DMA_GetState(hspi1.hdmarx) == HAL_DMA_STATE_RESET);
	while (HAL_DMA_GetState(hspi1.hdmatx) == HAL_DMA_STATE_BUSY|| HAL_DMA_GetState(hspi1.hdmatx) == HAL_DMA_STATE_RESET);

	return;
}
#endif

static void wizchip_critical_section_lock(void)
{
    __disable_irq();
}

static void wizchip_critical_section_unlock(void)
{
    __enable_irq();
}

void wizchip_spi_initialize(void)
{

}

void wizchip_cris_initialize(void)
{
	reg_wizchip_cris_cbfunc(wizchip_critical_section_lock, wizchip_critical_section_unlock);
}

void wizchip_initialize(void)
{
    /* Deselect the FLASH : chip select high */
    wizchip_deselect();

    /* CS function register */
    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);

    /* SPI function register */
#ifdef USE_SPI_DMA
    reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write, wizchip_read_burst, wizchip_write_burst);
#else
    reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write, wizchip_read_buf, wizchip_write_buf);
#endif
    /* W5x00 initialize */
    uint8_t temp;
  
#if (_WIZCHIP_ == W5100S)
    uint8_t memsize[2][4] = {{2, 2, 2, 2}, {2, 2, 2, 2}};
#elif (_WIZCHIP_ == W5500)
    uint8_t memsize[2][8] = {{2, 2, 2, 2, 2, 2, 2, 2}, {2, 2, 2, 2, 2, 2, 2, 2}};
#elif (_WIZCHIP_ == W6100)
    uint8_t memsize[2][8] = {{2, 2, 2, 2, 2, 2, 2, 2}, {2, 2, 2, 2, 2, 2, 2, 2}};
#endif
  
    if (ctlwizchip(CW_INIT_WIZCHIP, (void *)memsize) == -1)
    {
        printf(" W5x00 initialized fail\n");

        return;
    }
#if 0
    /* Check PHY link status */
    do
    {
        if (ctlwizchip(CW_GET_PHYLINK, (void *)&temp) == -1)
        {
            printf(" Unknown PHY link status\n");

            return;
        }
    } while (temp == PHY_LINK_OFF);
#endif
}

void wizchip_check(void)
{
#if (_WIZCHIP_ == W5100S)
    /* Read version register */
    if (getVER() != 0x51)
    {
        printf(" ACCESS ERR : VERSION != 0x51, read value = 0x%02x\n", getVER());

        while (1)
            ;
    }
#elif (_WIZCHIP_ == W5500)
    /* Read version register */
    if (getVERSIONR() != 0x04)
    {
        printf(" ACCESS ERR : VERSION != 0x04, read value = 0x%02x\n", getVERSIONR());

        while (1)
            ;
    }
#elif (_WIZCHIP_ == W6100)
    uint16_t cidr = getCIDR();
    /* Read version register */
    printf("cidr value = 0x%02x\n", cidr);
    if (cidr != 0x6100)
    {
        printf(" ACCESS ERR : VERSION != 0x6100, read value = 0x%02x\n", cidr);
        while (1)
            ;
    }
#endif
}

/* Network */
void network_initialize(wiz_NetInfo net_info)
{
    uint8_t syslock = SYS_NET_LOCK;
    ctlwizchip(CW_SYS_UNLOCK, &syslock);
    ctlnetwork(CN_SET_NETINFO, (void *)&net_info);
}

void print_network_information(wiz_NetInfo net_info)
{
    uint8_t tmp_str[8] = {
        0,
    };

    ctlnetwork(CN_GET_NETINFO, (void *)&net_info);
    ctlwizchip(CW_GET_ID, (void *)tmp_str);

    printf("==========================================================\n");
    printf(" %s network configuration\n\n", (char *)tmp_str);

    printf(" MAC         : %02X:%02X:%02X:%02X:%02X:%02X\n", net_info.mac[0], net_info.mac[1], net_info.mac[2], net_info.mac[3], net_info.mac[4], net_info.mac[5]);
    printf(" IP          : %d.%d.%d.%d\n", net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3]);
    printf(" Subnet Mask : %d.%d.%d.%d\n", net_info.sn[0], net_info.sn[1], net_info.sn[2], net_info.sn[3]);
    printf(" Gateway     : %d.%d.%d.%d\n", net_info.gw[0], net_info.gw[1], net_info.gw[2], net_info.gw[3]);
    printf(" DNS         : %d.%d.%d.%d\n", net_info.dns[0], net_info.dns[1], net_info.dns[2], net_info.dns[3]);
    print_ipv6_addr(" GW6 ", net_info.gw6);
	print_ipv6_addr(" LLA ", net_info.lla);
	print_ipv6_addr(" GUA ", net_info.gua);
	print_ipv6_addr(" SUB6", net_info.sn6);
    print_ipv6_addr(" DNS6", net_info.dns6);
    printf("==========================================================\n\n");
}

void print_ipv6_addr(uint8_t* name, uint8_t* ip6addr)
{
	printf("%s        : ", name);
	printf("%04X:%04X", ((uint16_t)ip6addr[0] << 8) | ((uint16_t)ip6addr[1]), ((uint16_t)ip6addr[2] << 8) | ((uint16_t)ip6addr[3]));
	printf(":%04X:%04X", ((uint16_t)ip6addr[4] << 8) | ((uint16_t)ip6addr[5]), ((uint16_t)ip6addr[6] << 8) | ((uint16_t)ip6addr[7]));
	printf(":%04X:%04X", ((uint16_t)ip6addr[8] << 8) | ((uint16_t)ip6addr[9]), ((uint16_t)ip6addr[10] << 8) | ((uint16_t)ip6addr[11]));
	printf(":%04X:%04X\r\n", ((uint16_t)ip6addr[12] << 8) | ((uint16_t)ip6addr[13]), ((uint16_t)ip6addr[14] << 8) | ((uint16_t)ip6addr[15]));
}
