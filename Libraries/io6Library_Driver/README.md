# io6Library

---

![](https://raw.githubusercontent.com/Wiznet/io6Library/master/Doxygen/img/io6Library.png)

---

[Goto Korean Version](https://midnightcow.tistory.com/141)

---

## Overview


io6Library is an IPv6 integrated Library that can easily integrate and manage user applications that use the [WIZnet](http://www.wiznet.io/) Hardwired Dual TCP/IP Stack Controller (<b> WIZCHIP </b>) product family.

io6Library is implemented to manage the code dependent on the user's specific MCU so that the user does not have to perform the porting operation of the io6Library according to the user MCU. (See <b>How to Use</b> for more information)
## Contents

io6Library can be classified into three types as follows.

 - Reigsters Defintion
   - Common Registers
     : Defines general registers such as network information, mode, interrupt, and so on.
   - Socket Registers
     : Define SOCKET regsiters such as socket mode, socket communication, socket interrupt, and so on.

 - Each WIZCHIP I/O Access function
   - Basic I/O function 
     : Basic unit function to access Input/Output through HOST interface (SPI, BUS, etc.) defined by WIZCHIP
   - Common Register Access Functions
     : Function to access <b>Common Register</b> based on <b>Basic I/O Function</b>
   - SOCKET Register Access Functions
     : Function to Access <b>SOCKET Register</b> based on <b>Basic I / O Function</b>

 - WIZCHIP control APIs for user application integration, management, and migration
   - SOCKET APIs
     : Like as BSD SOCKET API, SOCKET APIs provide function set that can be related socket commnuincation
   - Extra APIs
     : It provides functions to support the integration of user applications regardless of WIZCHIP specific Regiter/ Memory, Address Map, Features and so on.
     : It can be replaced with <b>WIZCHIP I / O Access funcions</b> for the small footprint of the User Application.

For more details, refer to <b>[io6Library.chm](https://github.com/Wiznet/io6Library/blob/master/Doxygen/doc/io6Library.chm)</b>.

io6Library.chm may not be up to date, so please refer the document made by [doxygen program](http://www.doxygen.nl/download.html) with <b> Doxyfile.dox </b> project. Doxygen program can made the document to <b>chm</b>, <b>html</b>, or <b>pdf</b> if you want to.

## Directory

 ![](https://raw.githubusercontent.com/midnightcow/MyDocument/master/Images/W6100/io6Library_structure.png)

ioLibrary has the same directory structure as the above figure, and the major directory is as follows.

 - [Ethernet](https://github.com/Wiznet/io6Library/tree/master/Ethernet)
   - WIZCHIP specific Directory (EX> [W6100](https://github.com/Wiznet/io6Library/tree/master/Ethernet/W6100) - w6100.h, w6100.c)
   - SOCKET API : [socket.h](https://github.com/Wiznet/io6Library/blob/master/Ethernet/socket.h), [socket.c](https://github.com/Wiznet/io6Library/blob/master/Ethernet/socket.c)
   - ioLibrary Configruation files : [wizchip_conf.h](https://github.com/Wiznet/io6Library/blob/master/Ethernet/wizchip_conf.h), [wizchip_conf.c](https://github.com/Wiznet/io6Library/blob/master/Ethernet/wizchip_conf.c)

 - [Internet](https://github.com/Wiznet/io6Library/tree/master/Internet)
   - Protcols for IP configuration (EX> DHCP, DNS)
   - Some protocols will be added

 - [Application](https://github.com/Wiznet/io6Library/tree/master/Application)
   - Application Socket Mode Definition : [Application.h](https://github.com/Wiznet/io6Library/blob/master/Application/Application.h)
   - [Loopback](https://github.com/Wiznet/io6Library/tree/master/Application/loopback) : TCP, UDP Basic Skeleton Code, [loopback.h](https://github.com/Wiznet/io6Library/blob/master/Application/loopback/loopback.h), [loopback.c](https://github.com/Wiznet/io6Library/blob/master/Application/loopback/loopback.h)

io6Library users will be able to use it immediately by modifying only a few defintion in <b>wizchip_conf.h</b>.
For more information, see <b>How to Use</b>.

## How to use

  ### io6Library Configuration

  Define the type and interface of WIZCHIP defined in wizchip_conf.h, to suit your intended usage.

  - Select the Hardwired Dual TCP/IP Stack Controller to use.
    In the following figure, Select one of list in the blue box and define the selected it to \_WIZCHIP\_ like as Red Box.

    ![](https://raw.githubusercontent.com/midnightcow/MyDocument/master/Images/W6100/def_wizchip.png)

  - Select the HOST Interface (Parallel Bus, Serial Bus Mode, etc.) that the user will use for WIZCHIP Access.
    In the following figure, Select one of list in the blue box and define the selected it to \_WIZCHIP\_IO\_MODE\_ like as Red Box.

    ![](https://raw.githubusercontent.com/midnightcow/MyDocument/master/Images/W6100/def_host_if.png)

    - <b>Only when Parallel Bus Mode</b> is used, <b>Must</b> set the memory bank base address of HOST to be assigned to WIZCHIPCHIP like as red box.

      ![](https://raw.githubusercontent.com/midnightcow/MyDocument/master/Images/W6100/def_bus_map.png)

  - WIZCHIP PHY Access Mode Configuration
    Like as the following figure, Select one of the two approaches of Ethernet PHY access mode defined in blue box and    define it as Red box.

    ![](https://github.com/midnightcow/MyDocument/blob/master/Images/W6100/def_wizphy_if.png)

    - \_PHY\_IO\_MODE\_PHYCR\_ : It provides simple control to Ethernet PHY of WIZCHIP thru PHY Command & Status Register such like as PHY Operation Mode and Link Status.
    - \_PHY\_\_IO\_MODE\_MII\_ : It provides direct control to Ethernet PHY Register of WIZCHIP PHY thru MDC/MDIO signal.


  ### Make user-defined functions for WIZCHIP I/O Access

  Make the basic Access I/O function by yourself according to your HOST interface.
  This is because the interface control method differs for each user HOST. So, You should make it.

  For example, if you define the following and control WIZCHIP using SPI1 of STM32FXXX

  ```c
  #define _WIZCHIP_IO_MODE_       _WIZCHIP_IO_MODE_SPI_VDM_
  ```

  Create basic unit functions such as WIZCHIP select / deselect, 1byte read / write, critical section enter / exit etc. via SPI interface as follows.
  
  Make your basic I/O access functions such as WIZCHIP select/deselect, 1byte read/write and critical section enter/exit through the SPI interface as shown belows.

  - WIZCHIP select/deselect : Function to set/reset any GPIO of STM32FXXX connected with CSn Pin of WIZCHIP

  ```c
  void your_wizchip_enable(void)
  {
     /* void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) */
     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET)
  }

  void your_wizchip_disable(void)
  {
     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET)
  }
  ```

  - WIZCHIP 1 byte read/write : Function to read and write 1 byte through SPI interface

  ```c
  /* Read 1 byte thru SPI */
  uint8_t your_spi_read_byte()
  {
     uint8_t ret;
     /*HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)*/
     HAL_SPI_Receive(SPI1, &ret, 1, 1000);
     return ret;
  }

  /* Write 1 byte thru SPI */
  void your_spi_wite_byte(uint8_t wd)
  {
    /* HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout); */
    HAL_SPI_Transmit(SPI1, &wd, 1, 1000);
  }
  ```

  - WIZCHIP Critical Section Enter/Exit : Function to protect against events such as interupts and task switching while WIZCHIP is accessing

  ```c
  void your_critical_enter(void)
  {
    __disable_irq();
  }

  void your_critical_exit(void)
  {
    __enable_irq();
  }
  ```

  - WIZCHIP N bytes read/write Function
    : It is not mandatory.
    : But, If you want to use WIZCHIP for high speed access using a peripheral such as DMA, make it as follows.
    : Even if you do not, you can use N bytes read/write accesses because it is performed by repeating your 1-byte read/write function.

   ```c
   void your_spi_dma_write_buf(uint8_t* pbuf, iodata_t len)
   {
     /* HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size) */
     HAL_SPI_Transmit_DMA(SPI1, pbuf, (uint16_t)len);
   }

   void your_spi_dma_read_buf(uint8_t* pbuf, iodata_t len)
   {
     /* HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size) */
     HAL_SPI_Receive_DMA(SPI1, pbuf, (uint16_t)len);
   }
   ```

### Register your defined Function to io6Library

   Register the your defined code made in the above examples before WIZCHIP initialization as below.

   ```c
   #include "wizchip_conf.h"  // Just only included for using ioLibrary

   void main(void)
   {
      uint16_t chip_id;
      wiz_NetTimeout tmp_timeout = 0;
      /*
      // Intialize your Target System such as SPI, UART, DMA, and etc.
      */

      /* Register your basic access function to io6Library */

      //WIZCHIP Enable/Disable
      reg_wizchip_cs_cbfunc(your_wizchip_enable, your_wizchip_disable);

      //WIZCHIP Critical Section
      reg_wizchip_cris_cbfunc(your_critical_enter, your_critical_exit);

      //WIZCHIP read/write function
      reg_wizchip_spi_cbfunc(your_spi_read_byte, your_spi_write_byte, 0, 0);

      // If you made DMA function for readign/writting function, you can register as following
      // reg_wizchip_spi_cbfunc(your_spi_read_byte, your_spi_write_byte, your_spi_dma_read_buf, your_spi_dma_write_buf);
      //

      /* For io6Library Read/Write Test */

      // Read Test
      ctlwizchip(CW_GET_ID,&chip_id); // Check WIZCHIP ID value for read test
      if(chip_id != 0x6100) printf("ERROR : HOST I/F READ\r\n") // It is just example for W6100 

      // Write Test
      ctlwizchip(CW_PHY_RESET, 0);  // Check phyically PHY Link Led to turn off and then on.

      /////////////////////////////////
      // Enjoy, WIZCHIP & io6Library //
      // Thank you                   //
      /////////////////////////////////
   }
   ```

## Examples for io6Library & [ioLibrary](https://github.com/Wiznet/ioLibrary_Driver)

You can refer many examples like as http, ftp, and so on related to io6Library & ioLibrary.

[WIZnet-ioLibrary Examples](https://github.com/Wiznet-ioLibrary)

## Available WIZCHIP sets

<table>
 <tr> <th>Product Name</th><th>Figures</th> </tr>
 <tr> <td>W6100-L(Q)</td> <td> <img src=https://raw.githubusercontent.com/Wiznet/io6Library/master/Doxygen/img/w6100.png width=320></td></tr>
 </table>

## Reference Site

  - WIZnet Homepage : https://www.wiznet.io/
  - WIZCHIP Set : https://www.wiznet.io/product/tcpip-chip/
  - WIZCHIP Modules : https://www.wiznet.io/product/network-module/
  - Technical Documents : [WiKi](http://wizwiki.net/), [WIZCHIP](http://wizwiki.net/https://wizwiki.net/wiki/doku.php?id=products:iethernet:start)
  - Q&A, Forum : https://forum.wiznet.io/

  - Simple IPv6 intrduction : https://maker.wiznet.io/wp-content/uploads/2019/04/WoW_part1.pdf
  - Simple W6100 Description : https://maker.wiznet.io/wp-content/uploads/2019/04/WoW_part2.pdf
