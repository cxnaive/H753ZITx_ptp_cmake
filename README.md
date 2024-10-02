# 1EEE1588v2 PTP Example for STM32H753ZITx

PTP Demo for STM32H753 (test on board NUCLEO-H753ZI)

## Features
thanks to: 
- https://github.com/chris1seto/Stm32H723NucleoPtpExample
- https://github.com/hasseb/stm32h7_atsame70_ptpd
- https://github.com/stm32-hotspot/STM32H7-LwIP-Examples

Using STM32CubeMX + LWIP to generate this CMAKE-Based project.

Messages will output to the on board ST-LINK's VCP Serial Port

## Performance
![image](https://github.com/cxnaive/H753ZITx_ptp_cmake/blob/main/doc/slave_demo.png)

average accuracy in ~50ns (offset less than 100ns)

## TODO
- use ETH DMA to get/set timestamp (LWIP_PTP)