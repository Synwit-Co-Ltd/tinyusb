DEPS_SUBMODULES += hw/mcu/synwit

CFLAGS += \
  -mthumb \
  -mabi=aapcs \
  -mcpu=cortex-m33 \
  -mfloat-abi=soft \
  -nostdlib -nostartfiles \
  -DCFG_TUSB_MCU=OPT_MCU_SWM341

# suppress warning caused by vendor mcu driver
CFLAGS += -Wno-error=unused-parameter -Wno-error=cast-align -Wno-error=cast-qual -Wno-error=sign-compare -Wno-error=aggressive-loop-optimizations

# All source paths should be relative to the top level.
LD_FILE = hw/bsp/$(BOARD)/swm341.ld

SRC_C += \
  src/portable/synwit/swm341/dcd_swm341.c \
  src/portable/synwit/swm341/hcd_swm341.c \
  hw/mcu/synwit/SWM341_Lib/CMSIS/DeviceSupport/system_SWM341.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_adc.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_can.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_cordic.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_crc.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_dac.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_div.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_dma.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_dma2d.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_exti.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_flash.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_gpio.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_i2c.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_jpeg.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_lcd.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_port.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_pwm.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_rtc.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_sdio.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_sdram.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_sfc.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_sleep.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_spi.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_timr.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_uart.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_usbd.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_usbh.c \
  hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver/SWM341_wdt.c

SRC_S += \
  hw/mcu/synwit/SWM341_Lib/CMSIS/DeviceSupport/startup/gcc/startup_SWM341.s

INC += \
  $(TOP)/hw/mcu/synwit/SWM341_Lib/CMSIS/CoreSupport \
  $(TOP)/hw/mcu/synwit/SWM341_Lib/CMSIS/DeviceSupport \
  $(TOP)/hw/mcu/synwit/SWM341_Lib/SWM341_StdPeriph_Driver
