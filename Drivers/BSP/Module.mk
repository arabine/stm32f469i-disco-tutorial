
LOCAL_DIR = $(call my-dir)/

SOURCES += $(addprefix $(LOCAL_DIR), stm32469i_discovery.c \
stm32469i_discovery_eeprom.c \
stm32469i_discovery_lcd.c \
stm32469i_discovery_sdram.c \
stm32469i_discovery_ts.c)


SOURCES += $(addprefix $(LOCAL_DIR), stm32f4xx_hal_msp.c \
stm32f4xx_it.c \
stm32f4xx_hal_timebase_TIM.c \
usart.c \
gpio.c \
freertos.c \
fatfs.c \
fmc.c \
usbh_platform.c \
dma2d.c \
usbh_conf.c \
quadspi.c \
i2c.c \
sd_diskio.c \
crc.c \
usb_host.c \
fatfs_platform.c \
sai.c \
tim.c \
system_stm32f4xx.c \
sdio.c \
ltdc.c \
dsihost.c \
bsp_driver_sd.c \
startup_stm32f469xx.s)

