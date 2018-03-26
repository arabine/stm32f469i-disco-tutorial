
LOCAL_DIR := $(call my-dir)

INCLUDES += $(LOCAL_DIR)/Inc

SOURCES += $(addprefix $(LOCAL_DIR)/Src/, stm32f4xx_hal_dma_ex.c \
stm32f4xx_hal_flash.c \
stm32f4xx_hal_sdram.c \
stm32f4xx_hal_rcc_ex.c \
stm32f4xx_hal_rcc.c \
stm32f4xx_hal_dsi.c \
stm32f4xx_hal_qspi.c \
stm32f4xx_hal_tim_ex.c \
stm32f4xx_hal_pwr.c \
stm32f4xx_hal_dma2d.c \
stm32f4xx_hal_sd.c \
stm32f4xx_hal_flash_ex.c \
stm32f4xx_hal_sai_ex.c \
stm32f4xx_hal_crc.c \
stm32f4xx_hal_i2c.c \
stm32f4xx_hal_dma.c \
stm32f4xx_hal_hcd.c \
stm32f4xx_hal_flash_ramfunc.c \
stm32f4xx_ll_usb.c \
stm32f4xx_hal.c \
stm32f4xx_ll_sdmmc.c \
stm32f4xx_hal_i2c_ex.c \
stm32f4xx_hal_tim.c \
stm32f4xx_hal_ltdc_ex.c \
stm32f4xx_hal_gpio.c \
stm32f4xx_hal_ltdc.c \
stm32f4xx_hal_cortex.c \
stm32f4xx_ll_fmc.c \
stm32f4xx_hal_sai.c \
stm32f4xx_hal_pwr_ex.c \
stm32f4xx_hal_uart.c) 
