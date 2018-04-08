 # *******************************************************************************
# Main makefile project
# This makefile calls all the modules defined in the config.mk file
# *******************************************************************************

# *******************************************************************************
# DEFAULT DEFINITIONS
# These definitions can be overloaded from the command line
# *******************************************************************************
PROJECT		?= $(MAKECMDGOALS)
TARGET 		?= release
BOARD 		?= host
OPTIM 		?= high
ENABLE_DEP 	?= true
ARCH		?= cm4-freertos-gcc

OUTDIR		:= $(TOPDIR)build/output/


# Export them to be sure that they are available in sub-makefiles
export PROJECT
export TARGET
export BOARD
export DEFINES
export OPTIM
export ENABLE_DEP
export ARCH
export OUTDIR

# *******************************************************************************
# APPLICATION DEFINITIONS
# List of modules and extra libraries needed to generate project targets
# *******************************************************************************
LIB_UTILITIES		:= Utilities/Fonts
LIB_STM32F4XX_HAL	:= Drivers/STM32F4xx_HAL_Driver
LIB_469I_DISCO_BSP 	:= Drivers/BSP
LIB_CMSIS 			:= Drivers/CMSIS
LIB_DRIVERS			:= Drivers/Components/ft6x06 Drivers/Components/otm8009a
LIB_LCD				:= Drivers/liblcd
LIB_FREERTOS		:= Middlewares/Third_Party/FreeRTOS
LIB_FATFS			:= Middlewares/Third_Party/FatFs
LIB_USB_HOST		:= Middlewares/ST/STM32_USB_Host_Library

LIB_ALL				:= $(LIB_STM32F4XX_HAL) $(LIB_469I_DISCO_BSP) $(LIB_CMSIS) $(LIB_FREERTOS) $(LIB_DRIVERS) $(LIB_FATFS) $(LIB_USB_HOST)

# *******************************************************************************
# SERVER CONFIGURATION
# *******************************************************************************
ifeq ($(PROJECT), blinky)

DEFINES += -DUSE_HAL_DRIVER -DSTM32F469xx
GCC_PREFIX := arm-none-eabi-

APP_MODULES 	:= projects/blinky $(LIB_ALL)
APP_LIBPATH 	:= 
APP_LIBS 		:= 
APP_LDSCRIPT	:= STM32F469NIHx_FLASH.ld
endif

# *******************************************************************************
# CLIENT CONFIGURATION
# *******************************************************************************
ifeq ($(PROJECT), lcd)

DEFINES += -DUSE_HAL_DRIVER -DSTM32F469xx -DTS_MULTI_TOUCH_SUPPORTED=1
GCC_PREFIX := arm-none-eabi-

APP_MODULES 	:= projects/lcd $(LIB_ALL) $(LIB_LCD)
APP_LIBPATH 	:= 
APP_LIBS 		:= 
APP_LDSCRIPT	:= STM32F469NIHx_FLASH.ld
endif

# *******************************************************************************
# BUILD ENGINE
# *******************************************************************************
include build/Main.mk

blinky: $(OBJECTS)
	$(call linker, $(OBJECTS), $(APP_LIBS), blinky)

lcd: $(OBJECTS)
	$(call linker, $(OBJECTS), $(APP_LIBS), lcd_logo)

flash:
	st-flash --serial 303636424646333533353335344433 --format ihex write $(OUTDIR)$(PROJECT).hex

clean:
	@echo "Cleaning generated files..."
	$(VERBOSE) $(RM) -rf $(OBJECTS) $(OUTDIR)*.d $(OUTDIR)*.gcov $(OUTDIR)*.gcov.htm



# *******************************************************************************
# END OF MAKEFILE
# *******************************************************************************
