/**
  ******************************************************************************
  * @file    LCD_Paint/Src/main.c
  * @author  MCD Application Team
  * @brief   This file provides main program functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "color.h"
#include "save.h"

/* Includes ------------------------------------------------------------------*/
#include "stm32469i_discovery.h"
#include "stm32469i_discovery_lcd.h"
#include "stm32469i_discovery_ts.h"
#include "stm32469i_discovery_sdram.h"
#include "stm32469i_discovery_sd.h"
#include <stdlib.h>
#include <stdio.h>

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Alias for LCD_FB_START_ADDRESS : frame buffer displayed on LCD */
#define LCD_FRAME_BUFFER              LCD_FB_START_ADDRESS

#define WVGA_RES_X                    800
#define WVGA_RES_Y                    480

#define LCD_SCREEN_WIDTH              WVGA_RES_X
#define LCD_SCREEN_HEIGHT             WVGA_RES_Y

#define ARGB8888_BYTE_PER_PIXEL       4
#define RGB888_BYTE_PER_PIXEL         3

/* Buffer LCD Converted to RGB888 in SDRAM in order to be copied to SD Card   */
/* Starts directly at end of LCD frame buffer                                 */
#define CONVERTED_FRAME_BUFFER        LCD_FRAME_BUFFER + (LCD_SCREEN_WIDTH * LCD_SCREEN_HEIGHT * ARGB8888_BYTE_PER_PIXEL)

/* Imported globals ----------------------------------------------------------*/
extern TS_StateTypeDef  TS_State;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t  Touchscreen_Calibration(void);
uint16_t TouchScreen_Get_Calibrated_X(uint16_t x);
uint16_t TouchScreen_Get_Calibrated_Y(uint16_t y);
uint8_t  TouchScreen_IsCalibrationDone(void);

extern LTDC_HandleTypeDef  hltdc_eval;

/* Private typedef -----------------------------------------------------------*/

typedef struct
{
  uint16_t bfType;  /* specifies the file type */
  uint32_t bfSize;  /* specifies the size in bytes of the bitmap file */
  uint16_t bfReserved1;  /* reserved : must be 0 */
  uint16_t bfReserved2;  /* reserved : must be 0 */
  uint32_t bOffBits;  /* species the offset in bytes from the bitmapfileheader to the bitmap bits */
  uint16_t Padding;   /* padding to multiple of 32 bits */
} BitMapFileHeader_Typedef;

typedef struct
{
  uint32_t biSize;  /* specifies the number of bytes required by the struct */
  uint32_t biWidth;  /* specifies width in pixels */
  uint32_t biHeight;  /* species height in pixels */
  uint16_t biPlanes; /* specifies the number of color planes, must be 1 */
  uint16_t biBitCount; /* specifies the number of bit per pixel */
  uint32_t biCompression; /* specifies the type of compression */
  uint32_t biSizeImage;  /* size of image in bytes */
  uint32_t biXPelsPerMeter;  /* number of pixels per meter in x axis */
  uint32_t biYPelsPerMeter;  /* number of pixels per meter in y axis */
  uint32_t biClrUsed;  /* number of colors used by the bitmap */
  uint32_t biClrImportant;  /* number of colors that are important */
} BitMapFileInfoHeader_Typedef;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
FATFS SDFatFs;   /* File system object for SD card logical drive */
FIL MyFile;      /* File object */
char SDPath[4];  /* SD card logical drive path */
static uint8_t buffer[_MAX_SS]; /* a work buffer for the f_mkfs() */

/* BMP file information to save the drawing pad to file BMP in RGB888 format */
static BitMapFileHeader_Typedef     bmpFileHeader;
static BitMapFileInfoHeader_Typedef bmpFileInfoHeader;

static uint8_t *  p_bmp_converted_pixel_data = (uint8_t *)CONVERTED_FRAME_BUFFER;

static uint32_t Radius = 10;
TS_StateTypeDef  TS_State = {0};

/* Private function prototypes -----------------------------------------------*/
static void Draw_Menu(void);
static void GetPosition(void);
static void Save_Picture(void);
static void LTDC_Operation(uint32_t Enable_LTDC);
static void Prepare_Picture(void);
static void Update_Color(void);
static void Update_Size(uint8_t size);



/* Imported variables */
extern TS_StateTypeDef TS_State;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t ts_calibration_done = 0;
static int16_t  A1, A2, B1, B2;
static int16_t aPhysX[2], aPhysY[2], aLogX[2], aLogY[2];
/* Private function prototypes -----------------------------------------------*/
static void TouchscreenCalibration_SetHint(void);
static void TouchScreen_Calibration_GetPhysValues(int16_t LogX, int16_t LogY, int16_t * pPhysX, int16_t * pPhysY) ;
static void TouchScreen_Calibration_WaitForPressedState(uint8_t Pressed) ;
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Performs the TS calibration
  * @param  None
  * @retval Status (TS_OK = 0/ TS_ERROR = 1 / TS_TIMEOUT = 1 / TS_DEVICE_NOT_FOUND = 3)
  */
uint8_t Touchscreen_Calibration(void)
{
  uint8_t ts_status = TS_OK;
  uint8_t i;
  uint16_t ts_SizeX;
  uint16_t ts_SizeY;

  ts_SizeX = BSP_LCD_GetXSize();
  ts_SizeY = BSP_LCD_GetYSize();

  TouchscreenCalibration_SetHint();

  /* Start touchscreen internal calibration and configuration + start */
  ts_status = BSP_TS_Init(ts_SizeX, ts_SizeY);
  if (ts_status != TS_OK)
  {
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 95, (uint8_t *)"ERROR", CENTER_MODE);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 80, (uint8_t *)"Touchscreen cannot be calibrated", CENTER_MODE);
    if(ts_status == TS_ERROR)
    {
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 65, (uint8_t *)"Touchscreen undefined error", CENTER_MODE);
    }
    else if(ts_status == TS_TIMEOUT)
    {
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 65, (uint8_t *)"Touchscreen Timeout", CENTER_MODE);
    }
    else
    {
      /* TS_DEVICE_NOT_FOUND */
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 65, (uint8_t *)"Touchscreen Not Found", CENTER_MODE);
    }
  }
  else
  {
    /* status == TS_OK */
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 65, (uint8_t *)"FT6x06 internal calibration passed", CENTER_MODE);

    /* Get touch points for SW calibration processing */
    aLogX[0] = 40;
    aLogY[0] = 40;
    aLogX[1] = BSP_LCD_GetXSize() - 40;
    aLogY[1] = BSP_LCD_GetYSize() - 40;

    for (i = 0; i < 2; i++)
    {
      TouchScreen_Calibration_GetPhysValues(aLogX[i], aLogY[i], &aPhysX[i], &aPhysY[i]);
    }

    /* Compute calibration coefficients */
    A1 = (1000 * ( aLogX[1] - aLogX[0])) / ( aPhysX[1] - aPhysX[0]);
    B1 = (1000 * aLogX[0]) - A1 * aPhysX[0];

    A2 = (1000 * ( aLogY[1] - aLogY[0])) / ( aPhysY[1] - aPhysY[0]);
    B2 = (1000 * aLogY[0]) - A2 * aPhysY[0];

    ts_calibration_done = 1;
  }

  return (ts_status);
}

/**
  * @brief  Display calibration hint
  * @param  None
  * @retval None
  */
static void TouchscreenCalibration_SetHint(void)
{
  /* Clear the LCD */
  BSP_LCD_Clear(LCD_COLOR_WHITE);

  /* Set Touchscreen Demo description */
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);

  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 35, (uint8_t *)"Before using the Touchscreen", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 15, (uint8_t *)"you need to calibrate it.", CENTER_MODE);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 7, (uint8_t *)"WAIT until the black circle appears", CENTER_MODE);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 30, (uint8_t *)"THEN Press precisly on the black circles", CENTER_MODE);
  BSP_LCD_SetFont(&Font12);
}


/**
  * @brief  Get Physical position
  * @param  LogX : logical X position
  * @param  LogY : logical Y position
  * @param  pPhysX : Physical X position
  * @param  pPhysY : Physical Y position
  * @retval None
  */
static void TouchScreen_Calibration_GetPhysValues(int16_t LogX, int16_t LogY, int16_t * pPhysX, int16_t * pPhysY)
{
  /* Draw the ring */

  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_FillCircle(LogX, LogY, 20);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillCircle(LogX, LogY, 10);

  /* Wait until pressed state on the touch panel */
  TouchScreen_Calibration_WaitForPressedState(1);

  /* Return as physical touch values the positions of first touch, even if double touched occurred */
  *pPhysX = TS_State.touchX[0];
  *pPhysY = TS_State.touchY[0];

  /* Wait until touch is released on touch panel */
  TouchScreen_Calibration_WaitForPressedState(0);

  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillCircle(LogX, LogY, 20);
}

/**
  * @brief  TouchScreen_Calibration_WaitForPressedState : wait until a particular press/depress action
  *         The function is managing anti-rebound : that is the awaited state when detected
  *         needs to be stable for a sufficient time (timeout time), otherwise a new sense to search
  *         for awaited state is performed. When awaited state is found and state is stable for timeout
  *         duration, the function is exited.
  * @param  uint8_t Pressed :  Awaited pressed state
  *         - Await touch (single/multiple) detection if Pressed == 1
  *         - Await no touch detection if Pressed == 0
  * @retval None
  */
static void TouchScreen_Calibration_WaitForPressedState(uint8_t Pressed)
{
  uint16_t TimeStart = 0;
  uint8_t  status = TS_OK;
  uint32_t exitFirstLevelWhileLoopReq = 0;  /* By default no exit request from first level while loop  */
  uint32_t exitSecondLevelWhileLoopReq = 0; /* By default no exit request from second level while loop */

  /* First level while loop entry */
  do
  {
    /* reset exit second level while loop in case it was set */
    exitSecondLevelWhileLoopReq = 0;

    /* Sense of touch state from touch IC until get the awaited state in parameter 'Pressed' */
    status = BSP_TS_GetState(&TS_State);
    if(status == TS_OK)
    {
      if (((Pressed == 0) && (TS_State.touchDetected == 0)) ||
          ((Pressed == 1) && ((TS_State.touchDetected == 1) || (TS_State.touchDetected == 2))))
      {
        /* Got awaited press state */
        /* Record in 'TimeStart' the time of awaited touch event for anti-rebound calculation */
        /* The state should persist for a minimum sufficient time */
        TimeStart = HAL_GetTick();

        /* Is state of the touch changing ? */
        /* Second level while loop entry */
        do
        {
          /* New sense of touch state from touch IC : to evaluate if state was stable */
          status = BSP_TS_GetState(&TS_State);
          if(status == TS_OK)
          {
            /* Is there a state change compared since having found the awaited state ? */
            if (((Pressed == 0) && ((TS_State.touchDetected == 1) || (TS_State.touchDetected == 2))) ||
                ((Pressed == 1) && ((TS_State.touchDetected == 0))))
            {
              /* Too rapid state change => anti-rebound management : restart first touch search */
              exitSecondLevelWhileLoopReq = 1; /* exit request from second level while loop */
            }
            else if ((HAL_GetTick() - 100) > TimeStart)
            {
              /* State have not changed for the timeout duration (stable touch for 100 ms) */
              /* This means the touch state is stable : can exit function */

              /* found valid touch, exit both while levels */
              exitSecondLevelWhileLoopReq = 1;
              exitFirstLevelWhileLoopReq  = 1;
            }

            /* Wait 10 ms before next sense of touch at next loop iteration */
            HAL_Delay(10);

          } /* of if(status == TS_OK) */
        }
        while (!exitSecondLevelWhileLoopReq);

      } /* of if (((Pressed == 0) && .... */

    } /* of if(status == TS_OK) */

    if(!exitFirstLevelWhileLoopReq)
    {
      /* Wait some time before next sense of touch at next loop iteration */
      HAL_Delay(10);
    }

  }
  while (!exitSecondLevelWhileLoopReq);
}

/**
  * @brief  Calibrate x position (to obtain X = calibrated(x))
  * @param  x : X position
  * @retval calibrated x
  */
uint16_t TouchScreen_Get_Calibrated_X(uint16_t x)
{
  return (((A1 * x) + B1) / 1000);
}

/**
  * @brief  Calibrate Y position
  * @param  y : Y position
  * @retval calibrated y
  */
uint16_t TouchScreen_Get_Calibrated_Y(uint16_t y)
{
  return (((A2 * y) + B2) / 1000);
}

/**check if the TS is calibrated
  * @param  None
* @retval calibration state (1 : calibrated / 0: no)
  */
uint8_t TouchScreen_IsCalibrationDone(void)
{
  return (ts_calibration_done);
}

#if 0

BSP_LCD_Init();
BSP_TS_Init(800,480);

BSP_LCD_LayerDefaultInit(0, 0xC0000000);
BSP_LCD_SelectLayer(0);
BSP_LCD_DisplayOn();
BSP_LCD_Clear(LCD_COLOR_WHITE);
BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
BSP_LCD_SetFont(myfont);


BSP_LCD_DisplayStringAt(0, 0, (uint8_t *)"Hello World", CENTER_MODE);
/* USER CODE END 2 */

/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{
/* USER CODE END WHILE */

/* USER CODE BEGIN 3 */


    for(uint32_t r=1;r<120;r+=1){

  BSP_LCD_SetTextColor(0xff000000+r*1398);
    BSP_LCD_DrawCircle(120, 160, r);
    HAL_Delay(5);
    }
    HAL_Delay(100);
    BSP_LCD_Clear(LCD_COLOR_BLACK);
}

#endif


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int example1(void)
{
  uint8_t  sdram_status = SDRAM_OK;
  uint8_t  lcd_status = LCD_OK;
  uint32_t ts_status  = TS_OK;

  p_bmp_converted_pixel_data = (uint8_t *)CONVERTED_FRAME_BUFFER;

  /* Configure LED1 and LED3 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED3);

  /*##-1- Initialize the SDRAM */
  sdram_status = BSP_SDRAM_Init();
  if(sdram_status != SDRAM_OK)
  {
    Error_Handler();
  }

  /*##-2- LCD Initialization #################################################*/
  /* Initialize the LCD DSI */
  lcd_status = BSP_LCD_Init() ;
  if(lcd_status != LCD_OK)
  {
    Error_Handler();
  }

  lcd_status = BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE);
  if(lcd_status != LCD_OK)
  {
    Error_Handler();
  }
  BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER_BACKGROUND, LCD_FB_START_ADDRESS);

  /* Clear the LCD Background layer */
  BSP_LCD_Clear(LCD_COLOR_ORANGE);


  /*##-3- Touch screen initialization ########################################*/
  BSP_TS_ResetTouchData(&TS_State);

  /* If calibration is not yet done, proceed with calibration */
  if (TouchScreen_IsCalibrationDone() == 0)
  {
    ts_status = Touchscreen_Calibration();
    if(ts_status == TS_OK)
    {
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 65, (uint8_t *)"Touchscreen calibration success.", CENTER_MODE);
    }
  } /* of if (TouchScreen_IsCalibrationDone() == 0) */



  /*##-4- Link the SD Card disk I/O driver ###################################*/

  /* Clear the LCD and display waiting message */
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 27, (uint8_t*)"Please WAIT few seconds", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 12, (uint8_t*)"Creating FAT file system on SD card", CENTER_MODE);

  if(FATFS_LinkDriver(&SD_Driver, SDPath) != 0)
  {
    /* FatFs Initialization Error */
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 3, (uint8_t*)"FAT FS Error !!", CENTER_MODE);
    Error_Handler();
  }

/*##-4- Register the file system object to the FatFs module ################*/
  if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
    {
    /* FatFs Initialization Error */
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 3, (uint8_t*)"FAT FS Error !!", CENTER_MODE);
    Error_Handler();
   }
  /* Create a FAT file system (format) on the logical drive */
  if(f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, buffer, sizeof(buffer)) != FR_OK)
  {
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 3, (uint8_t*)"FAT FS Error !!", CENTER_MODE);
    Error_Handler();
  }

  /*##-5- Draw the menu ######################################################*/
  Draw_Menu();

  /* Infinite loop */
  while (1)
  {
  /*##-6- Configure the touch screen and Get the position ####################*/
    GetPosition();
  }

}

/**
  * @brief  Configures and gets Touch screen position.
  * @param  None
  * @retval None
  */
static void GetPosition(void)
{
  static uint32_t color_width;
  static uint32_t color;
  uint16_t x, y;

  color_width = 35;

  /* Get Touch screen position */
  BSP_TS_GetState(&TS_State);
  if(TS_State.touchDetected)
  {
    /* A touch occured, read the touch coordinates */
    /* Get X and Y position of the first touch post calibrated */
    x = TouchScreen_Get_Calibrated_X(TS_State.touchX[0]);
    y = TouchScreen_Get_Calibrated_Y(TS_State.touchY[0]);

    if((x > (90 + Radius)) & (y > (7 + Radius) ) & ( x < (BSP_LCD_GetXSize() - (7 + Radius ))) & (y < (BSP_LCD_GetYSize() - (95 + Radius ))))
    {
      BSP_LCD_FillCircle((x), (y), Radius);
    }
    else if((x > 0 ) & ( x < 90 ))
    {
      if((y > 0 ) & (y < color_width ))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
        Update_Size(Radius);
      }
      else if((y > color_width ) & (y < (2 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
        Update_Size(Radius);
      }
      else if((y > (2 * color_width)) & (y < (3 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
        Update_Size(Radius);
      }
      else if((y > (3 * color_width)) & (y < (4 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_LIGHTMAGENTA);
        Update_Size(Radius);
      }
      else if((y > (4 * color_width)) & (y < (5 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
        Update_Size(Radius);
      }
      else if((y > (5 * color_width)) &(y < (6 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
        Update_Size(Radius);
      }
      else if((y > (6 * color_width)) &(y < (7 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_BROWN);
        Update_Size(Radius);
      }
      else if((y > (7 * color_width)) & (y < (8 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_RED);
        Update_Size(Radius);
      }
      else if((y > (8 * color_width)) & (y < (9 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_DARKMAGENTA);
        Update_Size(Radius);
      }
      else if((y > (9 * color_width)) & (y < (10 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
        Update_Size(Radius);
      }
      else if((y > (10 * color_width)) & (y < (11 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
        Update_Size(Radius);
      }
      else if((y > (11 * color_width)) & (y < (12 * color_width)))
      {
        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
        Update_Size(Radius);
      }
      else if((y > (12 * color_width)) & (y < (14 * color_width)))
      {
        /* Get the current text color */
        color = BSP_LCD_GetTextColor();
        BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
        /* Clear the working window */
        BSP_LCD_FillRect(97, 8, 694, 374);
        BSP_LCD_SetTextColor(color);
      }
      else
      {
        x = 0;
        y = 0;
      }
      Update_Color();
    }
    else if((x > 100) & (y > (BSP_LCD_GetYSize() - 90)) & (y < (BSP_LCD_GetYSize()) ) & ( x < 170))
    {
      Radius = 15;
      Update_Size(Radius);
    }
    else if((x > 170) & (y > (BSP_LCD_GetYSize() - 90)) & (y < (BSP_LCD_GetYSize()) ) & ( x < 240))
    {
      Radius = 10;
      Update_Size(Radius);
    }
    else if((x > 240) & (y > (BSP_LCD_GetYSize() - 90)) & (y < (BSP_LCD_GetYSize()) ) & ( x < 310))
    {
      Radius = 5;
      Update_Size(Radius);
    }
    else if(((x > (BSP_LCD_GetXSize()-5) ) & (y > (14 * color_width)) & (y < (15 * color_width))) | ((x < 90) & (y < 5)))
    {
      TS_State.touchX[0] = 0;
      TS_State.touchY[0] = 0;
    }
    else if((x > 320) & (y > (BSP_LCD_GetYSize() - 90)) & (x < 410) & (y < BSP_LCD_GetYSize() - 5))
    {
      Save_Picture();
    }
  }
}

/**
  * @brief  Draws the menu.
  * @param  None
  * @retval None
  */
static void Draw_Menu(void)
{
  /* Clear the LCD */
  BSP_LCD_Clear(LCD_COLOR_WHITE);

  /* Draw color image */
  BSP_LCD_DrawBitmap(0, 0, (uint8_t *)color);

  /* Draw save image */
  BSP_LCD_DrawBitmap(320, (BSP_LCD_GetYSize() - 90), (uint8_t *)save);

  /* Set Black as text color */
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

  /* Draw working window */
  BSP_LCD_DrawRect(91, 0, (BSP_LCD_GetXSize()-91), (BSP_LCD_GetYSize()-90));
  BSP_LCD_DrawRect(93, 3, (BSP_LCD_GetXSize()-96), (BSP_LCD_GetYSize()-96));
  BSP_LCD_DrawRect(95, 5, (BSP_LCD_GetXSize()-100), (BSP_LCD_GetYSize()-100));
  BSP_LCD_DrawRect(97, 7, (BSP_LCD_GetXSize()-104), (BSP_LCD_GetYSize()-104));

  /* Draw size icons */
  BSP_LCD_FillRect(100, (BSP_LCD_GetYSize() - 85), 210, 80);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillCircle(135, (BSP_LCD_GetYSize() - 45), 15);
  BSP_LCD_FillCircle(205, (BSP_LCD_GetYSize() - 45), 10);
  BSP_LCD_FillCircle(275, (BSP_LCD_GetYSize() - 45), 5);

  BSP_LCD_SetTextColor(LCD_COLOR_DARKRED);
  BSP_LCD_SetFont(&Font8);
  BSP_LCD_DisplayStringAt(430, (BSP_LCD_GetYSize() - 75), (uint8_t *)"Selected Color  Size", LEFT_MODE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_FillRect(450, (BSP_LCD_GetYSize() - 50), 30, 30);
  BSP_LCD_FillCircle(520, (BSP_LCD_GetYSize() - 35), Radius);
}

/**
  * @brief  Saves the picture in microSD.
  * @param  None
  * @retval None
  */
static void Save_Picture(void)
{
  FRESULT res1 = FR_OK;
  FRESULT res2 = FR_OK;
  FRESULT res3 = FR_OK;      /* FatFs function common result code */
  uint32_t byteswritten = 0;     /* File write count */
  uint32_t bmpHeaderByteCnt = 0;
  uint32_t bmpFileInfoHeaderByteCnt = 0;
  uint32_t bmpFilePixelBytesCnt = 0;
  static uint32_t counter = 0;
  uint8_t str[30];
  uint16_t tmp_size;

  /* Check if the SD card is plugged in the slot */
  if(BSP_SD_IsDetected() != SD_PRESENT)
  {
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t*)"No SD card detected !!", RIGHT_MODE);
    Error_Handler();
  }
  else
  {
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t*)"Saving BMP to SD card", RIGHT_MODE);

    /* Format the string */
    sprintf((char *)str, "image_%lu.bmp", counter);

    /* -- Prepare Bitmap file (BMP) header */

    bmpFileHeader.bfType = 0x4D42; /* BMP file type */

    /* Offset in bytes from start of file to first pixel data = size in bytes of complete BMP header          */
    /* careful the structure is padded on multiple of 32 bits by the compiler : the Padding should be removed */
    bmpFileHeader.bOffBits = sizeof(BitMapFileHeader_Typedef) +
                         sizeof(BitMapFileInfoHeader_Typedef) - sizeof(uint32_t) - sizeof(uint16_t);

    /* BMP complete file size is size of pad in RGB888 : 24bpp = 3 bytes per pixel + complete header size */
    bmpFileHeader.bfSize = ((BSP_LCD_GetXSize() - 105) * (BSP_LCD_GetYSize() - 105 + 1) * RGB888_BYTE_PER_PIXEL);
    bmpFileHeader.bfSize += bmpFileHeader.bOffBits;

    bmpFileHeader.bfReserved1 = 0x0000;
    bmpFileHeader.bfReserved2 = 0x0000;

    bmpFileInfoHeader.biSize = 40; /* 40 bytes in bitmap info header */
    bmpFileInfoHeader.biWidth = (BSP_LCD_GetXSize() - 105);
    bmpFileInfoHeader.biHeight = (BSP_LCD_GetYSize() - 105);
    bmpFileInfoHeader.biPlanes = 1; /* one single plane */
    bmpFileInfoHeader.biBitCount = 24; /* RGB888 : 24 bits per pixel */
    bmpFileInfoHeader.biCompression = 0; /* no compression */

    /* This is number of pixel bytes in file : sizeX * sizeY * RGB888_BYTE_PER_PIXEL */
    bmpFileInfoHeader.biSizeImage = ((BSP_LCD_GetXSize() - 105) * (BSP_LCD_GetYSize() - 105 + 1) * RGB888_BYTE_PER_PIXEL);

    bmpFileInfoHeader.biXPelsPerMeter = 0; /* not used */
    bmpFileInfoHeader.biYPelsPerMeter = 0; /* not used */
    bmpFileInfoHeader.biClrUsed = 0; /* not used */
    bmpFileInfoHeader.biClrImportant = 0; /* not used */

    /* -- End Prepare Bitmap file (BMP) header */

    /*##-1- Prepare the image to be saved ####################################*/
    Prepare_Picture();

    /* Disable the LTDC to avoid charging the bandwidth for nothing while the BMP file is */
    /* written to SD card */
    LTDC_Operation(0);

    /*##-2- Create and Open a new bmp file object with write access ##########*/
    if(f_open(&MyFile, (const char*)str, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
    {
      /* 'image.bmp' file Open for write Error */
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t*)"     BMP File Creation Error !!", RIGHT_MODE);
      Error_Handler();
    }
    else
    {
      /*##-3- Write data to the BMP file #####################################*/
      /* Write the BMP header step 1 : first write BMP header : all but padding is written to file */
        res1 = f_write(&MyFile, (uint16_t *)&(bmpFileHeader.bfType),
                       sizeof(uint16_t),
                       (void *)&bmpHeaderByteCnt);
        byteswritten += bmpHeaderByteCnt;

        /* LSB of size in bytes of BMP file */
        tmp_size = (uint16_t)(bmpFileHeader.bfSize & 0x0000FFFF);
        res1 = f_write(&MyFile, (uint16_t *)&(tmp_size),
                       sizeof(uint16_t),
                       (void *)&bmpHeaderByteCnt);
        byteswritten += bmpHeaderByteCnt;

        /* MSB of size in bytes of BMP file */
        tmp_size = (uint16_t)((bmpFileHeader.bfSize & 0xFFFF0000) >> 16);
        res1 = f_write(&MyFile, (uint16_t *)&(tmp_size),
                       sizeof(uint16_t),
                       (void *)&bmpHeaderByteCnt);
        byteswritten += bmpHeaderByteCnt;

        res1 = f_write(&MyFile, (uint16_t *)&(bmpFileHeader.bfReserved1),
                       sizeof(uint16_t),
                       (void *)&bmpHeaderByteCnt);
        byteswritten += bmpHeaderByteCnt;

        res1 = f_write(&MyFile, (uint16_t *)&(bmpFileHeader.bfReserved2),
                       sizeof(uint16_t),
                       (void *)&bmpHeaderByteCnt);
        byteswritten += bmpHeaderByteCnt;

        res1 = f_write(&MyFile, (uint32_t *)&(bmpFileHeader.bOffBits),
                       sizeof(uint32_t),
                       (void *)&bmpHeaderByteCnt);
        byteswritten += bmpHeaderByteCnt;

      if(res1 != FR_OK)
      {
      /* Reactivate LTDC */
        LTDC_Operation(1);
        f_close(&MyFile);
        BSP_LCD_ClearStringLine(BSP_LCD_GetYSize() - 20);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t*)"     BMP File Header Saving Error !!", RIGHT_MODE);
        Error_Handler();
      }

      byteswritten += bmpHeaderByteCnt;

      if(res1 == FR_OK)
      {
        /* Write the BMP header step 2 : second write BMP file info header */
        res2 = f_write(&MyFile, (BitMapFileInfoHeader_Typedef *)&bmpFileInfoHeader,
                       sizeof(BitMapFileInfoHeader_Typedef),
                       (void *)&bmpFileInfoHeaderByteCnt);


          if(res2 != FR_OK)
          {
            /* Reactivate LTDC */
            LTDC_Operation(1);
            f_close(&MyFile);
            BSP_LCD_ClearStringLine(BSP_LCD_GetYSize() - 20);
            BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t*)"     BMP File Header Info Saving Error !!", RIGHT_MODE);
            Error_Handler();
          }
      }

      byteswritten += bmpFileInfoHeaderByteCnt;

      if((res1 == FR_OK) && (res2 == FR_OK))
      {
        /* Write pixel data in the the BMP file */
        res3 = f_write(&MyFile, (uint8_t *)p_bmp_converted_pixel_data,
                       bmpFileInfoHeader.biSizeImage,
                       (void *)&bmpFilePixelBytesCnt);

          /* Reactivate LTDC */
          LTDC_Operation(1);

        if(res3 != FR_OK)
        {
          if(res3 == FR_DISK_ERR)
          {
              f_close(&MyFile);
              BSP_LCD_ClearStringLine(BSP_LCD_GetYSize() - 20);
              BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t*)"     File Saving Error DISKERR !!", RIGHT_MODE);
          }

          Error_Handler();
        }

      }

      byteswritten += bmpFilePixelBytesCnt;

      if((res1 != FR_OK) || (res2 != FR_OK) || (res3 != FR_OK) || (byteswritten == 0))
      {
        /* 'image.bmp' file Write or EOF Error */
        BSP_LCD_ClearStringLine(BSP_LCD_GetYSize() - 20);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t*)"     BMP File Saving Error !!", RIGHT_MODE);
        Error_Handler();
      }
      else
      {
        /*##-4- Close the open BMP file ######################################*/
        f_close(&MyFile);

        /* Success of the demo: no error occurrence */
        BSP_LED_On(LED1);

        BSP_LCD_ClearStringLine(BSP_LCD_GetYSize() - 20);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t*)"             BMP File Saved.", RIGHT_MODE);

        /* Wait for 2s */
        HAL_Delay(2000);

        BSP_LCD_ClearStringLine(BSP_LCD_GetYSize() - 20);

        /* Select Layer 1 */
        BSP_LED_Off(LED1);
        counter++;
      }
    }
  }
}

/**
  * @brief  Disable/Enable LTDC activity as in DSI Video mode the LTDC is all the time
  *         in active window pumping data with its DMA, it creates a huge bandwidth consumption
  *         that can penalize other IPs, here the save to SD functionality.
  * @param  Enable_LTDC : 0 to disable LTDC, 1 to re-enable LTDC
  * @retval None
  */
static void LTDC_Operation(uint32_t Enable_LTDC)
{
  /* Desactivate the DSI wrapper */
  DSI->WCR &= ~(DSI_WCR_DSIEN);

  if(Enable_LTDC == 1)
  {
    __HAL_LTDC_ENABLE(&hltdc_eval); /* Switch back On bit LTDCEN */
  }
  else if (Enable_LTDC == 0)
  {
    __HAL_LTDC_DISABLE(&hltdc_eval); /* Switch Off bit LTDCEN */
  }

  /* Reactivate the DSI Wrapper */
 DSI->WCR |= DSI_WCR_DSIEN;
}

/**
  * @brief  Prepares the picture to be saved in microSD.
  * @param  None
  * @retval None
  */
static void Prepare_Picture(void)
{
  const uint32_t x0 = 98;
  const uint32_t x1 = BSP_LCD_GetXSize()- 7;
  const uint32_t y0 = 7;
  const uint32_t y1 = BSP_LCD_GetYSize()- 98;
  uint32_t addrSrc = LCD_FRAME_BUFFER;
  uint32_t addrDst = CONVERTED_FRAME_BUFFER;
  static DMA2D_HandleTypeDef hdma2d_eval;
  uint32_t lineCnt = 0;

  /* Configure the DMA2D Mode, Color Mode and output offset : used to convert ARGB8888 to RGB888 */
  /* used in BMP file format                                                                     */
  hdma2d_eval.Init.Mode         = DMA2D_M2M_PFC;
  hdma2d_eval.Init.ColorMode    = DMA2D_RGB888; /* DMA2D Output format */
  hdma2d_eval.Init.OutputOffset = 0;

  /* Foreground Configuration */
  hdma2d_eval.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d_eval.LayerCfg[1].InputAlpha = 0xFF;
  hdma2d_eval.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888; /* DMA2D input format */
  hdma2d_eval.LayerCfg[1].InputOffset = 0;

  hdma2d_eval.Instance = DMA2D;

  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d_eval) == HAL_OK)
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d_eval, 1) != HAL_OK)
    {
      Error_Handler();
    }
  }
  else
  {
    Error_Handler();
  }

  /* Go to start of last drawing pad useful line from LCD frame buffer */
  addrSrc += (((y1 * BSP_LCD_GetXSize()) + x0) * ARGB8888_BYTE_PER_PIXEL);

  /* Copy and Convert picture from LCD frame buffer in ARGB8888 to Converted frame buffer in
   * RGB888 pixel format for all the useful lines of the drawing pad */
  for(lineCnt = y0; lineCnt <= y1; lineCnt++)
  {
    if (HAL_DMA2D_Start(&hdma2d_eval, addrSrc, addrDst, (x1 - x0), 1) == HAL_OK)
    {
      /* Polling For DMA transfer */
      HAL_DMA2D_PollForTransfer(&hdma2d_eval, 20);
    }

    /* Increment the destination address by one line RGB888, this will add one padding pixel */
    addrDst += ((x1 - x0) * RGB888_BYTE_PER_PIXEL) + RGB888_BYTE_PER_PIXEL;

    /* Decrement the source address by one line */
    addrSrc -= (BSP_LCD_GetXSize() * ARGB8888_BYTE_PER_PIXEL);
  }
}

/**
  * @brief  Updates the selected color
  * @param  None
  * @retval None
  */
static void Update_Color(void)
{
  static uint32_t color;

  /* Get the current text color */
  color = BSP_LCD_GetTextColor();

  /* Update the selected color icon */
  BSP_LCD_SetTextColor(color);
  BSP_LCD_FillRect(450, (BSP_LCD_GetYSize() - 50), 30, 30);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_DrawRect(450, (BSP_LCD_GetYSize() - 50), 30, 30);
  BSP_LCD_SetTextColor(color);
}

/**
  * @brief  Updates the selected size
  * @param  size: Size to be updated
  * @retval None
  */
static void Update_Size(uint8_t size)
{
  static uint32_t color;

  /* Get the current text color */
  color = BSP_LCD_GetTextColor();

  /* Update the selected size icon */
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillCircle(520, (BSP_LCD_GetYSize() - 35), 20);
  BSP_LCD_SetTextColor(color);
  BSP_LCD_FillCircle(520, (BSP_LCD_GetYSize() - 35), size);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_DrawCircle(520, (BSP_LCD_GetYSize() - 35), size);
  BSP_LCD_SetTextColor(color);
}



/**
  * @}
  */
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
