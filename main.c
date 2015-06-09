/**************************************************************************//**
 * @file main.c
 * @brief USB HID keyboard device example.
 * @version 3.20.10
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

/* CHIP */
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_rmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_wdog.h"

/* USB */
#include "em_usb.h"
#include "descriptors.h"

/* Proximity */
#include "prox.h"
#include "i2cspm.h"
#include "si114x_algorithm.h"
#include "rtcdriver.h"
#include "em_adc.h"
#include "bspconfig.h"

/* Graphics */
#include "graphics.h"
#include "background.h"
   
/* Leds */
#include "bsp.h"
   
/**************************************************************************//**
 *
 * This example shows how a HID keyboard can be implemented.
 *
 *****************************************************************************/

/**************************************************************************//**
 * Local defines
 *****************************************************************************/
/** Time (in ms) between periodic updates of the measurements. */
#define PERIODIC_UPDATE_MS      50

/*** USB Callbacks ***/
static const USBD_Callbacks_TypeDef callbacks =
{
  .usbReset        = NULL,
  .usbStateChange  = NULL,
  .setupCmd        = HIDKBD_SetupCmd,
  .isSelfPowered   = NULL,
  .sofInt          = NULL
};

static const USBD_Init_TypeDef usbInitStruct =
{
  .deviceDescriptor    = &USBDESC_deviceDesc,
  .configDescriptor    = USBDESC_configDesc,
  .stringDescriptors   = USBDESC_strings,
  .numberOfStrings     = sizeof(USBDESC_strings)/sizeof(void*),
  .callbacks           = &callbacks,
  .bufferingMultiplier = USBDESC_bufferingMultiplier,
  .reserved            = 0
};

/* RTC callback parameters. */
static void (*rtcCallback)(void*) = NULL;
static void * rtcCallbackArg = 0;

/** This flag tracks if we need to update the display
 *  (animations or measurements). */
static volatile int updateDisplay = true;
/** Variable to change between flappyBird mode and helicopter game mode */
static volatile int flappyBirdConf = 15;
/** Flag that is used whenever we have get an gesture process interrupt. */
static volatile bool processGestures = false;
/** Millisecond tick counter */
volatile uint32_t msTicks;
/** Variables for holding the position */
static int  dist         = 0;
/** Last observed gesture */
static gesture_t gestureInput = NONE;

/** Timer used for timing out gesturemode to save power. */
RTCDRV_TimerID_t gestureTimeoutTimerId;
/** Timer used for counting milliseconds. Used for gesture detection. */
RTCDRV_TimerID_t millisecondsTimerId;
/** Timer used for periodic update of the measurements. */
RTCDRV_TimerID_t periodicUpdateTimerId;
/** Timer used for animations (swiping) */
RTCDRV_TimerID_t animationTimerId;

/**************************************************************************//**
 * Local prototypes
 *****************************************************************************/
static void enableGestureMode(void);
static void getDist(void);
static void msTicksCallback(RTCDRV_TimerID_t id, void *user);
static void periodicUpdateCallback(RTCDRV_TimerID_t id, void *user);
static void memLcdCallback(RTCDRV_TimerID_t id, void *user);

/**************************************************************************//**
 * @brief main - the entrypoint after reset.
 *****************************************************************************/
int main(void)
{
  
  HIDKBD_Init_t hidInitStruct;
  I2CSPM_Init_TypeDef i2cInit  = I2CSPM_INIT_DEFAULT;
  WDOG_Init_TypeDef   wdogInit = WDOG_INIT_DEFAULT;
  bool             si1147_status = false;
  bool pushed;      /* New mouse state */
  bool keyPushed;   /* Current mouse status. */
  bool pushOk       = true; /* OK to accept new push */
  int  uThr    = 0;
  HIDKBD_MouseReport_t *report;
  
  int lastDist, lastThr, lastFlap;
  

  /* Chip errata */
  CHIP_Init();

  /* Go slow to reduce current consumption. */
  //CMU_HFRCOBandSet( cmuHFRCOBand_7MHz );

  /* Initialize GPIO */
  PROX_GpioSetup();
  /* Reverse bytes, as they are LSB first, but rather MSB first (or the other way round)
     Should probably rewrite the array and save it rewritten in flash, but this was easier */
  PROX_ReverseBytes(backgroundBee);
  /* Initialize WDOG, not that we need it of course */
  wdogInit.enable = false;
  wdogInit.perSel = wdogPeriod_257;
  CMU_ClockEnable( cmuClock_CORELE, true);
  WDOG_Init(&wdogInit);
  
  /* Misc timers. */
  RTCDRV_Init();
  RTCDRV_AllocateTimer(&gestureTimeoutTimerId);
  RTCDRV_AllocateTimer(&millisecondsTimerId);
  RTCDRV_AllocateTimer(&periodicUpdateTimerId);
  RTCDRV_AllocateTimer(&animationTimerId);

  /* Initialize and clear screen */
  GRAPHICS_Init();
  GRAPHICS_Clear();

  /* Initialize I2C driver, using standard rate. */
  I2CSPM_Init(&i2cInit);
  
  /* Get status from detector */
  si1147_status = Si1147_Detect_Device(I2C0, SI1147_ADDR);
  GRAPHICS_ShowStatus(si1147_status, true, false, false);
  
  if (!si1147_status)
  {
    while(1); //Die if it's not available
  }
  
  /*configure prox sensor to enter low power state*/
  Si1147_ConfigureDetection(I2C0, SI1147_ADDR, true);

  /* Set up periodic update of the display.
   * Note: Must update at a much lesser rate than the gesture detection
   * algorithm sample rate. */
  RTCDRV_StartTimer(periodicUpdateTimerId, rtcdrvTimerTypePeriodic,
                    PERIODIC_UPDATE_MS, periodicUpdateCallback, NULL);
  
  
  
  
  /* Initialize HID keyboard driver. */
  hidInitStruct.hidDescriptor = (void*)USBDESC_HidDescriptor;
  hidInitStruct.setReportFunc = NULL;
  HIDKBD_Init( &hidInitStruct );

  /* Make sure we are using the correct band of the USHFRCO */
  CMU->USHFRCOCONF = CMU_USHFRCOCONF_BAND_48MHZ;
  
  /* Initialize and start USB device stack. */
  USBD_Init( &usbInitStruct );
  
  /* Send an empty report to make sure nothing is pressed by default */
  HIDKBD_MouseEvent( (void*)&USBDESC_noMouseReport );

  /*
   * When using a debugger it is practical to uncomment the following three
   * lines to force host to re-enumerate the device.
   */
  /* USBD_Disconnect();      */
  /* USBTIMER_DelayMs(1000); */
  /* USBD_Connect();         */

  enableGestureMode();
  WDOG_Enable(true);

  while(1)
  {
      if (processGestures)
      {
        getDist();
        /* Check if interrupt pin still low (if it is we have another sample ready). */
        if (GPIO_PinInGet(gpioPortD, 5) == 0)
          getDist();
        processGestures = false;
      }
      if (updateDisplay)
      {        
        if (gestureInput != NONE)
        {
          /* Calculate if a button was pushed, and new threshold */
          if (dist > uThr)
          {
            if (pushOk) {
              //skipEM1 = 4;
              pushed = true;
            }
            else
              pushed = false;
            
            pushOk = false;
            uThr = dist;
          }
          if (dist < uThr - flappyBirdConf)
          {
             pushOk = true;
             pushed = false;
             uThr = dist + flappyBirdConf;
          }
        }
        else
        {
          /* No gesture detected */
          dist = 0;
          pushOk = true;
          pushed = false;
          uThr = flappyBirdConf;
        }
        if ((dist != lastDist) || (uThr*pushOk != lastThr) || (flappyBirdConf != lastFlap)) {
          // Update display
          GRAPHICS_DrawProx(dist, uThr*pushOk, flappyBirdConf);
          lastDist = dist;
          lastThr  = uThr*pushOk;
          lastFlap = flappyBirdConf;
        }
        updateDisplay = false;
        
        /* Reset timer for periodic update of the display */
        RTCDRV_StartTimer(periodicUpdateTimerId, rtcdrvTimerTypePeriodic,
                          PERIODIC_UPDATE_MS, periodicUpdateCallback, NULL);

        if ( pushed )
          report = (void*)&USBDESC_leftMouseReport;
        else
          report = (void*)&USBDESC_noMouseReport;

        /* Pass mouse report on to the HID mouse driver. */
        HIDKBD_MouseEvent( report );
        keyPushed = pushed;
      }
    /* Conserve energy */
      EMU_EnterEM1();
  }
}

/**************************************************************************//**
 * @brief Enable gesture mode.
 *****************************************************************************/
static void enableGestureMode(void)
{
  Si1147_ConfigureDetection(I2C0, SI1147_ADDR, false);
  Si1147_SetInterruptOutputEnable(I2C0, SI1147_ADDR, true);

  /* Start timer to count milliseconds - used for gesture detection */
  RTCDRV_StartTimer(millisecondsTimerId, rtcdrvTimerTypePeriodic,
                    5, msTicksCallback, NULL);
}

/**************************************************************************//**
 * @brief This function is called whenever a new gesture needs to be processed.
 *        It is responsible to deliver the position to the mouse.
 *****************************************************************************/
static void getDist(void)
{
  int  distNew      = 0;

  /* get prox sensor sample */
  gestureInput = Si1147_NewPosSample(I2C0, SI1147_ADDR, msTicks, &distNew);

  dist = (dist + distNew) >> 1;
    
  //updateDisplay = true;
}

/**************************************************************************//**
 * @brief   The actual callback for Memory LCD toggling
 * @param[in] id
 *   The id of the RTC timer (not used)
 *****************************************************************************/
static void memLcdCallback(RTCDRV_TimerID_t id, void *user)
{
  (void) id;
  (void) user;
  rtcCallback(rtcCallbackArg);
}

/**************************************************************************//**
 * @brief   Register a callback function at the given frequency.
 *
 * @param[in] pFunction  Pointer to function that should be called at the
 *                       given frequency.
 * @param[in] argument   Argument to be given to the function.
 * @param[in] frequency  Frequency at which to call function at.
 *
 * @return  0 for successful or
 *         -1 if the requested frequency does not match the RTC frequency.
 *****************************************************************************/
int RtcIntCallbackRegister(void (*pFunction)(void*),
                           void* argument,
                           unsigned int frequency)
{
  RTCDRV_TimerID_t timerId;
  rtcCallback    = pFunction;
  rtcCallbackArg = argument;

  RTCDRV_AllocateTimer(&timerId);

  RTCDRV_StartTimer(timerId, rtcdrvTimerTypePeriodic, 1000 / frequency,
                    memLcdCallback, NULL);

  return 0;
}

/**************************************************************************//**
 * @brief Callback used to count between measurement updates
 *****************************************************************************/
static void periodicUpdateCallback(RTCDRV_TimerID_t id, void *user)
{
  (void) id;
  (void) user;
  updateDisplay = true;
}



/**************************************************************************//**
 * @brief Callback used to count milliseconds using gestures
 *****************************************************************************/
static void msTicksCallback(RTCDRV_TimerID_t id, void *user)
{
  (void) id;
  (void) user;
  msTicks += 5;
}

/**************************************************************************//**
* @brief Unified GPIO Interrupt handler (pushbuttons)
*        PB0 Switches units within a measurement display
*        PB1 Starts the demo (quit splashscreen)
*****************************************************************************/
void GPIO_Unified_IRQ(void) 
{
  /* Get and clear all pending GPIO interrupts */
  uint32_t interruptMask = GPIO_IntGet();
  GPIO_IntClear(interruptMask);

  /* Act on interrupts */
  if (interruptMask & (1 << BSP_GPIO_PB1_PIN)) 
  {
    /* PB0: Switch between flappy bird mode and hilcopter game mode*/
    flappyBirdConf++;
  }
  if (interruptMask & (1 << BSP_GPIO_PB0_PIN)) 
  {
    /* PB0: Switch between flappy bird mode and hilcopter game mode*/
    flappyBirdConf--;
  }
  if (interruptMask & (1 << 5)) 
  {
    /* Interrupt from Si1147 on PD5 */
    processGestures = true;
    /* This should happen every 20 ms, if not something is wrong and we will kill the chip */
    WDOG_Feed();

  }
}

/**************************************************************************//**
* @brief GPIO Interrupt handler for even pins
*****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  GPIO_Unified_IRQ();
}

/**************************************************************************//**
* @brief GPIO Interrupt handler for odd pins
*****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  GPIO_Unified_IRQ();
}
