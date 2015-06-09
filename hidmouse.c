/***************************************************************************//**
 * @file hidkbd.c
 * @brief USB Human Interface Devices (HID) class keyboard driver.
 * @version 3.20.12
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/
#include "em_device.h"
#include "em_common.h"
#include "em_usb.h"
#include "hidmouse.h"

/**************************************************************************//**
 * @addtogroup HidKeyboard
 * @{ Implements a USB HID class keyboard.

@section hidkbd_intro HIDKBD implementation.

   The source code of the HIDKBD implementation resides in
   kits/common/drivers/hidkbd.c and hidkbd.h. This driver implements a basic
   USB keyboard. Refer to hidkbd examples for STK3700 and DK3750 for examples
   using this driver.

@section hidkbd_config HIDKBD device configuration options.

  This section contains a description of the configuration options for
  the driver. The options are @htmlonly #define's @endhtmlonly which are
  expected to be found in the application "usbconfig.h" header file.
  The values shown below are from the Giant Gecko STK3700 hidbkd example.

  @verbatim
// USB interface number. Interfaces are numbered from zero to one less than
// the number of concurrent interfaces supported by the configuration.
// The interfac numbers must be 0 standalone HID device, for a
// composite device which includes a HID interface it must not be in conflict
// with other device interfaces.
#define HIDKBD_INTERFACE_NO     0

// Endpoint address for HID input reports.
#define HIDKBD_INTR_IN_EP_ADDR  0x81

// The bInterval reported with the interrupt IN endpoint descriptor.
#define HIDKBD_POLL_RATE        24

// Timer id, see USBTIMER in the USB device stack documentation.
// The HID driver use a timer to implement the idle-rate defined in the HID class spec.
#define HIDKBD_IDLE_TIMER       0
  @endverbatim
 ** @} ***********************************************************************/

/** Default idle-rate recommended in the USB HID class specification. */
#define DEFAULT_IDLE_RATE  500

/** HID report descriptor created with usb.org's "HID Descriptor Tool(DT)"
    version 2.4. The descriptor defines an input report which matches
    the HIDKBD_KeyReport_t typedef, and an output report with 3 bits
    for controlling NumLock, CapsLock and ScrollLock keyboard LED's
*/
EFM32_ALIGN(4)
const char HIDKBD_MouseReportDescriptor[ 50 ] __attribute__ ((aligned(4))) =
{ 
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                    // USAGE (Mouse)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x03,                    //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x05,                    //     REPORT_SIZE (5)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    0xc0,                          //   END_COLLECTION
    0xc0                           // END_COLLECTION
};

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

static uint32_t                 tmpBuffer;
static uint8_t                  idleRate;
static void                     *hidDescriptor = NULL;
static HIDKBD_SetReportFunc_t   setReportFunc = NULL;

/* The last keyboard report sent to host. */
EFM32_ALIGN(4)
static HIDKBD_MouseReport_t lastSentReport __attribute__ ((aligned(4)));

/* The last keyboard report reported to the driver. */
EFM32_ALIGN(4)
static HIDKBD_MouseReport_t lastKnownReport __attribute__ ((aligned(4)));

static bool QueueEmpty( void );
static bool QueueFull( void );
static bool QueueGet( HIDKBD_MouseReport_t *element );
static void QueueInit( void );
static bool QueuePut( HIDKBD_MouseReport_t *element );

/**************************************************************************//**
 * @brief
 *   Timeout function for the idleRate timer. The idleRate is set by the host
 *   device driver with the SET_IDLE setup command.
 *   If idleRate is set to 0 the idleRate timer is completely stopped.
 *   This function will always send a keyboard report to host.
 *****************************************************************************/
static void IdleTimeout( void )
{
  /* If there is a keyboard event in the queue, we send it to the host. */
  /* If not we just resend the last one sent. */
  if ( !QueueEmpty() )
  {
    /* A new keyboard event. */
    QueueGet( &lastSentReport );
  }

  USBD_Write( HIDKBD_INTR_IN_EP_ADDR, &lastSentReport,
              sizeof( HIDKBD_MouseReport_t ), NULL );

  /* Schedule next idle event at current idle rate, idleRate unit is 4 ms. */
  USBTIMER_Start( HIDKBD_IDLE_TIMER, idleRate * 4, IdleTimeout );
}

/**************************************************************************//**
 * @brief
 *   Callback function called when the data stage of a USB_HID_SET_REPORT
 *   setup command has completed.
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK.
 *****************************************************************************/
static int OutputReportReceived( USB_Status_TypeDef status,
                                 uint32_t xferred,
                                 uint32_t remaining )
{
  (void) remaining;

  if (     ( status        == USB_STATUS_OK )
        && ( xferred       == 1             )
        && ( setReportFunc != NULL          ) )
  {
    setReportFunc( (uint8_t)tmpBuffer );
  }

  return USB_STATUS_OK;
}

/** @endcond */

/***************************************************************************//**
 * @brief
 *  Initialize HID Keyboard driver.
 *
 * @param[in] init
 *  Pointer to a HIDKBD_Init_t struct with configuration options.
 ******************************************************************************/
void HIDKBD_Init( HIDKBD_Init_t *init )
{
  hidDescriptor = init->hidDescriptor;
  setReportFunc = init->setReportFunc;
  memset( &lastSentReport,  0, sizeof( HIDKBD_MouseReport_t ) );
  memset( &lastKnownReport, 0, sizeof( HIDKBD_MouseReport_t ) );
}

/***************************************************************************//**
 * @brief
 *  Report a keyboard press/release event.
 *
 * @param[in] report
 *  Pointer to a HIDKBD_KeyReport_t struct with new key pressed information.
 ******************************************************************************/
void HIDKBD_MouseEvent( HIDKBD_MouseReport_t *report )
{
  lastKnownReport = *report;

  if ( idleRate != 0 )      /* Put keyboard events into a queue. */
  {
    /* Put the kbd event in the queue, it will be retrieved and reported */
    /* to host in the idleRate timeout function.                         */
    QueuePut( report );
  }
  else /* idleRate == 0, send report immediately. */
  {
    lastSentReport = *report;
    USBD_Write( HIDKBD_INTR_IN_EP_ADDR, &lastSentReport,
                sizeof( HIDKBD_MouseReport_t ), NULL );
  }
}

/**************************************************************************//**
 * @brief
 *   Handle USB setup commands. Implements HID class specific commands.
 *   This function must be called each time the device receive a setup command.
 *
 *
 * @param[in] setup Pointer to the setup packet received.
 *
 * @return USB_STATUS_OK if command accepted,
 *         USB_STATUS_REQ_UNHANDLED when command is unknown. In the latter case
 *         the USB device stack will handle the request.
 *****************************************************************************/
int HIDKBD_SetupCmd( const USB_Setup_TypeDef *setup )
{
  STATIC_UBUF( hidDesc, USB_HID_DESCSIZE );

  int retVal = USB_STATUS_REQ_UNHANDLED;

  if ( ( setup->Type      == USB_SETUP_TYPE_STANDARD       ) &&
       ( setup->Direction == USB_SETUP_DIR_IN              ) &&
       ( setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE )    )
  {
    /* A HID device must extend the standard GET_DESCRIPTOR command   */
    /* with support for HID descriptors.                              */
    switch (setup->bRequest)
    {
    case GET_DESCRIPTOR:
      /********************/
      if ( ( setup->wValue >> 8 ) == USB_HID_REPORT_DESCRIPTOR )
      {
        USBD_Write( 0, (void*)HIDKBD_MouseReportDescriptor,
                    EFM32_MIN(sizeof(HIDKBD_MouseReportDescriptor), setup->wLength),
                    NULL );
        retVal = USB_STATUS_OK;
      }
      else if ( ( setup->wValue >> 8 ) == USB_HID_DESCRIPTOR )
      {
        /* The HID descriptor might be misaligned ! */
        memcpy( hidDesc, hidDescriptor, USB_HID_DESCSIZE );
        USBD_Write( 0, hidDesc, EFM32_MIN(USB_HID_DESCSIZE, setup->wLength),
                    NULL );
        retVal = USB_STATUS_OK;
      }
      break;
    }
  }

  else if ( ( setup->Type      == USB_SETUP_TYPE_CLASS          ) &&
            ( setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE ) &&
            ( setup->wIndex    == HIDKBD_INTERFACE_NO           )    )
  {
    /* Implement the necessary HID class specific commands.           */
    switch ( setup->bRequest )
    {
    case USB_HID_SET_REPORT:
      /********************/
      if ( ( ( setup->wValue >> 8   ) == 2                ) &&  /* Output report */
           ( ( setup->wValue & 0xFF ) == 0                ) &&  /* Report ID     */
           ( setup->wLength           == 1                ) &&  /* Report length */
           ( setup->Direction         != USB_SETUP_DIR_IN )    )
      {
        USBD_Read( 0, (void*)&tmpBuffer, 1, OutputReportReceived );
        retVal = USB_STATUS_OK;
      }
      break;

    case USB_HID_GET_REPORT:
      /********************/
      if ( ( ( setup->wValue >> 8   ) == 1                ) &&  /* Input report  */
           ( ( setup->wValue & 0xFF ) == 0                ) &&  /* Report ID     */
           ( setup->wLength           == 8                ) &&  /* Report length */
           ( setup->Direction         == USB_SETUP_DIR_IN )    )
      {
        USBD_Write( HIDKBD_INTR_IN_EP_ADDR, &lastKnownReport,
                    sizeof( HIDKBD_MouseReport_t ), NULL );
        retVal = USB_STATUS_OK;
      }
      break;

    case USB_HID_SET_IDLE:
      /********************/
      if ( ( ( setup->wValue & 0xFF) == 0                ) &&  /* Report ID     */
           ( setup->wLength          == 0                ) &&
           ( setup->Direction        != USB_SETUP_DIR_IN )    )
      {
        idleRate = setup->wValue >> 8;
        if ( ( idleRate != 0 ) && ( idleRate < ( HIDKBD_POLL_RATE / 4 ) ) )
        {
          idleRate = HIDKBD_POLL_RATE / 4;
        }
        USBTIMER_Stop( HIDKBD_IDLE_TIMER );
        if ( idleRate != 0 )
        {
          IdleTimeout();
        }
        retVal = USB_STATUS_OK;
      }
      break;

    case USB_HID_GET_IDLE:
      /********************/
      if ( ( setup->wValue    == 0                ) &&  /* Report ID     */
           ( setup->wLength   == 1                ) &&
           ( setup->Direction == USB_SETUP_DIR_IN )    )
      {
        *(uint8_t*)&tmpBuffer = idleRate;
        USBD_Write( 0, (void*)&tmpBuffer, 1, NULL );
        retVal = USB_STATUS_OK;
      }
      break;
    }
  }

  return retVal;
}

/**************************************************************************//**
 * @brief
 *   Handle USB state change events, this function must be called each time
 *   the USB device state is changed.
 *
 * @param[in] oldState The device state the device has just left.
 * @param[in] newState The new device state.
 *****************************************************************************/
void HIDKBD_StateChangeEvent( USBD_State_TypeDef oldState,
                              USBD_State_TypeDef newState )
{
  if ( newState == USBD_STATE_CONFIGURED )
  {
    /* We have been configured, start HID functionality ! */
    if ( oldState != USBD_STATE_SUSPENDED ) /* Resume ?   */
    {
      idleRate = DEFAULT_IDLE_RATE / 4;     /* Unit is 4 millisecond. */
      QueueInit();
    }
    if ( idleRate )
    {
      USBTIMER_Start( HIDKBD_IDLE_TIMER, idleRate * 4, IdleTimeout );
    }
  }

  else if ( ( oldState == USBD_STATE_CONFIGURED ) &&
            ( newState != USBD_STATE_SUSPENDED  )    )
  {
    /* We have been de-configured, stop HID functionality */
    USBTIMER_Stop( HIDKBD_IDLE_TIMER );
  }

  else if ( newState == USBD_STATE_SUSPENDED )
  {
    /* We have been suspended, stop HID functionality */
    /* Reduce current consumption to below 2.5 mA.    */
    USBTIMER_Stop( HIDKBD_IDLE_TIMER );
  }
}

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

/* Minimal circular buffer implementation. */

#define QUEUE_SIZE 16             /* Must be 2^n !! */

typedef struct ringBuffer_t
{
  unsigned int        putIdx;
  unsigned int        getIdx;
  HIDKBD_MouseReport_t  buf[ QUEUE_SIZE ];
} ringBuffer_t;

ringBuffer_t queue;

static bool QueueEmpty( void )
{
  return ( queue.putIdx - queue.getIdx ) == 0;
}

static bool QueueFull( void )
{
  return ( queue.putIdx - queue.getIdx ) >= QUEUE_SIZE;
}

static bool QueueGet( HIDKBD_MouseReport_t *element )
{
  if ( !QueueEmpty() )
  {
    *element = queue.buf[ queue.getIdx++ & (QUEUE_SIZE - 1) ];
    queue.getIdx = ( queue.getIdx + 1 ) % QUEUE_SIZE;
    return true;
  }
  return false;
}

static void QueueInit( void )
{
  queue.getIdx = 0;
  queue.putIdx = 0;
}

static bool QueuePut( HIDKBD_MouseReport_t *element )
{
  if ( !QueueFull() )
  {
    queue.buf[ queue.putIdx++ & (QUEUE_SIZE - 1) ] = *element;
    queue.putIdx = ( queue.putIdx + 1 ) % QUEUE_SIZE;
    return true;
  }
  return false;
}

/** @endcond */
