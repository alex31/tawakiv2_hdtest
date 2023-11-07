#include "usb_msd.h"
#include "ch.h"
#include "hal.h"
#include "stdutil.h"
#include "usbStorage.h"
#include "sdLog.h"
#include <stdio.h>
#include <string.h>
#include <sdio.h>

/* USB mass storage configuration */
/* Turns on a LED when there is I/O activity on the USB port */
static void usbActivity(bool active)
{
  palWriteLine (LINE_LED4, active ? PAL_HIGH : PAL_LOW);
}

static USBMassStorageConfig msdConfig =
{
    &USBD,
    (BaseBlockDevice*)&SDCD1,
    USB_MS_DATA_EP,
    &usbActivity,
    "ChibiOS",
    "Pprz",
    "0.2"
};

static void    thdUsbStorage(void *arg);

static thread_t*	usbStorageThreadPtr=NULL;

static THD_WORKING_AREA(waThsUsbStorage, 1024);
void usbStorageStartPolling (void)
{
  usbStorageThreadPtr = chThdCreateStatic (waThsUsbStorage, sizeof(waThsUsbStorage),
					   NORMALPRIO, thdUsbStorage, NULL);

}


void usbStorageWaitForDeconnexion (void)
{
  if (usbStorageThreadPtr != NULL) 
    chThdWait (usbStorageThreadPtr);
  usbStorageThreadPtr = NULL;
}

void usbStorageStop (void)
{
  if (usbStorageThreadPtr != NULL) {
    chThdTerminate (usbStorageThreadPtr);
  }
}




static void     thdUsbStorage(void *arg)
{
  (void) arg; // unused
  chRegSetThreadName("UsbStorage:polling"); 
  uint32_t antiBounce=5;
  
  // Should use EXTI interrupt instead of active polling,
  // but in the chibios_opencm3 implementation, since EXTI is
  // used via libopencm3, ISR are routed on pprz/opencm3 and cannot
  // be used concurrently by chibios api
  // Should be fixed when using chibios-rt branch
  while (!chThdShouldTerminateX() && antiBounce) {
    const bool usbConnected = palReadLine (LINE_USB_VBUS);
    if (usbConnected)
      antiBounce--;
    else
      antiBounce=5;
    
    chThdSleepMilliseconds(20);
  }

  chRegSetThreadName("UsbStorage:connected"); 


  /* connect sdcard sdc interface sdio */
  if (sdioConnect () == false) 
    chThdExit (MSG_TIMEOUT);

  init_msd_driver (NULL, &msdConfig);
  
  /* watch the mass storage events */
  while (!chThdShouldTerminateX() && palReadLine (LINE_USB_VBUS)) {
    chThdSleepMilliseconds(10);
  }
  
  deinit_msd_driver();
  
  chThdSleepMilliseconds(500);
  sdioDisconnect ();
  
  systemReset ();
}
