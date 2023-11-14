#include <ch.h>
#include <hal.h>
#include "ramArch.h"
#include "stdutil.h"		
#include "ttyConsole.hpp"	
#include "leds.h"		
#include "internalSensors.hpp"
#include <array>
#include <algorithm>
#include "sdio.h"

/*
  TODO : pour le BMP390 et le LIS3MDL en I²C, l'ICM40605, on utilise une mémoire non cache et on force le flush : 
         choisir une des 2 solutions, mais pas les 2 !
         possibilité : ne pas disabler le cache de ram4, et faire un invalidate avant de lire le buffer de samples ADC
                       qui devra être aligné 32 et avec taille alignée 32 aussi
  
  
 */

/*
  ° cf ChibiOS_21.11_stable/os/common/startup/ARMCMx/compilers/GCC/ld/STM32H743xI.ld
  ° cf RM0433 page 104/3353 : memory map
 
  *  ADC3, I2C4, SPI6  : BDMA -> .RAM4 -> DMAMUX2
  *  SDMMC1, SPI2, I2C2, TIM1, TIM3, TIM4  : DMA -> .RAM0NC -> DMAMUX1 
  *  FDCAN : non connecté DMA

  * solution retenue : 
    + pour l'adc, buffer statique ou l'on specifie section .ram4
    + pour le BMP390 et le LIS3MDL en I²C, l'ICM40605, les librairies allouent un buffer
      aligné sur la pile et force la coherence de cache : il suffit que tous les appels
      se fassent depuis des thread ou la pile soit allouée sur dans la section ram4
      ->TODO : thread qui utilisent .ram4 et initialisation des periphériques 
           faite dans un thread plutot que dans main

    + pour le SDMMC : connecté à AXI (.ram0) 
      dans les fichiers linker script, nocache correspond à ram0nc pour les buffers
      internes du driver SDIO (hardwired)
    + driver sdLog modifié

 */


/*
  Câbler une LED sur la broche C0


  ° connecter B6 (uart1_tx) sur PROBE+SERIAL Rx AVEC UN JUMPER
  ° connecter B7 (uart1_rx) sur PROBE+SERIAL Tx AVEC UN JUMPER
  ° connecter C0 sur led0

 */

// see board.cfg for detail about LINE_CONTINUITY_GROUP
static const std::array sdLines = {LINE_CONTINUITY_GROUP};

static THD_WORKING_AREA(waGpioPulse, 304);
static void gpioPulse (void *)
{
  chRegSetThreadName("gpioPulse");

  for (const auto &l : sdLines) {
    palSetLineMode(l, PAL_MODE_INPUT_PULLDOWN);
    palSetLine(l);
  }
  
  while (true) {
    auto ts = TIME_I2MS(chVTGetSystemTime());
    auto limit = ts+50; // a complete cycle every 60 milliseconds
    
    for (const auto &output : sdLines) {

      // all other line are input pulldown
      for (const auto &input : sdLines) {
	if (input != output) {
	  palSetLineMode(input, PAL_MODE_INPUT_PULLDOWN);
	}
      }

      // the one which output is output pushpull 
      palSetLineMode(output, PAL_MODE_OUTPUT_PUSHPULL);
      palClearLine(output);
      chThdSleepMicroseconds(100);		// pulse negatif de 100 µs
      palSetLine(output);
      chThdSleepMilliseconds(1);		// pulse de 1 ms
      palClearLine(output);
      chThdSleepMicroseconds(100);
    }

    chThdSleepUntil(TIME_MS2I(limit)); // frequence de 20hz (50ms)
   }
}


int main (void)
{
  halInit();
  mpuConfigureNonCachedRam();
  chSysInit();
  initHeap();		// initialisation du "tas" pour permettre l'allocation mémoire dynamique 

  ledRegisterLine(LINE_LED1, LED_BLINKSLOW);
  ledRegisterLine(LINE_LED2, LED_BLINKSLOW);
  ledRegisterLine(LINE_LED3, LED_BLINKSLOW);
  ledRegisterLine(LINE_LED4, LED_BLINKSLOW);
							    
  
  consoleInit();	// initialisation des objets liés au shell
  consoleLaunch();      // lancement du shell
  //chThdSleep(TIME_INFINITE);
  ledSet(LINE_LED4, LED_BLINKFAST);
  chThdSleepSeconds(5);
  ledSet(LINE_LED3, LED_BLINKFAST);
  while(true) {
    cmd_sdiotest(chp, false, nullptr);
    chThdSleepSeconds(10);
  }
  chThdCreateStatic(waGpioPulse, sizeof(waGpioPulse), NORMALPRIO, &gpioPulse, NULL); 
  launchSensorsThd();
  
  chThdSleep(TIME_INFINITE);
}


