#include <ch.h>
#include <hal.h>
#include "stdutil.h"		// necessaire pour initHeap
#include "ttyConsole.h"		// fichier d'entête du shell
#include "leds.h"		// fichier d'entête du shell
//#include "internalSensors.hpp"
#include <array>
#include <algorithm>

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
  chSysInit();
  initHeap();		// initialisation du "tas" pour permettre l'allocation mémoire dynamique 

  ledRegisterLine(LINE_LED1, LED_BLINKSLOW);
  ledRegisterLine(LINE_LED2, LED_BLINKSLOW);
  ledRegisterLine(LINE_LED3, LED_BLINKSLOW);
  ledRegisterLine(LINE_LED4, LED_BLINKSLOW);
							    
  
  consoleInit();	// initialisation des objets liés au shell
  chThdCreateStatic(waGpioPulse, sizeof(waGpioPulse), NORMALPRIO, &gpioPulse, NULL); 
  // launchSensorsThd();
  // cette fonction en interne fait une boucle infinie, elle ne sort jamais
  // donc tout code situé après ne sera jamais exécuté.
  consoleLaunch();  // lancement du shell
  
  // main thread does nothing
  chThdSleep(TIME_INFINITE);
}


