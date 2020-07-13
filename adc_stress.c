#include <ch.h>
#include <hal.h>
#include <stdnoreturn.h>
#include "stdutil.h"
#include "adc_stress.h"
#include "adcHelper.h"

/*
  ° connecter B6 (uart1_tx) sur ftdi rx
  ° connecter B7 (uart1_rx) sur ftdi tx
  ° connecter C0 sur led0 
*/

#define VOLT_TO_ADC(x) ((uint32_t)((x/3.3f)*4095))

#define ADC_GRP1_NUM_CHANNELS   2
#define ADC_GRP1_BUF_DEPTH      1
static adcsample_t IN_DMA_SECTION_CLEAR(adcSamples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH]);
static void adc_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n);
/*
 * ADC conversion group.
 * Mode:        Linear buffer, 1 samples of 2 channels, SW triggered.
 * Channels:    IN11, thermal sensor
 */

/*
  ADC_SMPR1_SMP_SENSOR : surechentillonage du capteur interne de température
  ADC_CHANNEL_SENSOR   : n° de canal du capteur interne de température 
 */

static float scaleTemp (int fromTmp);

static THD_WORKING_AREA(waDisplayAdc, 512);
static noreturn void displayAdc (void *arg)
{
  (void)arg;
  chRegSetThreadName("displayAdc");
  chThdSleepMilliseconds(100);

  while (true) {
    const float internalTemp = scaleTemp(adcSamples[0]);
    const float potVolt = 3.3f * adcSamples[1]/4095.f;
    
    DebugTrace("temp=%.1f; potVolt=%.3f", internalTemp,
	       potVolt);
    chThdSleepMilliseconds(1000);
  }
}


static ADCConversionGroup adcgrpcfg;
void adcStressInit(void)
{
  adcgrpcfg = adcGetConfig(ADC_GRP1_NUM_CHANNELS,
			   ADC_CONTINUOUS,
			   &adc_cb,
			   ADC_CHANNEL_SENSOR,
			   LINE_C01_POTAR,
			   ADC_END);

  adcStart(&ADCD1, NULL);
  
  // initialise le capteur interne de température
  adcSTM32EnableTSVREFE();
  adcStartConversion(&ADCD1, &adcgrpcfg, adcSamples, ADC_GRP1_BUF_DEPTH);
  
  chThdCreateStatic(waDisplayAdc, sizeof(waDisplayAdc), NORMALPRIO+1, &displayAdc, NULL);
}

/* cette fonction renvoie la temperature interne du coeur ARM en fonction de la tension du capteur
   analogique intégré 

   la formule est donnée dans le reference manuel du microcontroleur

*/
static float scaleTemp (int fromTmp)
{
  const float sampleNorm = (float) fromTmp / 4095.0f;
  const float sampleVolt = sampleNorm * 3.3f;
  const float deltaVolt = sampleVolt - 0.76f;
  const float deltaCentigrade = deltaVolt / 2.5e-3f;
  const float temp = 25.0f + deltaCentigrade;

  return temp;
}

static void adc_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n)
{
  (void) adcp;
  (void) buffer;
  (void) n;
}
