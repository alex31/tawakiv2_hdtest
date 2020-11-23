#include <ch.h>
#include <hal.h>
#include <stdnoreturn.h>
#include "stdutil.h"
#include "adc_stress.h"

/*
  ° connecter B6 (uart1_tx) sur ftdi rx
  ° connecter B7 (uart1_rx) sur ftdi tx
  ° connecter C0 sur led0 
*/

#define GPT_TRIG FALSE
#define ADC_WATCHDOG TRUE


#define TIM6TRGO 0b1101
#define VOLT_TO_ADC(x) ((uint32_t)((x/3.3f)*4095))

#define ADC_GRP1_NUM_CHANNELS   2
#define ADC_GRP1_BUF_DEPTH      1
static adcsample_t IN_DMA_SECTION_CLEAR(adcSamples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH]);
static void adc_cb(ADCDriver *adcp);
static void adcErrorCb(ADCDriver *adcp, adcerror_t err);
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
static volatile size_t nbSamples=0U;
static volatile bool   adcWatchDogTriggered=false;

static THD_WORKING_AREA(waDisplayAdc, 512);
static noreturn void displayAdc (void *arg)
{
  (void)arg;
  chRegSetThreadName("displayAdc");
  chThdSleepMilliseconds(100);

  while (true) {
    const float internalTemp = scaleTemp(adcSamples[0]);
    const float potVolt = 3.3f * adcSamples[1]/4095.f;
    
    DebugTrace("temp=%.1f; potVolt=%.3f [%u Isr] %c", internalTemp,
      potVolt, adcGetNbSamples(), adcWatchDogTriggered ? 'T' : ' ');
    adcWatchDogTriggered = false;
    chThdSleepMilliseconds(1000);
  }
}

#if GPT_TRIG
static GPTConfig gpt6cfg1 = {
			     .frequency =  1e4,
			     .callback  =  NULL,
			     .cr2       =  TIM_CR2_MMS_1,  /* MMS = 010 = TRGO on Update Event.        */
			     .dier      =  0U
};
#endif

static const ADCConversionGroup adcgrpcfg = {
  .circular	= TRUE,   // continuous conversion
  .num_channels = ADC_GRP1_NUM_CHANNELS,
  .end_cb	= &adc_cb, // adc completed callback,
  .error_cb	= &adcErrorCb, // adc error callback,

#if ADC_WATCHDOG
  .cr1		= ADC_CR1_AWDSGL | ADC_CR1_AWDEN | ADC_CR1_AWDIE | (11 << ADC_CR1_AWDCH_Pos),
#else
  .cr1		= 0,
#endif
#if GPT_TRIG
  .cr2		= ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(TIM6TRGO),
#else
  .cr2		= ADC_CR2_SWSTART,
#endif
  .smpr1	= ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_480) | ADC_SMPR1_SMP_AN11(ADC_SAMPLE_480),
  .smpr2	= 0,                        
  .htr		= VOLT_TO_ADC(2.0),
  .ltr		= VOLT_TO_ADC(1.0),
  .sqr1		= 0,
  .sqr2		= 0,					   
  .sqr3		= ADC_SQR3_SQ1_N(ADC_CHANNEL_SENSOR) | 	ADC_SQR3_SQ2_N(ADC_CHANNEL_IN11)
};

void adcStressInit(void)
{
#if GPT_TRIG
  gptStart(&GPTD6, &gpt6cfg1);
  gptStartContinuous(&GPTD6, 5);
#endif
  
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

static void adc_cb(ADCDriver *adcp)
{
  (void) adcp;

  nbSamples++;
}

size_t adcGetNbSamples(void)
{
  return nbSamples;
}

static void adcErrorCb(ADCDriver *adcp, adcerror_t err)
{
  (void) adcp;

  if (err == ADC_ERR_WATCHDOG) {
    adcWatchDogTriggered=true;
    chSysLockFromISR();
    adcStartConversionI(&ADCD1, &adcgrpcfg, adcSamples, ADC_GRP1_BUF_DEPTH);
    chSysUnlockFromISR();
  }
}
