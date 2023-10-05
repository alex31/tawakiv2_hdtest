#include <ch.h>
#include <hal.h>
#include "stdutil.h"
#include "leds.h"
#include "i2cMaster.h"
#include <array>
#include <algorithm>
#include "internalSensors.hpp"
#include "spiPeriphIvensenseV3.h"
#include "sdLog.h"
#include "tlsf_malloc.h"
#include "sdio.h"

static bool initSensors(void);
static float getVbatVoltage(void);
static float getCoreTemp(void);
static bool  sdLogInit(void);
static void powerUnplugSurvey (void *arg);
static const uint16_t *TS_CAL1 =  (uint16_t *) 0x1FF1E820; //  30 °C,V=3.3V
static const uint16_t *TS_CAL2 =  (uint16_t *) 0x1FF1E840; //  110 °C,V=3.3V
static float scaleTemp(const adcsample_t rawt);
static float scaleVolt(const adcsample_t rawt);

/*
#                            _     _____                      _          
#                           | |   / ____|                    | |         
#                 ___     __| |  | |        __ _   _ __    __| |         
#                / __|   / _` |  | |       / _` | | '__|  / _` |         
#                \__ \  | (_| |  | |____  | (_| | | |    | (_| |         
#                |___/   \__,_|   \_____|  \__,_| |_|     \__,_|         
*/
static FileDes file=-1;




/*
#                  ___    _____      _____         
#                 / _ \  |  __ \    / ____|        
#                | |_| | | |  | |  | |             
#                |  _  | | |  | |  | |             
#                | | | | | |__| |  | |____         
#                |_| |_| |_____/    \_____|        
*/
enum SampleIndex {EXTBAT, VSENSE, VREFINT};
constexpr size_t   ADC_GRP1_NUM_CHANNELS =   3;
constexpr size_t   ADC_GRP1_BUF_DEPTH    =   1;
constexpr uint32_t EXTBAT_CHANNEL	 = ADC_CHANNEL_IN10;
constexpr uint32_t VSENSE_CHANNEL	 = ADC_CHANNEL_IN18;
constexpr uint32_t VREFINT_CHANNEL	 = ADC_CHANNEL_IN19;
static adcsample_t IN_DMA_SECTION_CLEAR(samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH]);
static const ADCConversionGroup adcgrpcfg = {
  .circular     = true,
  .num_channels = ADC_GRP1_NUM_CHANNELS,
  .end_cb       = nullptr,
  .error_cb     = [](hal_adc_driver*, uint32_t) {chSysHalt("adc cb error");},
  .cfgr         = ADC_CFGR_CONT_ENABLED,
  .cfgr2        = 0,
  .ccr          = 0U,
  .pcsel        = ADC_SELMASK_IN10 | ADC_SELMASK_IN18 | ADC_SELMASK_IN19,
  .ltr1         = 0,
  .htr1         = 0,
  .ltr2         = 0,
  .htr2         = 0,
  .ltr3         = 0,
  .htr3         = 0,
  .awd2cr       = 0,
  .awd3cr       = 0,
 .smpr         = {
   0,
   ADC_SMPR2_SMP_AN10(ADC_SMPR_SMP_810P5) |
   ADC_SMPR2_SMP_AN18(ADC_SMPR_SMP_810P5) |
   ADC_SMPR2_SMP_AN19(ADC_SMPR_SMP_810P5)
 },
 .sqr          = {
   ADC_SQR1_SQ1_N(EXTBAT_CHANNEL) | ADC_SQR1_SQ1_N(VSENSE_CHANNEL) |
   ADC_SQR1_SQ1_N(VREFINT_CHANNEL),
   0U, 0U, 0U
 }
};


/*
#                 _____   ___     _____         
#                |_   _| |__ \   / ____|        
#                  | |      ) | | |             
#                  | |     / /  | |             
#                 _| |_   / /_  | |____         
#                |_____| |____|  \_____|        
*/
#define I2C_FAST_400KHZ_DNF3_R200NS_F50NS_PCLK120MHZ_TIMINGR 0x30800F2D
#define STM32_CR1_DNF(n)         ((n << I2C_CR1_DNF_Pos) & I2C_CR1_DNF_Msk)

_Static_assert(STM32_I2C4SEL ==  STM32_I2C4SEL_PCLK4, "I2C4 source clock must be PCLK4 (see mcuconf.h)");
_Static_assert((STM32_D1HPRE == STM32_D1HPRE_DIV2), "PCLK4 should be 120Mhz (SYSCLK/->2<-/2)");
_Static_assert((STM32_D3PPRE4 == STM32_D3PPRE4_DIV2), "PCLK4 should be 120Mhz (SYSCLK/2/->2<-)");


static constexpr I2CConfig i2ccfg_400 __attribute__((unused)) = {
  .timingr = I2C_FAST_400KHZ_DNF3_R200NS_F50NS_PCLK120MHZ_TIMINGR, // Refer to the STM32H7 reference manual
  .cr1 = STM32_CR1_DNF(3), // Digital noise filter activated (timingr should be aware of that)
  .cr2 = 0, // Only the ADD10 bit can eventually be specified here (10-bit addressing mode)
} ;

/*
#                 ______   _____    _____         
#                /  ____| |  __ \  |_   _|        
#                | (___   | |__) |   | |          
#                 \___ \  |  ___/    | |          
#                .____) | | |       _| |_         
#                \_____/  |_|      |_____|        
*/

/* 7.5 Mhz, 8 bits word, CPHA= second (rising) edge, CPOL= high level idle state */
const SPIConfig spiCfg = {
  .circular         = true,
  .slave	    = false,
  .data_cb          = nullptr,
  .error_cb         = [](hal_spi_driver*) {chSysHalt("spi cb error");},
  .ssline = LINE_SPI6_CS_INTERNAL,
  .cfg1             = SPI_CFG1_MBR_DIV16 | SPI_CFG1_DSIZE_VALUE(7),
  .cfg2             = SPI_CFG2_CPOL | SPI_CFG2_CPHA
};


/*
#                 __  __            __ _                  _                    
#                |  \/  |          / _` |                | |                   
#                | \  / |   __ _  | (_| |  _ __     ___  | |_     ___          
#                | |\/| |  / _` |  \__, | | '_ \   / _ \ | __|   / _ \         
#                | |  | | | (_| |   __/ | | | | | |  __/ \ |_   | (_) |        
#                |_|  |_|  \__,_|  |___/  |_| |_|  \___|  \__|   \___/         
*/
static constexpr Lis3mdlConfig lisCfg = {
  .i2cp = &I2CD4,
  .numSlave = LIS3_SLAVE_SA1_HIGH,
  .regs = {
    .cr = {
      [0] = LIS3_CR1_DATARATE_80 |  LIS3_CR1_OMXY_ULTRAHIGH | LIS3_CR1_TEMP_ENABLE,
      [1] = LIS3_CR2_SCALE_4_GAUSS,
      [2] = LIS3_CR3_MODE_CONTINUOUS_CONV,
      [3] = LIS3_CR4_LITTLE_ENDIAN | LIS3_CR4_OMZ_ULTRAHIGH,
      [4] = LIS3_CR5_UPDATE_BDU |  LIS3_CR5_NORMALREAD},
    .intCfg = LIS3_INT_CFG_DISABLE,
    .threshold = 0U
  }
};
static Lis3mdlDriver lisd; 

/*
#                 ____                                              _                          
#                |  _ \                                            | |                         
#                | |_) |   __ _   _ __    ___    _ __ ___     ___  | |_     ___   _ __         
#                |  _ <   / _` | | '__|  / _ \  | '_ ` _ \   / _ \ | __|   / _ \ | '__|        
#                | |_) | | (_| | | |    | (_) | | | | | | | |  __/ \ |_   |  __/ | |           
#                |____/   \__,_| |_|     \___/  |_| |_| |_|  \___|  \__|   \___| |_|           
*/
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
static Bmp3xxConfig bmp3cfg = {
  .i2cp = &I2CD4,
  .slaveAddr = BMP3_ADDR_I2C_SEC,
  .settings = {
    .op_mode = BMP3_MODE_NORMAL,
    .press_en =  BMP3_ENABLE,
    .temp_en = BMP3_ENABLE,
    .odr_filter = {
      .press_os = BMP3_OVERSAMPLING_32X,
      .temp_os = BMP3_OVERSAMPLING_2X,
      .odr = BMP3_ODR_12_5_HZ // BMP3_ODR_6_25_HZ BMP3_ODR_200_HZ
    }
  },
  .settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS |
                  BMP3_SEL_TEMP_OS | BMP3_SEL_ODR /* | BMP3_SEL_DRDY_EN*/
};

#pragma GCC diagnostic pop
static Bmp3xxDriver bmp3p;


/*
#                 _____   __  __   _    _         
#                |_   _| |  \/  | | |  | |        
#                  | |   | \  / | | |  | |        
#                  | |   | |\/| | | |  | |        
#                 _| |_  | |  | | | |__| |        
#                |_____| |_|  |_|  \____/         
*/
#define TICKS_PER_PERIOD      10U
#define PWMDRIVER	      PWMD15
#define ICM_CLOCK_PWM_FREQ    32000U
#define TICK_FREQ (ICM_CLOCK_PWM_FREQ * TICKS_PER_PERIOD)

static PWMConfig pwmcfg = {     
  .frequency = ICM_CLOCK_PWM_FREQ * TICKS_PER_PERIOD,
  .period    = TICKS_PER_PERIOD,
  .callback  = NULL,            
  .channels  = {
    {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = NULL},
    {.mode = PWM_OUTPUT_DISABLED, .callback = NULL},
    {.mode = PWM_OUTPUT_DISABLED, .callback = NULL},
    {.mode = PWM_OUTPUT_DISABLED, .callback = NULL}
  },
  .cr2  = 0,
  .bdtr = 0,
  .dier = 0
};

const Inv3Config icmCfg =  {
  .spid = &SPID6,
  .commonOdr = COMMON_ODR_1KHZ,
  .gyroScale = GYRO_FS_SEL_15_625DPS,
  .accelScale = ACCEL_FS_SEL_2G,
  .externClockRef = true,
};
static Inv3Driver inv3d;

static THD_WORKING_AREA(waSensorsAcquire, 2*1024);	
static THD_WORKING_AREA(waBatterySurvey, 1024);	
static void sensorsAcquire (void *arg);		




/*
#                 ______                          _      _                                 
#                |  ____|                        | |    (_)                                
#                | |__     _   _   _ __     ___  | |_    _     ___    _ __    ___          
#                |  __|   | | | | | '_ \   / __| | __|  | |   / _ \  | '_ \  / __|         
#                | |      | |_| | | | | | | (__  \ |_   | |  | (_) | | | | | \__ \         
#                |_|       \__,_| |_| |_|  \___|  \__|  |_|   \___/  |_| |_| |___/         
*/
bool	launchSensorsThd(void)
{
  i2cStart(&I2CD4, &i2ccfg_400);
  spiStart(&SPID6, &spiCfg);
  adcStart(&ADCD3, NULL);
  // initialise la reference de tension interne
  adcSTM32EnableVREF(&ADCD3);
  // initialise le capteur interne de température
  adcSTM32EnableTS(&ADCD3);
  adcStartConversion(&ADCD3, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);

  while (not initSensors()) {
    chThdSleepSeconds(1);
  }
  while (not (sdLogInit())) {
    chThdSleepSeconds(5);
  }

  chThdCreateStatic(waSensorsAcquire, sizeof(waSensorsAcquire),
		    NORMALPRIO, &sensorsAcquire, NULL);
  chThdCreateStatic(waBatterySurvey, sizeof(waBatterySurvey),
		    NORMALPRIO, &powerUnplugSurvey, NULL);

  
  return true;
}
  


static bool initSensors(void)
{
  
  // MAGNETO init and factory test
  msg_t status  = lis3mdlStart(&lisd, &lisCfg);
  if (status != MSG_OK) {
    DebugTrace ("lis3mdlStart fails");
    ledSet(LINE_LED1, LED_BLINKFAST);
    return false;
  } else {
    ledSet(LINE_LED1, LED_BLINKSLOW);
    DebugTrace ("lis3mdlStart OK");
  }
  Lis3_ErrorMask msk;
  if ((msk = lis3mdlLaunchTest(&lisd)) != LIS3_TEST_PASS) {
    DebugTrace ("self test fails with mask = 0x%x", msk);
    ledSet(LINE_LED1, LED_BLINKFAST);
    return false;
  } else {
    DebugTrace ("self test OK");
    lis3mdlStart(&lisd, &lisCfg);
    ledSet(LINE_LED1, LED_BLINKSLOW);
  }

  // BAROMETER init
  if (bmp3xxStart(&bmp3p, &bmp3cfg) == MSG_OK) {
    DebugTrace ("bmp init OK");
    ledSet(LINE_LED2, LED_BLINKSLOW);
  } else {
    DebugTrace ("bmp init FAIL");
    ledSet(LINE_LED2, LED_BLINKFAST);
    return false;
  }

  // IMU init and factory test
  // start synchronization clock @32Khz
  pwmStart(&PWMD15, &pwmcfg);
  pwmEnableChannel(&PWMD15, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD15, 5000));

  if (inv3Init(&inv3d, &icmCfg) == true) {
     DebugTrace ("IMU init OK");
  } else {
    DebugTrace("IMU init FAIL");
    ledSet(LINE_LED3, LED_BLINKFAST);
    return false;
  }
  Vec3f accelDiff, gyroDiff, accelRatio, gyroRatio;
  bool selfTestOk = inv3RunSelfTest(icmCfg.spid, &accelDiff, &gyroDiff,
				    &accelRatio, &gyroRatio);
  
  if (selfTestOk) {
    ledSet(LINE_LED3, LED_BLINKSLOW);
    DebugTrace ("IMU factory test OK");
    for(size_t i=0; i<3; i++) {
      DebugTrace("PASS :: accRatio{%c} = %.3f, gyrRatio{%c} = %.3f [0.5 .. 1.5]",
		 'X' + i, accelRatio.v[i], 'X' + i, gyroRatio.v[i]);
    }
  } else {
    ledSet(LINE_LED3, LED_BLINKFAST);
    DebugTrace ("IMU factory FAIL");
    for(size_t i=0; i<3; i++) {
      DebugTrace("FAIL :: accRatio{%c} = %.3f, gyrRatio{%c} = %.3f [0.5 .. 1.5]",
		 'X' + i, accelRatio.v[i], 'X' + i, gyroRatio.v[i]);
    }
    return false;
  }
  
  if ((getVbatVoltage() < 9.0) || (getVbatVoltage() > 11.0)) {
    DebugTrace ("ADC BAT FAIL");
    ledSet(LINE_LED4, LED_BLINKFAST);
    return false;
  } else {
    ledSet(LINE_LED4, LED_BLINKSLOW);
  }
  
  return true;
}



static float getVbatVoltage(void)
{
  constexpr float R8 = 2'200.0f;
  constexpr float R20 = 18'000.0f;

  return scaleVolt(samples[EXTBAT]) * ((R8+R20) / R8);
}

static float getCoreTemp(void)
{
  return scaleTemp(samples[VSENSE]);
}


static float scaleTemp(const adcsample_t rawt)
{
  return ((80.0f / (*TS_CAL2 - *TS_CAL1)) * (rawt - *TS_CAL1)) + 30.0f;
}

static float scaleVolt(const adcsample_t rawt)
{
  return (rawt / 65535.0f) * 3.3f; 
}



static bool  sdLogInit(void)
{
  uint32_t freeSpaceInKo = 0;
  SdioError se;


  if (not sdioIsCardResponding()) {
    DebugTrace("sensors OK but µSD card not present, or not reponding\r\n");
    ledSet(LINE_LED1, LED_BLINKFAST);
    ledSet(LINE_LED4, LED_BLINKFAST);
    return false;
  }
  
  se = sdLogInit (&freeSpaceInKo);
  switch (se) {
  case SDLOG_OK : DebugTrace (" freeSpaceInKo = %lu Mo", freeSpaceInKo/1024); break;
  case SDLOG_FATFS_ERROR : DebugTrace ("sdLogInit: Fatfs error"); return false;
  case SDLOG_INTERNAL_ERROR : DebugTrace ("sdLogInit: Internal error"); return false;
  default: break;
  }

  se = sdLogOpenLog (&file, "TAWAKI_HDTEST", "hdlog.txt", 10, false, 50, false);
  switch (se) {
  case SDLOG_OK : DebugTrace ("sdOpenLog Ok"); break;
  case SDLOG_FATFS_ERROR : DebugTrace ("sdOpenLog: Fatfs error"); return false;
  case SDLOG_INTERNAL_ERROR : DebugTrace ("sdOpenLog: Internal error"); return false;
  default: break;
  }

  return true;
}

static void powerUnplugSurvey (void *arg)
{
  (void)arg;			
  chRegSetThreadName("power Unplug Survey");

  while (true) {
    if (getVbatVoltage() < 6.0) {
      BOARD_GROUP_DECLFOREACH(led, LINE_LEDS_GROUP) {
	ledSet(led, LED_OFF);
	palClearLine(led);
      } 
      sdLogCloseAllLogs(true);
      sdLogFinish();
      chThdSleepMilliseconds(4);
      systemDeepSleep();
    }
    chThdSleepMilliseconds(1);
  }
}


static void sensorsAcquire (void *arg)
{
  (void)arg;			
  chRegSetThreadName("sensors acquire");
  Vec3f mag;
  
  while (true) {
    // MAG
    lis3mdlWaitUntilDataReady(&lisd);
    if (lis3mdlFetch(&lisd, LIS3_STATUS_REG, LIS3_OUT_Z_H) != MSG_OK) {
      ledSet(LINE_LED1, LED_BLINKFAST);
    }
    lis3mdlFetch(&lisd, LIS3_TEMP_OUT_L, LIS3_TEMP_OUT_H);
    lis3mdlGetMag(&lisd, &mag);
    DebugTrace ("MAG x=%.3f y=%.3f z=%.3f temp=%.2f status=0x%x A=%.1f len=%0.2f",
		mag.v[0], mag.v[1], mag.v[2],
		lis3mdlGetTemp(&lisd),
		lis3mdlGetStatus(&lisd),
		atan2f(mag.v[1], mag.v[0]) * 180.0f / 3.1415926f,
		sqrtf(mag.v[0] * mag.v[0] +
		      mag.v[1] * mag.v[1] +
		      mag.v[2] * mag.v[2]));
    sdLogWriteLog(file, "MAG x=%.3f y=%.3f z=%.3f temp=%.2f status=0x%x A=%.1f len=%0.2f",
	       mag.v[0], mag.v[1], mag.v[2],
	       lis3mdlGetTemp(&lisd),
	       lis3mdlGetStatus(&lisd),
	       atan2f(mag.v[1], mag.v[0]) * 180.0f / 3.1415926f,
	       sqrtf(mag.v[0] * mag.v[0] +
		     mag.v[1] * mag.v[1] +
		     mag.v[2] * mag.v[2]));
    // BARO
    if (bmp3xxFetch(&bmp3p, BMP3_PRESS | BMP3_TEMP) == MSG_OK) {
      DebugTrace("Temp =%.2f, Press=%.2f mB",
		 bmp3xxGetTemp(&bmp3p)/100, bmp3xxGetPressure(&bmp3p)/10000.0f);
      sdLogWriteLog(file, "Temp =%.2f, Press=%.2f mB",
		    bmp3xxGetTemp(&bmp3p)/100, bmp3xxGetPressure(&bmp3p)/10000.0f);
      
    } else {
      DebugTrace ("bmp fetch FAIL");
      sdLogWriteLog(file, "bmp fetch FAIL");
      ledSet(LINE_LED2, LED_BLINKFAST);
    }

    // IMU
    Vec3f gyro={0}, acc={0};
    float temp=0;
    inv3GetVal(&inv3d, &temp, &gyro, &acc);
    DebugTrace("IMU temp= %.2f\r\n"
	       "IMU gyro=[x=%.2f, y=%.2f, z=%.2f]\r\n"
	       "IMU acc= [x=%.2f, y=%.2f, z=%.2f]",
	       temp, gyro.v[0], gyro.v[1],  gyro.v[2],
	       acc.v[0], acc.v[1],  acc.v[2]);
    sdLogWriteLog(file, "IMU temp= %.2f\r\n"
		  "IMU gyro=[x=%.2f, y=%.2f, z=%.2f]\r\n"
		  "IMU acc= [x=%.2f, y=%.2f, z=%.2f]",
		  temp, gyro.v[0], gyro.v[1],  gyro.v[2],
		  acc.v[0], acc.v[1],  acc.v[2]);
    
    const float vbat  = getVbatVoltage();
    const float coretemp = getCoreTemp();
    DebugTrace("vbat = %.2f core temp = %.1f",
	       vbat, coretemp);
    sdLogWriteLog(file, "vbat = %.2f core temp = %.1f",
		  vbat, coretemp);
    
    if ((vbat > 9.0) && (vbat < 11.0) && (coretemp < 60))
      ledSet(LINE_LED4, LED_BLINKSLOW);
    else
      ledSet(LINE_LED4, LED_BLINKFAST);
    
    chThdSleepSeconds(1);
  }
}



