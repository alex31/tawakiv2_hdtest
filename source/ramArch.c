#include <ch.h>
#include <hal.h>
#include "ramArch.h"


/*
  nocache regions are 
    ° ram0nc for sdmmc1
    ° ram3 for miscellanous ?
    ° ram4 for bdma attached peripherals (i2c4, spi6, adc3)
 */

extern const uint32_t __ram0nc_base__;
extern const uint32_t __ram0nc_size__;
extern const uint32_t __ram3_base__;
extern const uint32_t __ram3_size__;
extern const uint32_t __ram4_base__;
extern const uint32_t __ram4_size__;

static uint32_t getMPU_RASR_SIZE(const uint32_t ldSize)
{
  // 2^n -> n-1
  chDbgAssert(__builtin_popcount(ldSize) == 1U, "MPU region size must be 2^n");
  chDbgAssert(ldSize >= 32U, "MPU region size must be >= 32");
  return __builtin_ctz(ldSize) - 1U;
}


void mpuConfigureNonCachedRam(void)
{
  const uint32_t mpuSharedOption = MPU_RASR_ATTR_AP_RW_RW |
    MPU_RASR_ATTR_NON_CACHEABLE | MPU_RASR_ATTR_S |
    MPU_RASR_ENABLE;

  const uint32_t ram0nc_base = (uint32_t) &__ram0nc_base__;
  const uint32_t ram3_base = (uint32_t) &__ram3_base__;
  const uint32_t ram4_base = (uint32_t) &__ram4_base__;

  const uint32_t ram0nc_size = (uint32_t) &__ram0nc_size__;
  const uint32_t ram3_size = (uint32_t) &__ram3_size__;
  const uint32_t ram4_size = (uint32_t) &__ram4_size__;

  chDbgAssert(ram0nc_base == 0x24000000, "MPU ram0nc addr mismatch");
  chDbgAssert(ram3_base == 0x30040000, "MPU ram3 addr mismatch");
  chDbgAssert(ram4_base == 0x38000000, "MPU ram4 addr mismatch");

  chDbgAssert((ram0nc_base % ram0nc_size) == 0, "MPU ram0nc base addr must be size aligned");
  chDbgAssert((ram3_base % ram3_size) == 0, "MPU ram3 base addr must be size aligned");
  chDbgAssert((ram4_base % ram4_size) == 0, "MPU ram4 base addr must be size aligned");

  
  mpuConfigureRegion(MPU_REGION_6,
		     ram3_base,
		     getMPU_RASR_SIZE(ram3_size) | mpuSharedOption
		     );
  mpuConfigureRegion(MPU_REGION_5,
		     ram4_base,
		     getMPU_RASR_SIZE(ram4_size) | mpuSharedOption
		     );
  mpuConfigureRegion(MPU_REGION_4,
		     ram0nc_base,
		     getMPU_RASR_SIZE(ram0nc_size) | mpuSharedOption
		     );
  
  mpuEnable(MPU_CTRL_PRIVDEFENA);
  __ISB();
  __DSB();
  SCB_CleanInvalidateDCache();
}
