#pragma once

#define DSHOT_BIT_WIDTHS		16
#define DSHOT_DMA_BUFFER_SIZE	        (DSHOT_BIT_WIDTHS + \
					 DSHOT_PRE_FRAME_SILENT_SYNC_BITS + \
					 DSHOT_POST_FRAME_SILENT_SYNC_BITS )
#define DSHOT_SPEED_KHZ			600
#define DSHOT_PRE_FRAME_SILENT_SYNC_BITS  2 
#define DSHOT_POST_FRAME_SILENT_SYNC_BITS 2
#define DSHOT_TELEMETRY_BAUD		115200
#define DSHOT_CHANNELS			4
#define DSHOT_TELEMETRY_TIMEOUT_MS	30
#define DSHOT_AT_LEAST_ONE_32B_TIMER	(STM32_PWM_USE_TIM2 || STM32_PWM_USE_TIM5)
