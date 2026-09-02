#include "stm32g4xx.h"
#include "stm32g4xx_ll_rcc.h"

int main(void) {
	/* HSI is the reset clock; keep the skeleton independent of the future board clock tree. */
	LL_RCC_HSI_Enable();
	while(LL_RCC_HSI_IsReady() == 0) {
	}
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
	}
	SystemCoreClockUpdate();

	// ponytail: no board I/O until the STM32G431CBU6 pin map is supplied.
	for(;;)
		__WFI();
}
