#include <gpio.h>
#include <selector.h>

#define Sel0     	GPIOC, 13
#define Sel1     	GPIOC, 14
#define Sel2     	GPIOC, 15
#define Sel3     	GPIOD, 4

#define NORMAL				0
#define MANUEL				1
#define AUTOMATIQUE			2

void init_selector(void)
{
    // Enable GPIOC and GPIOD peripheral clock
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;

    gpio_config_input_pd(Sel0);
	gpio_config_input_pd(Sel1);
	gpio_config_input_pd(Sel2);
	gpio_config_input_pd(Sel3);

}

uint8_t get_selector_mode(void)
{
    int selector, old_selector = 0;
    uint8_t mode = NORMAL;
	selector = gpio_read(Sel0) + 2 * gpio_read(Sel1) + 4 * gpio_read(Sel2) + 8 * gpio_read(Sel3);

    old_selector = selector;
    switch (selector)
   	{
    	case 0:
    		mode = MANUEL;
       		break;
    	case 1:
    		mode = AUTOMATIQUE;
    		break;
    }
    return mode;
}
