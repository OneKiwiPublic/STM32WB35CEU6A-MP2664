#ifndef __BOARD_H
#define __BOARD_H

#define LED_PIN		LL_GPIO_PIN_2
#define LED_PORT	GPIOA

void led_init(void);
void led_on(void);
void led_off(void);
void led_toggle(void);
void usart1_init(void);

#endif /* __BOARD_H */
