#ifndef GPIO_MOTOR_H
#define GPIO_MOTOR_H


#define POSITION_NOT_REACHED	0
#define POSITION_REACHED       	1

void gpio_motor_init(void);
void gpio_motor_set_speed(float speed_r, float speed_l);
void gpio_motor_set_position(float position_r, float position_l, float speed_r, float speed_l);
void gpio_motor_stop(void);
uint8_t gpio_motor_position_reached(void);

#endif /* GPIO_MOTOR_H */
