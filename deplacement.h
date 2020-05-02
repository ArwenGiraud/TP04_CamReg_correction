#ifndef DEPLACEMENT_H
#define DEPLACEMENT_H

void lieu_obstacle(uint8_t ir_sensor_nb);
uint8_t taille_obstacle(void);
void contourne_obstacle(uint8_t type_obstacle);
void face_obstacle(float facteur_rotatif, uint8_t sens);

#endif /* DEPLACEMENT_H */
