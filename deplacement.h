#ifndef DEPLACEMENT_H
#define DEPLACEMENT_H

void recule(void);
uint8_t taille_obstacle(void);
void contourne_obstacle(uint8_t type_obstacle);
void tourne(float facteur_rotatif, uint8_t sens);
void deplacement_start(void);

#endif /* DEPLACEMENT_H */
