#ifndef __HAVERSINE_H__
#define __HAVERSINE_H__

#include "gps.h"

int points2distance(COORD *a, COORD *b);
UINT8 triangleStatus(COORD *c1, COORD *c2, COORD *c3);
UINT8 quadStatus(COORD *c1, COORD *c2, COORD *c3, COORD *c4);
#endif
