#include "dogwalk.h"

dogwalk_t walkmode = DOGWALK_DISABLED;
dogpark_t parkmode = DOGPARK_DISABLED;

void SetDogWalkMode(dogwalk_t const new_mode)
{
    walkmode = new_mode;
}

dogwalk_t GetDogWalkMode(void)
{
    return walkmode;
}


void SetDogParkMode(dogpark_t const new_mode)
{
    parkmode = new_mode;
}

dogpark_t GetDogParkMode(void)
{
    return parkmode;
}
