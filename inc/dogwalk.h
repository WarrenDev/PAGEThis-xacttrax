#if !defined(DOGWALK_H_)
#define DOGWALK_H_

typedef enum
{
    DOGWALK_DISABLED = 0,
    DOGWALK_ENABLED
} dogwalk_t;

typedef enum
{
    DOGPARK_DISABLED = 0,
    DOGPARK_ENABLED
} dogpark_t;

void SetDogWalkMode(dogwalk_t const new_mode);
dogwalk_t GetDogWalkMode(void);

void SetDogParkMode(dogpark_t const new_mode);
dogpark_t GetDogParkMode(void);

#endif
