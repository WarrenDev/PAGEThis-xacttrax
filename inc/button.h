#if !defined(BUTTON_H_)
#define BUTTON_H_

#include <wm_types.h>

bool GetSWButtonOvrd(int * val);
void ClrSWButtonOvrd(void);
void SetSWButtonOvrd(void);
void ProcessPowerButton(void);
bool InputButtonActive(void);
void ProcessIginitionButton();


#endif

