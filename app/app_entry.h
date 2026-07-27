#ifndef APP_ENTRY_H
#define APP_ENTRY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool App_Init(void);
void App_Process(void);

#ifdef __cplusplus
}
#endif

#endif