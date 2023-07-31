#include <stdarg.h>

#include "common.h"
#include "zjy122250/epaper.h"

void UI_InitEPD(void);
void UI_TestEPD(void);

void UI_TerminalInit();
void UI_TerminalRender();
void UI_TerminalPrintf(const char *format, ...);
void UI_TerminalFlush();
void UI_TerminalClear();