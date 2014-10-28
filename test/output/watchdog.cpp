#include "Marlin.h"

#ifdef USE_WATCHDOG
#include <avr/wdt.h>

#include "watchdog.h"
#include "ultralcd.h"











void watchdog_init()
{
#ifdef WATCHDOG_RESET_MANUAL
    
    
    wdt_reset();
    _WD_CONTROL_REG = _BV(_WD_CHANGE_BIT) | _BV(WDE);
    _WD_CONTROL_REG = _BV(WDIE) | WDTO_4S;
#else
    wdt_enable(WDTO_4S);
#endif
}


void watchdog_reset() 
{
    wdt_reset();
}






#ifdef WATCHDOG_RESET_MANUAL
ISR(WDT_vect)
{ 
    
    LCD_ALERTMESSAGEPGM("ERR:Please Reset");
    lcd_update();
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Something is wrong, please turn off the printer.");
    kill(); 
    while(1); 
}
#endif

#endif
