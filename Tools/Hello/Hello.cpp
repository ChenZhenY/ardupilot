/*
  simple hello world sketch
  Andrew Tridgell September 2011
*/

#include <AP_HAL/AP_HAL.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup()
{
    hal.console->printf("hello world\n");
    hal.console->printf("1212"); //add by czy, 2019.11.11
}

void loop()
{
    hal.scheduler->delay(1000);
    hal.console->printf("*\n");
    hal.console->printf("czy_helloworld");
}

AP_HAL_MAIN();
