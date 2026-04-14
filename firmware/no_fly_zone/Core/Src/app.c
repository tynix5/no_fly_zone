#include "app.h"
#include "nrf24.h"

static RadioParams testing;         // only app.c functions can access testing

void app_init(void)
{
    testing.this_addr = 2;
}

void app(void)
{
    while (1)
    {

    }
}