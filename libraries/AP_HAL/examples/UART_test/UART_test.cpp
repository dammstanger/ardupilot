/*
  simple test of UART interfaces
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  setup one UART at 115200
 */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(115200);
}


void setup(void)
{
    /*
      start all UARTs at 57600 with default buffer sizes
    */

    hal.scheduler->delay(1000); //Ensure that the uartA can be initialized

    setup_uart(hal.uartA, "uartA");  // console
    setup_uart(hal.uartB, "uartB");  // 1st GPS
    setup_uart(hal.uartC, "uartC");  // telemetry 1
    setup_uart(hal.uartD, "uartD");  // telemetry 2
    setup_uart(hal.uartE, "uartE");  // 2nd GPS
    setup_uart(hal.uartF, "uartF");  // 
 //   setup_uart(hal.console, "console");  //

}

static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->printf("Hello on UART %s at %.3f seconds\n",
                name, (double)(AP_HAL::millis() * 0.001f));
    uart->println("3.1415926\n");
    
}

void loop(void)
{
    test_uart(hal.uartA, "uartA");
    test_uart(hal.uartB, "uartB");
    test_uart(hal.uartC, "uartC");
    test_uart(hal.uartD, "uartD");
    test_uart(hal.uartE, "uartE");
    test_uart(hal.uartF, "uartF");

    // also do a raw printf() on some platforms, which prints to the
    // debug console
//#if HAL_OS_POSIX_IO
//    ::printf("Hello on debug console at %.3f seconds\n", (double)(AP_HAL::millis() * 0.001f));
//#endif
//    ::printf("this message is from console.\n");
    char Data[50]={0};
    int size=hal.uartF->available();
    int i=0;

    if(size>=50)
        size = 49;
    while(size--){
        Data[i++] = (char)hal.uartF->read();
    }
    hal.console->printf("console:uartF rev: %s \n",Data);
    i = hal.console->txspace();
    hal.console->printf("console hase: %d Bytes space\n",i);
    
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
