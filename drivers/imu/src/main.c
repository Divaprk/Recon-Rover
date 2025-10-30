#include <stdio.h>
#include "pico/stdlib.h"
#include "imu_orientation.h"

int main(void) {
    stdio_init_all();
    sleep_ms(500);
    run_imu_orientation();
    return 0;
}
