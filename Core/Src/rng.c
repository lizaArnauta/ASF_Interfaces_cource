#include "stm32f4xx_hal.h"
#include "rng.h"

static uint32_t rng_state = 0x12345678;

uint32_t xorshift32(void) {
    uint32_t x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state = x;
    return x;
}

void rng_seed(uint32_t seed) {
    rng_state = seed;
}
