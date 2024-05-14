#include "../../../tests/mmio.h"
#include <stdio.h>
#include <inttypes.h>

#define QDEC_ADDR 0x11000000L  // reference memeory address
#define MOTOR_ADDR 0x12000000L  // reference memeory address

int main(void) {
  printf("\n[STARTING TEST]\n\n");

  reg_write16(MOTOR_ADDR, 15000);
  reg_write8(MOTOR_ADDR + 0x08, 1);

  printf("Wait a bit...\n");

  reg_write8(MOTOR_ADDR + 0x0A, 1);

  printf("Wait a bit more...\n");
  printf("[Test done]\n");
}