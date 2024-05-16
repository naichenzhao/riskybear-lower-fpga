#include "../../../tests/mmio.h"
#include <stdio.h>
#include <inttypes.h>

#define QDEC_ADDR 0x11000000L  // reference memeory address
#define MOTOR_ADDR 0x12000000L  // reference memeory address

int main(void) {
  printf("\n[STARTING TEST]\n\n");

  // set initial direction
  reg_write16(MOTOR_ADDR, 2000);

  reg_write8(MOTOR_ADDR + 0x08, 1);
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  reg_write8(MOTOR_ADDR + 0x08, 0);

  // switch the implicit dir pin
  printf("Pause...\n");
  reg_write8(MOTOR_ADDR + 0x09, 1);

  reg_write8(MOTOR_ADDR + 0x08, 1);
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  reg_write8(MOTOR_ADDR + 0x08, 0);

  // flip the input speed
  printf("Pause...\n");
  reg_write16(MOTOR_ADDR, -1000);
  reg_write8(MOTOR_ADDR + 0x09, 0);

  reg_write8(MOTOR_ADDR + 0x08, 1);
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  reg_write8(MOTOR_ADDR + 0x08, 0);

  // switch the implicit dir pin
  printf("Pause...\n");
  reg_write8(MOTOR_ADDR + 0x09, 1);

  reg_write8(MOTOR_ADDR + 0x08, 1);
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  reg_write8(MOTOR_ADDR + 0x08, 0);


  // switch set the prescaler for the PWM
  printf("Pause...\n");
  reg_write32(MOTOR_ADDR + 0x04, 1);

  reg_write8(MOTOR_ADDR + 0x08, 1);
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  reg_write8(MOTOR_ADDR + 0x08, 0);

  // change duty cycle again
  printf("Pause...\n");
  reg_write16(MOTOR_ADDR, 3000);
  reg_write8(MOTOR_ADDR + 0x09, 0);

  reg_write8(MOTOR_ADDR + 0x08, 1);
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  reg_write8(MOTOR_ADDR + 0x08, 0);



  // change duty cycle again
  printf("Overflow\n");
  reg_write16(MOTOR_ADDR, 5000);
  reg_write8(MOTOR_ADDR + 0x09, 0);

  reg_write8(MOTOR_ADDR + 0x08, 1);
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  reg_write8(MOTOR_ADDR + 0x08, 0);


  // change duty cycle again
  printf("Underflow\n");
  reg_write16(MOTOR_ADDR, -5000);
  reg_write8(MOTOR_ADDR + 0x09, 0);

  reg_write8(MOTOR_ADDR + 0x08, 1);
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  printf("Wait a bit...\n");
  reg_write8(MOTOR_ADDR + 0x08, 0);


  printf("[Test done]\n");
}