#include "../../../tests/mmio.h"
#include <stdio.h>
#include <inttypes.h>

#define QDEC_ADDR 0x11000000L  // reference memeory address
#define MOTOR_ADDR 0x12000000L  // reference memeory address
#define JOINT_ADDR 0x13000000L  // reference memeory address


#define JOINT_TARGET    JOINT_ADDR + 0x00
#define JOINT_STATE     JOINT_ADDR + 0x08
#define JOINT_EN        JOINT_ADDR + 0x09
#define JOINT_DIR       JOINT_ADDR + 0x0A

#define JOINT_SPEED     JOINT_ADDR + 0x0C
#define JOINT_PRESC     JOINT_ADDR + 0x10

#define JOINT_POS       JOINT_ADDR + 0x18
#define JOINT_RST       JOINT_ADDR + 0x20

void wiggle() {
  reg_write8(JOINT_EN, 1);
  printf("  [Wait a bit...]\n");
  printf("  [Wait a bit...]\n");
  printf("  [Wait a bit...]\n");
  printf("  [Wait a bit...]\n");
  reg_write8(JOINT_EN, 0);

}

int main(void) {
  printf("\n[STARTING TEST]\n\n");

  // +----------------------------------------------+
  // | Test just the motor passthrough
  // +----------------------------------------------+
  // set initial direction
  puts("start");
  reg_write16(JOINT_SPEED, 2000);  
  wiggle();
  
  // switch the implicit dir pin
  puts("change dir");
  reg_write8(JOINT_DIR, 1);
  wiggle();

  // flip the input speed
  puts("change ds");
  reg_write16(JOINT_SPEED, -2000);
  reg_write8(JOINT_DIR, 0);
  wiggle();

  // switch the implicit dir pin
  puts("change dir, ds");
  reg_write8(JOINT_DIR, 1);
  wiggle();

  // // switch set the prescaler for the PWM
  // puts("add presc");
  // reg_write32(JOINT_PRESC, 1);
  // wiggle();

  // // change duty cycle again
  // puts("change ds");
  // reg_write16(JOINT_SPEED, 3000);
  // reg_write8(JOINT_DIR, 0);
  // wiggle();

  puts("Position0");
  reg_write8(JOINT_STATE, 1);
  wiggle();

  puts("Position1");
  reg_write64(JOINT_TARGET, 2000);
  wiggle();

  puts("Position2");
  reg_write64(JOINT_TARGET, -4000);
  wiggle();


  // change duty cycle again
  puts("Overflow");
  reg_write8(JOINT_STATE, 0);
  reg_write16(JOINT_SPEED, 5000);
  reg_write8(JOINT_DIR, 0);
  wiggle();


  // change duty cycle again
  puts("Underflow");
  reg_write16(JOINT_SPEED, -5000);
  reg_write8(JOINT_DIR, 0);
  wiggle();


  printf("[Test done]\n");
}