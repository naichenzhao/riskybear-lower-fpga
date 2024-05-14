package riskybear

import chisel3._
import chisel3.util._
import chisel3.experimental.{IntParam, BaseModule}
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.prci._
import freechips.rocketchip.subsystem.{BaseSubsystem, PBUS}
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{HasRegMap, RegField, RegFieldDesc}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.UIntIsOneOf
import freechips.rocketchip.util.{AsyncQueue, AsyncQueueParams}
import freechips.rocketchip.util.{SynchronizerShiftReg}

import riskybear._



// +---------------------------------------------------+
// | Motor module chaining two PWM modules
// +---------------------------------------------------+
class MotorChiselIO(val pwm_width: Int) extends Bundle {
  // Input pins
  val en = Input(Bool())
  val dir = Input(Bool())
  val speed_in = Input(SInt(pwm_width.W))

  // Output pins
  val motor_out_a = Output(Bool())
  val motor_out_b = Output(Bool())
}

class MotorChiselModule(val pwm_width: Int) extends Module {
  val io = IO(new MotorChiselIO(pwm_width))
  val motor_pin_a = Module(new MotorPWM(pwm_width))
  val motor_pin_b = Module(new MotorPWM(pwm_width))

  // Pass through en pins
  motor_pin_a.io.en := io.en
  motor_pin_b.io.en := io.en

  // pass through output pins
  io.motor_out_a := motor_pin_a.io.pwm_out
  io.motor_out_b := motor_pin_b.io.pwm_out

  // Set the speed of each PWM output
  when(io.speed_in > 0.S) {
    when(io.dir){
      motor_pin_a.io.duty_in := io.speed_in
      motor_pin_b.io.duty_in := 0.S
    }.otherwise{
      motor_pin_a.io.duty_in := 0.S
      motor_pin_b.io.duty_in := io.speed_in
    }
  }.otherwise{
    when(io.dir){
      motor_pin_a.io.duty_in := 0.S
      motor_pin_b.io.duty_in := -io.speed_in
    }.otherwise{
      motor_pin_a.io.duty_in := -io.speed_in
      motor_pin_b.io.duty_in := 0.S
    }
  }
}


// +---------------------------------------------------+
// | Custom PWM module for the motor
// +---------------------------------------------------+
class MotorPWMIO(val pwm_width: Int) extends Bundle {
  // Input pins
  val en = Input(Bool())
  val duty_in = Input(SInt(pwm_width.W))

  // Output pins
  val pwm_out = Output(Bool())
}

class MotorPWM(pwm_width: Int) extends Module {
  val io = IO(new MotorPWMIO(pwm_width))
  val pwm_val = RegInit(0.S(pwm_width.W))

  when(io.en) {
    pwm_val := pwm_val + 1.S
  }.otherwise{
    pwm_val := 0.S
  }
  io.pwm_out := io.en && (pwm_val < io.duty_in)
}

