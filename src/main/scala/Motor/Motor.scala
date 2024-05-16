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
  val presc = Input(UInt(32.W))
  val speed_in = Input(SInt(pwm_width.W))

  // Output pins
  val motor_out_a = Output(Bool())
  val motor_out_b = Output(Bool())
}

class MotorChiselModule(val pwm_width: Int) extends Module {
  val io = IO(new MotorChiselIO(pwm_width))
  val motor_pin_a = Module(new MotorPWM(pwm_width))
  val motor_pin_b = Module(new MotorPWM(pwm_width))

  // Pass through en values
  motor_pin_a.io.en := io.en
  motor_pin_b.io.en := io.en

  // Pass through pre-scaler values
  motor_pin_a.io.presc := io.presc
  motor_pin_b.io.presc := io.presc

  // pass through output pins
  io.motor_out_a := motor_pin_a.io.pwm_out
  io.motor_out_b := motor_pin_b.io.pwm_out

  // Set the speed of each PWM output
  when(io.speed_in > 0.S) {
    when(io.dir){
      motor_pin_a.io.duty_in := (io.speed_in).asUInt
      motor_pin_b.io.duty_in := 0.U
    }.otherwise{
      motor_pin_a.io.duty_in := 0.U
      motor_pin_b.io.duty_in := (io.speed_in).asUInt
    }
  }.otherwise{
    when(io.dir){
      motor_pin_a.io.duty_in := 0.U
      motor_pin_b.io.duty_in := (-io.speed_in).asUInt
    }.otherwise{
      motor_pin_a.io.duty_in := (-io.speed_in).asUInt
      motor_pin_b.io.duty_in := 0.U
    }
  }
}


// +---------------------------------------------------+
// | Custom PWM module for the motor
// +---------------------------------------------------+
class MotorPWMIO(val pwm_width: Int) extends Bundle {
  // Input pins
  val en = Input(Bool())
  val duty_in = Input(UInt(pwm_width.W))
  val presc = Input(UInt(32.W))

  // Output pins
  val pwm_out = Output(Bool())
}

class MotorPWM(pwm_width: Int) extends Module {
  val io = IO(new MotorPWMIO(pwm_width-1))
  val pwm_val = RegInit(0.U((pwm_width-1).W))
  val presc_val = RegInit(0.U(32.W))

  when(io.en) {
    when(presc_val === io.presc) {
      pwm_val := pwm_val + 1.U
      presc_val := 0.U
    }.otherwise{
      presc_val := presc_val + 1.U
    }
  }.otherwise{
    pwm_val := 0.U
  }
  io.pwm_out := io.en && (pwm_val <= io.duty_in) && (io.duty_in =/= 0.U)
}

