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
// | PID Controller module - Only the controller itself
// +---------------------------------------------------+
class PIDControllerModuleIO extends Bundle {
  // Input: current and desires position
  val target_pos = Input(SInt(64.W))
  val curr_pos = Input(SInt(64.W))

  //Output: Set speed
  val pid_out = Output(SInt(64.W))

  // PID controller parameters
  // val Kd = Input(SInt(32.W))
  // val Kp = Input(SInt(32.W))
  // val Ki = Input(SInt(32.W))
  // val shift = Input(UInt(32.W))
}


class  PIDControllerModule(Kp: Int = 2, Ki: Int = 0, Kd: Int = 0, shift: Int = 2) extends Module {
   val io = IO(new PIDControllerModuleIO())
   // Wires for error terms
   val P_error = Wire(SInt(64.W))
   val I_error = Wire(SInt(64.W))
   val D_error = Wire(SInt(64.W))
   
   // Wires for input terms
   val P_input = Wire(SInt(64.W))
   val I_input = Wire(SInt(64.W))
   val D_input = Wire(SInt(64.W))
   
   // calculate error values
   P_error := (io.target_pos - io.curr_pos)
   I_error := 0.S
   D_error := 0.S
   
  //  // Add pipeline stage(s) here if needed
  //  P_input := (P_error * io.Kp) >> io.shift
  //  I_input := (I_error * io.Ki) >> io.shift
  //  D_input := (D_error * io.Kd) >> io.shift

  // Add pipeline stage(s) here if needed
  P_input := (P_error * Kp.asSInt) >> shift.asUInt
  I_input := (I_error * Ki.asSInt) >> shift.asUInt
  D_input := (D_error * Kd.asSInt) >> shift.asUInt

   io.pid_out := P_input + I_input + D_input
}








