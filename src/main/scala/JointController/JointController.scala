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
// | Joint Controller module - Only the controller itself
// | --> Input: 
// |        - target_pos, target_vel
// |        - current_pos
// |        - min_speed, cutoff_speed
// | <-- Output:
// |        - pos_out, vel_out
// +---------------------------------------------------+
class JointControlleModuleIO extends Bundle {
    // Input: current and desires position
    val target_pos = Input(SInt(64.W))
    val target_vel = Input(SInt(64.W))
    val curr_pos = Input(SInt(64.W))

    //Output: Set speed
    val pos_out = Output(SInt(32.W))
    val vel_out = Output(SInt(32.W))

    // Mininum Values
    val min_speed = Input(SInt(32.W))
    val cutoff_speed = Input(SInt(32.W))
}


class  JointControllerModule(Kp: Int = 4, Ki: Int = 0, Kd: Int = 0, shift: Int = 2) extends Module {
    val io = IO(new PIDControllerModuleIO())
    // Wires for error terms
    val error_pos = Wire(SInt(64.W))
    val error_vel = Wire(SInt(64.W))
    val error_acc = Wire(SInt(64.W))
    
    // Wires for positional input terms
    val P_input_pos = Wire(SInt(64.W))
    val I_input_pos = Wire(SInt(64.W))
    val D_input_pos = Wire(SInt(64.W))

    // Wires for positional input terms
    val P_input_vel = Wire(SInt(64.W))
    val I_input_vel = Wire(SInt(64.W))
    val D_input_vel = Wire(SInt(64.W))

    
    
    // calculate error values
    error_pos := (io.target_pos - io.curr_pos)
    error_vel := 0.S
    error_sum := RegNext(error_sum) + error_pos

    // Add pipeline stage(s) here if needed
    P_input_pos := (error_pos * Kp.asSInt) >> shift.asUInt
    I_input_pos := ((error_sum * Ki.asSInt) >> shift.asUInt) >> 22.U // Estimate a 60MHz clock as dividing by 2^22 = 4194304
    D_input_pos := (error_vel * Kd.asSInt) >> shift.asUInt

    val pos_val = Wire(SInt(32.W))
    pos_val := RegNext(P_input_pos + I_input_pos + D_input_pos)



    // Calculate output for position control
    when(pos_val < io.cutoff_speed && pos_val > -io.cutoff_speed) {
        io.pos_out := 0.S
    }.elsewhen(pos_val < io.min_speed && pos_val > -io.min_speed){
        when (pos_val > 0.S) {
        io.pos_out := io.min_speed
        }.otherwise{
        io.pos_out := -io.min_speed
        }
    }.otherwise{
        io.pos_out := pos_val
    }
}



