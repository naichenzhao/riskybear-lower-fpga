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
    val clock = Input(Clock())
    val reset = Input(Bool())

    // Input: current and desires position
    val target_pos = Input(SInt(64.W))
    val target_vel = Input(SInt(64.W))
    val curr_pos = Input(SInt(64.W))

    //Output: Set speed
    val pos_target = Output(SInt(32.W))
    val vel_target = Output(SInt(32.W))

    val pos_out = Output(SInt(64.W))
    val vel_out = Output(SInt(64.W))

    // Mininum Values
    val min_speed = Input(SInt(32.W))
    val cutoff_speed = Input(SInt(32.W))
}


class  JointControllerChiselModule(Kp: Int = -64, Ki: Int = 0, Kd: Int = 0, shift: Int = 6) extends Module {
    val io = IO(new JointControlleModuleIO())

    // Output value wires
    val pos_val = Wire(SInt(32.W))
    val vel_val = Wire(SInt(32.W))

    // Calculate our current velocity
    val get_vel = Module(new Differentiator)       // Differentiator used to get the velocity
    get_vel.io.value := io.curr_pos

    io.pos_out := RegNext(io.curr_pos)
    io.vel_out := RegNext(get_vel.io.result)

    // Define PID modules
    val PID_pos = Module(new PIDModule(Kp = Kp, Ki = Ki, Kd = Kd, shift = shift))
    val PID_vel = Module(new PIDModule(Kp = Kp, Ki = Ki, Kd = Kd, shift = shift))

    PID_pos.io.target := io.target_pos
    PID_pos.io.current := io.curr_pos
    PID_vel.io.target := io.target_vel
    PID_vel.io.current := get_vel.io.result

    pos_val := PID_pos.io.result                    // By default, we monomize positon error to zero
    vel_val := PID_vel.io.result + io.target_vel    // For velocity control, we need to also pass ff

    // Calculate output for position control
    when(pos_val < io.cutoff_speed && pos_val > -io.cutoff_speed) {
        io.pos_target := 0.S
    }.elsewhen(pos_val < io.min_speed && pos_val > -io.min_speed){
        when (pos_val > 0.S) {
        io.pos_target := io.min_speed
        }.otherwise{
        io.pos_target := -io.min_speed
        }
    }.otherwise{
        io.pos_target := pos_val
    }


    // Calculate output for velocity
    when(vel_val < io.cutoff_speed && vel_val > -io.cutoff_speed) {
        io.vel_target := 0.S
    }.elsewhen(vel_val < io.min_speed && vel_val > -io.min_speed){
        when (vel_val > 0.S) {
        io.vel_target := io.min_speed
        }.otherwise{
        io.vel_target := -io.min_speed
        }
    }.otherwise{
        io.vel_target := vel_val
    }
}



