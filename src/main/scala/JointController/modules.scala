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

// +--------------------------------------------------------------------+
// | Integrator module
// |    - Performs a discrete-time integral over an input
// |    - We use a prescaler on the input cause hardware go fast
// +--------------------------------------------------------------------+
class IntegIO extends Bundle {
    val value = Input(SInt(64.W))
    val result = Output(SInt(64.W))
}

// Assuming a clock speed of 60MHz, we want our integrator to run at 10kHz
class  Integrator(presc: Int = 6000, maxval: Int = 1000, minval: Int = -1000) extends Module {
    val io = IO(new IntegIO())
    val presc_cnt = RegInit(0.U(32.W))
    val summation = RegInit(0.S(64.W))

    // Add the current value when the prescalar triggers
    when(presc_cnt >= presc.U) {
        summation := summation + io.value
        presc_cnt := 0.U
    }.otherwise{
        presc_cnt := presc_cnt + 1.U
    }

    // Apply the offset - divide by 10000 since we are running at 10kHz
    val base_res = Wire(SInt(64.W))
    base_res := summation/10000.S

    // Add min/max clamps on the result value
    when(base_res > maxval.S) {
        io.result := maxval.S
    }.elsewhen(base_res < minval.S) {
        io.result := minval.S
    }.otherwise{
        io.result := base_res
    }
}



// +--------------------------------------------------------------------+
// | Differentiator module
// |    - Performs a discrete-time derivative over an input
// |    - We use a prescaler on the input cause hardware go fast
// +--------------------------------------------------------------------+
class DiffIO extends Bundle {
    val value = Input(SInt(64.W))
    val result = Output(SInt(64.W))
}

// Assuming a clock speed of 60MHz, we want our differentiator to run at 10kHz
class  Differentiator(presc: Int = 6000) extends Module {
    val io = IO(new DiffIO())
    val presc_cnt = RegInit(0.U(32.W))
    val last_val = RegInit(0.S(64.W))
    val diff_val = RegInit(0.S(64.W))

    // Get the difference of two points based on the prescaler
    when(presc_cnt >= presc.U) {
        diff_val := io.value - last_val
        last_val := io.value
        presc_cnt := 0.U
    }.otherwise{
        presc_cnt := presc_cnt + 1.U
    }

    // We multiply by 10000 since we are 
    io.result := diff_val * 10000.S
}



// +--------------------------------------------------------------------+
// | PID Module
// |    - Given an error term, we get the PID control result on that error
// |    - This is to help simply the joint controller 
// |        cause we need PID for position and velocity
// +--------------------------------------------------------------------+
class PIDIO extends Bundle {
    val target = Input(SInt(64.W))
    val current = Input(SInt(64.W))

    val result = Output(SInt(64.W))
}

// Assuming a clock speed of 60MHz, we want our differentiator to run at 10kHz
class  PIDModule(Kp: Int = 4, Ki: Int = 0, Kd: Int = 0, shift: Int = 2) extends Module {
    val io = IO(new PIDIO())
    val error = Wire(SInt(64.W))

    // Wires for positional input terms
    val P_input = Wire(SInt(64.W))
    val I_input = Wire(SInt(64.W))
    val D_input = Wire(SInt(64.W))
    
    // Modules to get PID terms
    val error_integ = Module(new Integrator)        // error_integ - Integrator to get the I term
    val error_diff = Module(new Differentiator)     // error_diff - Differentiator to get the D term
    error_integ.io.value := error
    error_diff.io.value := error

     // Add pipeline stage(s) here if needed
    error := io.current - io.target
    P_input := (error * Kp.asSInt) >> shift.asUInt
    I_input := (error_integ.io.result * Ki.asSInt) >> shift.asUInt
    D_input := (error_diff.io.result * Kd.asSInt) >> shift.asUInt

    // Return back the PID result
    io.result := RegNext(P_input + I_input + D_input) 
}
