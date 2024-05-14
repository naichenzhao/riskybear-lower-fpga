package riskybear

import chisel3._
import chisel3.util._
import chisel3.experimental.{IntParam, BaseModule}
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.prci._
import freechips.rocketchip.subsystem.{BaseSubsystem, PBUS}
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{HasRegMap, RegField}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.UIntIsOneOf
import freechips.rocketchip.util.{AsyncQueue, AsyncQueueParams}
import freechips.rocketchip.util.{SynchronizerShiftReg}

import riskybear._


class QDECChiselIO(val w: Int) extends Bundle {
  val clock = Input(Clock())
  val reset = Input(Bool())
  val center = Input(Bool())

  val gpio_a = Input(Bool())
  val gpio_b = Input(Bool())

  val count = Output(UInt(w.W))
}


class QDECChiselModule(val width: Int) extends Module {
  val io = IO(new QDECChiselIO(width))

  // Define counter register
  val enc_cnt = RegInit(0.U(width.W))
  io.count := enc_cnt

  // Use the synchronizer
  val A_sync = SynchronizerShiftReg(io.gpio_a, 2, Some("syncreg_a"))
  val B_sync = SynchronizerShiftReg(io.gpio_b, 2, Some("syncreg_b"))

  val A_prev = RegNext(A_sync)
  val B_prev = RegNext(B_sync)

  val A_rise = Wire(Bool())
  val A_fall = Wire(Bool())
  val B_rise = Wire(Bool())
  val B_fall = Wire(Bool())

  A_rise := ~A_prev && A_sync
  A_fall := A_prev && ~A_sync
  B_rise := ~B_prev && B_sync
  B_fall := B_prev && ~B_sync

  when(io.center) {
    enc_cnt := 0.U
  }.otherwise{
    when(A_rise){
      enc_cnt := Mux(B_sync, enc_cnt-1.U, enc_cnt+1.U)
    }.elsewhen(A_fall){
      enc_cnt := Mux(B_sync, enc_cnt+1.U, enc_cnt-1.U)
    }.elsewhen(B_rise){
      enc_cnt := Mux(A_sync, enc_cnt+1.U, enc_cnt-1.U)
    }.elsewhen(B_fall){
      enc_cnt := Mux(A_sync, enc_cnt-1.U, enc_cnt+1.U)
    }
  }
}




