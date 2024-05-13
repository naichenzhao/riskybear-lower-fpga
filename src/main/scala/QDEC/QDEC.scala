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

case class QDECParams(
  address: BigInt,
  counterSize: Int)

case object QDECKey extends Field[Option[QDECParams]](None)

class QDECIO() extends Bundle {
  val gpio_a = Input(Bool())
  val gpio_b = Input(Bool())
}

trait HasQDECIO {
  def io: QDECIO
}

class QDECChiselIO(val w: Int) extends Bundle {
  val clock = Input(Clock())
  val reset = Input(Bool())
  val center = Input(Bool())

  val gpio_a = Input(Bool())
  val gpio_b = Input(Bool())

  val count = Output(UInt(w.W))
}


class QDECMMIOChiselModule(val width: Int) extends Module {
  val io = IO(new QDECChiselIO(width))

  // Define counter register
  val enc_cnt = RegInit(0.U(width.W))
  io.count := enc_cnt

  // Use the synchronizer
  val a_sync = SynchronizerShiftReg(io.gpio_a, 2, Some("syncreg_a"))
  val b_sync = SynchronizerShiftReg(io.gpio_b, 2, Some("syncreg_b"))

  val a_prev_val = RegNext(a_sync)
  val a_trig = Wire(Bool())

  a_trig  := ~a_prev_val & a_sync
  when(io.center) {
    enc_cnt := 0.U
  }.otherwise{
    enc_cnt := Mux(a_trig & b_sync, enc_cnt+1.U, enc_cnt-1.U)
  }
}


class QDECTL(params: QDECParams, beatBytes: Int)(implicit p: Parameters) extends ClockSinkDomain(ClockSinkParameters())(p) {
  val device = new SimpleDevice("qdec", Seq("ucbbar, qdec")) 
  val node = TLRegisterNode(Seq(AddressSet(params.address, 4096-1)), device, "reg/control", beatBytes=beatBytes)

  override lazy val module = new QDECImpl
  class QDECImpl extends Impl with HasQDECIO {
    val io = IO(new QDECIO)
    withClockAndReset(clock, reset) {
      val impl = Module(new QDECMMIOChiselModule(params.counterSize))
      val rst_qdec = Wire(DecoupledIO(Bool()))

      impl.io.clock := clock
      impl.io.reset := reset.asBool
      impl.io.center := rst_qdec.valid
      rst_qdec.ready:= 1.U

      impl.io.gpio_a := io.gpio_a
      impl.io.gpio_b := io.gpio_b

      node.regmap(
      0x00 -> Seq(
        RegField.w(1, rst_qdec)),
      0x04 -> Seq(
        RegField.r(params.counterSize, impl.io.count))
      )
    }
  }



}


trait CanHavePeripheryQDEC { this: BaseSubsystem =>
  private val portName = "qdec"
  private val pbus = locateTLBusWrapper(PBUS)

  val qdecio = p(QDECKey) match {
    case Some(params) => {
      val qdec = LazyModule(new QDECTL(params, pbus.beatBytes)(p))
      qdec.clockNode := pbus.fixedClockNode
      pbus.coupleTo(portName) { qdec.node := TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ }

      val qdecio = InModuleBody {
        val qdecio = IO(new QDECIO).suggestName("qdec_io")
        qdec.module.io := qdecio
        qdecio
      }
      Some(qdecio)
    }
    case None => None
  }
}


class WithQDEC(address: BigInt = 0x9000, counterSize: Int = 32) extends Config((site, here, up) => {
  case QDECKey => Some(QDECParams(address = address, counterSize=counterSize))
})



