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

case class QDECParams(
  address: BigInt,
  counterSize: Int, 
  channels: Int)

case object QDECKey extends Field[Option[QDECParams]](None)

class QDECPortIO() extends Bundle {
    val gpio_a = Input(Bool())
    val gpio_b = Input(Bool())
}

class QDECTopIO(channels: Int) extends Bundle {
  val port = Vec(channels, new QDECPortIO())
}

trait HasQDECTopIO {
  def io: QDECTopIO
}


class QDECTL(params: QDECParams, beatBytes: Int)(implicit p: Parameters) extends ClockSinkDomain(ClockSinkParameters())(p) {
  val device = new SimpleDevice("qdec", Seq("ucbbar, qdec")) 
  val node = TLRegisterNode(Seq(AddressSet(params.address, 4096-1)), device, "reg/control", beatBytes=beatBytes)

  override lazy val module = new QDECImpl
  class QDECImpl extends Impl with HasQDECTopIO {
    val io = IO(new QDECTopIO(params.channels))
    withClockAndReset(clock, reset) {
      var regMap = Seq[(Int, Seq[freechips.rocketchip.regmapper.RegField])]()

      val regMaps = for (i <- 0 until params.channels) yield {
        val impl = Module(new QDECChiselModule(params.counterSize))
        val rst_qdec = Wire(DecoupledIO(Bool()))

        impl.io.clock := clock
        impl.io.reset := reset.asBool
        impl.io.center := rst_qdec.valid
        rst_qdec.ready:= 1.U

        impl.io.gpio_a := io.port(i).gpio_a
        impl.io.gpio_b := io.port(i).gpio_b

        regMap = regMap ++ Seq(
          (0x00 + (i*0x10)) -> Seq(
            RegField.r(params.counterSize, impl.io.count, RegFieldDesc(s"ch${i}_qdec_cnt", s"Channel ${i} qdec read count"))),
          (0x08 + (i*0x10)) -> Seq(
            RegField.w(1, rst_qdec, RegFieldDesc(s"ch${i}_qdec_rst", s"Channel ${i} qdec reset pin"))),
        )

      }
      node.regmap(regMap:_*)
    }
  }
}


trait CanHavePeripheryQDEC { this: BaseSubsystem =>
  private val portName = "qdec"
  private val pbus = locateTLBusWrapper(PBUS)

  val qdec_out = p(QDECKey) match {
    case Some(params) => {
      val qdec = LazyModule(new QDECTL(params, pbus.beatBytes)(p))
      qdec.clockNode := pbus.fixedClockNode
      pbus.coupleTo(portName) { qdec.node := TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ }

      val qdec_out = InModuleBody {
        val out = IO(Vec(params.channels, new QDECPortIO()).suggestName("qdec_port"))
        (out zip qdec.module.io.port) foreach {case (t, m) =>
          t :<>= m
        }
        out
      }
      Some(qdec_out)
    }
    case None => None
  }
}


class WithQDEC(address: BigInt = 0x9000, counterSize: Int = 64, channels: Int = 8) extends Config((site, here, up) => {
  case QDECKey => Some(QDECParams(address = address, counterSize=counterSize, channels=channels))
})