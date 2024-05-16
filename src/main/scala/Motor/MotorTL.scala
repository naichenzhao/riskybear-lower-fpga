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

case class MotorParams(
  address: BigInt,
  pwmWidth: Int, 
  channels: Int)

case object MotorKey extends Field[Option[MotorParams]](None)

class MotorPortIO() extends Bundle {
    val motor_out_a = Output(Bool())
    val motor_out_b = Output(Bool())
}

class MotorTopIO(channels: Int) extends Bundle {
  val port = Vec(channels, new MotorPortIO())
}

trait HasMotorTopIO {
  def io: MotorTopIO
}

class MotorTL(params: MotorParams, beatBytes: Int)(implicit p: Parameters) extends ClockSinkDomain(ClockSinkParameters())(p) {
  val device = new SimpleDevice("Motor", Seq("ucbbar, Motor")) 
  val node = TLRegisterNode(Seq(AddressSet(params.address, 4096-1)), device, "reg/control", beatBytes=beatBytes)

  override lazy val module = new MotorImpl
  class MotorImpl extends Impl with HasMotorTopIO {
    val io = IO(new MotorTopIO(params.channels))
    val max_PWM = (1.S << (params.pwmWidth-1)) - 1.S
    withClockAndReset(clock, reset) {
      var regMap = Seq[(Int, Seq[freechips.rocketchip.regmapper.RegField])]()

      val regMaps = for (i <- 0 until params.channels) yield {
        val impl = Module(new MotorChiselModule(params.pwmWidth))
        val motor_en = RegInit(0.U(1.W))
        val motor_dir = RegInit(0.U(1.W))
        val motor_presc = RegInit(0.U(32.W))
        val motor_speed = RegInit(0.U(16.W))

        impl.io.en := motor_en
        impl.io.dir := motor_dir
        impl.io.presc := motor_presc

        // Input motor speed - We also check for overflow and underflow
        when(motor_speed.asSInt > max_PWM){
          impl.io.speed_in := max_PWM
        }.elsewhen(motor_speed.asSInt < -max_PWM){
          impl.io.speed_in := -max_PWM
        }.otherwise{
          impl.io.speed_in := motor_speed(12, 0).asSInt
        }
        

        io.port(i).motor_out_a := impl.io.motor_out_a
        io.port(i).motor_out_b := impl.io.motor_out_b

        regMap = regMap ++ Seq(
          (0x00 + (i*0x10)) -> Seq(
            RegField.w(16, motor_speed, RegFieldDesc(s"ch${i}_motor_speed", s"Channel ${i} motor set speed"))),
          (0x04 + (i*0x10)) -> Seq(
            RegField.w(32, motor_presc, RegFieldDesc(s"ch${i}_motor_presc", s"Channel ${i} motor PWM prescaler"))),
          (0x08 + (i*0x10)) -> Seq(
            RegField.w(1, motor_en, RegFieldDesc(s"ch${i}_motor_en", s"Channel ${i} motor enable pin"))),
          (0x09 + (i*0x10)) -> Seq(
            RegField.w(1, motor_dir, RegFieldDesc(s"ch${i}_motor_dir", s"Channel ${i} motor implicit direction pin"))),
        )
      }
      node.regmap(regMap:_*)
    }



  }
}



trait CanHavePeripheryMotor { this: BaseSubsystem =>
  private val portName = "motor"
  private val pbus = locateTLBusWrapper(PBUS)

  val motor_out = p(MotorKey) match {
    case Some(params) => {
      val motor = LazyModule(new MotorTL(params, pbus.beatBytes)(p))
      motor.clockNode := pbus.fixedClockNode
      pbus.coupleTo(portName) { motor.node := TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ }

      val motor_out = InModuleBody {
        val out = IO(Vec(params.channels, new MotorPortIO()).suggestName("motor_port"))
        (out zip motor.module.io.port) foreach {case (t, m) =>
          t :<>= m
        }
        out
      }
      Some(motor_out)
    }
    case None => None
  }
}


class WithMotor(address: BigInt = 0x9000, pwmWidth: Int = 13, channels: Int = 8) extends Config((site, here, up) => {
  case MotorKey => Some(MotorParams(address = address, pwmWidth=pwmWidth, channels=channels))
})