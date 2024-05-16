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

case class RobotJointParams(
  address: BigInt,
  channels: Int,
  
  pwmWidth: Int,
  counterSize: Int)

case object RobotJointKey extends Field[Option[RobotJointParams]](None)

class RobotJointPortIO() extends Bundle {
  // Output for each motor
  val motor_out_a = Output(Bool())
  val motor_out_b = Output(Bool())

  // Input for QDEC
  val qdec_a = Input(Bool())
  val qdec_b = Input(Bool())
}

class RobotJointTopIO(channels: Int) extends Bundle {
  val port = Vec(channels, new RobotJointPortIO())
}

trait HasRobotJointTopIO {
  def io: RobotJointTopIO
}

class RobotJointTL(params: RobotJointParams, beatBytes: Int)(implicit p: Parameters) extends ClockSinkDomain(ClockSinkParameters())(p) {
  val device = new SimpleDevice("RobotJoint", Seq("ucbbar, RobotJoint")) 
  val node = TLRegisterNode(Seq(AddressSet(params.address, 4096-1)), device, "reg/control", beatBytes=beatBytes)

  override lazy val module = new RobotJointImpl
  class RobotJointImpl extends Impl with HasRobotJointTopIO {
    val io = IO(new RobotJointTopIO(params.channels))
    val max_PWM = (1.S << (params.pwmWidth-1)) - 1.S
    withClockAndReset(clock, reset) {
      var regMap = Seq[(Int, Seq[freechips.rocketchip.regmapper.RegField])]()

      val regMaps = for (i <- 0 until params.channels) yield {
        val motor = Module(new MotorChiselModule(params.pwmWidth)) // PWM control motor module
        val encoder = Module(new QDECChiselModule(params.counterSize)) // Encoder module
        val PID = Module(new PIDControllerModule()) // PID Controller module

        // State wires
        //  State 0: Speed control
        //  State 1: Positional control
        val joint_state = RegInit(0.U(1.W))
        val joint_target = RegInit(0.U(64.W))
        val joint_speed = RegInit(0.S(16.W))

        // Motor MMIO registers
        val motor_en = RegInit(0.U(1.W))
        val motor_dir = RegInit(0.U(1.W))
        val motor_presc = RegInit(0.U(32.W))
        val motor_speed = RegInit(0.U(16.W))

        // Encoder MMIO registers
        val encoder_rst = Wire(DecoupledIO(Bool()))

         // PID MMIO registers
        val PID_Kp = RegInit(0.U(64.W))
        val PID_Ki = RegInit(0.U(64.W))
        val PID_Kd = RegInit(2.U(64.W))
        val PID_shift = RegInit(3.U(32.W))

        // Motor passthrough values
        motor.io.en := motor_en
        motor.io.dir := motor_dir
        motor.io.presc := motor_presc

        // Encoder passsthrough values
        encoder.io.clock := clock
        encoder.io.reset := reset.asBool
        encoder.io.center := encoder_rst.valid
        encoder_rst.ready:= 1.U

        // PID passthrough values
        PID.io.target_pos := joint_target.asSInt
        PID.io.curr_pos := encoder.io.count.asSInt

        // PID.io.Kd := PID_Kd.asSInt
        // PID.io.Kp := PID_Kp.asSInt
        // PID.io.Ki := PID_Ki.asSInt
        // PID.io.shift := PID_shift

        // Input selector - Select whether or not we want to do speed or PID control
        when(joint_state.asBool) {
          joint_speed := PID.io.pid_out(15, 0).asSInt
        }.otherwise{
          joint_speed := motor_speed.asSInt
        }
        
        // Input motor speed - We also check for overflow and underflow
        when(joint_speed > max_PWM){
          motor.io.speed_in := max_PWM
        }.elsewhen(joint_speed < -max_PWM){
          motor.io.speed_in := -max_PWM
        }.otherwise{
          motor.io.speed_in := joint_speed(12, 0).asSInt
        }
        
        // Set any external-facing wires
        io.port(i).motor_out_a := motor.io.motor_out_a
        io.port(i).motor_out_b := motor.io.motor_out_b
        encoder.io.gpio_a := io.port(i).qdec_a
        encoder.io.gpio_b := io.port(i).qdec_b

        val MMIO_SHIFT = 0x100

        regMap = regMap ++ Seq(
          // Control MMIO
          (0x00 + (i*MMIO_SHIFT)) -> Seq(
            RegField.w(64, joint_target, RegFieldDesc(s"ch${i}_joint_target", s"Channel ${i} joint set target position"))),
          (0x08 + (i*MMIO_SHIFT)) -> Seq(
            RegField.w(1, joint_state, RegFieldDesc(s"ch${i}_joint_mode", s"Channel ${i} joint mode"))),
          (0x09 + (i*MMIO_SHIFT)) -> Seq(
            RegField.w(1, motor_en, RegFieldDesc(s"ch${i}_joint_en", s"Channel ${i} joint enable pin"))),
          (0x0A + (i*MMIO_SHIFT)) -> Seq(
            RegField.w(1, motor_dir, RegFieldDesc(s"ch${i}_joint_dir", s"Channel ${i} joinr implicit direction pin"))),
          
          // Motor MMIO
          (0x0C + (i*MMIO_SHIFT)) -> Seq(
            RegField.w(16, motor_speed, RegFieldDesc(s"ch${i}_motor_speed", s"Channel ${i} motor set speed"))),
          (0x10 + (i*MMIO_SHIFT)) -> Seq(
            RegField.w(32, motor_presc, RegFieldDesc(s"ch${i}_motor_presc", s"Channel ${i} motor PWM prescaler"))),

          // QDEC MMIO
          (0x18 + (i*MMIO_SHIFT)) -> Seq(
            RegField.r(params.counterSize, encoder.io.count, RegFieldDesc(s"ch${i}_qdec_cnt", s"Channel ${i} qdec read count"))),
          (0x20 + (i*MMIO_SHIFT)) -> Seq(
            RegField.w(1, encoder_rst, RegFieldDesc(s"ch${i}_qdec_rst", s"Channel ${i} qdec reset pin"))),

          // // PID MMIO
          // (0x28 + (i*MMIO_SHIFT)) -> Seq(
          //   RegField.r(64, PID_Kp, RegFieldDesc(s"ch${i}_PID_Kp", s"Channel ${i} PID proportional constant"))),
          // (0x30 + (i*MMIO_SHIFT)) -> Seq(
          //   RegField.w(64, PID_Ki,RegFieldDesc(s"ch${i}_PID_Ki", s"Channel ${i} PID integral constant"))),
          // (0x38 + (i*MMIO_SHIFT)) -> Seq(
          //   RegField.r(64, PID_Kd, RegFieldDesc(s"ch${i}_PID_Kd", s"Channel ${i} PID derivative constant"))),
          // (0x40 + (i*MMIO_SHIFT)) -> Seq(
          //   RegField.w(64, PID_shift, RegFieldDesc(s"ch${i}_PID_shift", s"Channel ${i} PID shift variable"))),

        )
      }
      node.regmap(regMap:_*)
    }
  }
}


trait CanHavePeripheryRobotJoint { this: BaseSubsystem =>
  private val portName = "RobotJoint"
  private val pbus = locateTLBusWrapper(PBUS)

  val joint_out = p(RobotJointKey) match {
    case Some(params) => {
      val joint = LazyModule(new RobotJointTL(params, pbus.beatBytes)(p))
      joint.clockNode := pbus.fixedClockNode
      pbus.coupleTo(portName) { joint.node := TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ }

      val joint_out = InModuleBody {
        val out = IO(Vec(params.channels, new RobotJointPortIO()).suggestName("joint_port"))
        (out zip joint.module.io.port) foreach {case (t, m) =>
          t :<>= m
        }
        out
      }
      Some(joint_out)
    }
    case None => None
  }
}


class WithRobotJoint(address: BigInt = 0x13000000, channels: Int = 8, pwmWidth: Int = 13,  counterSize:Int = 64) extends Config((site, here, up) => {
  case RobotJointKey => Some(RobotJointParams(address = address, pwmWidth=pwmWidth, channels=channels, counterSize=counterSize))
})