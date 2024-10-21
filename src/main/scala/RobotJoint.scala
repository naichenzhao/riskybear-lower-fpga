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
    withClockAndReset(clock, reset) {
      var regMap = Seq[(Int, Seq[freechips.rocketchip.regmapper.RegField])]()

      val regMaps = for (i <- 0 until params.channels) yield {
        val motor = Module(new MotorChiselModule(params.pwmWidth)) // PWM control motor module
        val encoder = Module(new QDECChiselModule(params.counterSize)) // Encoder module
        val controller = Module(new JointControllerChiselModule()) // Joint Controller module

        // State/target registers
        val joint_target_pos = RegInit(0.U(64.W))
        val joint_target_vel = RegInit(0.U(64.W))
        val joint_state = RegInit(0.U(1.W))

        // Motor MMIO registers
        val motor_en = RegInit(0.U(32.W))
        val motor_dir = RegInit(0.U(32.W))
        val motor_presc = RegInit(0.U(32.W))

        // Encoder MMIO registers
        val encoder_rst = Wire(DecoupledIO(Bool()))

        // Controller MMIO registers
        val min_speed = RegInit(850.U(32.W))
        val cutoff_speed = RegInit(2.U(32.W))

        
        // Pass through velocity/positon values
        val pos_out = RegInit(0.U(64.W))
        val vel_out = RegInit(0.U(64.W))
        pos_out := controller.io.pos_out.asUInt
        vel_out := controller.io.vel_out.asUInt

        // Motor passthrough values
        motor.io.en := motor_en.orR
        motor.io.dir := motor_dir.orR
        motor.io.presc := motor_presc

        // Encoder passsthrough values
        encoder.io.clock := clock
        encoder.io.reset := reset.asBool
        encoder.io.center := encoder_rst.valid
        encoder_rst.ready:= 1.U

        // joint controller passthrough values
        controller.io.clock := clock
        controller.io.reset := reset.asBool
        controller.io.target_pos := joint_target_pos.asSInt
        controller.io.target_vel := joint_target_vel.asSInt
        controller.io.curr_pos := encoder.io.count.asSInt

        controller.io.min_speed := min_speed.asSInt
        controller.io.cutoff_speed := cutoff_speed.asSInt

        // Input selector - Select motor mode
        //    - 0: Feedback position control
        //    - 1: Feedback velocity control
        //    - 2: Feed-forward velocity control
        when(joint_state === 1.U) {
          motor.io.speed_in := controller.io.pos_target(15, 0).asSInt
        }.elsewhen(joint_state === 2.U) {
          motor.io.speed_in := controller.io.vel_target(15, 0).asSInt
        }.otherwise{
          motor.io.speed_in := joint_target_vel.asSInt
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
            RegField(32, joint_target_pos, RegFieldDesc(s"ch${i}_target_pos", s"Channel ${i} joint set target position"))),
          (0x04 + (i*MMIO_SHIFT)) -> Seq(
            RegField(32, joint_target_vel, RegFieldDesc(s"ch${i}_target_vel", s"Channel ${i} joint set target velocity"))),
          (0x08 + (i*MMIO_SHIFT)) -> Seq(
            RegField(32, joint_state, RegFieldDesc(s"ch${i}_joint_mode", s"Channel ${i} joint mode"))),
          
          (0x0C + (i*MMIO_SHIFT)) -> Seq(
            RegField(32, motor_en, RegFieldDesc(s"ch${i}_joint_en", s"Channel ${i} joint enable pin"))),
          (0x10 + (i*MMIO_SHIFT)) -> Seq(
            RegField(32, motor_dir, RegFieldDesc(s"ch${i}_joint_dir", s"Channel ${i} joinr implicit direction pin"))),
          (0x14 + (i*MMIO_SHIFT)) -> Seq(
            RegField(32, motor_presc, RegFieldDesc(s"ch${i}_motor_presc", s"Channel ${i} motor PWM prescaler"))),

          // QDEC MMIO
          (0x20 + (i*MMIO_SHIFT)) -> Seq(
            RegField.r(64, pos_out, RegFieldDesc(s"ch${i}_joint_pos", s"Channel ${i} qdec read count"))),
          (0x28 + (i*MMIO_SHIFT)) -> Seq(
            RegField.r(64, vel_out, RegFieldDesc(s"ch${i}_joint_vel", s"Channel ${i} qdec read count"))),
          (0x30 + (i*MMIO_SHIFT)) -> Seq(
            RegField.w(1, encoder_rst, RegFieldDesc(s"ch${i}_qdec_rst", s"Channel ${i} qdec reset pin"))),

          // Joint controller MMIO
          (0x34 + (i*MMIO_SHIFT)) -> Seq(
            RegField.r(32, min_speed, RegFieldDesc(s"ch${i}_controller_min_speed", s"Channel ${i} controller minimum motor speed"))),
          (0x38 + (i*MMIO_SHIFT)) -> Seq(
            RegField.w(32, cutoff_speed, RegFieldDesc(s"ch${i}_controller_cutoff_speed", s"Channel ${i} controller cutoff motor speed")))

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


class WithRobotJoint(address: BigInt = 0x13000000, channels: Int = 8, pwmWidth: Int = 13,  counterSize:Int = 32) extends Config((site, here, up) => {
  case RobotJointKey => Some(RobotJointParams(address = address, pwmWidth=pwmWidth, channels=channels, counterSize=counterSize))
})