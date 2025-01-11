package frc.robot.wrist;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class WristSubsystem extends StateMachine<WristState> {
  private final TalonFX motor;
  private double motorAngle;
  private double lowestSeenAngle = Double.MAX_VALUE;
  private double highestSeenAngle = Double.MIN_VALUE;
  private static final double MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE = 90.0;
    private final StaticBrake brakeNeutralRequest = new StaticBrake();


  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withEnableFOC(false);
  //private final PositionVoltage pidRequest =
    //  new PositionVoltage(0).withEnableFOC(false);


  public WristSubsystem(TalonFX motor) {
    super(SubsystemPriority.WRIST, WristState.PRE_MATCH_HOMING);
    motor.getConfigurator().apply(RobotConfig.get().wrist().motorConfig());

    this.motor = motor;
  }

  public void setState(WristState newState) {
    if (getState() != WristState.PRE_MATCH_HOMING) {

      setStateFromRequest(newState);
    }
  }

  public boolean atGoal() {
    return switch (getState()) {
      case ALGAE_BACKWARD_NET -> MathUtil.isNear(WristState.ALGAE_BACKWARD_NET.angle, motorAngle, 1);
      case ALGAE_FORWARD_NET -> MathUtil.isNear(WristState.ALGAE_FORWARD_NET.angle, motorAngle, 1);
      case ALGAE_PROCESSOR -> MathUtil.isNear(WristState.ALGAE_PROCESSOR.angle, motorAngle, 1);
      case CORAL_SCORE_LV1 -> MathUtil.isNear(WristState.CORAL_SCORE_LV1.angle, motorAngle, 1);
      case CORAL_SCORE_LV2 -> MathUtil.isNear(WristState.CORAL_SCORE_LV2.angle, motorAngle, 1);
      case CORAL_SCORE_LV3 -> MathUtil.isNear(WristState.CORAL_SCORE_LV3.angle, motorAngle, 1);
      case CORAL_SCORE_LV4 -> MathUtil.isNear(WristState.CORAL_SCORE_LV4.angle, motorAngle, 1);
      case GROUND_ALGAE_INTAKE -> MathUtil.isNear(WristState.GROUND_ALGAE_INTAKE.angle, motorAngle, 1);
      case GROUND_CORAL_INTAKE -> MathUtil.isNear(WristState.GROUND_CORAL_INTAKE.angle, motorAngle, 1);
      case IDLE -> MathUtil.isNear(WristState.IDLE.angle, motorAngle, 1);

      case PRE_MATCH_HOMING -> true;
      case SOURCE_INTAKE -> MathUtil.isNear(WristState.SOURCE_INTAKE.angle, motorAngle, 1);
      case UNJAM -> MathUtil.isNear(WristState.UNJAM.angle, motorAngle, 1);
      default -> false;
    };
  }

  @Override
  protected void collectInputs() {
    motorAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    if (DriverStation.isDisabled()) {
      lowestSeenAngle = Math.min(lowestSeenAngle, motorAngle);
      highestSeenAngle = Math.max(highestSeenAngle,motorAngle);
    }
  }

  @Override
  protected void afterTransition(WristState newState) {
    switch (newState) {
      case ALGAE_BACKWARD_NET -> {
        motor.setControl(
            motionMagicRequest.withPosition(
                Units.degreesToRotations(clamp(WristState.ALGAE_BACKWARD_NET.angle))));
      }

      case ALGAE_FORWARD_NET -> {
        motor.setControl(
            motionMagicRequest.withPosition(
                Units.degreesToRotations(clamp(WristState.ALGAE_FORWARD_NET.angle))));
      }
      case ALGAE_PROCESSOR -> {
        motor.setControl(
            motionMagicRequest.withPosition(
                Units.degreesToRotations(clamp(WristState.ALGAE_PROCESSOR.angle))));
      }
      case CORAL_SCORE_LV1 -> {
        motor.setControl(
            motionMagicRequest.withPosition(
                Units.degreesToRotations(clamp(WristState.CORAL_SCORE_LV1.angle))));
      }
      case CORAL_SCORE_LV2 -> {
        motor.setControl(
            motionMagicRequest.withPosition(
                Units.degreesToRotations(clamp(WristState.CORAL_SCORE_LV2.angle))));
      }
      case CORAL_SCORE_LV3 -> {
        motor.setControl(
            motionMagicRequest.withPosition(
                Units.degreesToRotations(clamp(WristState.CORAL_SCORE_LV3.angle))));
      }
      case CORAL_SCORE_LV4 -> {
        motor.setControl(
            motionMagicRequest.withPosition(
                Units.degreesToRotations(clamp(WristState.CORAL_SCORE_LV4.angle))));
      }
      case GROUND_ALGAE_INTAKE -> {
        motor.setControl(
            motionMagicRequest.withPosition(
                Units.degreesToRotations(clamp(WristState.GROUND_ALGAE_INTAKE.angle))));
      }
      case GROUND_CORAL_INTAKE -> {
        motor.setControl(
            motionMagicRequest.withPosition(
                Units.degreesToRotations(clamp(WristState.GROUND_CORAL_INTAKE.angle))));
      }
      case IDLE -> {
        motor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(clamp(WristState.IDLE.angle))));

      }
      case SOURCE_INTAKE -> {
        motor.setControl(
            motionMagicRequest.withPosition(
                Units.degreesToRotations(clamp(WristState.SOURCE_INTAKE.angle))));
      }
      case UNJAM -> {
        motor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(clamp(WristState.UNJAM.angle))));

      }
      default -> {}
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();


    if (getState() == WristState.PRE_MATCH_HOMING && (highestSeenAngle-lowestSeenAngle)>MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE) {
      if (DriverStation.isEnabled()) {
        motor.setPosition(

          Units.degreesToRotations(
              RobotConfig.get().wrist().minAngle() + (motorAngle - lowestSeenAngle)));

      setStateFromRequest(WristState.IDLE);
      } else {
        motor.setControl(brakeNeutralRequest);
      }
    }



  }

  private static double clamp(double armAngle) {
    return MathUtil.clamp(
        armAngle, RobotConfig.get().wrist().minAngle(), RobotConfig.get().wrist().maxAngle());
  }
}
