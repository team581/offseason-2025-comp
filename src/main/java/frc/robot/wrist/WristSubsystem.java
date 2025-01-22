package frc.robot.wrist;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
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
  private double collisionAvoidanceGoal;
  private static final double MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE = 90.0;
  private final StaticBrake brakeNeutralRequest = new StaticBrake();

  private final PositionVoltage motionMagicRequest =
      new PositionVoltage(0).withEnableFOC(false).withOverrideBrakeDurNeutral(true);

  // private final PositionVoltage pidRequest =
  //  new PositionVoltage(0).withEnableFOC(false).withOverrideBrakeDurNeutral(true);

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

  public double getAngle() {
    return motorAngle;
  }

  public void setCollisionAvoidanceGoal(double angle) {
    collisionAvoidanceGoal = angle;
    DogLog.log("Wrist/CollisionAvoidanceGoalAngle", collisionAvoidanceGoal);
  }

  public boolean atGoal() {
    return switch (getState()) {
      default -> MathUtil.isNear(getState().angle, motorAngle, 1);
      case COLLISION_AVOIDANCE -> MathUtil.isNear(collisionAvoidanceGoal, motorAngle, 1);
      case PRE_MATCH_HOMING -> true;
    };
  }

  @Override
  protected void collectInputs() {
    motorAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    if (DriverStation.isDisabled()) {
      lowestSeenAngle = Math.min(lowestSeenAngle, motorAngle);
      highestSeenAngle = Math.max(highestSeenAngle, motorAngle);
    }
  }

  @Override
  protected void afterTransition(WristState newState) {
    switch (newState) {
      default -> {
        motor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(clamp(newState.angle))));
      }
      case COLLISION_AVOIDANCE -> {
        motor.setControl(
            motionMagicRequest.withPosition(
                Units.degreesToRotations(clamp(collisionAvoidanceGoal))));
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Wrist/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Wrist/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Wrist/Position", motorAngle);
    if (DriverStation.isDisabled()) {
      DogLog.log("Wrist/LowestAngle", lowestSeenAngle);
      DogLog.log("Wrist/HighestAngle", highestSeenAngle);
    }

    switch (getState()) {
      case PRE_MATCH_HOMING -> {
        if ((highestSeenAngle - lowestSeenAngle) > MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE) {
          if (DriverStation.isEnabled()) {
            motor.setControl(
                motionMagicRequest.withPosition(
                    Units.degreesToRotations(
                        RobotConfig.get().wrist().minAngle() + (motorAngle - lowestSeenAngle))));

            setStateFromRequest(WristState.IDLE);
          } else {
            motor.setControl(brakeNeutralRequest);
          }
        }
      }
      case COLLISION_AVOIDANCE -> {
        motor.setControl(
            motionMagicRequest.withPosition(
                Units.degreesToRotations(clamp(collisionAvoidanceGoal))));
      }

      default -> {}
    }
  }

  private static double clamp(double armAngle) {
    return MathUtil.clamp(
        armAngle, RobotConfig.get().wrist().minAngle(), RobotConfig.get().wrist().maxAngle());
  }
}
