package frc.robot.arm;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.FeatureFlags;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.util.tuning.TunablePid;

public class ArmSubsystem extends StateMachine<ArmState> {
  private final TalonFX motor;
  private double motorAngle;
  private double motorCurrent;
  private double lowestSeenAngle = Double.MAX_VALUE;
  private double highestSeenAngle = Double.MIN_VALUE;
  private double collisionAvoidanceGoal;
  private static final double MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE = 120.0;
  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();

  private double averageMotorCurrent;
  private final LinearFilter linearFilter = LinearFilter.movingAverage(5);

  // private final MotionMagicVoltage motionMagicRequest =
  //     new MotionMagicVoltage(0.0).withEnableFOC(false);

  private final PositionVoltage pidRequest = new PositionVoltage(0).withEnableFOC(false);

  public ArmSubsystem(TalonFX motor) {
    super(
        SubsystemPriority.ARM,
        RobotConfig.IS_PRACTICE_BOT ? ArmState.PRE_MATCH_HOMING : ArmState.HOLDING_UPRIGHT);
    motor.getConfigurator().apply(RobotConfig.get().arm().motorConfig());

    this.motor = motor;

    // In field calibration mode, boot arm to lower hardstop angle
    if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      motor.setPosition(Units.degreesToRotations(RobotConfig.get().arm().homingPosition()));
    }

    TunablePid.of("Arm", motor, RobotConfig.get().arm().motorConfig());
  }

  public void setState(ArmState newState) {
    switch (getState()) {
      default -> {
        setStateFromRequest(newState);
      }
    }
  }

  public double getAngle() {
    return motorAngle;
  }

  public void setCollisionAvoidanceGoal(double angle) {
    collisionAvoidanceGoal = angle;
    DogLog.log("Arm/CollisionAvoidanceGoalAngle", collisionAvoidanceGoal);
  }

  public boolean atGoal() {
    return switch (getState()) {
      default -> MathUtil.isNear(getState().getAngle(), motorAngle, 2, -180, 180);
      case COLLISION_AVOIDANCE -> MathUtil.isNear(collisionAvoidanceGoal, motorAngle, 1, -180, 180);
      case PRE_MATCH_HOMING -> false;
    };
  }

  @Override
  protected void collectInputs() {
    motorAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    if (DriverStation.isDisabled()) {
      lowestSeenAngle = Math.min(lowestSeenAngle, motorAngle);
      highestSeenAngle = Math.max(highestSeenAngle, motorAngle);
    }

    motorCurrent = motor.getStatorCurrent().getValueAsDouble();
    averageMotorCurrent = linearFilter.calculate(motorCurrent);
  }

  @Override
  protected void afterTransition(ArmState newState) {
    switch (newState) {
      case COLLISION_AVOIDANCE -> {
        motor.setControl(pidRequest.withPosition(Units.degreesToRotations(collisionAvoidanceGoal)));
      }
      case PRE_MATCH_HOMING -> {
        motor.setControl(coastNeutralRequest);
      }
      default -> {
        motor.setControl(pidRequest.withPosition(Units.degreesToRotations(newState.getAngle())));
      }
    }
  }

  @Override
  protected ArmState getNextState(ArmState currentState) {
    if (currentState == ArmState.PRE_MATCH_HOMING) {
      if (rangeOfMotionGood()) {
        if (DriverStation.isEnabled()) {
          motor.setPosition(
              Units.degreesToRotations(
                  RobotConfig.get().arm().homingPosition() + (motorAngle - lowestSeenAngle)));

          return ArmState.HOLDING_UPRIGHT;
        } else {
          motor.setControl(brakeNeutralRequest);
        }
      }
    }

    return currentState;
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Arm/StatorCurrent", motorCurrent);
    DogLog.log("Arm/AverageStatorCurrent", averageMotorCurrent);
    DogLog.log("Arm/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Arm/NormalizedAngle", MathUtil.clamp(motorAngle, -180, 180));
    DogLog.log("Arm/RawAngle", motorAngle);
    DogLog.log("Arm/AtGoal", atGoal());
    if (DriverStation.isDisabled()) {
      DogLog.log("Arm/LowestAngle", lowestSeenAngle);
      DogLog.log("Arm/HighestAngle", highestSeenAngle);
    }
    if (rangeOfMotionGood()) {
      DogLog.clearFault("Arm not seen range of motion");
    } else if (RobotConfig.IS_PRACTICE_BOT) {
      DogLog.logFault("Arm not seen range of motion", AlertType.kWarning);
    }

    switch (getState()) {
      case COLLISION_AVOIDANCE -> {
        motor.setControl(pidRequest.withPosition(Units.degreesToRotations(collisionAvoidanceGoal)));
      }
      default -> {}
    }
  }

  public boolean rangeOfMotionGood() {
    return (highestSeenAngle - lowestSeenAngle) > MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE;
  }
}
