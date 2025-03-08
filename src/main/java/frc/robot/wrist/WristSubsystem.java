package frc.robot.wrist;

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

public class WristSubsystem extends StateMachine<WristState> {
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
  private LinearFilter linearFilter = LinearFilter.movingAverage(5);

  // private final MotionMagicVoltage motionMagicRequest =
  //     new MotionMagicVoltage(0.0).withEnableFOC(false);

  private final PositionVoltage pidRequest = new PositionVoltage(0).withEnableFOC(false);

  public WristSubsystem(TalonFX motor) {
    super(SubsystemPriority.WRIST, WristState.PRE_MATCH_HOMING);
    motor.getConfigurator().apply(RobotConfig.get().wrist().motorConfig());

    this.motor = motor;

    // In field calibration mode, boot wrist to lower hardstop angle
    if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      motor.setPosition(Units.degreesToRotations(RobotConfig.get().wrist().homingPosition()));
    }
  }

  public void setState(WristState newState) {
    switch (getState()) {
      case PRE_MATCH_HOMING -> {
        if (newState == WristState.MID_MATCH_HOMING) {
          setStateFromRequest(newState);
        }
      }
      case MID_MATCH_HOMING -> {}
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
    DogLog.log("Wrist/CollisionAvoidanceGoalAngle", collisionAvoidanceGoal);
  }

  public boolean atGoal() {
    return switch (getState()) {
      default -> MathUtil.isNear(getState().angle, motorAngle, 2);
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

    motorCurrent = motor.getStatorCurrent().getValueAsDouble();
    averageMotorCurrent = linearFilter.calculate(motorCurrent);
  }

  @Override
  protected void afterTransition(WristState newState) {
    switch (newState) {
      case MID_MATCH_HOMING -> {
        motor.setVoltage(-0.5);
      }
      case COLLISION_AVOIDANCE -> {
        motor.setControl(
            pidRequest.withPosition(Units.degreesToRotations(clamp(collisionAvoidanceGoal))));
      }
      case PRE_MATCH_HOMING -> {
        motor.setControl(coastNeutralRequest);
      }
      default -> {
        motor.setControl(pidRequest.withPosition(Units.degreesToRotations(clamp(newState.angle))));
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Wrist/StatorCurrent", motorCurrent);
    DogLog.log("Wrist/AverageStatorCurrent", averageMotorCurrent);
    DogLog.log("Wrist/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Wrist/Angle", motorAngle);
    DogLog.log("Wrist/AtGoal", atGoal());
    if (DriverStation.isDisabled()) {
      DogLog.log("Wrist/LowestAngle", lowestSeenAngle);
      DogLog.log("Wrist/HighestAngle", highestSeenAngle);
    }
    if (rangeOfMotionGood()) {
      DogLog.clearFault("Wrist not seen range of motion");
    } else {
      DogLog.logFault("Wrist not seen range of motion", AlertType.kWarning);
    }

    switch (getState()) {
      case PRE_MATCH_HOMING -> {
        if (rangeOfMotionGood()) {
          if (DriverStation.isEnabled()) {
            motor.setPosition(
                Units.degreesToRotations(
                    RobotConfig.get().wrist().minAngle() + (motorAngle - lowestSeenAngle)));

            setStateFromRequest(WristState.CORAL_STOWED);
          } else {
            motor.setControl(brakeNeutralRequest);
          }
        }
      }
      case COLLISION_AVOIDANCE -> {
        motor.setControl(
            pidRequest.withPosition(Units.degreesToRotations(clamp(collisionAvoidanceGoal))));
      }
      default -> {}
    }
  }

  @Override
  protected WristState getNextState(WristState currentState) {
    if (currentState == WristState.MID_MATCH_HOMING
        && averageMotorCurrent > RobotConfig.get().wrist().homingCurrentThreshold()) {
      motor.setPosition(Units.degreesToRotations(RobotConfig.get().wrist().homingPosition()));
      return WristState.CORAL_STOWED;
    }

    // Don't do anything
    return currentState;
  }

  public boolean rangeOfMotionGood() {
    return (highestSeenAngle - lowestSeenAngle) > MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE;
  }

  private static double clamp(double armAngle) {
    return MathUtil.clamp(
        armAngle, RobotConfig.get().wrist().minAngle(), RobotConfig.get().wrist().maxAngle());
  }
}
