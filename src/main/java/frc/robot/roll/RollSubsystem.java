package frc.robot.roll;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.claw.ClawSubsystem;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class RollSubsystem extends StateMachine<RollState> {
  private final TalonFX motor;
  private double motorAngle;
  private double motorCurrent;
  private double averageMotorCurrent;
  private double smartStowAngle;
  private double coralScoreDirection;

  private LinearFilter linearFilter = LinearFilter.movingAverage(5);

  private final ClawSubsystem intake;

  private final PositionVoltage motionMagicRequest =
      new PositionVoltage(RollState.CORAL_HORIZONTAL.angle).withEnableFOC(false);

  public RollSubsystem(TalonFX motor, ClawSubsystem intake) {
    super(SubsystemPriority.ROLL, RollState.UNHOMED);

    motor.getConfigurator().apply(RobotConfig.get().roll().motorConfig());

    this.motor = motor;
    this.intake = intake;
  }

  @Override
  protected void afterTransition(RollState newState) {
    switch (newState) {
      case HOMING -> {
        motor.setVoltage(1);
      }
      case CORAL_SCORE -> {
        coralScoreDirection = getCoralScoreDirection();
        motor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(clamp(coralScoreDirection))));
      }
      case SMART_STOW -> {
        smartStowAngle = getSmartStowDirection();
        motor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(clamp(smartStowAngle))));
      }
      case UNHOMED -> motor.disable();
      default -> {
        motor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(clamp(getState().angle))));
      }
    }
  }

  @Override
  protected void collectInputs() {
    motorAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    motorCurrent = motor.getStatorCurrent().getValueAsDouble();
    averageMotorCurrent = linearFilter.calculate(motorCurrent);
  }

  @Override
  protected RollState getNextState(RollState currentState) {
    if (currentState == RollState.HOMING
        && averageMotorCurrent > RobotConfig.get().roll().homingCurrentThreshold()) {
      motor.setPosition(Units.degreesToRotations(RobotConfig.get().roll().homingPosition()));
      return RollState.CORAL_HORIZONTAL;
    }

    // Don't do anything
    return currentState;
  }

  private double getCoralScoreDirection() {
    if (intake.getLeftSensor()) {
      return -90;
    }
    return 90;
  }

  private double getSmartStowDirection() {
    if (intake.getLeftSensor() && intake.getRightSensor()) {
      return 45;
    } else if (intake.getLeftSensor()) {
      return 45;
    } else {
      return -45;
    }
  }

  public void setState(RollState newState) {
    setState(newState, false);
  }

  public void setState(RollState newState, boolean recheckSensors) {
    switch (getState()) {
      case HOMING -> {}
      case UNHOMED -> {
        if (newState == RollState.HOMING) {
          setStateFromRequest(RollState.HOMING);
        }
      }
      case CORAL_SCORE, SMART_STOW -> {
        if (newState == getState() && recheckSensors) {
          afterTransition(newState);
        } else {
          setStateFromRequest(newState);
        }
      }
      default -> setStateFromRequest(newState);
    }
  }

  public boolean atGoal() {
    return switch (getState()) {
      case HOMING -> false;
      case CORAL_SCORE -> MathUtil.isNear(coralScoreDirection, motorAngle, 3);
      case SMART_STOW -> MathUtil.isNear(smartStowAngle, motorAngle, 2);
      default -> MathUtil.isNear(getState().angle, motorAngle, 3);
    };
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Roll/StatorCurrent", motorCurrent);
    DogLog.log("Roll/AverageStatorCurrent", averageMotorCurrent);
    DogLog.log("Roll/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Roll/Angle", motorAngle);
    DogLog.log("Roll/AtGoal", atGoal());

    if (getState() == RollState.UNHOMED) {
      DogLog.logFault("Roll Unhomed", AlertType.kWarning);
    } else {
      DogLog.clearFault("Roll Unhomed");
    }
  }

  private static double clamp(double rollAngle) {
    return MathUtil.clamp(
        rollAngle, RobotConfig.get().roll().minAngle(), RobotConfig.get().roll().maxAngle());
  }
}
