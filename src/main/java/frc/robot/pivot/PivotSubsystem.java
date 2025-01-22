package frc.robot.pivot;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class PivotSubsystem extends StateMachine<PivotState> {
  private final TalonFX motor;
  private double motorAngle;
  private double motorCurrent;

  private final IntakeSubsystem intake;

  private final PositionVoltage motionMagicRequest =
      new PositionVoltage(PivotState.STOWED.angle).withEnableFOC(false);

  public PivotSubsystem(TalonFX motor, IntakeSubsystem intake) {
    super(SubsystemPriority.PIVOT, PivotState.STOWED);

    motor.getConfigurator().apply(RobotConfig.get().pivot().motorConfig());

    this.motor = motor;
    this.intake = intake;
  }

  @Override
  protected void afterTransition(PivotState newState) {
    switch (newState) {
      case HOMING -> {
        // TODO: Set homing voltage to like 2ish
        motor.setVoltage(0);
      }
      case CORAL_SCORE -> {
        motor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(getScoreDirection())));
      }
      default -> {
        motor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(getState().angle)));
      }
    }
  }

  @Override
  protected PivotState getNextState(PivotState currentState) {
    if (currentState == PivotState.HOMING
        && motorCurrent > RobotConfig.get().pivot().homingCurrentThreshold()) {
      motor.setPosition(RobotConfig.get().pivot().homingPosition());
      return PivotState.STOWED;
    }

    // Don't do anything
    return currentState;
  }

  @Override
  protected void collectInputs() {
    motorAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    motorCurrent = motor.getStatorCurrent().getValueAsDouble();
  }

  private double getScoreDirection() {
    if (!intake.getBottomSensor()) {
      return 90;
    }

    return -90;
  }

  public void setState(PivotState newState) {
    if (getState() != PivotState.HOMING) {
      setStateFromRequest(newState);
    }
  }

  public boolean atGoal() {
    return switch (getState()) {
      case HOMING -> false;
      case CORAL_SCORE -> MathUtil.isNear(getScoreDirection(), motorAngle, 1);
      default -> MathUtil.isNear(getState().angle, motorAngle, 1);
    };
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Pivot/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Pivot/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Pivot/Position", motor.getPosition().getValueAsDouble() * 360);
    DogLog.log("Pivot/PositionAngle", motorAngle);
  }
}
