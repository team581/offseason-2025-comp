package frc.robot.roll;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class RollSubsystem extends StateMachine<RollState> {
  private final TalonFX motor;
  private double motorAngle;
  private double motorCurrent;

  private final IntakeSubsystem intake;

  private final PositionVoltage motionMagicRequest =
      new PositionVoltage(RollState.STOWED.angle).withEnableFOC(false);

  public RollSubsystem(TalonFX motor, IntakeSubsystem intake) {
    super(SubsystemPriority.ROLL, RollState.STOWED);

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
  protected RollState getNextState(RollState currentState) {
    if (currentState == RollState.HOMING
        && motorCurrent > RobotConfig.get().roll().homingCurrentThreshold()) {
      motor.setPosition(Units.degreesToRotations(RobotConfig.get().roll().homingPosition()));
      return RollState.STOWED;
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

  public void setState(RollState newState) {
    if (getState() != RollState.HOMING) {
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
    DogLog.log("Roll/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Roll/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Roll/Position", motorAngle);
    if (DriverStation.isEnabled()) {
      if (getState() == RollState.HOMING) {
        DogLog.logFault("Roll Unhomed", AlertType.kWarning);
      }
    } else {
      if (getState() != RollState.HOMING) {
        DogLog.clearFault("Roll Unhomed");
      }
    }
  }
}
