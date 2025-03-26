package frc.robot.intake_deploy;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.util.tuning.TunablePid;

public class DeploySubsystem extends StateMachine<DeployState> {
  private static final double TOLERANCE = 1.0;

  private final TalonFX motor;
  private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(false);
  private final LinearFilter currentFilter = LinearFilter.movingAverage(7);

  private double currentAngle = 0.0;
  private double rawCurrent = 0.0;
  private double filteredCurrent = 0.0;

  public DeploySubsystem(TalonFX motor) {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);

    motor.getConfigurator().apply(RobotConfig.get().deploy().motorConfig());

    this.motor = motor;
    TunablePid.of("Deploy", motor, RobotConfig.get().deploy().motorConfig());
  }

  @Override
  protected void afterTransition(DeployState newState) {
    switch (newState) {
      case UNHOMED -> {
        motor.disable();
      }
      case HOMING -> {
        motor.setVoltage(RobotConfig.get().deploy().homingVoltage());
      }
      default ->
          motor.setControl(
              positionRequest.withPosition(Units.degreesToRotations(newState.getAngle())));
    }
  }

  @Override
  protected DeployState getNextState(DeployState currentState) {
    return switch (currentState) {
      case HOMING -> {
        if (filteredCurrent > RobotConfig.get().deploy().homingCurrentThreshold()) {
          // TODO: Use dedicated homing end position
          motor.setPosition(Units.degreesToRotations(clamp(RobotConfig.get().deploy().maxAngle())));
          yield DeployState.STOWED;
        }
        yield currentState;
      }
      default -> currentState;
    };
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("Deploy/AtGoal", atGoal());
    DogLog.log("Deploy/Angle", currentAngle);
    DogLog.log("Deploy/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Deploy/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Deploy/CurrentThreshold", RobotConfig.get().deploy().homingCurrentThreshold());
  }

  @Override
  protected void collectInputs() {
    rawCurrent = motor.getStatorCurrent().getValueAsDouble();
    filteredCurrent = currentFilter.calculate(rawCurrent);

    currentAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
  }

  public boolean atGoal() {
    return switch (getState()) {
      case UNHOMED, HOMING -> false;
      default -> MathUtil.isNear(clamp(getState().getAngle()), currentAngle, TOLERANCE);
    };
  }

  public void setState(DeployState newState) {
    switch (getState()) {
      case HOMING -> {}
      case UNHOMED -> {
        if (newState == DeployState.HOMING) {
          setStateFromRequest(DeployState.HOMING);
        }
      }
      default -> {
        setStateFromRequest(newState);
      }
    }
  }

  private static double clamp(double deployAngle) {
    return MathUtil.clamp(
        deployAngle, RobotConfig.get().deploy().minAngle(), RobotConfig.get().deploy().maxAngle());
  }
}
