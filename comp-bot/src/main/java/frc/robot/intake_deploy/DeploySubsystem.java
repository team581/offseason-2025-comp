package frc.robot.intake_deploy;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.GlobalConfig;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class DeploySubsystem extends StateMachineSubsystem<DeployState> {
  private static final double TOLERANCE = 1.0;

  private final TalonFX motor;
  private final CoastOut coastRequest = new CoastOut();
  private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(false);
  private final LinearFilter currentFilter = LinearFilter.movingAverage(7);

  private double currentAngle = 0.0;

  private double rawCurrent = 0.0;
  private double filteredCurrent = 0.0;

  public DeploySubsystem(TalonFX motor) {
    super(
        SubsystemPriority.DEPLOY,
        RobotBase.isSimulation() ? DeployState.STOWED : DeployState.UNHOMED);

    motor.getConfigurator().apply(RobotConfig.get().deploy().motorConfig());

    this.motor = motor;
    TunablePid.of("Deploy", motor, RobotConfig.get().deploy().motorConfig());

    if (GlobalConfig.IS_DEVELOPMENT) {
      DogLog.tunable(
          "Deploy/SetAngleDeg", 0.0, angle -> motor.setPosition(Units.degreesToRotations(angle)));
    }
  }

  @Override
  protected void afterTransition(DeployState newState) {
    switch (newState) {
      case UNHOMED -> {
        motor.setControl(coastRequest);
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
          motor.setPosition(
              Units.degreesToRotations(RobotConfig.get().deploy().homingEndPosition()));
          yield DeployState.STOWED;
        }
        yield currentState;
      }
      default -> currentState;
    };
  }

  @Override
  public void whileInState(DeployState currentState) {
    DogLog.log("Deploy/AtGoal", atGoal());
    DogLog.log("Deploy/Angle", currentAngle);
    DogLog.log("Deploy/FilteredStatorCurrent", filteredCurrent);
  }

  @Override
  protected void collectInputs() {
    switch (getState()) {
      case UNHOMED, HOMING -> {
        rawCurrent = motor.getStatorCurrent().getValueAsDouble();
        filteredCurrent = currentFilter.calculate(rawCurrent);
      }
      default -> {
        rawCurrent = 0;
        filteredCurrent = 0;
      }
    }

    currentAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
  }

  public boolean atGoal() {
    return switch (getState()) {
      case UNHOMED, HOMING -> false;
      default -> MathUtil.isNear(clamp(getState().getAngle()), currentAngle, TOLERANCE, -180, 180);
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

  public double getAngle() {
    return currentAngle;
  }

  @Override
  public void simulationPeriodic() {
    var deploySimulation =
        SimKit.positionMechanism(
            "deploy",
            (mechanism) ->
                mechanism
                    .addMotor(motor)
                    .withMinPosition(
                        Units.degreesToRotations(RobotConfig.get().deploy().minAngle()))
                    .withMaxPosition(
                        Units.degreesToRotations(RobotConfig.get().deploy().maxAngle())));

    if (getState() == DeployState.HOMING) {
      motor.setPosition(RobotConfig.get().deploy().homingEndPosition());
      setStateFromRequest(DeployState.STOWED);
    }

    deploySimulation.update();
  }
}
