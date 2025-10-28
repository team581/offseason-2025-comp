package frc.robot.elevator;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.FeatureFlags;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class ElevatorSubsystem extends StateMachineSubsystem<ElevatorState> {
  private static final double TOLERANCE = 0;
  private static final double NEAR_TOLERANCE = 0;

  private final TalonFX motor;

  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(ElevatorState.STOWED.getHeight());

  private double height = 0;
  private double lowestSeenHeight = Double.POSITIVE_INFINITY;
  private final CoastOut coastRequest = new CoastOut();

  public ElevatorSubsystem(TalonFX motor) {
    super(SubsystemPriority.ELEVATOR, ElevatorState.PRE_MATCH_HOMING);

    this.motor = motor;

    motor.getConfigurator().apply(RobotConfig.get().elevator().motorConfig());

    TunablePid.of("Elevator/motor", motor, RobotConfig.get().elevator().motorConfig());
  }

  public void setState(ElevatorState newState) {
    switch (getState()) {
      case PRE_MATCH_HOMING -> {
        if (DriverStation.isEnabled()) {
          setStateFromRequest(newState);
        }
      }
      default -> {
        setStateFromRequest(newState);
      }
    }
  }

  public double getHeight() {
    return height;
  }

  @Override
  protected void collectInputs() {
    height = motor.getPosition().getValueAsDouble();

    if (DriverStation.isDisabled()) {
      lowestSeenHeight = Math.min(lowestSeenHeight, height);
    }
  }

  private static double clampHeight(double height) {
    return MathUtil.clamp(
        height, RobotConfig.get().elevator().minHeight(), RobotConfig.get().elevator().maxHeight());
  }

  @Override
  protected void afterTransition(ElevatorState newState) {
    switch (newState) {
      default -> {
        motor.setControl(positionRequest.withPosition(clampHeight(newState.getHeight())));
      }
    }
  }

  public void customPeriodic() {
    DogLog.log("Elevator/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Elevator/Height", height);
    DogLog.log("Elevator/AtGoal", atGoal());

    switch (getState()) {
      default -> {}
    }

    if (DriverStation.isDisabled() && FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      motor.setControl(coastRequest);
    }
  }

  @Override
  protected void beforeTransition(ElevatorState oldState, ElevatorState newState) {
    DogLog.log("Elevator/OldState", oldState);
    DogLog.log("Elevator/NewState", newState);

    if (oldState == ElevatorState.PRE_MATCH_HOMING
        && newState != ElevatorState.PRE_MATCH_HOMING
        && DriverStation.isEnabled()) {
      // We are enabled and still in pre match homing
      // Reset the motor positions, and then transition to idle state
      double homingEndHeight = RobotConfig.get().elevator().homingEndHeight();
      var homedHeight = homingEndHeight + (height - lowestSeenHeight);

      motor.setPosition(homedHeight);
      // Refresh sensor data now that position is set
      collectInputs();
    }
  }

  public boolean atGoal() {
    return switch (getState()) {
      case MID_MATCH_HOMING -> false;
      case PRE_MATCH_HOMING, UNJAM -> true;
      default ->
          MathUtil.isNear(
              getState().getHeight(),
              height,
              getState().getHeight() == 0.0 ? TOLERANCE + 1.0 : TOLERANCE);
    };
  }

  public boolean nearGoal(ElevatorState state) {
    return nearGoal(state, NEAR_TOLERANCE);
  }

  public boolean nearGoal(ElevatorState state, double tolerance) {
    return MathUtil.isNear(state.getHeight(), height, tolerance);
  }

  public boolean nearGoal() {
    return switch (getState()) {
      case PRE_MATCH_HOMING, UNJAM -> true;
      default -> MathUtil.isNear(getState().getHeight(), height, NEAR_TOLERANCE);
    };
  }

  @Override
  public void simulationPeriodic() {
    var elevatorSimulation =
        SimKit.positionMechanism(
            "elevator",
            (mechanism) ->
                mechanism
                    .addMotor(motor, ChassisReference.Clockwise_Positive)
                    .withMinPosition(RobotConfig.get().elevator().minHeight())
                    .withMaxPosition(RobotConfig.get().elevator().maxHeight()));

    elevatorSimulation.update();

    if (DriverStation.isDisabled()) {
      elevatorSimulation.seedPosition(0);
    }
  }
}
