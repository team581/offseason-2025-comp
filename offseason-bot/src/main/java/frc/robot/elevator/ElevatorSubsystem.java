package frc.robot.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class ElevatorSubsystem extends StateMachineSubsystem<ElevatorState> {
  public static final double CARRIAGE_HEIGHT_FROM_FLOOR_METERS = Units.inchesToMeters(22.1);

  private static final double TOLERANCE = 5.0;
  private static final double NEAR_TOLERANCE = 20.0;

  private final TalonFX motor;

  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(ElevatorState.STOWED.defaultHeight);

  private double height = 0.0;
  private double lowestSeenHeight = Double.POSITIVE_INFINITY;

  public ElevatorSubsystem(TalonFX motor) {
    super(SubsystemPriority.ELEVATOR, ElevatorState.PRE_MATCH_HOMING);
    TunablePid.of("Elevator", motor, RobotConfig.get().elevator().motorConfig());

    motor.getConfigurator().apply(RobotConfig.get().elevator().motorConfig());
    this.motor = motor;
  }

  private static double clampHeight(double unclamped) {
    return MathUtil.clamp(
        unclamped,
        RobotConfig.get().elevator().minHeight(),
        RobotConfig.get().elevator().maxHeight());
  }

  @Override
  protected void afterTransition(ElevatorState newState) {
    switch (newState) {
      default -> motor.setControl(positionRequest.withPosition(clampHeight(newState.getHeight())));
    }
  }

  @Override
  protected void beforeTransition(ElevatorState oldState, ElevatorState newState) {

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

  @Override
  public void whileInState(ElevatorState currentState) {
    DogLog.log("Elevator/Height", height);
    DogLog.log("Elevator/AtGoal", atGoal());
    DogLog.log("Elevator/NearGoal", nearGoal());
    DogLog.log("Elevator/Goal", currentState.getHeight());
    DogLog.log("Elevator/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Elevator/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Elevator/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());


    if (DriverStation.isDisabled()) {
      var homingEndHeight = RobotConfig.get().elevator().homingEndHeight();
      var estimatedHeight = homingEndHeight + (height - lowestSeenHeight);
      if (!MathUtil.isNear(estimatedHeight, homingEndHeight, 2.0)) {
        DogLog.logFault("ELEVATOR NOT IN AUTO POSITION", AlertType.kError);
      } else {
        DogLog.clearFault("ELEVATOR NOT IN AUTO POSITION");
      }
    }
  }

  @Override
  protected void collectInputs() {
    height = motor.getPosition().getValueAsDouble();
    if (DriverStation.isDisabled()) {
      lowestSeenHeight = Math.min(lowestSeenHeight, height);
    }
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

  public boolean atGoal(ElevatorState goal) {
    return switch (goal) {
      case UNJAM -> true;
      case PRE_MATCH_HOMING, UNTUNED -> false;
      default -> MathUtil.isNear(clampHeight(goal.getHeight()), height, TOLERANCE);
    };
  }

  public boolean atGoal() {
    return atGoal(getState());
  }

  public boolean nearGoal(ElevatorState goal) {
    return switch (goal) {
      case UNJAM -> true;
      case PRE_MATCH_HOMING, UNTUNED -> false;
      default -> MathUtil.isNear(clampHeight(goal.getHeight()), height, NEAR_TOLERANCE);
    };
  }

  public boolean nearGoal() {
    return nearGoal(getState());
  }

  public double getHeight() {
    return height;
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
