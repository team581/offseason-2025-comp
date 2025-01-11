package frc.robot.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ElevatorSubsystem extends StateMachine<ElevatorState> {
  private static final double TOLERANCE = RobotConfig.get().elevator().tolerance();

  private static double rotationsToInches(double rotations) {
    return Units.degreesToRotations(rotations)
        * (RobotConfig.get().elevator().rotationsToDistance());
  }

  private static double inchesToRotations(double inches) {
    return Units.rotationsToDegrees(inches / (RobotConfig.get().elevator().rotationsToDistance()));
  }

  private static double clampHeight(double height) {
    return MathUtil.clamp(
        height, RobotConfig.get().elevator().minHeight(), RobotConfig.get().elevator().maxHeight());
  }

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(ElevatorState.STOWED.value);

  // Homing
  private double lowestSeenHeight = 0.0;
  private double height;

  public ElevatorSubsystem(TalonFX topMotor, TalonFX bottomMotor) {
    super(SubsystemPriority.ELEVATOR, ElevatorState.PRE_MATCH_HOMING);
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;
  }

  public void setState(ElevatorState newState) {
    if (getState() != ElevatorState.PRE_MATCH_HOMING) {

      setStateFromRequest(newState);
    }
  }

  @Override
  public void disabledPeriodic() {
    double currentHeight = height;

    if (currentHeight < lowestSeenHeight) {
      lowestSeenHeight = currentHeight;
    }
  }

  @Override
  protected void collectInputs() {
    // Calculate average height of the two motors
    height =
        (rotationsToInches(Units.rotationsToDegrees(topMotor.getPosition().getValueAsDouble()))
                + rotationsToInches(
                    Units.rotationsToDegrees(bottomMotor.getPosition().getValueAsDouble())))
            / 2.0;
  }

  @Override
  protected void afterTransition(ElevatorState newState) {
    switch (newState) {
      case NET -> {
        topMotor.setPosition(Units.degreesToRotations(clampHeight(ElevatorState.NET.value)));
        bottomMotor.setPosition(Units.degreesToRotations(clampHeight(ElevatorState.NET.value)));
      }
      case PROCESSOR -> {
        topMotor.setPosition(Units.degreesToRotations(clampHeight(ElevatorState.PROCESSOR.value)));
        bottomMotor.setPosition(
            Units.degreesToRotations(clampHeight(ElevatorState.PROCESSOR.value)));
      }
      case CORAL_L1 -> {
        topMotor.setPosition(Units.degreesToRotations(clampHeight(ElevatorState.CORAL_L1.value)));
        bottomMotor.setPosition(
            Units.degreesToRotations(clampHeight(ElevatorState.CORAL_L1.value)));
      }
      case CORAL_L2 -> {
        topMotor.setPosition(Units.degreesToRotations(clampHeight(ElevatorState.CORAL_L2.value)));
        bottomMotor.setPosition(
            Units.degreesToRotations(clampHeight(ElevatorState.CORAL_L2.value)));
      }
      case CORAL_L3 -> {
        topMotor.setPosition(Units.degreesToRotations(clampHeight(ElevatorState.CORAL_L3.value)));
        bottomMotor.setPosition(
            Units.degreesToRotations(clampHeight(ElevatorState.CORAL_L3.value)));
      }
      case CORAL_L4 -> {
        topMotor.setPosition(Units.degreesToRotations(clampHeight(ElevatorState.CORAL_L4.value)));
        bottomMotor.setPosition(
            Units.degreesToRotations(clampHeight(ElevatorState.CORAL_L4.value)));
      }
      case GROUND_ALGAE_INTAKE -> {
        topMotor.setPosition(
            Units.degreesToRotations(clampHeight(ElevatorState.GROUND_ALGAE_INTAKE.value)));
        bottomMotor.setPosition(
            Units.degreesToRotations(clampHeight(ElevatorState.GROUND_ALGAE_INTAKE.value)));
      }
      case GROUND_CORAL_INTAKE -> {
        topMotor.setPosition(
            Units.degreesToRotations(clampHeight(ElevatorState.GROUND_CORAL_INTAKE.value)));
        bottomMotor.setPosition(
            Units.degreesToRotations(clampHeight(ElevatorState.GROUND_CORAL_INTAKE.value)));
      }
      case STOWED -> {
        topMotor.setPosition(Units.degreesToRotations(clampHeight(ElevatorState.STOWED.value)));
        bottomMotor.setPosition(Units.degreesToRotations(clampHeight(ElevatorState.STOWED.value)));
      }
      case INTAKE_CORAL_STATION -> {
        topMotor.setPosition(
            Units.degreesToRotations(clampHeight(ElevatorState.INTAKE_CORAL_STATION.value)));
        bottomMotor.setPosition(
            Units.degreesToRotations(clampHeight(ElevatorState.INTAKE_CORAL_STATION.value)));
      }
      case UNJAM -> {
        topMotor.setPosition(Units.degreesToRotations(clampHeight(ElevatorState.UNJAM.value)));
        bottomMotor.setPosition(Units.degreesToRotations(clampHeight(ElevatorState.UNJAM.value)));
      }
      default -> {}
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    if (DriverStation.isEnabled() && getState() == ElevatorState.PRE_MATCH_HOMING) {
      // We are enabled and still in pre match homing
      // Reset the motor positions, and then transition to idle state
      double homingEndPosition = RobotConfig.get().elevator().homingEndPosition();
      double homedPosition = homingEndPosition + (height - lowestSeenHeight);
      topMotor.setPosition(Units.degreesToRotations(inchesToRotations(homedPosition)));
      bottomMotor.setPosition(Units.degreesToRotations(inchesToRotations(homedPosition)));

      setStateFromRequest(ElevatorState.STOWED);
    }
    DogLog.log("Elevator/Height", height);
  }

  public boolean atGoal(ElevatorState elevatorState) {
    return switch (getState()) {
      case PRE_MATCH_HOMING, UNJAM -> true;
      default -> MathUtil.isNear(height, elevatorState.value, TOLERANCE);
    };
  }
}
