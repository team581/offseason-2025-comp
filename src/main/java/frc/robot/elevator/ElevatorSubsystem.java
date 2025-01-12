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

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(ElevatorState.STOWED.value);

  // Homing
  private double lowestSeenHeight = 0.0;
  private double averageMeasuredHeight;
  private double collisionAvoidanceGoal;

  public ElevatorSubsystem(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.ELEVATOR, ElevatorState.PRE_MATCH_HOMING);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    // Motor Configs
    leftMotor.getConfigurator().apply(RobotConfig.get().elevator().leftMotorConfig());
    rightMotor.getConfigurator().apply(RobotConfig.get().elevator().rightMotorConfig());
  }

  public void setState(ElevatorState newState) {
    if (getState() != ElevatorState.PRE_MATCH_HOMING) {

      setStateFromRequest(newState);
    }
  }

  public double getHeight() {
    return averageMeasuredHeight;
  }

  public void setCollisionAvoidanceGoal(double height) {
    collisionAvoidanceGoal = height;
  }

  @Override
  protected void collectInputs() {
    // Calculate average height of the two motors
    averageMeasuredHeight =
        (rotationsToInches(Units.rotationsToDegrees(leftMotor.getPosition().getValueAsDouble()))
                + rotationsToInches(
                    Units.rotationsToDegrees(rightMotor.getPosition().getValueAsDouble())))
            / 2.0;
  }

  @Override
  public void disabledPeriodic() {
    double currentHeight = averageMeasuredHeight;

    if (currentHeight < lowestSeenHeight) {
      lowestSeenHeight = currentHeight;
    }
  }

  @Override
  protected void afterTransition(ElevatorState newState) {
    switch (newState) {
      case CLIMBING -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.CLIMBING.value))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.CLIMBING.value))));
      }
      case NET -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.NET.value))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.NET.value))));
      }
      case PROCESSOR -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.PROCESSOR.value))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.PROCESSOR.value))));
      }
      case ALGAE_DISLODGE_L2 -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.ALGAE_DISLODGE_L2.value))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.ALGAE_DISLODGE_L2.value))));
      }
      case ALGAE_DISLODGE_L3 -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.ALGAE_DISLODGE_L3.value))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.ALGAE_DISLODGE_L3.value))));
      }
      case ALGAE_INTAKE_L2 -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.ALGAE_INTAKE_L2.value))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.ALGAE_INTAKE_L2.value))));
      }
      case ALGAE_INTAKE_L3 -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.ALGAE_INTAKE_L3.value))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.ALGAE_INTAKE_L3.value))));
      }
      case CORAL_L1 -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.CORAL_L1.value))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.CORAL_L1.value))));
      }
      case CORAL_L2 -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.CORAL_L2.value))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.CORAL_L2.value))));
      }
      case CORAL_L3 -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.CORAL_L3.value))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.CORAL_L3.value))));
      }
      case CORAL_L4 -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.CORAL_L4.value))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.CORAL_L4.value))));
      }
      case GROUND_ALGAE_INTAKE -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.GROUND_ALGAE_INTAKE.value))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.GROUND_ALGAE_INTAKE.value))));
      }
      case GROUND_CORAL_INTAKE -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.GROUND_CORAL_INTAKE.value))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.GROUND_CORAL_INTAKE.value))));
      }
      case STOWED -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.STOWED.value))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.STOWED.value))));
      }
      case INTAKING_CORAL_STATION -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.INTAKING_CORAL_STATION.value))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.INTAKING_CORAL_STATION.value))));
      }
      case UNJAM -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.UNJAM.value))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clampHeight(ElevatorState.UNJAM.value))));
      }
      default -> {}
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    switch (getState()) {
      case PRE_MATCH_HOMING -> {
        if (DriverStation.isEnabled()) {
          // We are enabled and still in pre match homing
          // Reset the motor positions, and then transition to idle state
          double homingEndPosition = RobotConfig.get().elevator().homingEndPosition();
          double homedPosition = homingEndPosition + (averageMeasuredHeight - lowestSeenHeight);
          leftMotor.setPosition(Units.degreesToRotations(inchesToRotations(homedPosition)));
          rightMotor.setPosition(Units.degreesToRotations(inchesToRotations(homedPosition)));

          setStateFromRequest(ElevatorState.STOWED);
        }
      }

      case COLLISION_AVOIDANCE -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(inchesToRotations(clampHeight(collisionAvoidanceGoal)))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(inchesToRotations(clampHeight(collisionAvoidanceGoal)))));
      }

      default -> {}
    }

    DogLog.log("Elevator/Height", averageMeasuredHeight);
  }

  public boolean atGoal() {
    return switch (getState()) {
      case PRE_MATCH_HOMING, UNJAM -> true;
      case COLLISION_AVOIDANCE ->
          MathUtil.isNear(collisionAvoidanceGoal, averageMeasuredHeight, TOLERANCE);
      default -> MathUtil.isNear(getState().value, averageMeasuredHeight, TOLERANCE);
    };
  }
}
