package frc.robot.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ElevatorSubsystem extends StateMachine<ElevatorState> {
  private static final double TOLERANCE = RobotConfig.get().elevator().tolerance();

  private static double clampHeight(double height) {
    return MathUtil.clamp(
        height, RobotConfig.get().elevator().minHeight(), RobotConfig.get().elevator().maxHeight());
  }

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(ElevatorState.STOWED.height);

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
    DogLog.log("Elevator/CollisionAvoidanceGoalHeight", collisionAvoidanceGoal);
  }

  @Override
  protected void collectInputs() {
    // Calculate average height of the two motors
    averageMeasuredHeight =
        (leftMotor.getPosition().getValueAsDouble() + rightMotor.getPosition().getValueAsDouble())
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
            positionRequest.withPosition(clampHeight(ElevatorState.CLIMBING.height)));
        rightMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.CLIMBING.height)));
      }
      case NET -> {
        leftMotor.setControl(positionRequest.withPosition(clampHeight(ElevatorState.NET.height)));
        rightMotor.setControl(positionRequest.withPosition(clampHeight(ElevatorState.NET.height)));
      }
      case PROCESSOR -> {
        leftMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.PROCESSOR.height)));
        rightMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.PROCESSOR.height)));
      }
      case ALGAE_DISLODGE_L2 -> {
        leftMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.ALGAE_DISLODGE_L2.height)));
        rightMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.ALGAE_DISLODGE_L2.height)));
      }
      case ALGAE_DISLODGE_L3 -> {
        leftMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.ALGAE_DISLODGE_L3.height)));
        rightMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.ALGAE_DISLODGE_L3.height)));
      }
      case ALGAE_INTAKE_L2 -> {
        leftMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.ALGAE_INTAKE_L2.height)));
        rightMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.ALGAE_INTAKE_L2.height)));
      }
      case ALGAE_INTAKE_L3 -> {
        leftMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.ALGAE_INTAKE_L3.height)));
        rightMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.ALGAE_INTAKE_L3.height)));
      }
      case CORAL_L1_PLACE -> {
        leftMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.CORAL_L1_PLACE.height)));
        rightMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.CORAL_L1_PLACE.height)));
      }
      case CORAL_L2_PLACE -> {
        leftMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.CORAL_L2_PLACE.height)));
        rightMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.CORAL_L2_PLACE.height)));
      }
      case CORAL_L3_PLACE -> {
        leftMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.CORAL_L3_PLACE.height)));
        rightMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.CORAL_L3_PLACE.height)));
      }
      case CORAL_L4_PLACE -> {
        leftMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.CORAL_L4_PLACE.height)));
        rightMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.CORAL_L4_PLACE.height)));
      }
      case GROUND_ALGAE_INTAKE -> {
        leftMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.GROUND_ALGAE_INTAKE.height)));
        rightMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.GROUND_ALGAE_INTAKE.height)));
      }
      case GROUND_CORAL_INTAKE -> {
        leftMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.GROUND_CORAL_INTAKE.height)));
        rightMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.GROUND_CORAL_INTAKE.height)));
      }
      case STOWED -> {
        leftMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.STOWED.height)));
        rightMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.STOWED.height)));
      }
      case INTAKING_CORAL_STATION -> {
        leftMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.INTAKING_CORAL_STATION.height)));
        rightMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.INTAKING_CORAL_STATION.height)));
      }
      case UNJAM -> {
        leftMotor.setControl(positionRequest.withPosition(clampHeight(ElevatorState.UNJAM.height)));
        rightMotor.setControl(
            positionRequest.withPosition(clampHeight(ElevatorState.UNJAM.height)));
      }
      default -> {}
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Elevator/Left/StatorCurrent", leftMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Elevator/Right/StatorCurrent", rightMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Elevator/Left/AppliedVoltage", leftMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Elevator/Right/AppliedVoltage", rightMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Elevator/Height", averageMeasuredHeight);

    switch (getState()) {
      case PRE_MATCH_HOMING -> {
        if (DriverStation.isEnabled()) {
          // We are enabled and still in pre match homing
          // Reset the motor positions, and then transition to idle state
          double homingEndHeight = RobotConfig.get().elevator().homingEndHeight();
          double homedHeight = homingEndHeight + (averageMeasuredHeight - lowestSeenHeight);
          leftMotor.setPosition(homedHeight);
          rightMotor.setPosition(homedHeight);

          setStateFromRequest(ElevatorState.STOWED);
        }
      }

      case COLLISION_AVOIDANCE -> {
        leftMotor.setControl(positionRequest.withPosition(clampHeight(collisionAvoidanceGoal)));
        rightMotor.setControl(positionRequest.withPosition(clampHeight(collisionAvoidanceGoal)));
      }

      default -> {}
    }
  }

  public boolean atGoal() {
    return switch (getState()) {
      case PRE_MATCH_HOMING, UNJAM -> true;
      case COLLISION_AVOIDANCE ->
          MathUtil.isNear(collisionAvoidanceGoal, averageMeasuredHeight, TOLERANCE);
      default -> MathUtil.isNear(getState().height, averageMeasuredHeight, TOLERANCE);
    };
  }
}
