package frc.robot.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
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

  private double leftMotorCurrent;
  private double rightMotorCurrent;

  private LinearFilter linearFilter = LinearFilter.movingAverage(5);

  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(ElevatorState.STOWED.height);

  // Homing
  private double leftHeight = 0;
  private double rightHeight = 0;
  private double lowestSeenHeightLeft = 0.0;
  private double lowestSeenHeightRight = 0.0;

  private double averageMeasuredHeight = 0;
  private double collisionAvoidanceGoal = ElevatorState.STOWED.height;

  // Mid-match homing
  private double averageMotorCurrent;

  public ElevatorSubsystem(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.ELEVATOR, ElevatorState.PRE_MATCH_HOMING);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    // Motor Configs
    leftMotor.getConfigurator().apply(RobotConfig.get().elevator().leftMotorConfig());
    rightMotor.getConfigurator().apply(RobotConfig.get().elevator().rightMotorConfig());
  }

  public void setState(ElevatorState newState) {
    switch (getState()) {
      case PRE_MATCH_HOMING -> {
        if (newState == ElevatorState.MID_MATCH_HOMING) {
          setStateFromRequest(newState);
        }
      }
      case MID_MATCH_HOMING -> {}
      default -> {
        setStateFromRequest(newState);
      }
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
    leftHeight = leftMotor.getPosition().getValueAsDouble();
    rightHeight = rightMotor.getPosition().getValueAsDouble();

    averageMeasuredHeight = (leftHeight + rightHeight) / 2.0;

    leftMotorCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    rightMotorCurrent = rightMotor.getStatorCurrent().getValueAsDouble();

    averageMotorCurrent = linearFilter.calculate((leftMotorCurrent + rightMotorCurrent) / 2.0);

    if (DriverStation.isDisabled()) {
      lowestSeenHeightLeft = Math.min(lowestSeenHeightLeft, leftHeight);
      lowestSeenHeightRight = Math.min(lowestSeenHeightRight, rightHeight);
    }
  }

  @Override
  protected void afterTransition(ElevatorState newState) {
    switch (newState) {
      default -> {
        leftMotor.setControl(positionRequest.withPosition(clampHeight(newState.height)));
        rightMotor.setControl(positionRequest.withPosition(clampHeight(newState.height)));
      }
      case MID_MATCH_HOMING -> {
        leftMotor.setVoltage(-0.5);
        rightMotor.setVoltage(-0.5);
      }
      case COLLISION_AVOIDANCE -> {
        leftMotor.setControl(positionRequest.withPosition(clampHeight(collisionAvoidanceGoal)));
        rightMotor.setControl(positionRequest.withPosition(clampHeight(collisionAvoidanceGoal)));
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Elevator/Left/StatorCurrent", leftMotorCurrent);
    DogLog.log("Elevator/Right/StatorCurrent", rightMotorCurrent);
    DogLog.log("Elevator/AverageStatorCurrent", averageMotorCurrent);
    DogLog.log("Elevator/Left/AppliedVoltage", leftMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Elevator/Right/AppliedVoltage", rightMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Elevator/Left/Height", leftHeight);
    DogLog.log("Elevator/Right/Height", rightHeight);
    DogLog.log("Elevator/Height", averageMeasuredHeight);
    DogLog.log("Elevator/AtGoal", atGoal());

    switch (getState()) {
      case PRE_MATCH_HOMING -> {
        if (DriverStation.isEnabled()) {
          // We are enabled and still in pre match homing
          // Reset the motor positions, and then transition to idle state
          double homingEndHeight = RobotConfig.get().elevator().homingEndHeight();
          var leftHomedHeight = homingEndHeight + (leftHeight - lowestSeenHeightLeft);
          var rightHomedHeight = homingEndHeight + (rightHeight - lowestSeenHeightRight);
          // TODO: Restore elevator homing
          leftMotor.setPosition(homingEndHeight);
          rightMotor.setPosition(homingEndHeight);

          setStateFromRequest(ElevatorState.STOWED);
        }
      }

      case COLLISION_AVOIDANCE -> {
        leftMotor.setControl(positionRequest.withPosition(clampHeight(collisionAvoidanceGoal)));
        rightMotor.setControl(positionRequest.withPosition(clampHeight(collisionAvoidanceGoal)));
      }
      default -> {}
    }

    var usedHeight =
        getState() == ElevatorState.COLLISION_AVOIDANCE
            ? collisionAvoidanceGoal
            : getState().height;

    if (MathUtil.isNear(0, usedHeight, 1.0)
        && MathUtil.isNear(0, getHeight(), 1.0)
        && getState() != ElevatorState.MID_MATCH_HOMING) {
      leftMotor.disable();
      rightMotor.disable();
    }
  }

  @Override
  protected ElevatorState getNextState(ElevatorState currentState) {
    if (currentState == ElevatorState.MID_MATCH_HOMING
        && averageMotorCurrent > RobotConfig.get().elevator().homingCurrentThreshold()) {
      leftMotor.setPosition(RobotConfig.get().elevator().homingEndHeight());
      rightMotor.setPosition(RobotConfig.get().elevator().homingEndHeight());
      return ElevatorState.STOWED;
    }

    // Don't do anything
    return currentState;
  }

  public boolean atGoal() {
    return switch (getState()) {
      case PRE_MATCH_HOMING, MID_MATCH_HOMING, UNJAM -> true;
      case COLLISION_AVOIDANCE ->
          MathUtil.isNear(collisionAvoidanceGoal, averageMeasuredHeight, TOLERANCE);
      // This state is only used when it's safe to cancel the move partway
      // Since the next state is same setpoint, different wrist angle
      case CORAL_CENTERED_L4_RAISE_WRIST ->
          averageMeasuredHeight > ElevatorState.CORAL_CENTERED_L4_RAISE_WRIST.height - 8;
      case CORAL_DISPLACED_L4_RAISE_WRIST ->
          averageMeasuredHeight > ElevatorState.CORAL_DISPLACED_L4_RAISE_WRIST.height - 8;
      default -> MathUtil.isNear(getState().height, averageMeasuredHeight, TOLERANCE);
    };
  }
}
