package frc.robot.wrist;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.math.MathHelpers;
import com.team581.util.state_machines.StateMachine;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class WristSubsystem extends StateMachine<WristState> {

  private static final double TOLERANCE = 0;
  private static final double NEAR_TOLERANCE = 0;

  private final TalonFX motor;
  private double rawMotorAngle;
  private double motorAngle;
  private double motorCurrent;
  private double lowestSeenAngle = Double.POSITIVE_INFINITY;
  private double highestSeenAngle = Double.NEGATIVE_INFINITY;
  private static final double MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE = 0;
  private static final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();
  private final ElevatorSubsystem elevator;
  private boolean elevatorIsGoingDown = false;
  private boolean elevatorIsGoingDownDebounced = false;
  private double previousElevatorHeight = Double.POSITIVE_INFINITY;
  private final Debouncer debouncer = new Debouncer(0, DebounceType.kBoth);

  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0.0).withEnableFOC(false);
  private final MotionMagicExpoVoltage autoMotionMagicExpoRequest =
      new MotionMagicExpoVoltage(0.0).withEnableFOC(false);

  public WristSubsystem(TalonFX motor, ElevatorSubsystem elevator) {
    super(SubsystemPriority.WRIST, WristState.PRE_MATCH_HOMING);
    motor.getConfigurator().apply(RobotConfig.get().wrist().motorConfig());

    this.motor = motor;
    this.elevator = elevator;

    TunablePid.of("Wrist", motor, RobotConfig.get().wrist().motorConfig());
  }

  public void setState(WristState newState) {
    switch (getState()) {
      case PRE_MATCH_HOMING -> {
        if (DriverStation.isEnabled() && rangeOfMotionGood()) {
          setStateFromRequest(newState);
        }
      }
      default -> setStateFromRequest(newState);
    }
  }

  public double getAngle() {
    return motorAngle;
  }

  public double getRawAngle() {
    return rawMotorAngle;
  }

  private void makeGetMotionMagicRequest(double wristRotations) {
    if (DriverStation.isTeleop()) {
      motor.setControl(motionMagicRequest.withPosition(wristRotations));
      DogLog.log("Wrist/MotionMagicStrategy", "Teleop");
    } else {
      motor.setControl(autoMotionMagicExpoRequest.withPosition(wristRotations));
      DogLog.log("Wrist/MotionMagicStrategy", "Expo");
    }
  }

  public boolean atGoal() {
    return switch (getState()) {
      default -> MathUtil.isNear(getState().getAngle(), rawMotorAngle, TOLERANCE, 0, 0);
      case PRE_MATCH_HOMING -> false;
    };
  }

  public boolean nearGoal() {
    return switch (getState()) {
      default -> MathUtil.isNear(getState().getAngle(), rawMotorAngle, NEAR_TOLERANCE, 0, 0);
      case PRE_MATCH_HOMING -> false;
    };
  }

  public boolean nearGoal(WristState state) {
    return nearGoal(state, NEAR_TOLERANCE);
  }

  public boolean nearGoal(WristState state, double tolerance) {
    return MathUtil.isNear(state.getAngle(), rawMotorAngle, tolerance, 0, 0);
  }

  @Override
  protected void collectInputs() {
    rawMotorAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    motorAngle = MathHelpers.angleModulus(rawMotorAngle);

    if (DriverStation.isDisabled()) {
      elevatorIsGoingDown = elevator.getHeight() < previousElevatorHeight;
      elevatorIsGoingDownDebounced = debouncer.calculate(elevatorIsGoingDown);

      if (elevatorIsGoingDownDebounced) {
        lowestSeenAngle = Double.POSITIVE_INFINITY;
      }

      lowestSeenAngle = Math.min(lowestSeenAngle, rawMotorAngle);
      highestSeenAngle = Math.max(highestSeenAngle, rawMotorAngle);

      previousElevatorHeight = elevator.getHeight();
    }
    motorCurrent = motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  protected void afterTransition(WristState newState) {}

  public void customPeriodic() {
    DogLog.log("Wrist/StatorCurrent", motorCurrent);
    DogLog.log("Wrist/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Wrist/Angle", motorAngle);
    DogLog.log("Wrist/RawAngle", rawMotorAngle);

    DogLog.log("Wrist/AtGoal", atGoal());

    if (DriverStation.isDisabled()) {
      DogLog.log("Wrist/LowestAngle", lowestSeenAngle);
      DogLog.log("Wrist/HighestAngle", highestSeenAngle);
      DogLog.log("Wrist/ElevatorIsGoingDown", elevatorIsGoingDown);
      DogLog.log("Wrist/ElevatorIsGoingDownDebounced", elevatorIsGoingDownDebounced);
    }
    if (rangeOfMotionGood()) {
      DogLog.clearFault("WRIST NOT HOMED");
    } else {
      DogLog.logFault("WRIST NOT HOMED", AlertType.kWarning);
    }

    switch (getState()) {
      case PRE_MATCH_HOMING -> {
        if (rangeOfMotionGood()) {
          if (DriverStation.isDisabled()) {
            motor.setControl(brakeNeutralRequest);
          }
        } else {
          motor.setControl(coastNeutralRequest);
        }
      }
      default -> {
        makeGetMotionMagicRequest(Units.degreesToRotations(getState().getAngle()));
      }
    }
  }

  public boolean rangeOfMotionGood() {
    return Math.abs(highestSeenAngle - lowestSeenAngle) > MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE;
  }

  @Override
  protected void beforeTransition(WristState oldState, WristState newState) {
    DogLog.log("Wrist/OldState", oldState);
    DogLog.log("Wrist/NewState", newState);

    if (oldState == WristState.PRE_MATCH_HOMING
        && newState != WristState.PRE_MATCH_HOMING
        && DriverStation.isEnabled()) {
      DogLog.clearFault("Wrist/WRIST NOT HOMED");
      var actualWristAngle =
          RobotConfig.get().wrist().homingPosition() + (rawMotorAngle - lowestSeenAngle);
      motor.setPosition(Units.degreesToRotations(actualWristAngle));
      collectInputs();
    }
  }
}
