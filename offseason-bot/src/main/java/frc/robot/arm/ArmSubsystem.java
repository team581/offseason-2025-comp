package frc.robot.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.math.MathHelpers;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.FeatureFlags;
import frc.robot.config.RobotConfig;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Map;
import java.util.OptionalDouble;

public class ArmSubsystem extends StateMachineSubsystem<ArmState> {
  public static final double ARM_LENGTH_METERS = Units.inchesToMeters(37.416);

  private static final InterpolatingDoubleTreeMap CORAL_TX_TO_ARM_ANGLE_TABLE =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(2.66, 4.93),
          Map.entry(0.0, 0.0),
          Map.entry(-6.5, -2.37),
          Map.entry(-11.0, -6.67));

  private static final double TOLERANCE = 2.0;
  private static final double NEAR_TOLERANCE = 35.0;

  private final TalonFX motor;
  private double rawMotorAngle;
  private double motorAngle;
  private double motorCurrent;
  private double lowestSeenAngle = Double.POSITIVE_INFINITY;
  private double highestSeenAngle = Double.NEGATIVE_INFINITY;
  private double handoffOffset = 0;
  private static final double MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE = 90.0;
  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();
  private final VelocityVoltage spinToWin = new VelocityVoltage(0.6);
  private boolean lollipopMode = false;
  private final ElevatorSubsystem elevator;
  private boolean elevatorIsGoingDown = false;
  private boolean elevatorIsGoingDownDebounced = false;
  private double previousElevatorHeight = Double.POSITIVE_INFINITY;
  private final Debouncer debouncer = new Debouncer(1.0, DebounceType.kBoth);
  private final LinearFilter handoffAdjustmentTxFilter = LinearFilter.movingAverage(7);

  public void setLollipopMode(boolean lollipopMode) {
    this.lollipopMode = lollipopMode;
    DogLog.log("Arm/LollipopMode", lollipopMode);
  }

  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0.0).withEnableFOC(false);
  private final MotionMagicExpoVoltage autoMotionMagicExpoRequest =
      new MotionMagicExpoVoltage(0.0).withEnableFOC(false);

  public ArmSubsystem(TalonFX motor, ElevatorSubsystem elevator) {
    super(SubsystemPriority.ARM, ArmState.PRE_MATCH_HOMING);
    motor.getConfigurator().apply(RobotConfig.get().arm().motorConfig());

    this.motor = motor;
    this.elevator = elevator;

    // In field calibration mode, boot arm to lower hardstop angle
    if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      motor.setPosition(Units.degreesToRotations(RobotConfig.get().arm().homingPosition()));
    }

    TunablePid.of("Arm", motor, RobotConfig.get().arm().motorConfig());

    DogLog.tunable(
        "Arm/ChangeEncoderPositionDeg",
        0.0,
        newPositionDeg -> motor.setPosition(Units.degreesToRotations(newPositionDeg)));
  }

  public void setCoralHandoffOffset(OptionalDouble tx) {
    handoffOffset =
        CORAL_TX_TO_ARM_ANGLE_TABLE.get(handoffAdjustmentTxFilter.calculate(tx.orElse(0)));
  }

  public void setState(ArmState newState) {
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

  private void makeGetMotionMagicRequest(double armRotations) {
    if (DriverStation.isTeleop() || lollipopMode) {
      motor.setControl(motionMagicRequest.withPosition(armRotations));
      DogLog.log("Arm/MotionMagicStrategy", "Teleop");
    } else {
      motor.setControl(autoMotionMagicExpoRequest.withPosition(armRotations));
      DogLog.log("Arm/MotionMagicStrategy", "Expo");
    }
  }

  public boolean atGoal() {
    return switch (getState()) {
      default -> MathUtil.isNear(getState().getAngle(), rawMotorAngle, TOLERANCE, -180, 180);
      case CORAL_HANDOFF -> MathUtil.isNear(usedHandoffAngle, motorAngle, TOLERANCE, -180, 180);
      case PRE_MATCH_HOMING -> false;
    };
  }

  public boolean nearGoal() {
    return switch (getState()) {
      default -> MathUtil.isNear(getState().getAngle(), rawMotorAngle, NEAR_TOLERANCE, -180, 180);
      case PRE_MATCH_HOMING -> false;
    };
  }

  public boolean nearGoal(ArmState state) {
    return nearGoal(state, NEAR_TOLERANCE);
  }

  public boolean nearGoal(ArmState state, double tolerance) {
    return MathUtil.isNear(state.getAngle(), rawMotorAngle, tolerance, -180, 180);
  }

  @Override
  protected void collectInputs() {
    usedHandoffAngle = ArmState.CORAL_HANDOFF.getAngle() + handoffOffset;
    rawMotorAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    motorAngle = MathHelpers.angleModulus(rawMotorAngle);

    if (DriverStation.isDisabled()) {
      elevatorIsGoingDown = elevator.getHeight() < previousElevatorHeight;
      elevatorIsGoingDownDebounced = debouncer.calculate(elevatorIsGoingDown);

      // If elevator is going down, reset these values
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
  protected void afterTransition(ArmState newState) {}

  public void customPeriodic() {
    DogLog.log("Arm/StatorCurrent", motorCurrent);
    DogLog.log("Arm/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Arm/Angle", motorAngle);
    DogLog.log("Arm/RawAngle", rawMotorAngle);

    DogLog.log("Arm/AtGoal", atGoal());

    if (DriverStation.isDisabled()) {
      DogLog.log("Arm/LowestAngle", lowestSeenAngle);
      DogLog.log("Arm/HighestAngle", highestSeenAngle);
      DogLog.log("Arm/ElevatorIsGoingDown", elevatorIsGoingDown);
      DogLog.log("Arm/ElevatorIsGoingDownDebounced", elevatorIsGoingDownDebounced);
    }
    if (rangeOfMotionGood()) {
      DogLog.clearFault("ARM NOT HOMED");
    } else {
      DogLog.logFault("ARM NOT HOMED", AlertType.kWarning);
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
      case CORAL_HANDOFF -> {
        makeGetMotionMagicRequest(Units.degreesToRotations(usedHandoffAngle));
      }
      default -> {
        makeGetMotionMagicRequest(Units.degreesToRotations(getState().getAngle()));
      }
    }
  }

  public boolean rangeOfMotionGood() {
    return Math.abs(highestSeenAngle - lowestSeenAngle) > MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE;
  }

  private final TalonFXConfiguration simMotorConfig = new TalonFXConfiguration();
  private TrapezoidProfile.Constraints simConstraints;
  private boolean simDidInit = false;

  private double usedHandoffAngle = ArmState.CORAL_HANDOFF.getAngle();

  @Override
  protected void beforeTransition(ArmState oldState, ArmState newState) {
    DogLog.log("Arm/OldState", oldState);
    DogLog.log("Arm/NewState", newState);

    if (oldState == ArmState.PRE_MATCH_HOMING
        && newState != ArmState.PRE_MATCH_HOMING
        && DriverStation.isEnabled()) {
      DogLog.clearFault("Arm/ARM NOT HOMED");
      var actualArmAngle =
          RobotConfig.get().arm().homingPosition() + (rawMotorAngle - lowestSeenAngle);
      motor.setPosition(Units.degreesToRotations(actualArmAngle));
      // Refresh sensor data now that position is set
      collectInputs();
    }
  }

  @Override
  public void disabledInit() {
    if (RobotBase.isSimulation()) {
      // reset position to be 0*
      var motorSim = motor.getSimState();
      motorSim.setRawRotorPosition(rawMotorAngle);
    }
  }

  @Override
  public void simulationPeriodic() {
    var armSimulation = SimKit.positionMechanism("arm", (mechanism) -> mechanism.addMotor(motor));

    if (getState() == ArmState.PRE_MATCH_HOMING) {
      motor.setPosition(0);
      setStateFromRequest(ArmState.HOLDING_UPRIGHT);
    }

    armSimulation.update();

    if (DriverStation.isDisabled()) {
      armSimulation.seedPosition(rawMotorAngle);
    }
  }
}
