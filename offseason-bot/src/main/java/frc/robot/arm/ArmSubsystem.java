package frc.robot.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team581.math.MathHelpers;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.FeatureFlags;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class ArmSubsystem extends StateMachineSubsystem<ArmState> {
  public static final double ARM_LENGTH_METERS = Units.inchesToMeters(16.0);

  private static final double MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE = 90.0;
  private static final double TOLERANCE = 2.0;
  private static final double NEAR_TOLERANCE = 35.0;

  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();
  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0.0).withEnableFOC(false);

  private final TalonFX motor;

  private double rawMotorAngle = 0.0;
  private double motorAngle = 0.0;
  private double motorCurrent = 0.0;
  private double usedSetpoint = ArmState.STOWED.getAngle();
  private double lowestSeenAngle = Double.POSITIVE_INFINITY;
  private double highestSeenAngle = Double.NEGATIVE_INFINITY;

  /** Whether the arm should force itself in the down position. */
  private boolean forceClawDown = false;

  public ArmSubsystem(TalonFX motor) {
    super(SubsystemPriority.ARM, ArmState.PRE_MATCH_HOMING);
    motor.getConfigurator().apply(RobotConfig.get().arm().motorConfig());

    this.motor = motor;

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

  public boolean atGoal() {
    return switch (getState()) {
      case PRE_MATCH_HOMING -> false;
      default -> MathUtil.isNear(usedSetpoint, rawMotorAngle, TOLERANCE, -180, 180);
    };
  }

  public boolean nearGoal() {
    return switch (getState()) {
      default -> MathUtil.isNear(usedSetpoint, rawMotorAngle, NEAR_TOLERANCE, -180, 180);
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
    rawMotorAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());

    if (getState() == ArmState.PRE_MATCH_HOMING) {
      motorAngle = RobotConfig.get().arm().homingPosition() + (rawMotorAngle - lowestSeenAngle);
    } else {
      motorAngle = MathHelpers.angleModulus(rawMotorAngle);
    }

    if (DriverStation.isDisabled()) {
      lowestSeenAngle = Math.min(lowestSeenAngle, rawMotorAngle);
      highestSeenAngle = Math.max(highestSeenAngle, rawMotorAngle);
    }

    motorCurrent = motor.getStatorCurrent().getValueAsDouble();
    usedSetpoint = calculateUsedSetpoint();
  }

  @Override
  protected void afterTransition(ArmState newState) {
    usedSetpoint = calculateUsedSetpoint();

    motor.setControl(motionMagicRequest.withPosition(Units.degreesToRotations(usedSetpoint)));
  }

  public boolean rangeOfMotionGood() {
    return Math.abs(highestSeenAngle - lowestSeenAngle) > MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE;
  }

  public void setClawInCradle(boolean forceClawDown) {
    if (this.forceClawDown != forceClawDown) {
      this.forceClawDown = forceClawDown;

      // Rerun state actions, since the used setpoint may have changed
      afterTransition(getState());
    }
  }

  @Override
  protected void whileInState(ArmState state) {
    DogLog.log("Arm/StatorCurrent", motorCurrent, Amps);
    DogLog.log("Arm/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble(), Volts);
    DogLog.log("Arm/Angle", motorAngle, Degrees);
    DogLog.log("Arm/RawAngle", rawMotorAngle, Degrees);
    DogLog.log("Arm/UsedSetpoint", usedSetpoint, Degrees);
    DogLog.log("Arm/AtGoal", atGoal());
    DogLog.log("Arm/ForceClawDown", forceClawDown);

    if (DriverStation.isDisabled()) {
      DogLog.log("Arm/LowestAngle", lowestSeenAngle, Degrees);
      DogLog.log("Arm/HighestAngle", highestSeenAngle, Degrees);
      if (!MathUtil.isNear(ArmState.CORAL_HANDOFF.getAngle(), motorAngle, 8.0, -180, 180)) {
        DogLog.logFault("ARM NOT IN AUTO POSITION");
      } else {
        DogLog.clearFault("ARM NOT IN AUTO POSITION");
      }
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
      case CLIMBING -> {
        if (DriverStation.isDisabled()) {
          motor.setControl(brakeNeutralRequest);
        }
      }
      default -> {}
    }
  }

  @Override
  protected void beforeTransition(ArmState oldState, ArmState newState) {
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
  public void simulationPeriodic() {
    var armSimulation =
        SimKit.positionMechanism(
            "arm", (mechanism) -> mechanism.addMotor(motor, ChassisReference.Clockwise_Positive));

    if (getState() == ArmState.PRE_MATCH_HOMING) {
      motor.setPosition(0);
      setStateFromRequest(ArmState.CORAL_HANDOFF);
    }

    armSimulation.update();

    if (DriverStation.isDisabled()) {
      armSimulation.seedPosition(0);
    }
  }

  private static double clamp(double armAngle) {
    return MathUtil.clamp(
        armAngle, RobotConfig.get().arm().minAngle(), RobotConfig.get().arm().maxAngle());
  }

  private double calculateUsedSetpoint() {
    return clamp((forceClawDown ? ArmState.CORAL_HANDOFF : getState()).getAngle());
  }
}
