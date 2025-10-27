package frc.robot.arm;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
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
  private double lowestSeenAngle = Double.POSITIVE_INFINITY;
  private double highestSeenAngle = Double.NEGATIVE_INFINITY;

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

  private void makeGetMotionMagicRequest(double armRotations) {
    motor.setControl(motionMagicRequest.withPosition(armRotations));
  }

  public boolean atGoal() {
    return switch (getState()) {
      case PRE_MATCH_HOMING -> false;
      default -> MathUtil.isNear(getState().getAngle(), rawMotorAngle, TOLERANCE, -180, 180);
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
      default -> {
        makeGetMotionMagicRequest(Units.degreesToRotations(clamp(getState().getAngle())));
      }
    }
  }

  public boolean rangeOfMotionGood() {
    return Math.abs(highestSeenAngle - lowestSeenAngle) > MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE;
  }

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
  public void simulationPeriodic() {
    var armSimulation = SimKit.positionMechanism("arm", (mechanism) -> mechanism.addMotor(motor));

    if (getState() == ArmState.PRE_MATCH_HOMING) {
      motor.setPosition(0);
      setStateFromRequest(ArmState.STOWED);
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
}
