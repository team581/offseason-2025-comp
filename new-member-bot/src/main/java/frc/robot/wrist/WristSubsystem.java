package frc.robot.wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class WristSubsystem extends StateMachineSubsystem<WristState> {
  private static final double TOLERANCE = 3.0;
  private static final double NEAR_TOLERANCE = 5.0;

  private final TalonFX motor;
  private double rawMotorAngle;
  private double motorAngle;
  private double motorCurrent;
  private double lowestSeenAngle = Double.POSITIVE_INFINITY;
  private double highestSeenAngle = Double.NEGATIVE_INFINITY;
  private static final StaticBrake BRAKE_NEUTRAL_REQUEST = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();

  private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(false);

  public WristSubsystem(TalonFX motor) {
    super(SubsystemPriority.WRIST, WristState.PRE_MATCH_HOMING);
    motor.getConfigurator().apply(RobotConfig.get().wrist().motorConfig());

    this.motor = motor;

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

  private void positionVoltageRequest(double wristRotations) {
    if (DriverStation.isTeleop()) {
      motor.setControl(positionRequest.withPosition(wristRotations));
      DogLog.log("Wrist/PostionVoltage", "Teleop");
    }
  }

  public boolean atGoal(WristState state) {
    return switch (state) {
      default -> MathUtil.isNear(state.getAngle(), motorAngle, TOLERANCE);
      case PRE_MATCH_HOMING, MID_MATCH_HOMING -> false;
    };
  }

  public boolean atGoal() {
    return atGoal(getState());
  }

  public boolean nearGoal(WristState state) {
    return switch (state) {
      default -> MathUtil.isNear(state.getAngle(), motorAngle, NEAR_TOLERANCE);
      case PRE_MATCH_HOMING, MID_MATCH_HOMING -> false;
    };
  }

  public boolean nearGoal() {
    return nearGoal(getState());
  }

  @Override
  protected void collectInputs() {
    rawMotorAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    // TODO: remove if statemnet maybe
    if (getState() == WristState.PRE_MATCH_HOMING) {
      motorAngle = RobotConfig.get().wrist().homingPosition() - (highestSeenAngle - rawMotorAngle);
    } else {
      motorAngle = rawMotorAngle;
    }

    lowestSeenAngle = Math.min(lowestSeenAngle, rawMotorAngle);
    highestSeenAngle = Math.max(highestSeenAngle, rawMotorAngle);

    motorCurrent = motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  protected void whileInState(WristState state) {
    DogLog.log("Wrist/StatorCurrent", motorCurrent, Amps);
    DogLog.log("Wrist/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble(), Volts);
    DogLog.log("Wrist/MotorAngle", motorAngle, Degrees);
    DogLog.log("Wrist/RawMotorAngle", rawMotorAngle, Degrees);

    DogLog.log("Wrist/AtGoal", atGoal());

    if (rangeOfMotionGood()) {
      DogLog.clearFault("WRIST NOT HOMED");
    } else {
      DogLog.logFault("WRIST NOT HOMED", AlertType.kWarning);
    }

    switch (state) {
      case PRE_MATCH_HOMING -> {
        if (rangeOfMotionGood()) {
          if (DriverStation.isDisabled()) {
            motor.setControl(coastNeutralRequest);
          }
        } else {
          motor.setControl(coastNeutralRequest);
        }
      }
      default -> {
        positionVoltageRequest(Units.degreesToRotations(getState().getAngle()));
      }
    }
  }

  public boolean rangeOfMotionGood() {
    return Math.abs(highestSeenAngle - lowestSeenAngle)
        >= RobotConfig.get().wrist().rangeOfMotionDeg() / 2.0;
  }

  @Override
  protected void beforeTransition(WristState oldState, WristState newState) {
    DogLog.log("Wrist/OldState", oldState);
    DogLog.log("Wrist/NewState", newState);

    if (oldState == WristState.PRE_MATCH_HOMING
        && newState != WristState.PRE_MATCH_HOMING
        && DriverStation.isEnabled()
        && Robot.isReal()) {
      DogLog.clearFault("Wrist/WRIST NOT HOMED");

      motor.setPosition(Units.degreesToRotations(motorAngle));
      collectInputs();
    }
  }

  @Override
  public void simulationPeriodic() {
    var wristSimulation =
        SimKit.positionMechanism("wrist", (mechanism) -> mechanism.addMotor(motor));

    if (getState() == WristState.PRE_MATCH_HOMING || getState() == WristState.MID_MATCH_HOMING) {
      motor.setPosition(0);
      setStateFromRequest(WristState.STOWED);
    }

    wristSimulation.update();

    if (DriverStation.isDisabled()) {
      wristSimulation.seedPosition(rawMotorAngle);
    }
  }
}
