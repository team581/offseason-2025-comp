package frc.robot.climber;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ClimberSubsystem extends StateMachine<ClimberState> {
  private static final double TOLERANCE = 3;
  private final TalonFX motor;
  private final CANcoder encoder;
  private final Debouncer motorDirectionDebouncer = new Debouncer(1.0, DebounceType.kBoth);
  private double motorDirection = 0;
  private double cancoderDirection = 0;
  private boolean climberDirectionBad = false;
  private double currentAngle;
  private double motorAngle;
  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();

  public ClimberSubsystem(TalonFX motor, CANcoder encoder) {
    super(SubsystemPriority.CLIMBER, ClimberState.STOWED);

    this.motor = motor;
    this.encoder = encoder;

    motor.getConfigurator().apply(RobotConfig.get().climber().motorConfig());
    encoder.getConfigurator().apply(RobotConfig.get().climber().cancoderConfig());

    DogLog.log("Climber/DirectionBad", climberDirectionBad);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    if (!climberDirectionBad) {
      climberDirectionBad =
          motorDirectionDebouncer.calculate(
              cancoderDirection != 0 && motorDirection != 0 && cancoderDirection != motorDirection);
    }

    if (climberDirectionBad) {
      DogLog.logFault("Climber Direction Bad", AlertType.kError);
      DogLog.log("Climber/DirectionBad", climberDirectionBad);
    }
    if (getState() == ClimberState.STOWED && !atGoal()) {
      DogLog.logFault("Climber stowed and not at goal", AlertType.kWarning);
    } else {
      DogLog.clearFault("Climber stowed and not at goal");
    }

    if (DriverStation.isDisabled()) {
      if (getState() == ClimberState.STOWED) {
        motor.setControl(coastNeutralRequest);
      } else {
        motor.setControl(brakeNeutralRequest);
      }
    } else if (climberDirectionBad || atGoal()) {
      motor.disable();
    } else if (currentAngle < clamp(getState().angle)) {
      motor.setVoltage(getState().forwardsVoltage);
    } else {
      motor.setVoltage(getState().backwardsVoltage);
    }

    if (RobotConfig.IS_DEVELOPMENT) {
      if (atGoal()) {
        DogLog.log("Climber/Status", "At goal");
      } else if (currentAngle < clamp(getState().angle)) {
        DogLog.log("Climber/Status", "Too low");
      } else {
        DogLog.log("Climber/Status", "Too high");
      }
    }
  }

  public void setState(ClimberState newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected void collectInputs() {
    currentAngle = Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble());
    motorAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());

    cancoderDirection = Math.signum(motor.getVelocity().getValueAsDouble());
    motorDirection = Math.signum(encoder.getVelocity().getValueAsDouble());

    DogLog.log("Climber/Cancoder/Direction", cancoderDirection);
    DogLog.log("Climber/Cancoder/Angle", currentAngle);

    DogLog.log("Climber/Motor/Direction", motorDirection);
    DogLog.log("Climber/Motor/Angle", motorAngle);

    DogLog.log("Climber/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Climber/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Climber/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
  }

  public boolean atGoal() {
    return MathUtil.isNear(clamp(getState().angle), currentAngle, TOLERANCE);
  }

  private double clamp(double angle) {
    return MathUtil.clamp(
        angle, RobotConfig.get().climber().minAngle(), RobotConfig.get().climber().maxAngle());
  }
}
