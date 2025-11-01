package frc.robot.climber;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.GlobalConfig;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class ClimberSubsystem extends StateMachineSubsystem<ClimberState> {
  private static final double TOLERANCE = 1.5;
  private final TalonFX climbMotor;
  private final CANcoder encoder;
  private final TalonFX grabMotor;
  private final CANrange canRange;
  private final Debouncer canRangeDebouncer = new Debouncer(0.25, DebounceType.kBoth);

  private final CoastOut coastNeutralRequest = new CoastOut();
  private double currentAngle = 0.0;
  private double climberMotorAngle = 0.0;
  private boolean holdingCage = false;

  public ClimberSubsystem(
      TalonFX climbMotor, CANcoder encoder, TalonFX grabMotor, CANrange canRange) {
    super(SubsystemPriority.CLIMBER, ClimberState.STOWED);

    this.climbMotor = climbMotor;
    this.encoder = encoder;
    this.grabMotor = grabMotor;
    this.canRange = canRange;

    climbMotor.getConfigurator().apply(RobotConfig.get().climber().climbMotorConfig());
    encoder.getConfigurator().apply(RobotConfig.get().climber().cancoderConfig());
    grabMotor.getConfigurator().apply(RobotConfig.get().climber().grabMotorConfig());
    canRange.getConfigurator().apply(RobotConfig.get().climber().canRangeConfig());
  }

  @Override
  protected void afterTransition(ClimberState newState) {
    if (newState == ClimberState.STOWED && !atGoal()) {
      DogLog.logFault("Climber stowed and not at goal", AlertType.kWarning);
    } else {
      DogLog.clearFault("Climber stowed and not at goal");
    }
  }

  @Override
  public void whileInState(ClimberState currentState) {
    if (DriverStation.isDisabled()) {
      if (currentState == ClimberState.STOWED) {
        climbMotor.setControl(coastNeutralRequest);
      } else {
        climbMotor.disable();
      }
      grabMotor.disable();
      return;
    }

    var clampedSetpoint = clamp(currentState.angle);

    if (atGoal()) {
      climbMotor.disable();
    } else if (currentAngle < clampedSetpoint) {
      climbMotor.setVoltage(currentState.forwardsVoltage);
    } else {
      climbMotor.setVoltage(currentState.backwardsVoltage);
    }

    if (currentState == ClimberState.LINEUP && !holdingCage) {
      grabMotor.setVoltage(12.0);
    } else {
      grabMotor.disable();
    }

    if (GlobalConfig.IS_DEVELOPMENT) {
      if (atGoal()) {
        DogLog.log("Climber/Status", "At goal");
      } else if (currentAngle < clampedSetpoint) {
        DogLog.log("Climber/Status", "Too low");
      } else {
        DogLog.log("Climber/Status", "Too high");
      }
    }
  }

  public void setState(ClimberState newState) {
    setStateFromRequest(newState);
  }

  public boolean holdingCage() {
    return holdingCage;
  }

  @Override
  protected void collectInputs() {
    currentAngle = Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble());
    climberMotorAngle = Units.rotationsToDegrees(climbMotor.getPosition().getValueAsDouble());
    holdingCage = canRangeDebouncer.calculate(canRange.getIsDetected().getValue());

    DogLog.log("Climber/Cancoder/Angle", currentAngle, Degrees);
    DogLog.log("Climber/ClimbMotor/Angle", climberMotorAngle, Degrees);
    DogLog.log("Climber/HoldingCage", holdingCage);
  }

  public boolean atGoal() {
    var goal = clamp(getState().angle);
    if (currentAngle < goal) {
      return false;
    }
    if (currentAngle > goal + TOLERANCE) {
      return false;
    }
    return true;
  }

  private static double clamp(double angle) {
    return MathUtil.clamp(
        angle, RobotConfig.get().climber().minAngle(), RobotConfig.get().climber().maxAngle());
  }
}
