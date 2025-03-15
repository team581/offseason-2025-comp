package frc.robot.climber;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ClimberSubsystem extends StateMachine<ClimberState> {
  private static final double TOLERANCE = 1.5;
  private final TalonFX climbMotor;
  private final CANcoder encoder;
  private final TalonFX grabMotor;
  private final CANrange canRange;
  private final Debouncer climbMotorDirectionDebouncer = new Debouncer(1.0, DebounceType.kBoth);
  private final Debouncer canRangeDebouncer = new Debouncer(0.25, DebounceType.kBoth);
  private double climbMotorDirection = 0;
  private double cancoderDirection = 0;
  private boolean climberDirectionBad = false;
  private double currentAngle;
  private double cilmberMotorAngle;
  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();
  private boolean holdingCage = false;
  private boolean autoLineup = false;

  private Timer lineupTimer = new Timer();

  public ClimberSubsystem(
      TalonFX climbMotor, CANcoder encoder, TalonFX grabMotor, CANrange canrange) {
    super(SubsystemPriority.CLIMBER, ClimberState.STOWED);

    this.climbMotor = climbMotor;
    this.encoder = encoder;
    this.grabMotor = grabMotor;
    this.canRange = canrange;

    climbMotor.getConfigurator().apply(RobotConfig.get().climber().climbMotorConfig());
    encoder.getConfigurator().apply(RobotConfig.get().climber().cancoderConfig());
    grabMotor.getConfigurator().apply(RobotConfig.get().climber().grabMotorConfig());
    canrange.getConfigurator().apply(RobotConfig.get().climber().canRangeConfig());

    DogLog.log("Climber/DirectionBad", climberDirectionBad);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    // if (!climberDirectionBad) {
    //   climberDirectionBad =
    //       climbMotorDirectionDebouncer.calculate(
    //           cancoderDirection != 0
    //               && climbMotorDirection != 0
    //               && cancoderDirection != climbMotorDirection);
    // }

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
        climbMotor.setControl(coastNeutralRequest);
      } else {
        climbMotor.setControl(brakeNeutralRequest);
      }
    } else if (climberDirectionBad || atGoal()) {
      climbMotor.disable();
    } else if (currentAngle < clamp(getState().angle)) {
      climbMotor.setVoltage(getState().forwardsVoltage);
    } else {
      climbMotor.setVoltage(getState().backwardsVoltage);
    }

    if (getState() == ClimberState.LINEUP && !holdingCage) {
      grabMotor.setVoltage(-12);
    } else {
      grabMotor.disable();
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

    if (!autoLineup && lineupTimer.hasElapsed(105.0)) {
      if (getState() == ClimberState.STOWED) {
        setState(ClimberState.AUTO_LINEUP);
      }
      autoLineup = true;
    }
  }

  public void setState(ClimberState newState) {
    if (getState() == ClimberState.AUTO_LINEUP) {
      if (newState == ClimberState.LINEUP
          || newState == ClimberState.HANGING
          || newState == ClimberState.HANGING_2
          || newState == ClimberState.HANGING_3) {
        setStateFromRequest(newState);
      } else {
        return;
      }
    }

    setStateFromRequest(newState);
  }

  public boolean holdingCage() {
    return holdingCage;
  }

  @Override
  protected void collectInputs() {
    currentAngle = Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble());
    cilmberMotorAngle = Units.rotationsToDegrees(climbMotor.getPosition().getValueAsDouble());

    cancoderDirection = Math.signum(climbMotor.getVelocity().getValueAsDouble());
    climbMotorDirection = Math.signum(encoder.getVelocity().getValueAsDouble());

    holdingCage = canRangeDebouncer.calculate(canRange.getIsDetected().getValue());

    DogLog.log("Climber/Cancoder/Direction", cancoderDirection);
    DogLog.log("Climber/Cancoder/Angle", currentAngle);

    DogLog.log("Climber/ClimbMotor/Direction", climbMotorDirection);
    DogLog.log("Climber/ClimbMotor/Angle", cilmberMotorAngle);

    DogLog.log("Climber/HoldingCage", holdingCage);

    DogLog.log("Climber/AppliedVoltage", climbMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Climber/StatorCurrent", climbMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Climber/SupplyCurrent", climbMotor.getSupplyCurrent().getValueAsDouble());
  }

  @Override
  public void teleopInit() {
    super.teleopInit();
    autoLineup = false;
    lineupTimer.reset();
    lineupTimer.start();
  }

  public boolean atGoal() {
    return MathUtil.isNear(clamp(getState().angle), currentAngle, TOLERANCE);
  }

  private double clamp(double angle) {
    return MathUtil.clamp(
        angle, RobotConfig.get().climber().minAngle(), RobotConfig.get().climber().maxAngle());
  }
}
