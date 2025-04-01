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
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ClimberSubsystem extends StateMachine<ClimberState> {
  private final TalonFX climbMotor;
  private final CANcoder encoder;
  private final TalonFX grabMotor;
  private final CANrange canRange;
  private final Debouncer cancoderBackwardsDebouncer = new Debouncer(1.0, DebounceType.kRising);
  private final Debouncer canRangeDebouncer = new Debouncer(0.25, DebounceType.kBoth);
  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();
  private double climbMotorDirection = 0;
  private double cancoderDirection = 0;
  private boolean cancoderBackwardDebounced = false;
  private double currentAngle = 0.0;
  private double cilmberMotorAngle = 0.0;
  private boolean holdingCage = false;

  public ClimberSubsystem(
      TalonFX climbMotor, CANcoder encoder, TalonFX grabMotor, CANrange canRange) {
    super(SubsystemPriority.CLIMBER, ClimberState.STOPPED);

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
  public void robotPeriodic() {
    super.robotPeriodic();

    if (DriverStation.isDisabled()) {
      if (getState() == ClimberState.STOPPED) {
        climbMotor.setControl(coastNeutralRequest);
      } else {
        climbMotor.setControl(brakeNeutralRequest);
      }
    }

    switch (getState()) {
      case STOPPED -> {
        climbMotor.disable();
      }
      case LINEUP_FORWARD -> {
        climbMotor.setVoltage(getState().forwardsVoltage);
        if (cancoderBackwardDebounced) {
          setStateFromRequest(ClimberState.LINEUP_BACKWARD);
          DogLog.timestamp("Climber/LineupForwardStartedFlip");
        }
      }
      case LINEUP_BACKWARD -> {
        if (!atGoal()) {
          climbMotor.setVoltage(getState().forwardsVoltage);
        } else {
          climbMotor.disable();
        }
        if (!holdingCage) {
          grabMotor.setVoltage(-0.0);
        } else {
          grabMotor.disable();
          setStateFromRequest(ClimberState.HANGING);
        }
      }
      case HANGING -> {
        if (!atGoal()) {
          climbMotor.setVoltage(getState().forwardsVoltage);
        } else {
          climbMotor.disable();
        }
      }
      default -> {}
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
    if (newState == ClimberState.LINEUP_FORWARD || newState == ClimberState.STOPPED) {
      setStateFromRequest(newState);
    } else {
      return;
    }
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
    cancoderBackwardDebounced =
        cancoderBackwardsDebouncer.calculate(
            (cancoderDirection != 0 && climbMotorDirection != 0)
                && cancoderDirection < climbMotorDirection);

    holdingCage = canRangeDebouncer.calculate(canRange.getIsDetected().getValue());

    DogLog.log("Climber/Cancoder/Angle", currentAngle);

    DogLog.log("Climber/ClimbMotor/Angle", cilmberMotorAngle);

    DogLog.log("Climber/HoldingCage", holdingCage);

    DogLog.log("Climber/AppliedVoltage", climbMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Climber/StatorCurrent", climbMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Climber/SupplyCurrent", climbMotor.getSupplyCurrent().getValueAsDouble());
  }

  public boolean atGoal() {
    var goal = clamp(getState().angle);
    return currentAngle >= goal;
  }

  private static double clamp(double angle) {
    return MathUtil.clamp(
        angle, RobotConfig.get().climber().minAngle(), RobotConfig.get().climber().maxAngle());
  }
}
