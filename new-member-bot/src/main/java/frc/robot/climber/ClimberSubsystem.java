package frc.robot.climber;

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
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class ClimberSubsystem extends StateMachineSubsystem<ClimberState> {
  private static final double PASS_ANGLE_CHECK = 0.0;
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
  protected ClimberState getNextState(ClimberState currentState) {
      return switch (currentState) {
        case LINEUP_FORWARD -> {
          if (currentAngle < PASS_ANGLE_CHECK) {
            DogLog.timestamp("Climber/LineupForwardStartedFlip");
            yield ClimberState.LINEUP_BACKWARD;
          }
          yield currentState;
        }
        case LINEUP_BACKWARD -> holdingCage ? ClimberState.HANGING : currentState;
        default -> currentState;
      };
  }

  @Override
  public void whileInState(ClimberState currentState) {
    switch (getState()) {
      case STOPPED -> {
        if (DriverStation.isDisabled()) {
          climbMotor.setControl(coastNeutralRequest);
        } else {
          climbMotor.disable();
        }
        grabMotor.disable();
      }
      case LINEUP_FORWARD -> {
        climbMotor.setVoltage(getState().forwardsVoltage);
        grabMotor.disable();
      }
      case LINEUP_BACKWARD -> {
        if (atGoal()) {
          climbMotor.disable();
        } else {
          climbMotor.setVoltage(getState().forwardsVoltage);
        }
        grabMotor.setVoltage(12.0);
      }
      case HANGING -> {
        if (atGoal()) {
          climbMotor.disable();
        } else {
          climbMotor.setVoltage(getState().forwardsVoltage);
        }
        grabMotor.disable();
      }
    }

    if (GlobalConfig.IS_DEVELOPMENT) {
      if (atGoal()) {
        DogLog.log("Climber/Status", "At goal");
      } else {
        DogLog.log("Climber/Status", "Too low");
      }
    }
  }

  public void setState(ClimberState newState) {
    switch (newState) {
      case HANGING -> {
        if (getState() == ClimberState.LINEUP_BACKWARD && atGoal()) {
          setStateFromRequest(newState);
        }
      }
      default -> setStateFromRequest(newState);
    }
  }

  public boolean holdingCage() {
    return holdingCage;
  }

  @Override
  protected void collectInputs() {
    if (getState() == ClimberState.STOPPED) {
      return;
    }

    currentAngle = Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble());
    climberMotorAngle = Units.rotationsToDegrees(climbMotor.getPosition().getValueAsDouble());
    holdingCage = canRangeDebouncer.calculate(canRange.getIsDetected().getValue());

    DogLog.log("Climber/Cancoder/Angle", currentAngle);
    DogLog.log("Climber/ClimbMotor/Angle", climberMotorAngle);
    DogLog.log("Climber/HoldingCage", holdingCage);
  }

  public boolean atGoal() {
    return currentAngle >= clamp(getState().angle);
  }

  private static double clamp(double angle) {
    return MathUtil.clamp(
        angle, RobotConfig.get().climber().minAngle(), RobotConfig.get().climber().maxAngle());
  }
}
