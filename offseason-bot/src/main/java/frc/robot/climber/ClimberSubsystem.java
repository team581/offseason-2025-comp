package frc.robot.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.GlobalConfig;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class ClimberSubsystem extends StateMachineSubsystem<ClimberState> {
  private static final double PASS_ANGLE_CHECK = 0.0;
  private final TalonFX climbMotor;
  private final CANcoder encoder;

  private final LinearFilter cancoderVelocityFilter = LinearFilter.movingAverage(7);

  private final CoastOut coastNeutralRequest = new CoastOut();
  private double cancoderVelocity = 0;
  private double currentAngle = 0.0;
  private double climberMotorAngle = 0.0;

  public ClimberSubsystem(TalonFX climbMotor, CANcoder encoder) {
    super(SubsystemPriority.CLIMBER, ClimberState.STOPPED);

    this.climbMotor = climbMotor;
    this.encoder = encoder;

    climbMotor.getConfigurator().apply(RobotConfig.get().climber().climbMotorConfig());
    encoder.getConfigurator().apply(RobotConfig.get().climber().cancoderConfig());
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
      }
      case LINEUP_FORWARD -> {
        climbMotor.setVoltage(getState().forwardsVoltage);

        if (currentAngle < PASS_ANGLE_CHECK) {
          setStateFromRequest(ClimberState.LINEUP_BACKWARD);
          DogLog.timestamp("Climber/LineupForwardStartedFlip");
        }
      }
      case LINEUP_BACKWARD -> {
        if (atGoal()) {
          climbMotor.disable();
        } else {
          climbMotor.setVoltage(getState().forwardsVoltage);
        }
      }
      case HANGING -> {
        if (atGoal()) {
          climbMotor.disable();
        } else {
          climbMotor.setVoltage(getState().forwardsVoltage);
        }
      }
    }

    if (GlobalConfig.IS_DEVELOPMENT) {
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
    switch (newState) {
      case LINEUP_FORWARD, STOPPED -> setStateFromRequest(newState);
      case HANGING -> {
        if (getState() == ClimberState.LINEUP_BACKWARD && atGoal()) {
          setStateFromRequest(newState);
        }
      }
      default -> {}
    }
  }

  @Override
  protected void collectInputs() {
    currentAngle = Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble());
    climberMotorAngle = Units.rotationsToDegrees(climbMotor.getPosition().getValueAsDouble());
    cancoderVelocity = cancoderVelocityFilter.calculate(encoder.getVelocity().getValueAsDouble());

    DogLog.log("Climber/CANCoderVelocity", cancoderVelocity, RotationsPerSecond);

    DogLog.log("Climber/Cancoder/Angle", currentAngle, Degrees);

    DogLog.log("Climber/ClimbMotor/Angle", climberMotorAngle, Degrees);

    DogLog.log("Climber/AppliedVoltage", climbMotor.getMotorVoltage().getValueAsDouble(), Volts);
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
