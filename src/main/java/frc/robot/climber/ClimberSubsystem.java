package frc.robot.climber;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ClimberSubsystem extends StateMachine<ClimberState> {
  // TODO: Only enable this with adult supervision or else the climber will genuinely destroy itself
  // again
  private static final boolean CLIMBER_ENABLED = false;
  private static final double TOLERANCE = 3;
  private final TalonFX motor;
  private final CANcoder encoder;
  private double motorDirection = 0;
  private double cancoderDirection = 0;
  private boolean climberDirectionBad = false;
  private double currentAngle;
  private double motorAngle;

  public ClimberSubsystem(TalonFX motor, CANcoder encoder) {
    super(SubsystemPriority.CLIMBER, ClimberState.STOWED);

    motor.getConfigurator().apply(RobotConfig.get().climber().motorConfig());
    encoder.getConfigurator().apply(RobotConfig.get().climber().cancoderConfig());

    this.motor = motor;
    this.encoder = encoder;
    DogLog.log("Climber/DirectionBad", climberDirectionBad);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    if (cancoderDirection != motorDirection) {
      climberDirectionBad = true;
      DogLog.logFault("Climber Direction Bad", AlertType.kError);
      DogLog.log("Climber/DirectionBad", climberDirectionBad);
    }

    if (climberDirectionBad || atGoal() || !CLIMBER_ENABLED) {
      motor.disable();
    } else if (currentAngle < clamp(getState().angle)) {
      motor.setVoltage(getState().forwardsVoltage);
    } else {
      motor.setVoltage(getState().backwardsVoltage);
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
