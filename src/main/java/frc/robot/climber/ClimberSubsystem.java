package frc.robot.climber;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ClimberSubsystem extends StateMachine<ClimberState> {
  // TODO: Only enable this with adult supervision or else the climber will genuinely destroy itself
  // again
  private static final boolean CLIMBER_ENABLED = false;
  private static final double TOLERANCE = 3;
  private final TalonFX motor;
  private final CANcoder encoder;
  private double currentAngle;

  public ClimberSubsystem(TalonFX motor, CANcoder encoder) {
    super(SubsystemPriority.CLIMBER, ClimberState.STOWED);

    motor.getConfigurator().apply(RobotConfig.get().climber().motorConfig());
    encoder.getConfigurator().apply(RobotConfig.get().climber().cancoderConfig());

    this.motor = motor;
    this.encoder = encoder;
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    if (!CLIMBER_ENABLED) {
      motor.disable();
      return;
    }

    if (atGoal()) {
      motor.disable();
    } else {
      if (currentAngle < clamp(getState().angle)) {
        motor.setVoltage(12.0);
      } else {
        motor.setVoltage(-12.0);
      }
    }
  }

  public void setState(ClimberState newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected void collectInputs() {
    currentAngle = Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble());
    DogLog.log(
        "Climber/MotorAngle", Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()));
    DogLog.log("Climber/CancoderAngle", currentAngle);
    DogLog.log("Climber/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Climber/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Climber/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
  }

  public boolean atGoal() {
    return MathUtil.isNear(clamp(getState().angle), currentAngle, TOLERANCE);
  }

  private double clamp(double angle) {
    return MathUtil.clamp(angle, RobotConfig.get().climber().minAngle(), RobotConfig.get().climber().maxAngle());
  }
}
