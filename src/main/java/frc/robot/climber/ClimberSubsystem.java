package frc.robot.climber;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ClimberSubsystem extends StateMachine<ClimberState> {
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

    if (atGoal()) {
      motor.disable();
    } else {
      if (currentAngle < getState().angle) {
        motor.setVoltage(0);
      } else {
        motor.setVoltage(-0);
      }
    }
  }

  public void setState(ClimberState newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected void collectInputs() {
    currentAngle = Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble());
    DogLog.log("Climber/CurrentAngle", currentAngle);
  }

  public boolean atGoal() {
    return MathUtil.isNear(getState().angle, currentAngle, TOLERANCE);
  }
}
