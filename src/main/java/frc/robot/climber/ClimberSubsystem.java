package frc.robot.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import frc.robot.config.FeatureFlags;
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
  private TempClimberState tempState = TempClimberState.STOPPED;

  public ClimberSubsystem(TalonFX motor, CANcoder encoder) {
    super(SubsystemPriority.CLIMBER, ClimberState.STOWED);

    motor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(70)
                        .withSupplyCurrentLimit(50))
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake)));
    encoder.getConfigurator().apply(RobotConfig.get().climber().cancoderConfig());

    this.motor = motor;
    this.encoder = encoder;
    DogLog.log("Climber/DirectionBad", climberDirectionBad);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    switch (tempState) {
      case STOPPED -> motor.disable();
      case UP -> motor.setVoltage(12);
      case DOWN -> motor.setVoltage(-12);
    }

    DogLog.log("Climber/TempState", tempState);
    DogLog.log("Climber/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Climber/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Climber/OutputVoltage", motor.getMotorVoltage().getValueAsDouble());
  }

  public void setState(ClimberState newState) {
    setStateFromRequest(newState);
  }

  public void setState(TempClimberState newState) {
    tempState = newState;
  }

  @Override
  protected void collectInputs() {
    if (FeatureFlags.CLIMBER_ENABLED.getAsBoolean()) {
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
  }

  public boolean atGoal() {
    return MathUtil.isNear(clamp(getState().angle), currentAngle, TOLERANCE);
  }

  private double clamp(double angle) {
    return MathUtil.clamp(
        angle, RobotConfig.get().climber().minAngle(), RobotConfig.get().climber().maxAngle());
  }
}
