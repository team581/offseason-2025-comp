package frc.robot.singulator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class SingulatorSubsystem extends StateMachine<SingulatorState> {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private double rawRightCurrent = 0.0;
  private double rawLeftCurrent = 0.0;

  private final LinearFilter leftFilter = LinearFilter.movingAverage(5);
  private final LinearFilter rightFilter = LinearFilter.movingAverage(5);
  private double filteredRightCurrent = 0.0;
  private double filteredLeftCurrent = 0.0;

  private final double jamCurrentThreshold = 999.0;

  public SingulatorSubsystem(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.SINGULATOR, SingulatorState.IDLE);

    leftMotor.getConfigurator().apply(RobotConfig.get().singulator().leftMotorConfig());
    rightMotor.getConfigurator().apply(RobotConfig.get().singulator().rightMotorConfig());
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }

  @Override
  protected SingulatorState getNextState(SingulatorState currentState) {
    return switch (currentState) {
      default -> {
        if (isLeftJammed()) {
          yield SingulatorState.UNJAM_LEFT_ONLY;
        }
        if (isRightJammed()) {
          yield SingulatorState.UNJAM_RIGHT_ONLY;
        }
        yield currentState;
      }
    };
  }

  @Override
  protected void afterTransition(SingulatorState newState) {
    switch (newState) {
      case UNTUNED, STOPPED -> {
        leftMotor.disable();
        rightMotor.disable();
      }
      case UNJAM_LEFT_ONLY -> {
        leftMotor.setVoltage(newState.voltsLeft);
        rightMotor.disable();
      }
      case UNJAM_RIGHT_ONLY -> {
        leftMotor.disable();
        rightMotor.setVoltage(newState.voltsRight);
      }
      default -> {
        leftMotor.setVoltage(newState.voltsLeft);
        rightMotor.setVoltage(newState.voltsRight);
      }
    }
  }

  @Override
  protected void collectInputs() {
    rawLeftCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    rawRightCurrent = rightMotor.getStatorCurrent().getValueAsDouble();

    filteredLeftCurrent = leftFilter.calculate(rawLeftCurrent);
    filteredRightCurrent = rightFilter.calculate(rawRightCurrent);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("Singulator/Left/Current", filteredLeftCurrent);
    DogLog.log("Singulator/Right/Current", filteredRightCurrent);
  }

  private boolean isLeftJammed() {
    return filteredLeftCurrent > jamCurrentThreshold;
  }

  private boolean isRightJammed() {
    return filteredRightCurrent > jamCurrentThreshold;
  }

  public void setState(SingulatorState newState) {
    setStateFromRequest(newState);
  }
}
