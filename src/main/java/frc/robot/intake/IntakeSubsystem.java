package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class IntakeSubsystem extends StateMachine<IntakeState> {
  private static final Debouncer LEFT_DEBOUNCER = RobotConfig.get().intake().leftDebouncer();
  private static final Debouncer RIGHT_DEBOUNCER = RobotConfig.get().intake().rightDebouncer();

  private final TalonFX motor;
  private final DigitalInput leftSensor;
  private final DigitalInput rightSensor;

  private boolean leftSensorRaw = false;
  private boolean rightSensorRaw = false;
  private boolean leftSensorDebounced = false;
  private boolean rightSensorDebounced = false;
  private boolean hasGP = false;

  public IntakeSubsystem(TalonFX motor, DigitalInput leftSensor, DigitalInput rightSensor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE_NO_GP);

    motor.getConfigurator().apply(RobotConfig.get().intake().motorConfig());

    this.motor = motor;
    this.leftSensor = leftSensor;
    this.rightSensor = rightSensor;
  }

  @Override
  protected void collectInputs() {
    leftSensorRaw = leftSensor.get();
    rightSensorRaw = rightSensor.get();
    leftSensorDebounced = LEFT_DEBOUNCER.calculate(leftSensorRaw);
    rightSensorDebounced = RIGHT_DEBOUNCER.calculate(rightSensorRaw);
    hasGP = leftSensorDebounced || rightSensorDebounced;
  }

  public boolean getLeftSensor() {
    return leftSensorDebounced;
  }

  public boolean getRightSensor() {
    return rightSensorDebounced;
  }

  public boolean getHasGP() {
    return hasGP;
  }

  public void setState(IntakeState newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case IDLE_NO_GP -> {
        motor.setVoltage(0.0);
      }
      case IDLE_W_ALGEA -> {
        motor.setVoltage(1.0);
      }
      case IDLE_W_CORAL -> {
        motor.setVoltage(0.0);
      }
      case INTAKING_ALGEA -> {
        motor.setVoltage(6.0);
      }
      case INTAKING_CORAL -> {
        motor.setVoltage(6.0);
      }
      case SCORE_ALGEA_NET -> {
        motor.setVoltage(-4.0);
      }
      case SCORE_ALGEA_PROCESSOR -> {
        motor.setVoltage(-8.0);
      }
      case SCORE_CORAL -> {
        motor.setVoltage(-8.0);
      }
      case OUTTAKING -> {
        motor.setVoltage(-6.0);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Intake/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Intake/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Intake/LeftSensorRaw", leftSensorRaw);
    DogLog.log("Intake/RightSensorRaw", rightSensorRaw);
    DogLog.log("Intake/LeftSensorDebounced", leftSensorDebounced);
    DogLog.log("Intake/RightSensorDebounced", rightSensorDebounced);
    DogLog.log("Intake/HasGP", hasGP);
  }
}
