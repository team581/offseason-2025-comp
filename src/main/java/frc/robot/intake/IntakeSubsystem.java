package frc.robot.intake;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class IntakeSubsystem extends StateMachine<IntakeState> {
  private static final Debouncer LEFT_DEBOUNCER = RobotConfig.get().intake().leftDebouncer();
  private static final Debouncer RIGHT_DEBOUNCER = RobotConfig.get().intake().rightDebouncer();

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final CANifier leftSensor;
  private final CANifier rightSensor;

  private boolean leftSensorRaw = false;
  private boolean rightSensorRaw = false;
  private boolean leftSensorDebounced = false;
  private boolean rightSensorDebounced = false;
  private boolean hasGP = false;

  public IntakeSubsystem(
      TalonFX leftMotor, TalonFX rightMotor, CANifier leftSensor, CANifier rightSensor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE_NO_GP);

    leftMotor.getConfigurator().apply(RobotConfig.get().intake().leftMotorConfig());
    rightMotor.getConfigurator().apply(RobotConfig.get().intake().rightMotorConfig());
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.leftSensor = leftSensor;
    this.rightSensor = rightSensor;
  }

  @Override
  protected void collectInputs() {
    leftSensorRaw = leftSensor.getGeneralInput(GeneralPin.LIMF);
    rightSensorRaw = rightSensor.getGeneralInput(GeneralPin.LIMF);
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
        leftMotor.disable();
        rightMotor.disable();
      }
      case IDLE_W_ALGAE -> {
        leftMotor.setVoltage(0.0);
        rightMotor.setVoltage(0.0);
      }
      case IDLE_W_CORAL -> {
        leftMotor.setVoltage(0.0);
        rightMotor.setVoltage(0.0);
      }
      case INTAKING_ALGAE -> {
        leftMotor.setVoltage(0.0);
        rightMotor.setVoltage(0.0);
      }
      case INTAKING_CORAL -> {
        leftMotor.setVoltage(0.0);
        rightMotor.setVoltage(0.0);
      }
      case SCORE_ALGEA_NET -> {
        leftMotor.setVoltage(-0.0);
        rightMotor.setVoltage(-0.0);
      }
      case SCORE_ALGEA_PROCESSOR -> {
        leftMotor.setVoltage(-0.0);
        rightMotor.setVoltage(-0.0);
      }
      case SCORE_CORAL -> {
        leftMotor.setVoltage(-0.0);
        rightMotor.setVoltage(-0.0);
      }
      case OUTTAKING -> {
        leftMotor.setVoltage(-0.0);
        rightMotor.setVoltage(-0.0);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Intake/LeftMotor/StatorCurrent", leftMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/LeftMotor/SupplyCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Intake/LeftMotor/AppliedVoltage", leftMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Intake/RightMotor/StatorCurrent", rightMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/RightMotor/SupplyCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Intake/RightMotor/AppliedVoltage", rightMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Intake/Sensors/LeftSensorRaw", leftSensorRaw);
    DogLog.log("Intake/Sensors/RightSensorRaw", rightSensorRaw);
    DogLog.log("Intake/Sensors/LeftSensorDebounced", leftSensorDebounced);
    DogLog.log("Intake/Sensors/RightSensorDebounced", rightSensorDebounced);
    DogLog.log("Intake/HasGP", hasGP);
  }
}
