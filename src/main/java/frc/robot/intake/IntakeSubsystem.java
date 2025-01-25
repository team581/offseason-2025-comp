package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class IntakeSubsystem extends StateMachine<IntakeState> {
  private static final double ALGAE_INTAKE_CURRENT = 30;
  private static final double CORAL_INTAKE_CURRENT = 30;

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  private boolean topSensorRaw = false;
  private boolean bottomSensorRaw = false;
  private boolean topSensorDebounced = false;
  private boolean bottomSensorDebounced = false;
  private double topMotorCurrent;
  private double bottomMotorCurrent;
  private boolean hasGP = false;

  public IntakeSubsystem(TalonFX topMotor, TalonFX bottomMotor) {

    super(SubsystemPriority.INTAKE, IntakeState.IDLE_NO_GP);

    topMotor.getConfigurator().apply(RobotConfig.get().intake().topMotorConfig());
    bottomMotor.getConfigurator().apply(RobotConfig.get().intake().bottomMotorConfig());
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;
  }

  @Override
  protected void collectInputs() {
    topMotorCurrent = topMotor.getStatorCurrent().getValueAsDouble();
    bottomMotorCurrent = bottomMotor.getStatorCurrent().getValueAsDouble();
    var averageCurrent = (topMotorCurrent + bottomMotorCurrent) / 2;

    hasGP =
        switch (getState()) {
          case INTAKING_ALGAE -> averageCurrent > ALGAE_INTAKE_CURRENT;
          case INTAKING_CORAL -> averageCurrent > CORAL_INTAKE_CURRENT;
          case SCORE_CORAL -> (averageCurrent < 3) ? false : true;
          default -> false;
        };
  }

  public boolean getTopSensor() {
    return topSensorDebounced;
  }

  public boolean getBottomSensor() {
    return bottomSensorDebounced;
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
        topMotor.disable();
        bottomMotor.disable();
      }
      case IDLE_W_ALGAE -> {
        topMotor.setVoltage(1.5);
        bottomMotor.setVoltage(1.5);
      }
      case IDLE_W_CORAL -> {
        topMotor.setVoltage(0.25);
        bottomMotor.setVoltage(0.25);
      }
      case INTAKING_ALGAE -> {
        topMotor.setVoltage(4.0);
        bottomMotor.setVoltage(4.0);
      }
      case INTAKING_CORAL -> {
        topMotor.setVoltage(10.0);
        bottomMotor.setVoltage(10.0);
      }
      case SCORE_ALGEA_NET -> {
        topMotor.setVoltage(-1.0);
        bottomMotor.setVoltage(-1.0);
      }
      case SCORE_ALGEA_PROCESSOR -> {
        topMotor.setVoltage(-1.0);
        bottomMotor.setVoltage(-1.0);
      }
      case SCORE_CORAL -> {
        topMotor.setVoltage(-1.0);
        bottomMotor.setVoltage(-1.0);
      }
      case OUTTAKING -> {
        topMotor.setVoltage(-1.0);
        bottomMotor.setVoltage(-1.0);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Intake/TopMotor/StatorCurrent", topMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/TopMotor/SupplyCurrent", topMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Intake/TopMotor/AppliedVoltage", topMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log(
        "Intake/BottomMotor/StatorCurrent", bottomMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log(
        "Intake/BottomMotor/SupplyCurrent", bottomMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log(
        "Intake/BottomMotor/AppliedVoltage", bottomMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Intake/Sensors/TopSensorRaw", topSensorRaw);
    DogLog.log("Intake/Sensors/BottomSensorRaw", bottomSensorRaw);
    DogLog.log("Intake/Sensors/TopSensorDebounced", topSensorDebounced);
    DogLog.log("Intake/Sensors/BottomSensorDebounced", bottomSensorDebounced);
    DogLog.log("Intake/HasGP", hasGP);
  }
}
