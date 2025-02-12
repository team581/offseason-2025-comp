package frc.robot.intake;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.util.VelocityDetector;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class IntakeSubsystem extends StateMachine<IntakeState> {
  private IntakeConfig CONFIG = RobotConfig.get().intake();
  private final TalonFX topMotor;
  private final TalonFX bottomMotor;
  private final CANdi candi;
  private final Debouncer rightDebouncer = CONFIG.rightDebouncer();
  private final Debouncer leftDebouncer = CONFIG.leftDebouncer();

  private boolean rightSensorRaw = false;
  private boolean leftSensorRaw = false;
  private boolean rightSensorDebounced = false;
  private boolean leftSensorDebounced = false;
  private boolean sensorsHaveGP = false;

  private final VelocityDetector topMotorDetection = CONFIG.topDetector();
  private final VelocityDetector bottomMotorDetection = CONFIG.bottomDetector();
  private double topMotorVelocity = 0.0;
  private double bottomMotorVelocity = 0.0;
  private boolean topMotorHasGp = false;
  private boolean bottomMotorHasGp = false;

  public IntakeSubsystem(TalonFX topMotor, TalonFX bottomMotor, CANdi candi) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE_NO_GP);

    topMotor.getConfigurator().apply(CONFIG.topMotorConfig());
    bottomMotor.getConfigurator().apply(CONFIG.bottomMotorConfig());
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;
    this.candi = candi;
  }

  @Override
  protected void collectInputs() {
    topMotorVelocity = topMotor.getVelocity().getValueAsDouble();
    bottomMotorVelocity = bottomMotor.getVelocity().getValueAsDouble();
    topMotorHasGp = topMotorDetection.hasGamePiece(topMotorVelocity, timeout(0.5));
    bottomMotorHasGp = bottomMotorDetection.hasGamePiece(bottomMotorVelocity, timeout(0.5));

    rightSensorRaw = candi.getS1State().getValue() != S1StateValue.Low;
    leftSensorRaw = candi.getS2State().getValue() != S2StateValue.Low;

    // Sensors are switched around on practice bot
    if (RobotConfig.IS_PRACTICE_BOT) {
      var temp = rightSensorRaw;
      rightSensorRaw = leftSensorRaw;
      leftSensorRaw = temp;
    }

    rightSensorDebounced = rightDebouncer.calculate(rightSensorRaw);
    leftSensorDebounced = leftDebouncer.calculate(leftSensorRaw);

    sensorsHaveGP = rightSensorDebounced || leftSensorDebounced;
  }

  public boolean isCoralCentered() {
    return leftSensorDebounced && rightSensorDebounced;
  }

  public boolean getRightSensor() {
    return rightSensorDebounced;
  }

  public boolean getLeftSensor() {
    return leftSensorDebounced;
  }

  public boolean getHasGP() {
    return switch (getState()) {
      case INTAKING_CORAL, INTAKING_ALGAE -> topMotorHasGp && bottomMotorHasGp;
      default -> sensorsHaveGP;
    };
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
        topMotor.setVoltage(2.0);
        bottomMotor.setVoltage(2.0);
      }
      case IDLE_W_CORAL -> {
        topMotor.setVoltage(0.25);
        bottomMotor.setVoltage(0.25);
      }
      case INTAKING_ALGAE -> {
        topMotor.setVoltage(5.0);
        bottomMotor.setVoltage(5.0);
        topMotorDetection.reset(1);
        bottomMotorDetection.reset(1);
      }
      case INTAKING_CORAL -> {
        topMotor.setVoltage(10.0);
        bottomMotor.setVoltage(10.0);
        topMotorDetection.reset(0);
        bottomMotorDetection.reset(0);
      }
      case SCORE_ALGAE_NET -> {
        topMotor.setVoltage(-1.0);
        bottomMotor.setVoltage(-1.0);
      }
      case SCORE_ALGAE_PROCESSOR -> {
        topMotor.setVoltage(-1.0);
        bottomMotor.setVoltage(-1.0);
      }
      case SCORE_CORAL -> {
        topMotor.setVoltage(-2.0);
        bottomMotor.setVoltage(-2.0);
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

    DogLog.log("Intake/TopMotor/AppliedVoltage", topMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log(
        "Intake/BottomMotor/AppliedVoltage", bottomMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Intake/Sensors/RightSensorRaw", rightSensorRaw);
    DogLog.log("Intake/Sensors/LeftSensorRaw", leftSensorRaw);
    DogLog.log("Intake/Sensors/RightSensorDebounced", rightSensorDebounced);
    DogLog.log("Intake/Sensors/LeftSensorDebounced", leftSensorDebounced);
    DogLog.log("Intake/SensorsHaveGP", sensorsHaveGP);
  }
}
