package frc.robot.intake;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig;
import frc.robot.util.VelocityDetector;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class IntakeSubsystem extends StateMachine<IntakeState> {

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;
  private final CANdi candi;
  private final Debouncer rightDebouncer = RobotConfig.get().intake().rightDebouncer();
  private final Debouncer leftDebouncer = RobotConfig.get().intake().leftDebouncer();

  private boolean rightSensorRaw = false;
  private boolean leftSensorRaw = false;
  private boolean rightSensorDebounced = false;
  private boolean leftSensorDebounced = false;
  private boolean sensorsHaveGP = false;

  // TODO: add minimum velocities
  private final VelocityDetector topMotorDetection = new VelocityDetector(0.0, 0.0);
  private final VelocityDetector bottomMotorDetection = new VelocityDetector(0.0, 0.0);
  private boolean topMotorHasGp = false;
  private boolean bottomMotorHasGp = false;

  public IntakeSubsystem(TalonFX topMotor, TalonFX bottomMotor, CANdi candi) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE_NO_GP);

    topMotor.getConfigurator().apply(RobotConfig.get().intake().topMotorConfig());
    bottomMotor.getConfigurator().apply(RobotConfig.get().intake().bottomMotorConfig());
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;
    this.candi = candi;
  }

  @Override
  protected void collectInputs() {
    topMotorHasGp =
        topMotorDetection.hasGamePiece(
            Units.rotationsToDegrees(topMotor.getVelocity().getValueAsDouble()));
    bottomMotorHasGp =
        bottomMotorDetection.hasGamePiece(
            Units.rotationsToDegrees(bottomMotor.getVelocity().getValueAsDouble()));

    rightSensorRaw = candi.getS1State().getValue() != S1StateValue.Low;
    leftSensorRaw = candi.getS2State().getValue() != S2StateValue.Low;
    rightSensorDebounced = rightDebouncer.calculate(rightSensorRaw);
    leftSensorDebounced = leftDebouncer.calculate(leftSensorRaw);

    sensorsHaveGP = rightSensorDebounced || leftSensorDebounced;
  }

  public boolean getRightSensor() {
    return rightSensorDebounced;
  }

  public boolean getLeftSensor() {
    return leftSensorDebounced;
  }

  public boolean getHasGP() {
    return switch (getState()) {
      // case INTAKING_CORAL, INTAKING_ALGAE -> topMotorHasGp && bottomMotorHasGp;
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
        topMotorDetection.reset();
        bottomMotorDetection.reset();
      }
      case INTAKING_CORAL -> {
        topMotor.setVoltage(10.0);
        bottomMotor.setVoltage(10.0);
        topMotorDetection.reset();
        bottomMotorDetection.reset();
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
    //    DogLog.log("Intake/TopMotor/CalulatedMotorCurrent", calculateTopIntakeMotorCurrent);
    //    DogLog.log("Intake/BottomMotor/CalculateMotorCurrent", calculateBottomIntakeMotorCurrent);
  }
}
