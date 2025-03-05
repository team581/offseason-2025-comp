package frc.robot.intake;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.config.FeatureFlags;
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

  private final TorqueCurrentFOC algaeHoldRequest =
      new TorqueCurrentFOC(RobotConfig.get().intake().algaeHoldCurrent())
          .withMaxAbsDutyCycle(RobotConfig.get().intake().algaeHoldMaxDutyCycle());

  private boolean rightSensorRaw = false;
  private boolean leftSensorRaw = false;
  private boolean rightSensorDebounced = false;
  private boolean leftSensorDebounced = false;
  private boolean sensorsHaveGP = false;

  private double topMotorVelocity = 0.0;
  private double bottomMotorVelocity = 0.0;

  private final VelocityDetector topMotorAlgaeDetection = new VelocityDetector(32, 0.2);
  private final VelocityDetector bottomMotorAlgaeDetection = new VelocityDetector(30, 0.2);
  private boolean topMotorAlgaeVelocityGp = false;
  private boolean bottomMotorAlgaeVelocityGp = false;
  private final VelocityDetector topMotorCoralDetection = new VelocityDetector(78, 0.2);
  private final VelocityDetector bottomMotorCoralDetection = new VelocityDetector(78, 0.2);
  private boolean topMotorCoralVelocityGp = false;
  private boolean bottomMotorCoralVelocityGp = false;

  public IntakeSubsystem(TalonFX topMotor, TalonFX bottomMotor, CANdi candi) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE_NO_GP);

    topMotor.getConfigurator().apply(RobotConfig.get().intake().topMotorConfig());
    bottomMotor.getConfigurator().apply(RobotConfig.get().intake().bottomMotorConfig());
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;
    this.candi = candi;
  }

  private final Debouncer topAlgaeVelocityDebouncer = new Debouncer(0.0, DebounceType.kRising);
  private final Debouncer bottomAlgaeVelocityDebouncer = new Debouncer(0.0, DebounceType.kRising);

  @Override
  protected void collectInputs() {
    topMotorVelocity = topMotor.getVelocity().getValueAsDouble();
    bottomMotorVelocity = bottomMotor.getVelocity().getValueAsDouble();

    topMotorAlgaeVelocityGp =
        topAlgaeVelocityDebouncer.calculate(
            topMotorAlgaeDetection.hasGamePiece(topMotorVelocity, 20));
    bottomMotorAlgaeVelocityGp =
        bottomAlgaeVelocityDebouncer.calculate(
            bottomMotorAlgaeDetection.hasGamePiece(bottomMotorVelocity, 20));

    topMotorCoralVelocityGp = topMotorCoralDetection.hasGamePiece(topMotorVelocity, 65);
    bottomMotorCoralVelocityGp = bottomMotorCoralDetection.hasGamePiece(bottomMotorVelocity, 65);
    DogLog.log("Intake/TopMotor/Velocity", topMotorVelocity);
    DogLog.log("Intake/BottomMotor/Velocity", bottomMotorVelocity);

    if (RobotConfig.get().intake().sensorFlipped()) {
      leftSensorRaw = candi.getS2State().getValue() != S2StateValue.Low;
      rightSensorRaw = candi.getS1State().getValue() != S1StateValue.Low;
    } else {
      leftSensorRaw = candi.getS2State().getValue() == S2StateValue.Low;
      rightSensorRaw = candi.getS1State().getValue() == S1StateValue.Low;
    }

    leftSensorDebounced = leftDebouncer.calculate(leftSensorRaw);
    rightSensorDebounced = rightDebouncer.calculate(rightSensorRaw);

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
      case INTAKING_CORAL ->
          FeatureFlags.INTAKE_VELOCITY_CORAL_DETECTION.getAsBoolean()
              ? topMotorCoralVelocityGp && bottomMotorCoralVelocityGp && sensorsHaveGP
              : sensorsHaveGP;
      case INTAKING_ALGAE -> topMotorAlgaeVelocityGp && bottomMotorAlgaeVelocityGp;
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
        topMotor.setVoltage(10.0);
        bottomMotor.setVoltage(10.0);
      }
      case IDLE_W_CORAL -> {
        topMotor.setVoltage(0.25);
        bottomMotor.setVoltage(0.25);
      }
      case INTAKING_ALGAE -> {
        topMotor.setVoltage(10.0);
        bottomMotor.setVoltage(10.0);
        topMotorAlgaeDetection.reset();
        bottomMotorAlgaeDetection.reset();
      }
      case INTAKING_CORAL -> {
        topMotor.setVoltage(10.0);
        bottomMotor.setVoltage(10.0);
        topMotorCoralDetection.reset();
        bottomMotorCoralDetection.reset();
      }
      case SCORE_ALGAE_NET_FORWARD -> {
        topMotor.setVoltage(-10.0);
        bottomMotor.setVoltage(-10.0);
      }
      case SCORE_ALGAE_NET_BACK -> {
        topMotor.setVoltage(-10.0);
        bottomMotor.setVoltage(-10.0);
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
