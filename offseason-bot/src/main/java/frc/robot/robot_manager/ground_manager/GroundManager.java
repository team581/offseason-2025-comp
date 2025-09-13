package frc.robot.robot_manager.ground_manager;

import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake_deploy.DeployState;
import frc.robot.intake_deploy.DeploySubsystem;
import frc.robot.singulator.SingulatorState;
import frc.robot.singulator.SingulatorSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class GroundManager extends StateMachine<GroundState> {
  public final IntakeSubsystem intake;
  public final DeploySubsystem deploy;
  public final SingulatorSubsystem singulator;

  // private final CANdi sensor;

  // private final Debouncer topDebouncer = RobotConfig.get().intake().debouncer();

  private static final boolean RAW = false;
  private static final boolean DEBOUNCED = false;

  public GroundManager(
      IntakeSubsystem intake, DeploySubsystem deploy, SingulatorSubsystem singulator /* ,
      CANdi sensor */) {
    super(
        SubsystemPriority.GROUND_MANAGER,
        RobotBase.isSimulation() ? GroundState.IDLE_NO_GP : GroundState.DEPLOY_NOT_HOMED);

    this.intake = intake;
    this.deploy = deploy;
    this.singulator = singulator;
    // this.sensor = sensor;
  }

  @Override
  protected GroundState getNextState(GroundState currentState) {
    return switch (currentState) {
      case DEPLOY_HOMING ->
          deploy.getState() == DeployState.STOWED ? GroundState.IDLE_NO_GP : currentState;
      case INTAKING -> getHasGP() ? GroundState.IDLE_GP : currentState;
      default -> currentState;
    };
  }

  @Override
  protected void afterTransition(GroundState newState) {
    switch (newState) {
      case DEPLOY_HOMING -> {
        intake.setState(IntakeState.STOPPED);
        deploy.rehome();
        singulator.setState(SingulatorState.IDLE);
      }
      case DEPLOY_NOT_HOMED -> {
        intake.setState(IntakeState.STOPPED);
        deploy.setState(DeployState.UNHOMED);
        singulator.setState(SingulatorState.IDLE);
      }
      case IDLE_NO_GP, IDLE_GP -> {
        intake.setState(IntakeState.IDLE);
        deploy.setState(DeployState.STOWED);
        singulator.setState(SingulatorState.IDLE);
      }
      case INTAKING -> {
        intake.setState(IntakeState.INTAKING);
        deploy.setState(DeployState.FLOOR_INTAKE);
        singulator.setState(SingulatorState.INTAKING);
      }
      case UNJAM_LEFT -> {
        intake.setState(IntakeState.OUTTAKING);
        deploy.setState(DeployState.OUTTAKE);
        singulator.setState(SingulatorState.UNJAM_LEFT_ONLY);
      }
      case UNJAM_RIGHT -> {
        intake.setState(IntakeState.OUTTAKING);
        deploy.setState(DeployState.OUTTAKE);
        singulator.setState(SingulatorState.UNJAM_RIGHT_ONLY);
      }
      default -> {}
    }
  }

  private boolean homingOrUnhomed = true;

  @Override
  protected void collectInputs() {
    // raw = sensor.getS2State().getValue() == S2StateValue.High;
    // debounced = topDebouncer.calculate(raw);

    homingOrUnhomed =
        getState() == GroundState.DEPLOY_HOMING || getState() == GroundState.DEPLOY_NOT_HOMED;
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("GroundManager/Sensor/Debounced", DEBOUNCED);
    DogLog.log("GroundManager/Sensor/Raw", RAW);
    DogLog.log("GroundManager/State", getState());
  }

  public boolean getHasGP() {
    return DEBOUNCED;
  }

  public void rehomeRequest() {
    setStateFromRequest(GroundState.DEPLOY_HOMING);
  }

  public void intakeRequest() {
    if (homingOrUnhomed) {
      return;
    }

    setStateFromRequest(GroundState.INTAKING);
  }

  public void stowRequest() {
    if (homingOrUnhomed) {
      return;
    }

    if (getHasGP()) {
      setStateFromRequest(GroundState.IDLE_GP);
      return;
    }
    setStateFromRequest(GroundState.IDLE_NO_GP);
  }
}
