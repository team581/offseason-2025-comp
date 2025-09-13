package frc.robot.robot_manager.ground_manager;

import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
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
  private static final boolean TOP_DEBOUNCED = false;
  private static final boolean BOTTOM_DEBOUNCED = false;


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
    if (singulator.isLeftJammed()) {
      return GroundState.UNJAM_LEFT;
    }
    if (singulator.isRightJammed()) {
      return GroundState.UNJAM_RIGHT;
    }
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
      case INTAKING, INTAKE_THEN_HANDOFF_WAIT -> {
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
      case L1_WAIT -> {
        intake.setState(IntakeState.SCORING);
        deploy.setState(DeployState.L1_SCORE);
        singulator.setState(SingulatorState.IDLE);
      }
      default -> {}
    }
  }

  @Override
  protected void collectInputs() {
    topRaw = topSensor.getS2State().getValue() == S2StateValue.High;
    bottomRaw = bottomSensor.getS2State().getValue() == S2StateValue.High;
    topDebounced = topDebouncer.calculate(topRaw);
    bottomDebounced = bottomDebouncer.calculate(bottomRaw);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("GroundManager/Sensor/Debounced", DEBOUNCED);
    DogLog.log("GroundManager/Sensor/Raw", RAW);
    DogLog.log("GroundManager/State", getState());
  }

  private void setState(GroundState newState) {
    switch (deploy.getState()) {
      case UNHOMED, REHOME -> {}
      default -> setStateFromRequest(newState);
    }
  }

  public boolean getTopHasGP() {
    return topDebounced;
  }
  private void setState(GroundState newState) {
    switch (deploy.getState()) {
      case UNHOMED, REHOME -> {}
      default -> setStateFromRequest(newState);
    }
  }

  public void rehomeRequest() {
    setStateFromRequest(GroundState.DEPLOY_HOMING);
  }

  public void intakeRequest() {
    setState(GroundState.INTAKING);
  }

  public void stowRequest() {
    if (getTopHasGP()) {
      setState(GroundState.IDLE_GP);
      return;
    }
    setState(GroundState.IDLE_NO_GP);
  }

  public void hardL1Request() {
    switch (getState()) {
      case L1_WAIT, L1_HARD_WAIT -> setState(GroundState.L1_HARD_SCORE);
      default -> setState(GroundState.L1_WAIT);
    }
  }

  public void l1Request() {
    switch (getState()) {
      case L1_WAIT, L1_HARD_WAIT -> setState(GroundState.L1_SCORE);
      default -> setState(GroundState.L1_WAIT);
    }
  }

  public void hardL1WaitRequest() {
    switch (getState()) {
      case L1_WAIT, L1_HARD_WAIT -> setState(GroundState.L1_HARD_WAIT);
      default -> setState(GroundState.L1_WAIT);
    }
  }

  public void l1WaitRequest() {
    setState(GroundState.L1_WAIT);
  }

  public void intakeThenHandoffRequest() {
    if (getState() == GroundState.INTAKING
        || DriverStation.isAutonomous()
        || getState() == GroundState.HANDOFF_WAIT
        || getState() == GroundState.HANDOFF_RELEASE) {
      setState(GroundState.INTAKE_THEN_HANDOFF_WAIT);
    } else {
      setState(GroundState.HANDOFF_WAIT);
    }
  }

  public void handoffReleaseRequest() {
    setState(GroundState.HANDOFF_RELEASE);
  }

  public void climbRequest() {
    setState(GroundState.CLIMB);
  }
}
