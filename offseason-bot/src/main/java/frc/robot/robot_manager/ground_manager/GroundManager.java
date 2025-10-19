package frc.robot.robot_manager.ground_manager;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S2StateValue;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.RobotConfig;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake_deploy.DeployState;
import frc.robot.intake_deploy.DeploySubsystem;
import frc.robot.singulator.SingulatorState;
import frc.robot.singulator.SingulatorSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class GroundManager extends StateMachineSubsystem<GroundState> {
  public final IntakeSubsystem intake;
  public final DeploySubsystem deploy;
  public final SingulatorSubsystem singulator;

  private final CANdi topSensor;
  private final CANdi bottomSensor;

  private final Debouncer topDebouncer = RobotConfig.get().singulator().topDebouncer();
  private final Debouncer bottomDebouncer = RobotConfig.get().singulator().bottonDebouncer();

  private boolean topRaw = false;
  private boolean bottomRaw = false;

  private boolean topDebounced = false;
  private boolean bottomDebounced = false;

  public GroundManager(
      IntakeSubsystem intake,
      DeploySubsystem deploy,
      SingulatorSubsystem singulator,
      CANdi topSensor,
      CANdi bottomSensor) {
    super(
        SubsystemPriority.GROUND_MANAGER,
        RobotBase.isSimulation() ? GroundState.IDLE_NO_GP : GroundState.DEPLOY_NOT_HOMED);

    this.intake = intake;
    this.deploy = deploy;
    this.singulator = singulator;
    this.topSensor = topSensor;
    this.bottomSensor = bottomSensor;
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
      case INTAKING -> getTopHasGP() ? GroundState.IDLE_GP : currentState;
      case INTAKE_THEN_HANDOFF_WAIT -> getTopHasGP() ? GroundState.HANDOFF_WAIT : currentState;
      case HANDOFF_RELEASE, OUTTAKING -> getTopHasGP() ? currentState : GroundState.IDLE_NO_GP;

      // TODO: Adjust timeouts and make work for INTAKE_THEN_HANDOFF
      case UNJAM_LEFT, UNJAM_RIGHT -> timeout(0) ? GroundState.INTAKING : currentState;
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

      case CLIMB -> {
        intake.setState(IntakeState.STOPPED);
        deploy.setState(DeployState.STOWED);
        singulator.setState(SingulatorState.STOPPED);
      }

      case OUTTAKING -> {
        intake.setState(IntakeState.OUTTAKING);
        deploy.setState(DeployState.OUTTAKE);
        singulator.setState(SingulatorState.OUTTAKING);
      }
      case HANDOFF_WAIT -> {
        intake.setState(IntakeState.HANDOFF);
        deploy.setState(DeployState.HANDOFF);
        singulator.setState(SingulatorState.IDLE);
      }
      case HANDOFF_RELEASE -> {
        intake.setState(IntakeState.HANDOFF);
        deploy.setState(DeployState.HANDOFF);
        singulator.setState(SingulatorState.HANDOFF);
      }
    }
  }

  @Override
  protected void collectInputs() {
    topRaw = topSensor.getS2State().getValue() == S2StateValue.High;
    bottomRaw = bottomSensor.getS2State().getValue() == S2StateValue.High;
    if (RobotBase.isSimulation()) {
      topRaw =
          switch (getState()) {
            case HANDOFF_RELEASE -> !timeout(0.5);
            case IDLE_NO_GP -> false;
            case IDLE_GP -> true;
            case HANDOFF_WAIT -> true;
            case INTAKING, INTAKE_THEN_HANDOFF_WAIT -> timeout(2);
            default -> false;
          };

      bottomRaw =
          switch (getState()) {
            case HANDOFF_RELEASE -> !timeout(0.5);
            case IDLE_NO_GP -> false;
            case IDLE_GP -> true;
            case HANDOFF_WAIT -> true;
            case INTAKING, INTAKE_THEN_HANDOFF_WAIT -> timeout(1.9);
            default -> false;
          };
    }
    topDebounced = topDebouncer.calculate(topRaw);
    bottomDebounced = bottomDebouncer.calculate(bottomRaw);
  }

  @Override
  public void whileInState(GroundState currentState) {
    DogLog.log("GroundManager/Sensors/TopRaw", topRaw);
    DogLog.log("GroundManager/Sensors/BottomRaw", bottomRaw);
    DogLog.log("GroundManager/Sensors/TopDebounced", topDebounced);
    DogLog.log("GroundManager/Sensors/BottomDebounced", bottomDebounced);
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

  public boolean getBottomHasGP() {
    return bottomDebounced;
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
