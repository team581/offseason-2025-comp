package frc.robot.robot_manager.ground_manager;

import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.DSOptions;
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

  private final DigitalInput topSensor;
  private final DigitalInput bottomSensor;

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
      DigitalInput topSensor,
      DigitalInput bottomSensor) {
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

    return switch (currentState) {
      case DEPLOY_HOMING ->
          deploy.getState() == DeployState.STOWED ? GroundState.IDLE_NO_GP : currentState;
      case INTAKING -> {
        if (getTopHasGP()) {
          yield GroundState.IDLE_GP;
        }

        // Prevent from going in an unjamming loop
        if (timeout(0.5)) {
          if (intake.isJammed()) {
            yield GroundState.AUTO_UNJAM;
          }
          if (singulator.isLeftJammed()) {
            yield GroundState.AUTO_UNJAM_LEFT;
          }
          if (singulator.isRightJammed()) {
            yield GroundState.AUTO_UNJAM_RIGHT;
          }
        }

        yield currentState;
      }
      case IDLE_GP -> !getTopHasGP() ? GroundState.IDLE_NO_GP : currentState;
      case IDLE_NO_GP -> getTopHasGP() ? GroundState.IDLE_GP : currentState;
      case AUTO_UNJAM_LEFT, AUTO_UNJAM_RIGHT, AUTO_UNJAM ->
          timeout(0.25) ? GroundState.INTAKING : currentState;
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
        if(DriverStation.isAutonomous()) {
          deploy.setState(DeployState.FLOOR_INTAKE);
        } else {
          deploy.setState(DeployState.STOWED);

        }
        intake.setState(IntakeState.IDLE);
        singulator.setState(SingulatorState.IDLE);
      }
      case INTAKING -> {
        intake.setState(IntakeState.INTAKING);
        deploy.setState(DeployState.FLOOR_INTAKE);
        singulator.setState(SingulatorState.INTAKING);
      }
      case AUTO_UNJAM_LEFT -> {
        intake.setState(IntakeState.OUTTAKING);
        deploy.setState(DeployState.OUTTAKE);
        singulator.setState(SingulatorState.UNJAM_LEFT_ONLY);
      }
      case AUTO_UNJAM_RIGHT -> {
        intake.setState(IntakeState.OUTTAKING);
        deploy.setState(DeployState.OUTTAKE);
        singulator.setState(SingulatorState.UNJAM_RIGHT_ONLY);
      }

      case CLIMB -> {
        intake.setState(IntakeState.STOPPED);
        deploy.setState(DeployState.FLOOR_INTAKE);
        singulator.setState(SingulatorState.STOPPED);
      }

      case OUTTAKING, AUTO_UNJAM -> {
        intake.setState(IntakeState.OUTTAKING);
        deploy.setState(DeployState.OUTTAKE);
        singulator.setState(SingulatorState.OUTTAKING);
      }
    }
  }

  @Override
  protected void collectInputs() {
    topRaw = topSensor.get();
    bottomRaw = bottomSensor.get();
    if (RobotBase.isSimulation() || DSOptions.SENSOR_BROKEN.getAsBoolean()) {
      topRaw =
          switch (getState()) {
            case IDLE_NO_GP -> false;
            case IDLE_GP -> true;
            case OUTTAKING, AUTO_UNJAM -> false;
            case INTAKING -> timeout(2);
            default -> false;
          };

      bottomRaw =
          switch (getState()) {
            case IDLE_NO_GP -> false;
            case IDLE_GP -> true;
            case OUTTAKING, AUTO_UNJAM -> false;
            case INTAKING -> timeout(1.9);
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
    if (getState()!=GroundState.DEPLOY_HOMING){
      setState(GroundState.INTAKING);
    }
  }

  public void outtakeRequest() {
    setState(GroundState.OUTTAKING);
  }

  public void stowRequest() {
    if (getTopHasGP()) {
      setState(GroundState.IDLE_GP);
      return;
    }
    setState(GroundState.IDLE_NO_GP);
  }

  public void climbRequest() {
    setState(GroundState.CLIMB);
  }
}
