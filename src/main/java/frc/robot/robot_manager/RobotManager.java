package frc.robot.robot_manager;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveState;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionSubsystem;

public class RobotManager extends StateMachine<RobotState> {
  private final VisionSubsystem vision;
  private final ImuSubsystem imu;

  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;
  private final IntakeSubsystem intake;

  public RobotManager(
      IntakeSubsystem intake,
      VisionSubsystem vision,
      ImuSubsystem imu,
      SwerveSubsystem swerve,
      LocalizationSubsystem localization) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE_NO_GP);
    this.intake = intake;
    this.vision = vision;
    this.imu = imu;
    this.swerve = swerve;
    this.localization = localization;
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case IDLE_NO_GP,
              IDLE_ALGAE,
              IDLE_CORAL,
              PROCESSOR_WAITING,
              NET_WAITING,
              CORAL_L1_WAITING,
              CORAL_L2_WAITING,
              CORAL_L3_WAITING,
              CORAL_L4_WAITING,
              CLIMBING_1_LINEUP,
              CLIMBING_2_HANGING ->
          currentState;

        //   TODO: add check for PREPARE_TO_SCORE transitions
      case PROCESSOR_PREPARE_TO_SCORE -> true ? RobotState.PROCESSOR_SCORING : currentState;
      case NET_PREPARE_TO_SCORE -> true ? RobotState.NET_SCORING : currentState;

      case CORAL_L1_PREPARE_TO_SCORE -> true ? RobotState.CORAL_L1_SCORING : currentState;
      case CORAL_L2_PREPARE_TO_SCORE -> true ? RobotState.CORAL_L2_SCORING : currentState;
      case CORAL_L3_PREPARE_TO_SCORE -> true ? RobotState.CORAL_L3_SCORING : currentState;
      case CORAL_L4_PREPARE_TO_SCORE -> true ? RobotState.CORAL_L4_SCORING : currentState;

        // Dislodging
      case DISLODGE_ALGAE_L2_WAIT -> true ? RobotState.DISLODGE_ALGAE_L2_PUSHING : currentState;
      case DISLODGE_ALGAE_L3_WAIT -> true ? RobotState.DISLODGE_ALGAE_L3_PUSHING : currentState;

      case DISLODGE_ALGAE_L2_PUSHING ->
          true
              ? (intake.getHasGP() ? RobotState.CORAL_L2_PREPARE_TO_SCORE : RobotState.IDLE_NO_GP)
              : currentState;
      case DISLODGE_ALGAE_L3_PUSHING ->
          true
              ? (intake.getHasGP() ? RobotState.CORAL_L3_PREPARE_TO_SCORE : RobotState.IDLE_NO_GP)
              : currentState;

        // Scoring
      case CORAL_L1_SCORING,
              CORAL_L2_SCORING,
              CORAL_L3_SCORING,
              CORAL_L4_SCORING,
              PROCESSOR_SCORING,
              NET_SCORING ->
          !intake.getHasGP() ? RobotState.IDLE_NO_GP : currentState;

        // Intaking
      case INTAKE_ALGAE_FLOOR, INTAKE_ALGAE_L2, INTAKE_ALGAE_L3 ->
          intake.getHasGP() ? RobotState.IDLE_ALGAE : currentState;
      case INTAKE_CORAL_FLOOR_HORIZONTAL, INTAKE_CORAL_FLOOR_UPRIGHT, INTAKE_CORAL_STATION ->
          intake.getHasGP() ? RobotState.IDLE_CORAL : currentState;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    // TODO: Implement
    switch (newState) {
      case SCORE_ASSIST -> {
        if (DriverStation.isTeleop()) {
          swerve.setState(SwerveState.SCORE_ASSIST);
        } else {
          swerve.setState(SwerveState.SCORE_ASSIST);
        }
      }
      case PURPLE_ALIGN -> {
        if (DriverStation.isTeleop()) {
          swerve.setState(SwerveState.PURPLE_ALIGN);
        } else {
          swerve.setState(SwerveState.PURPLE_ALIGN);
        }
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    // Continuous state actions
    switch (getState()) {
      default -> {}
    }
  }

  public void confirmScore() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          INTAKE_ALGAE_FLOOR,
          INTAKE_ALGAE_L2,
          INTAKE_ALGAE_L3,
          DISLODGE_ALGAE_L2_WAIT,
          DISLODGE_ALGAE_L3_WAIT,
          DISLODGE_ALGAE_L2_PUSHING,
          DISLODGE_ALGAE_L3_PUSHING,
          INTAKE_CORAL_FLOOR_HORIZONTAL,
          INTAKE_CORAL_FLOOR_UPRIGHT,
          INTAKE_CORAL_STATION -> {}

      case PROCESSOR_WAITING, IDLE_ALGAE ->
          setStateFromRequest(RobotState.PROCESSOR_PREPARE_TO_SCORE);
      case NET_WAITING -> setStateFromRequest(RobotState.NET_PREPARE_TO_SCORE);
      case PROCESSOR_WAITING, IDLE_ALGAE ->
          setStateFromRequest(RobotState.PROCESSOR_PREPARE_TO_SCORE);
      case NET_WAITING -> setStateFromRequest(RobotState.NET_PREPARE_TO_SCORE);

      case CORAL_L1_WAITING -> setStateFromRequest(RobotState.CORAL_L1_PREPARE_TO_SCORE);
      case CORAL_L2_WAITING -> setStateFromRequest(RobotState.CORAL_L2_PREPARE_TO_SCORE);
      case CORAL_L3_WAITING -> setStateFromRequest(RobotState.CORAL_L3_PREPARE_TO_SCORE);
      case CORAL_L4_WAITING -> setStateFromRequest(RobotState.CORAL_L4_PREPARE_TO_SCORE);

        // change default coral score level or algea score if needed
      default -> setStateFromRequest(RobotState.CORAL_L2_PREPARE_TO_SCORE);
      case CORAL_L1_WAITING -> setStateFromRequest(RobotState.CORAL_L1_PREPARE_TO_SCORE);
      case CORAL_L2_WAITING -> setStateFromRequest(RobotState.CORAL_L2_PREPARE_TO_SCORE);
      case CORAL_L3_WAITING -> setStateFromRequest(RobotState.CORAL_L3_PREPARE_TO_SCORE);
      case CORAL_L4_WAITING -> setStateFromRequest(RobotState.CORAL_L4_PREPARE_TO_SCORE);

        // change default coral score level or algea score if needed
      default -> setStateFromRequest(RobotState.CORAL_L2_PREPARE_TO_SCORE);
    }
  }
  }

  public void stowRequest() {}

  public void nextClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP -> setStateFromRequest(RobotState.CLIMBING_2_HANGING);
      case CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
    }
  }

  public void previousClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_2_HANGING -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
      case CLIMBING_1_LINEUP -> {
        setStateFromRequest(RobotState.IDLE_NO_GP);
      }
      default -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
    }
  }
}
