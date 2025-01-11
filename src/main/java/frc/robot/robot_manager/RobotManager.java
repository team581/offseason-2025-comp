package frc.robot.robot_manager;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveState;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import frc.robot.wrist.WristState;
import frc.robot.wrist.WristSubsystem;

public class RobotManager extends StateMachine<RobotState> {
  public final LocalizationSubsystem localization;

  private final VisionSubsystem vision;
  private final ImuSubsystem imu;

  private final SwerveSubsystem swerve;
  private final IntakeSubsystem intake;
  private final WristSubsystem wrist;

  private final Limelight topLimelight;
  private final Limelight bottomLimelight;
  private final Limelight backLimelight;

  public RobotManager(
      IntakeSubsystem intake,
      WristSubsystem wrist,
      VisionSubsystem vision,
      ImuSubsystem imu,
      SwerveSubsystem swerve,
      LocalizationSubsystem localization,
      Limelight topLimelight,
      Limelight bottomLimelight,
      Limelight backLimelight) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE_NO_GP);
    this.intake = intake;
    this.wrist = wrist;
    this.vision = vision;
    this.imu = imu;
    this.swerve = swerve;
    this.localization = localization;
    this.topLimelight = topLimelight;
    this.bottomLimelight = bottomLimelight;
    this.backLimelight = backLimelight;
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case IDLE_NO_GP,
              IDLE_ALGAE,
              IDLE_CORAL,
              PROCESSOR_WAITING,
              NET_BACK_WAITING,
              NET_FORWARD_WAITING,
              CORAL_L1_WAITING,
              CORAL_L2_WAITING,
              CORAL_L3_WAITING,
              CORAL_L4_WAITING,
              CLIMBING_1_LINEUP,
              CLIMBING_2_HANGING ->
          currentState;

        //   TODO: add check for PREPARE_TO_SCORE transitions
      case PROCESSOR_PREPARE_TO_SCORE -> true ? RobotState.PROCESSOR_SCORING : currentState;
      case NET_BACK_PREPARE_TO_SCORE -> true ? RobotState.NET_BACK_SCORING : currentState;
      case NET_FORWARD_PREPARE_TO_SCORE -> true ? RobotState.NET_FORWARD_SCORING : currentState;

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
              NET_BACK_SCORING,
              NET_FORWARD_SCORING ->
          !intake.getHasGP() ? RobotState.IDLE_NO_GP : currentState;

        // Intaking
      case INTAKE_ALGAE_FLOOR, INTAKE_ALGAE_L2, INTAKE_ALGAE_L3 ->
          intake.getHasGP() ? RobotState.IDLE_ALGAE : currentState;
      case INTAKE_CORAL_FLOOR_HORIZONTAL, INTAKE_CORAL_FLOOR_UPRIGHT, INTAKE_CORAL_STATION ->
          intake.getHasGP() ? RobotState.IDLE_CORAL : currentState;

      case SCORE_ASSIST -> currentState;
      case PURPLE_ALIGN -> currentState;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    // TODO: Implement
    switch (newState) {
      case SCORE_ASSIST -> {
        // Demo of how to set the state of limelight to coral detection
        bottomLimelight.setState(LimelightState.CORAL);
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
      case IDLE_NO_GP -> {
        wrist.setState(WristState.IDLE);
        intake.setState(IntakeState.IDLE_NO_GP);
      }
      case IDLE_ALGAE -> {
        wrist.setState(WristState.IDLE);
        intake.setState(IntakeState.IDLE_W_ALGEA);
      }
      case IDLE_CORAL -> {
        wrist.setState(WristState.IDLE);
        intake.setState(IntakeState.IDLE_W_CORAL);
      }
      case INTAKE_ALGAE_FLOOR -> {
        wrist.setState(WristState.GROUND_ALGAE_INTAKE);
        intake.setState(IntakeState.INTAKING_ALGEA);
      }
      case INTAKE_ALGAE_L2 -> {
        wrist.setState(WristState.CORAL_SCORE_LV2);
        intake.setState(IntakeState.INTAKING_ALGEA);
      }
      case INTAKE_ALGAE_L3 -> {
        wrist.setState(WristState.CORAL_SCORE_LV3);
        intake.setState(IntakeState.INTAKING_ALGEA);
      }
      case INTAKE_CORAL_STATION -> {
        wrist.setState(WristState.SOURCE_INTAKE);
        intake.setState(IntakeState.INTAKING_CORAL);
      }
      case INTAKE_CORAL_FLOOR_UPRIGHT, INTAKE_CORAL_FLOOR_HORIZONTAL -> {
        wrist.setState(WristState.GROUND_CORAL_INTAKE);
        intake.setState(IntakeState.INTAKING_CORAL);
      }
      case CORAL_L1_WAITING, CORAL_L1_PREPARE_TO_SCORE -> {
        wrist.setState(WristState.CORAL_SCORE_LV1);
        intake.setState(IntakeState.IDLE_W_CORAL);
      }
      case CORAL_L1_SCORING -> {
        wrist.setState(WristState.CORAL_SCORE_LV1);
        intake.setState(IntakeState.SCORE_CORAL);
      }
      case CORAL_L2_WAITING, CORAL_L2_PREPARE_TO_SCORE -> {
        wrist.setState(WristState.CORAL_SCORE_LV2);
        intake.setState(IntakeState.IDLE_W_CORAL);
      }
      case CORAL_L2_SCORING -> {
        wrist.setState(WristState.CORAL_SCORE_LV2);
        intake.setState(IntakeState.SCORE_CORAL);
      }
      case CORAL_L3_WAITING, CORAL_L3_PREPARE_TO_SCORE -> {
        wrist.setState(WristState.CORAL_SCORE_LV3);
        intake.setState(IntakeState.IDLE_W_CORAL);
      }
      case CORAL_L3_SCORING -> {
        wrist.setState(WristState.CORAL_SCORE_LV3);
        intake.setState(IntakeState.SCORE_CORAL);
      }
      case CORAL_L4_WAITING, CORAL_L4_PREPARE_TO_SCORE -> {
        wrist.setState(WristState.CORAL_SCORE_LV4);
        intake.setState(IntakeState.IDLE_W_CORAL);
      }
      case CORAL_L4_SCORING -> {
        wrist.setState(WristState.CORAL_SCORE_LV4);
        intake.setState(IntakeState.SCORE_CORAL);
      }
      case NET_BACK_WAITING, NET_BACK_PREPARE_TO_SCORE -> {
        wrist.setState(WristState.ALGAE_BACKWARD_NET);
        intake.setState(IntakeState.IDLE_W_ALGEA);
      }
      case NET_BACK_SCORING -> {
        wrist.setState(WristState.ALGAE_BACKWARD_NET);
        intake.setState(IntakeState.SCORE_ALGEA_NET);
      }
      case NET_FORWARD_WAITING, NET_FORWARD_PREPARE_TO_SCORE -> {
        wrist.setState(WristState.ALGAE_FORWARD_NET);
        intake.setState(IntakeState.IDLE_W_ALGEA);
      }
      case NET_FORWARD_SCORING -> {
        wrist.setState(WristState.ALGAE_FORWARD_NET);
        intake.setState(IntakeState.SCORE_ALGEA_NET);
      }
      case PROCESSOR_WAITING, PROCESSOR_PREPARE_TO_SCORE -> {
        wrist.setState(WristState.ALGAE_PROCESSOR);
        intake.setState(IntakeState.IDLE_W_ALGEA);
      }
      case PROCESSOR_SCORING -> {
        wrist.setState(WristState.ALGAE_PROCESSOR);
        intake.setState(IntakeState.SCORE_ALGEA_PROCESSOR);
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
      case NET_BACK_WAITING -> setStateFromRequest(RobotState.NET_BACK_PREPARE_TO_SCORE);
      case NET_FORWARD_WAITING -> setStateFromRequest(RobotState.NET_FORWARD_PREPARE_TO_SCORE);

        // change default coral score level or algea score if needed
      default -> setStateFromRequest(RobotState.CORAL_L2_PREPARE_TO_SCORE);
      case CORAL_L1_WAITING -> setStateFromRequest(RobotState.CORAL_L1_PREPARE_TO_SCORE);
      case CORAL_L2_WAITING -> setStateFromRequest(RobotState.CORAL_L2_PREPARE_TO_SCORE);
      case CORAL_L3_WAITING -> setStateFromRequest(RobotState.CORAL_L3_PREPARE_TO_SCORE);
      case CORAL_L4_WAITING -> setStateFromRequest(RobotState.CORAL_L4_PREPARE_TO_SCORE);
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
