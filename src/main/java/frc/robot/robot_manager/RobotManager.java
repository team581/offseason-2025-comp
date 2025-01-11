package frc.robot.robot_manager;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.elevator.ElevatorState;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SnapUtil;
import frc.robot.pivot.PivotState;
import frc.robot.pivot.PivotSubsystem;
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
  private final ElevatorSubsystem elevator;
  private final PivotSubsystem pivot;

  private final Limelight topPurpleLimelight;
  private final Limelight bottomCoralLimelight;
  private final Limelight backwardsTagLimelight;

  public RobotManager(
      IntakeSubsystem intake,
      WristSubsystem wrist,
      ElevatorSubsystem elevator,
      PivotSubsystem pivot,
      VisionSubsystem vision,
      ImuSubsystem imu,
      SwerveSubsystem swerve,
      LocalizationSubsystem localization,
      Limelight topPurpleLimelight,
      Limelight bottomCoralLimelight,
      Limelight backwardsTagLimelight) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE_NO_GP);
    this.intake = intake;
    this.wrist = wrist;
    this.elevator = elevator;
    this.pivot = pivot;
    this.vision = vision;
    this.imu = imu;
    this.swerve = swerve;
    this.localization = localization;
    this.topPurpleLimelight = topPurpleLimelight;
    this.bottomCoralLimelight = bottomCoralLimelight;
    this.backwardsTagLimelight = backwardsTagLimelight;
    backwardsTagLimelight.setState(LimelightState.TAGS);
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
              CLIMBING_2_HANGING,
              DISLODGE_ALGAE_L2_WAIT,
              DISLODGE_ALGAE_L3_WAIT ->
          currentState;

      case PROCESSOR_PREPARE_TO_SCORE ->
          wrist.atGoal() && elevator.atGoal() && pivot.atGoal()
              ? RobotState.PROCESSOR_SCORING
              : currentState;
      case NET_BACK_PREPARE_TO_SCORE ->
          wrist.atGoal() && elevator.atGoal() && pivot.atGoal()
              ? RobotState.NET_BACK_SCORING
              : currentState;
      case NET_FORWARD_PREPARE_TO_SCORE ->
          wrist.atGoal() && elevator.atGoal() && pivot.atGoal()
              ? RobotState.NET_FORWARD_SCORING
              : currentState;

      case CORAL_L1_PREPARE_TO_SCORE ->
          wrist.atGoal() && elevator.atGoal() && pivot.atGoal()
              ? RobotState.CORAL_L1_SCORING
              : currentState;
      case CORAL_L2_PREPARE_TO_SCORE ->
          wrist.atGoal() && elevator.atGoal() && pivot.atGoal()
              ? RobotState.CORAL_L2_SCORING
              : currentState;
      case CORAL_L3_PREPARE_TO_SCORE ->
          wrist.atGoal() && elevator.atGoal() && pivot.atGoal()
              ? RobotState.CORAL_L3_SCORING
              : currentState;
      case CORAL_L4_PREPARE_TO_SCORE ->
          wrist.atGoal() && elevator.atGoal() && pivot.atGoal()
              ? RobotState.CORAL_L4_SCORING
              : currentState;

        // Dislodging
      case DISLODGE_ALGAE_L2_PUSHING ->
          wrist.atGoal() && elevator.atGoal() && pivot.atGoal()
              ? (intake.getHasGP() ? RobotState.CORAL_L2_PREPARE_TO_SCORE : RobotState.IDLE_NO_GP)
              : currentState;
      case DISLODGE_ALGAE_L3_PUSHING ->
          wrist.atGoal() && elevator.atGoal() && pivot.atGoal()
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
          intake.getHasGP() ? currentState : RobotState.IDLE_NO_GP;

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
      case IDLE_NO_GP -> {
        wrist.setState(WristState.IDLE);
        intake.setState(IntakeState.IDLE_NO_GP);
        elevator.setState(ElevatorState.STOWED);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.CORAL);
        topPurpleLimelight.setState(LimelightState.TAGS);
      }
      case IDLE_ALGAE -> {
        wrist.setState(WristState.IDLE);
        intake.setState(IntakeState.IDLE_W_ALGAE);
        elevator.setState(ElevatorState.STOWED);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.TAGS);
      }
      case IDLE_CORAL -> {
        wrist.setState(WristState.IDLE);
        intake.setState(IntakeState.IDLE_W_CORAL);
        elevator.setState(ElevatorState.STOWED);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.TAGS);
      }
      case INTAKE_ALGAE_FLOOR -> {
        wrist.setState(WristState.GROUND_ALGAE_INTAKE);
        intake.setState(IntakeState.INTAKING_ALGAE);
        elevator.setState(ElevatorState.GROUND_ALGAE_INTAKE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.CORAL);

        topPurpleLimelight.setState(LimelightState.TAGS);
      }
      case INTAKE_ALGAE_L2 -> {
        wrist.setState(WristState.ALGAE_INTAKE_LV2);
        intake.setState(IntakeState.INTAKING_ALGAE);
        elevator.setState(ElevatorState.ALGAE_INTAKE_L2);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.CORAL);
        topPurpleLimelight.setState(LimelightState.PURPLE);
      }
      case INTAKE_ALGAE_L3 -> {
        wrist.setState(WristState.ALGAE_INTAKE_LV3);
        intake.setState(IntakeState.INTAKING_ALGAE);
        elevator.setState(ElevatorState.ALGAE_INTAKE_L3);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.CORAL);
        topPurpleLimelight.setState(LimelightState.PURPLE);
      }
      case INTAKE_CORAL_STATION -> {
        wrist.setState(WristState.INTAKING_CORAL_STATION);
        intake.setState(IntakeState.INTAKING_CORAL);
        elevator.setState(ElevatorState.INTAKING_CORAL_STATION);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.CORAL);
        topPurpleLimelight.setState(LimelightState.TAGS);
      }
      case INTAKE_CORAL_FLOOR_UPRIGHT -> {
        wrist.setState(WristState.GROUND_CORAL_INTAKE);
        intake.setState(IntakeState.INTAKING_CORAL);
        elevator.setState(ElevatorState.GROUND_CORAL_INTAKE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.CORAL);
        topPurpleLimelight.setState(LimelightState.TAGS);
      }
      case INTAKE_CORAL_FLOOR_HORIZONTAL -> {
        wrist.setState(WristState.GROUND_CORAL_INTAKE);
        intake.setState(IntakeState.INTAKING_CORAL);
        elevator.setState(ElevatorState.GROUND_CORAL_INTAKE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.INTAKING_CORAL_HORIZONTAL);
        bottomCoralLimelight.setState(LimelightState.CORAL);
        topPurpleLimelight.setState(LimelightState.TAGS);
      }
      case DISLODGE_ALGAE_L2_WAIT -> {
        wrist.setState(WristState.DISLODGE_L2_LOW);
        intake.setState(IntakeState.IDLE_W_CORAL);
        elevator.setState(ElevatorState.ALGAE_DISLODGE_L2);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.PURPLE);
      }
      case DISLODGE_ALGAE_L2_PUSHING -> {
        wrist.setState(WristState.DISLODGE_L2_HIGH);
        intake.setState(IntakeState.IDLE_W_CORAL);
        elevator.setState(ElevatorState.ALGAE_DISLODGE_L2);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.PURPLE);
      }
      case DISLODGE_ALGAE_L3_WAIT -> {
        wrist.setState(WristState.DISLODGE_L3_LOW);
        intake.setState(IntakeState.IDLE_W_CORAL);
        elevator.setState(ElevatorState.ALGAE_DISLODGE_L3);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.PURPLE);
      }
      case DISLODGE_ALGAE_L3_PUSHING -> {
        wrist.setState(WristState.DISLODGE_L3_HIGH);
        intake.setState(IntakeState.IDLE_W_CORAL);
        elevator.setState(ElevatorState.ALGAE_DISLODGE_L3);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.PURPLE);
      }
      case CORAL_L1_WAITING, CORAL_L1_PREPARE_TO_SCORE -> {
        wrist.setState(WristState.CORAL_SCORE_LV1);
        intake.setState(IntakeState.IDLE_W_CORAL);
        elevator.setState(ElevatorState.CORAL_L1);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.CORAL_SCORE);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.PURPLE);
      }
      case CORAL_L1_SCORING -> {
        wrist.setState(WristState.CORAL_SCORE_LV1);
        intake.setState(IntakeState.SCORE_CORAL);
        elevator.setState(ElevatorState.CORAL_L1);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.CORAL_SCORE);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.PURPLE);
      }
      case CORAL_L2_WAITING, CORAL_L2_PREPARE_TO_SCORE -> {
        wrist.setState(WristState.CORAL_SCORE_LV2);
        intake.setState(IntakeState.IDLE_W_CORAL);
        elevator.setState(ElevatorState.CORAL_L2);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.CORAL_SCORE);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.PURPLE);
      }
      case CORAL_L2_SCORING -> {
        wrist.setState(WristState.CORAL_SCORE_LV2);
        intake.setState(IntakeState.SCORE_CORAL);
        elevator.setState(ElevatorState.CORAL_L2);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.CORAL_SCORE);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.PURPLE);
      }
      case CORAL_L3_WAITING, CORAL_L3_PREPARE_TO_SCORE -> {
        wrist.setState(WristState.CORAL_SCORE_LV3);
        intake.setState(IntakeState.IDLE_W_CORAL);
        elevator.setState(ElevatorState.CORAL_L3);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.CORAL_SCORE);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.PURPLE);
      }
      case CORAL_L3_SCORING -> {
        wrist.setState(WristState.CORAL_SCORE_LV3);
        intake.setState(IntakeState.SCORE_CORAL);
        elevator.setState(ElevatorState.CORAL_L3);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.CORAL_SCORE);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.PURPLE);
      }
      case CORAL_L4_WAITING, CORAL_L4_PREPARE_TO_SCORE -> {
        wrist.setState(WristState.CORAL_SCORE_LV4);
        intake.setState(IntakeState.IDLE_W_CORAL);
        elevator.setState(ElevatorState.CORAL_L4);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.CORAL_SCORE);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.PURPLE);
      }
      case CORAL_L4_SCORING -> {
        wrist.setState(WristState.CORAL_SCORE_LV4);
        intake.setState(IntakeState.SCORE_CORAL);
        elevator.setState(ElevatorState.CORAL_L4);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.CORAL_SCORE);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.PURPLE);
      }
      case NET_BACK_WAITING, NET_BACK_PREPARE_TO_SCORE -> {
        wrist.setState(WristState.ALGAE_BACKWARD_NET);
        intake.setState(IntakeState.IDLE_W_ALGAE);
        elevator.setState(ElevatorState.NET);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.TAGS);
      }
      case NET_BACK_SCORING -> {
        wrist.setState(WristState.ALGAE_BACKWARD_NET);
        intake.setState(IntakeState.SCORE_ALGEA_NET);
        elevator.setState(ElevatorState.NET);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.TAGS);
      }
      case NET_FORWARD_WAITING, NET_FORWARD_PREPARE_TO_SCORE -> {
        wrist.setState(WristState.ALGAE_FORWARD_NET);
        intake.setState(IntakeState.IDLE_W_ALGAE);
        elevator.setState(ElevatorState.NET);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.TAGS);
      }
      case NET_FORWARD_SCORING -> {
        wrist.setState(WristState.ALGAE_FORWARD_NET);
        intake.setState(IntakeState.SCORE_ALGEA_NET);
        elevator.setState(ElevatorState.NET);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.TAGS);
      }
      case PROCESSOR_WAITING, PROCESSOR_PREPARE_TO_SCORE -> {
        wrist.setState(WristState.ALGAE_PROCESSOR);
        intake.setState(IntakeState.IDLE_W_ALGAE);
        elevator.setState(ElevatorState.PROCESSOR);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getProcessorAngle());
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.TAGS);
      }
      case PROCESSOR_SCORING -> {
        wrist.setState(WristState.ALGAE_PROCESSOR);
        intake.setState(IntakeState.SCORE_ALGEA_PROCESSOR);
        elevator.setState(ElevatorState.PROCESSOR);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getProcessorAngle());
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.TAGS);
      }
      case CLIMBING_1_LINEUP -> {
        wrist.setState(WristState.IDLE);
        intake.setState(IntakeState.IDLE_NO_GP);
        elevator.setState(ElevatorState.STOWED);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.TAGS);
      }
      case CLIMBING_2_HANGING -> {
        wrist.setState(WristState.IDLE);
        intake.setState(IntakeState.IDLE_NO_GP);
        elevator.setState(ElevatorState.STOWED);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        pivot.setState(PivotState.STOWED);
        bottomCoralLimelight.setState(LimelightState.TAGS);
        topPurpleLimelight.setState(LimelightState.TAGS);
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

  public void stowRequest() {
    if (intake.getHasGP()) {
      setStateFromRequest(RobotState.IDLE_CORAL);
    } else {
      setStateFromRequest(RobotState.IDLE_NO_GP);
    }
  }

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
