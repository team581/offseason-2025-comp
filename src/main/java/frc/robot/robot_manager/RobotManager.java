package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefAlignState;
import frc.robot.climber.ClimberState;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.elevator.ElevatorState;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.lights.LightsState;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.purple.Purple;
import frc.robot.robot_manager.collision_avoidance.CollisionAvoidance;
import frc.robot.roll.RollState;
import frc.robot.roll.RollSubsystem;
import frc.robot.swerve.SnapUtil;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.CameraHealth;
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
  public final IntakeSubsystem intake;
  public final WristSubsystem wrist;
  public final ElevatorSubsystem elevator;
  public final RollSubsystem roll;
  public final ClimberSubsystem climber;

  private final Limelight elevatorPurpleLimelight;
  private final Limelight frontCoralLimelight;
  private final Limelight backTagLimelight;

  private final LightsSubsystem lights;

  private final Purple purple;

  private GamePieceMode gamePieceMode;

  public RobotManager(
      IntakeSubsystem intake,
      WristSubsystem wrist,
      ElevatorSubsystem elevator,
      RollSubsystem roll,
      VisionSubsystem vision,
      ImuSubsystem imu,
      SwerveSubsystem swerve,
      LocalizationSubsystem localization,
      Limelight elevatorPurpleLimelight,
      Limelight frontCoralLimelight,
      Limelight backTagLimelight,
      LightsSubsystem lights,
      Purple purple,
      ClimberSubsystem climber) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE_NO_GP);
    this.intake = intake;
    this.wrist = wrist;
    this.elevator = elevator;
    this.roll = roll;
    this.vision = vision;
    this.imu = imu;
    this.swerve = swerve;
    this.localization = localization;
    this.elevatorPurpleLimelight = elevatorPurpleLimelight;
    this.frontCoralLimelight = frontCoralLimelight;
    this.backTagLimelight = backTagLimelight;
    this.lights = lights;
    this.purple = purple;
    this.climber = climber;
  }

  private double reefSnapAngle = 0.0;
  private Pose2d nearestReefSidePose = new Pose2d();

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case IDLE_NO_GP,
              IDLE_ALGAE,
              IDLE_CORAL,
              PROCESSOR_WAITING,
              NET_BACK_WAITING,
              NET_FORWARD_WAITING,
              CORAL_L1_2_LINEUP,
              CORAL_L1_3_PLACE,
              CORAL_L2_2_LINEUP,
              CORAL_L2_3_PLACE,
              CORAL_L3_2_LINEUP,
              CORAL_L3_3_PLACE,
              CORAL_L4_2_LINEUP,
              CORAL_L4_3_PLACE,
              CLIMBING_1_LINEUP,
              CLIMBING_2_HANGING,
              DISLODGE_ALGAE_L2_WAIT,
              DISLODGE_ALGAE_L3_WAIT,
              UNJAM ->
          currentState;

      case PROCESSOR_PREPARE_TO_SCORE ->
          wrist.atGoal() && elevator.atGoal() && roll.atGoal()
              ? RobotState.PROCESSOR_SCORING
              : currentState;
      case NET_BACK_PREPARE_TO_SCORE ->
          wrist.atGoal() && elevator.atGoal() && roll.atGoal()
              ? RobotState.NET_BACK_SCORING
              : currentState;
      case NET_FORWARD_PREPARE_TO_SCORE ->
          wrist.atGoal() && elevator.atGoal() && roll.atGoal()
              ? RobotState.NET_FORWARD_SCORING
              : currentState;

      case CORAL_L1_1_APPROACH ->
          AutoAlign.isCloseToReefSide(localization.getPose(), nearestReefSidePose)
              ? RobotState.CORAL_L1_2_LINEUP
              : currentState;
      case CORAL_L2_1_APPROACH ->
          AutoAlign.isCloseToReefSide(localization.getPose(), nearestReefSidePose)
              ? RobotState.CORAL_L2_2_LINEUP
              : currentState;
      case CORAL_L3_1_APPROACH ->
          AutoAlign.isCloseToReefSide(localization.getPose(), nearestReefSidePose)
              ? RobotState.CORAL_L3_2_LINEUP
              : currentState;
      case CORAL_L4_1_APPROACH ->
          AutoAlign.isCloseToReefSide(localization.getPose(), nearestReefSidePose)
              ? RobotState.CORAL_L4_2_LINEUP
              : currentState;

      // Dislodging
      case DISLODGE_ALGAE_L2_PUSHING ->
          wrist.atGoal() && elevator.atGoal() && roll.atGoal()
              ? (intake.getHasGP() ? RobotState.CORAL_L2_2_LINEUP : RobotState.IDLE_NO_GP)
              : currentState;
      case DISLODGE_ALGAE_L3_PUSHING ->
          wrist.atGoal() && elevator.atGoal() && roll.atGoal()
              ? (intake.getHasGP() ? RobotState.CORAL_L3_2_LINEUP : RobotState.IDLE_NO_GP)
              : currentState;

      // Scoring
      case PROCESSOR_SCORING, NET_FORWARD_SCORING ->
          intake.getHasGP() ? currentState : RobotState.IDLE_NO_GP;

      case CORAL_L1_4_RELEASE, CORAL_L2_4_RELEASE, CORAL_L3_4_RELEASE, CORAL_L4_4_RELEASE ->
          wrist.atGoal() && elevator.atGoal() && timeout(0.5)
              ? RobotState.IDLE_NO_GP
              : currentState;

      // Intaking
      case INTAKE_ALGAE_FLOOR, INTAKE_ALGAE_L2, INTAKE_ALGAE_L3 ->
          intake.getHasGP() ? RobotState.IDLE_ALGAE : currentState;
      case INTAKE_CORAL_FLOOR_HORIZONTAL, INTAKE_CORAL_FLOOR_UPRIGHT ->
          intake.getHasGP() ? RobotState.IDLE_CORAL : currentState;
      case REHOME -> roll.getState() == RollState.STOWED ? RobotState.IDLE_NO_GP : currentState;
      case INTAKE_CORAL_STATION ->
          intake.getHasGP() ? RobotState.AFTER_INTAKE_CORAL_STATION : currentState;
      case NET_BACK_SCORING -> intake.getHasGP() ? currentState : RobotState.AFTER_NET_BACK_WAITING;
      case PRE_INTAKE_CORAL_STATION ->
          wrist.atGoal() && elevator.atGoal() ? RobotState.INTAKE_CORAL_STATION : currentState;
      case PRE_NET_BACK_WAITING ->
          wrist.atGoal() && elevator.atGoal() ? RobotState.NET_BACK_WAITING : currentState;

      case AFTER_INTAKE_CORAL_STATION ->
          wrist.atGoal() && elevator.atGoal() ? RobotState.IDLE_CORAL : currentState;
      case AFTER_NET_BACK_WAITING ->
          wrist.atGoal() && elevator.atGoal() ? RobotState.IDLE_NO_GP : currentState;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case IDLE_NO_GP -> {
        intake.setState(IntakeState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, WristState.IDLE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        // Game piece mode can change without a state transition, so we update lights when the game
        // piece mode is updated
        climber.setState(ClimberState.STOWED);
      }
      case IDLE_ALGAE -> {
        intake.setState(IntakeState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.STOWED, WristState.IDLE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.STOWED);
        frontCoralLimelight.setState(LimelightState.TAGS);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.IDLE_WITH_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case IDLE_CORAL -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.STOWED, WristState.IDLE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.STOWED);
        frontCoralLimelight.setState(LimelightState.TAGS);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.IDLE_WITH_CORAL);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_ALGAE_FLOOR -> {
        intake.setState(IntakeState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.GROUND_ALGAE_INTAKE, WristState.GROUND_ALGAE_INTAKE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.STOWED);
        frontCoralLimelight.setState(LimelightState.CORAL);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.IDLE_NO_GP_ALGAE_MODE);
        climber.setState(ClimberState.STOWED);
      }

      case INTAKE_ALGAE_L2 -> {
        intake.setState(IntakeState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L2, WristState.ALGAE_INTAKE_L2);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.STOWED);
        frontCoralLimelight.setState(LimelightState.TAGS);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.IDLE_NO_GP_ALGAE_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_ALGAE_L3 -> {
        intake.setState(IntakeState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L3, WristState.ALGAE_INTAKE_L3);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.STOWED);
        frontCoralLimelight.setState(LimelightState.TAGS);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.IDLE_NO_GP_ALGAE_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_CORAL_STATION -> {
        intake.setState(IntakeState.INTAKING_CORAL);
        moveSuperstructure(ElevatorState.INTAKING_CORAL_STATION, WristState.INTAKING_CORAL_STATION);
        roll.setState(RollState.STOWED);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getCoralStationAngle(localization.getPose()));
        frontCoralLimelight.setState(LimelightState.TAGS);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_CORAL_FLOOR_UPRIGHT -> {
        intake.setState(IntakeState.INTAKING_CORAL);
        moveSuperstructure(ElevatorState.GROUND_CORAL_INTAKE, WristState.GROUND_CORAL_INTAKE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.STOWED);
        frontCoralLimelight.setState(LimelightState.CORAL);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_CORAL_FLOOR_HORIZONTAL -> {
        intake.setState(IntakeState.INTAKING_CORAL);
        moveSuperstructure(ElevatorState.GROUND_CORAL_INTAKE, WristState.GROUND_CORAL_INTAKE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.INTAKING_CORAL_HORIZONTAL);
        frontCoralLimelight.setState(LimelightState.CORAL);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case DISLODGE_ALGAE_L2_WAIT -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.ALGAE_DISLODGE_L2, WristState.DISLODGE_L2_LOW);
        roll.setState(RollState.STOWED);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.IDLE_NO_GP_ALGAE_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case DISLODGE_ALGAE_L2_PUSHING -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.ALGAE_DISLODGE_L2, WristState.DISLODGE_L2_HIGH);
        roll.setState(RollState.STOWED);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.IDLE_NO_GP_ALGAE_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case DISLODGE_ALGAE_L3_WAIT -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.ALGAE_DISLODGE_L3, WristState.DISLODGE_L3_LOW);
        roll.setState(RollState.STOWED);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.IDLE_NO_GP_ALGAE_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case DISLODGE_ALGAE_L3_PUSHING -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.ALGAE_DISLODGE_L3, WristState.DISLODGE_L3_HIGH);
        roll.setState(RollState.STOWED);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.IDLE_NO_GP_ALGAE_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L1_1_APPROACH, CORAL_L2_1_APPROACH, CORAL_L3_1_APPROACH, CORAL_L4_1_APPROACH -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.STOWED, WristState.IDLE);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L1_2_LINEUP -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L1_LINEUP, WristState.CORAL_SCORE_LINEUP_L1);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L1_3_PLACE -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L1_PLACE, WristState.CORAL_SCORE_PLACING_L1);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L1_4_RELEASE -> {
        intake.setState(IntakeState.SCORE_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L1_RELEASE, WristState.CORAL_SCORE_PLACING_L1);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L2_2_LINEUP -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L2_LINEUP, WristState.CORAL_SCORE_LINEUP_L2);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L2_3_PLACE -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L2_PLACE, WristState.CORAL_SCORE_PLACING_L2);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L2_4_RELEASE -> {
        intake.setState(IntakeState.SCORE_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L2_RELEASE, WristState.CORAL_SCORE_PLACING_L2);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L3_2_LINEUP -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L3_LINEUP, WristState.CORAL_SCORE_LINEUP_L3);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L3_3_PLACE -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L3_PLACE, WristState.CORAL_SCORE_PLACING_L3);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L3_4_RELEASE -> {
        intake.setState(IntakeState.SCORE_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L3_RELEASE, WristState.CORAL_SCORE_PLACING_L3);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L4_2_LINEUP -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L4_LINEUP, WristState.CORAL_SCORE_LINEUP_L4);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L4_3_PLACE -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L4_PLACE, WristState.CORAL_SCORE_PLACING_L4);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L4_4_RELEASE -> {
        intake.setState(IntakeState.SCORE_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L4_RELEASE, WristState.CORAL_SCORE_PLACING_L4);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case NET_BACK_WAITING, NET_BACK_PREPARE_TO_SCORE -> {
        intake.setState(IntakeState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.NET, WristState.ALGAE_BACKWARD_NET);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getBackwardNetDirection());
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case NET_BACK_SCORING -> {
        intake.setState(IntakeState.SCORE_ALGEA_NET);
        moveSuperstructure(ElevatorState.NET, WristState.ALGAE_BACKWARD_NET);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getBackwardNetDirection());
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case NET_FORWARD_WAITING, NET_FORWARD_PREPARE_TO_SCORE -> {
        intake.setState(IntakeState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.NET, WristState.ALGAE_FORWARD_NET);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getForwardNetDirection());
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case NET_FORWARD_SCORING -> {
        intake.setState(IntakeState.SCORE_ALGEA_NET);
        moveSuperstructure(ElevatorState.NET, WristState.ALGAE_FORWARD_NET);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getForwardNetDirection());
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case PROCESSOR_WAITING, PROCESSOR_PREPARE_TO_SCORE -> {
        intake.setState(IntakeState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.PROCESSOR, WristState.ALGAE_PROCESSOR);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getProcessorAngle());
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case PROCESSOR_SCORING -> {
        intake.setState(IntakeState.SCORE_ALGEA_PROCESSOR);
        moveSuperstructure(ElevatorState.PROCESSOR, WristState.ALGAE_PROCESSOR);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getProcessorAngle());
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CLIMBING_1_LINEUP -> {
        intake.setState(IntakeState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, WristState.IDLE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.LINEUP);
      }
      case CLIMBING_2_HANGING -> {
        intake.setState(IntakeState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, WristState.IDLE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.HANGING);
      }
      case UNJAM -> {
        wrist.setState(WristState.UNJAM);
        intake.setState(IntakeState.OUTTAKING);
        elevator.setState(ElevatorState.UNJAM);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.STOWED);
      }
      case REHOME -> {
        wrist.setState(WristState.IDLE);
        intake.setState(IntakeState.IDLE_NO_GP);
        elevator.setState(ElevatorState.STOWED);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.HOMING);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.STOWED);
      }
      case PRE_INTAKE_CORAL_STATION -> {
        intake.setState(IntakeState.INTAKING_CORAL);
        elevator.setState(ElevatorState.PRE_INTAKE_CORAL_STATION);
        wrist.setState(WristState.PRE_INTAKE_CORAL_STATION);
        roll.setState(RollState.STOWED);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getCoralStationAngle(localization.getPose()));
        frontCoralLimelight.setState(LimelightState.CORAL);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case PRE_NET_BACK_WAITING -> {
        intake.setState(IntakeState.IDLE_W_ALGAE);
        elevator.setState(ElevatorState.NET);
        wrist.setState(WristState.IDLE);
        elevator.setState(ElevatorState.NET);
        wrist.setState(WristState.IDLE);

        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getBackwardNetDirection());
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case AFTER_INTAKE_CORAL_STATION -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        elevator.setState(ElevatorState.PRE_INTAKE_CORAL_STATION);
        wrist.setState(WristState.PRE_INTAKE_CORAL_STATION);
        roll.setState(RollState.STOWED);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getCoralStationAngle(localization.getPose()));
        frontCoralLimelight.setState(LimelightState.CORAL);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case AFTER_NET_BACK_WAITING -> {
        intake.setState(IntakeState.IDLE_NO_GP);
        elevator.setState(ElevatorState.NET);
        wrist.setState(WristState.IDLE);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getBackwardNetDirection());
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("RobotManager/NearestReefSidePose", nearestReefSidePose);

    // Continuous state actions

    // Update snaps
    switch (getState()) {
      case CORAL_L1_1_APPROACH,
          CORAL_L1_2_LINEUP,
          CORAL_L1_3_PLACE,
          CORAL_L1_4_RELEASE,
          CORAL_L2_1_APPROACH,
          CORAL_L2_2_LINEUP,
          CORAL_L2_3_PLACE,
          CORAL_L2_4_RELEASE,
          CORAL_L3_1_APPROACH,
          CORAL_L3_2_LINEUP,
          CORAL_L3_3_PLACE,
          CORAL_L3_4_RELEASE,
          CORAL_L4_1_APPROACH,
          CORAL_L4_2_LINEUP,
          CORAL_L4_3_PLACE,
          CORAL_L4_4_RELEASE,
          DISLODGE_ALGAE_L2_WAIT,
          DISLODGE_ALGAE_L3_WAIT,
          DISLODGE_ALGAE_L2_PUSHING,
          DISLODGE_ALGAE_L3_PUSHING,
          INTAKE_ALGAE_L2,
          INTAKE_ALGAE_L3 -> {
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
      }
      case INTAKE_CORAL_STATION -> {
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getCoralStationAngle(localization.getPose()));
      }
      default -> {}
    }

    // Update lights
    switch (getState()) {
      case CORAL_L1_1_APPROACH,
          CORAL_L1_2_LINEUP,
          CORAL_L1_3_PLACE,
          CORAL_L1_4_RELEASE,
          CORAL_L2_1_APPROACH,
          CORAL_L2_2_LINEUP,
          CORAL_L2_3_PLACE,
          CORAL_L2_4_RELEASE,
          CORAL_L3_1_APPROACH,
          CORAL_L3_2_LINEUP,
          CORAL_L3_3_PLACE,
          CORAL_L3_4_RELEASE,
          CORAL_L4_1_APPROACH,
          CORAL_L4_2_LINEUP,
          CORAL_L4_3_PLACE,
          CORAL_L4_4_RELEASE -> {
        lights.setState(getLightStateForScoring());
      }
      default -> {}
    }

    if (DriverStation.isDisabled()) {
      if (elevatorPurpleLimelight.getCameraHealth() == CameraHealth.OFFLINE
          || frontCoralLimelight.getCameraHealth() == CameraHealth.OFFLINE
          || backTagLimelight.getCameraHealth() == CameraHealth.OFFLINE) {
        lights.setState(LightsState.ERROR);
      } else if (!wrist.rangeOfMotionGood()) {
        lights.setState(LightsState.UNHOMED);
      } else {
        lights.setState(LightsState.HEALTHY);
      }

      var currentSuperstructurePosition =
          new SuperstructurePosition(elevator.getHeight(), wrist.getAngle());
      CollisionAvoidance.plan(currentSuperstructurePosition, currentSuperstructurePosition);
    }
  }

  @Override
  protected void collectInputs() {
    super.collectInputs();
    nearestReefSidePose = AutoAlign.getClosestReefSide(localization.getPose());
    reefSnapAngle = nearestReefSidePose.getRotation().getDegrees();
  }

  public void setGamePieceMode(GamePieceMode newMode) {
    gamePieceMode = newMode;

    switch (getState()) {
      case CORAL_L1_4_RELEASE,
          CORAL_L2_4_RELEASE,
          CORAL_L3_4_RELEASE,
          CORAL_L4_4_RELEASE,
          CLIMBING_1_LINEUP,
          NET_BACK_SCORING,
          NET_FORWARD_SCORING,
          PROCESSOR_SCORING,
          CLIMBING_2_HANGING,
          UNJAM,
          REHOME -> {}
      case IDLE_NO_GP, IDLE_ALGAE, IDLE_CORAL -> {
        stowRequest();
      }
      case INTAKE_ALGAE_FLOOR -> {
        if (newMode == GamePieceMode.CORAL) {
          intakeFloorCoralHorizontalRequest();
        }
      }
      case INTAKE_ALGAE_L2, DISLODGE_ALGAE_L2_WAIT, DISLODGE_ALGAE_L2_PUSHING -> {
        if (newMode == GamePieceMode.CORAL) {
          l2CoralLineupRequest();
        }
      }
      case INTAKE_ALGAE_L3, DISLODGE_ALGAE_L3_WAIT, DISLODGE_ALGAE_L3_PUSHING -> {
        if (newMode == GamePieceMode.CORAL) {
          l3CoralLineupRequest();
        }
      }
      case INTAKE_CORAL_STATION -> {
        if (newMode == GamePieceMode.ALGAE) {
          stowRequest();
        }
      }
      case INTAKE_CORAL_FLOOR_UPRIGHT, INTAKE_CORAL_FLOOR_HORIZONTAL -> {
        if (newMode == GamePieceMode.ALGAE) {
          intakeFloorAlgaeRequest();
        }
      }
      case CORAL_L1_1_APPROACH, CORAL_L1_2_LINEUP, CORAL_L1_3_PLACE -> {
        if (newMode == GamePieceMode.ALGAE) {
          processorWaitingRequest();
        }
      }
      case CORAL_L2_1_APPROACH, CORAL_L2_2_LINEUP, CORAL_L2_3_PLACE -> {
        if (newMode == GamePieceMode.ALGAE) {
          intakeAlgaeL2Request();
        }
      }
      case CORAL_L3_1_APPROACH, CORAL_L3_2_LINEUP, CORAL_L3_3_PLACE -> {
        if (newMode == GamePieceMode.ALGAE) {
          intakeAlgaeL3Request();
        }
      }
      case CORAL_L4_1_APPROACH, CORAL_L4_2_LINEUP, CORAL_L4_3_PLACE -> {
        if (newMode == GamePieceMode.ALGAE) {
          algaeNetRequest();
        }
      }
      case NET_BACK_WAITING,
          NET_BACK_PREPARE_TO_SCORE,
          NET_FORWARD_WAITING,
          NET_FORWARD_PREPARE_TO_SCORE -> {
        if (newMode == GamePieceMode.CORAL) {
          l4CoralLineupRequest();
        }
      }
      case PROCESSOR_WAITING, PROCESSOR_PREPARE_TO_SCORE -> {
        if (newMode == GamePieceMode.CORAL) {
          l1CoralLineupRequest();
        }
      }
      default -> {}
    }
  }

  public void stowRequest() {
    DogLog.timestamp("Debug/StowRequest");
    if (intake.getHasGP()) {
      if (gamePieceMode == GamePieceMode.CORAL) {
        setStateFromRequest(RobotState.IDLE_CORAL);
      } else {
        setStateFromRequest(RobotState.IDLE_ALGAE);
      }
    } else {
      setStateFromRequest(RobotState.IDLE_NO_GP);

      if (gamePieceMode == GamePieceMode.CORAL) {
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
      } else {
        lights.setState(LightsState.IDLE_NO_GP_ALGAE_MODE);
      }
    }
  }

  public void intakeFloorRequest() {
    if (gamePieceMode == GamePieceMode.ALGAE) {
      intakeFloorAlgaeRequest();
    } else {
      intakeFloorCoralHorizontalRequest();
    }
  }

  public void intakeFloorAlgaeRequest() {
    gamePieceMode = GamePieceMode.ALGAE;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.INTAKE_ALGAE_FLOOR);
    }
  }

  public void intakeFloorCoralHorizontalRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.INTAKE_CORAL_FLOOR_HORIZONTAL);
    }
  }

  public void intakeStationRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.PRE_INTAKE_CORAL_STATION);
    }
  }

  public void lowLineupRequest() {
    if (gamePieceMode == GamePieceMode.ALGAE) {
      processorWaitingRequest();
    } else {
      l1CoralLineupRequest();
    }
  }

  public void processorWaitingRequest() {
    gamePieceMode = GamePieceMode.ALGAE;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.PROCESSOR_WAITING);
    }
  }

  public void l1CoralLineupRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.CORAL_L1_1_APPROACH);
    }
  }

  public void l2LineupRequest() {
    if (gamePieceMode == GamePieceMode.ALGAE) {
      intakeAlgaeL2Request();
    } else {
      l2CoralLineupRequest();
    }
  }

  private void intakeAlgaeL2Request() {
    gamePieceMode = GamePieceMode.ALGAE;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.INTAKE_ALGAE_L2);
    }
  }

  public void l2CoralLineupRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.CORAL_L2_1_APPROACH);
    }
  }

  public void l3LineupRequest() {
    if (gamePieceMode == GamePieceMode.ALGAE) {
      intakeAlgaeL3Request();
    } else {
      l3CoralLineupRequest();
    }
  }

  private void intakeAlgaeL3Request() {
    gamePieceMode = GamePieceMode.ALGAE;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.INTAKE_ALGAE_L3);
    }
  }

  public void l3CoralLineupRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.CORAL_L3_1_APPROACH);
    }
  }

  public void highLineupRequest() {
    if (gamePieceMode == GamePieceMode.ALGAE) {
      algaeNetRequest();
    } else {
      l4CoralLineupRequest();
    }
  }

  public void algaeNetRequest() {
    if (!vision.isAnyTagLimelightOnline()
        || AutoAlign.shouldNetScoreForwards(localization.getPose())) {
      algaeNetForwardRequest();
    } else {
      algaeNetBackRequest();
    }
  }

  private void algaeNetForwardRequest() {
    gamePieceMode = GamePieceMode.ALGAE;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.NET_FORWARD_WAITING);
    }
  }

  private void algaeNetBackRequest() {
    gamePieceMode = GamePieceMode.ALGAE;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.PRE_NET_BACK_WAITING);
    }
  }

  public void l4CoralLineupRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.CORAL_L4_1_APPROACH);
    }
  }

  public void confirmScoreRequest() {
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

      case CORAL_L1_1_APPROACH -> setStateFromRequest(RobotState.CORAL_L1_2_LINEUP);
      case CORAL_L1_2_LINEUP -> setStateFromRequest(RobotState.CORAL_L1_3_PLACE);
      case CORAL_L1_3_PLACE -> setStateFromRequest(RobotState.CORAL_L1_4_RELEASE);

      case CORAL_L2_1_APPROACH -> setStateFromRequest(RobotState.CORAL_L2_2_LINEUP);
      case CORAL_L2_2_LINEUP -> setStateFromRequest(RobotState.CORAL_L2_3_PLACE);
      case CORAL_L2_3_PLACE -> setStateFromRequest(RobotState.CORAL_L2_4_RELEASE);

      case CORAL_L3_1_APPROACH -> setStateFromRequest(RobotState.CORAL_L3_2_LINEUP);
      case CORAL_L3_2_LINEUP -> setStateFromRequest(RobotState.CORAL_L3_3_PLACE);
      case CORAL_L3_3_PLACE -> setStateFromRequest(RobotState.CORAL_L3_4_RELEASE);

      case CORAL_L4_1_APPROACH -> setStateFromRequest(RobotState.CORAL_L4_2_LINEUP);
      case CORAL_L4_2_LINEUP -> setStateFromRequest(RobotState.CORAL_L4_3_PLACE);
      case CORAL_L4_3_PLACE -> setStateFromRequest(RobotState.CORAL_L4_4_RELEASE);

      default -> setStateFromRequest(RobotState.CORAL_L1_1_APPROACH);
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

  public void unjamRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.UNJAM);
    }
  }

  public void rehomeRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.REHOME);
    }
  }

  private void moveSuperstructure(ElevatorState elevatorGoal, WristState wristGoal) {
    var maybeIntermediaryPosition =
        CollisionAvoidance.plan(
            new SuperstructurePosition(elevator.getHeight(), wrist.getAngle()),
            new SuperstructurePosition(elevatorGoal.height, wristGoal.angle));

    if (maybeIntermediaryPosition.isPresent()) {
      var intermediaryPosition = maybeIntermediaryPosition.get();

      // A collision was detected, so we need to go to an intermediary point
      elevator.setCollisionAvoidanceGoal(intermediaryPosition.elevatorHeight());
      elevator.setState(ElevatorState.COLLISION_AVOIDANCE);
      DogLog.log("RobotManager/CollisionAvoidance/Elevator", intermediaryPosition.elevatorHeight());

      wrist.setCollisionAvoidanceGoal(intermediaryPosition.wristAngle());
      wrist.setState(WristState.COLLISION_AVOIDANCE);
      DogLog.log("RobotManager/CollisionAvoidance/Wrist", intermediaryPosition.wristAngle());

    } else {
      // No collision, go straight to goal state
      elevator.setState(elevatorGoal);
      wrist.setState(wristGoal);
      DogLog.log("RobotManager/CollisionAvoidance/Elevator", 0);
      DogLog.log("RobotManager/CollisionAvoidance/Wrist", 0);
    }
    DogLog.log("RobotManager/CollisionAvoidanceTriggered", maybeIntermediaryPosition.isPresent());
  }

  private ReefAlignState getReefAlignState() {
    return AutoAlign.getReefAlignState(
        localization.getPose(),
        purple.getPurpleState(),
        frontCoralLimelight
            .getInterpolatedTagResult()
            .or(backTagLimelight::getInterpolatedTagResult),
        CameraHealth.combine(
            frontCoralLimelight.getCameraHealth(), backTagLimelight.getCameraHealth()));
  }

  private LightsState getLightStateForScoring() {
    return switch (getReefAlignState()) {
      case CAMERA_DEAD -> LightsState.ERROR;
      case HAS_PURPLE_ALIGNED -> LightsState.SCORE_ALIGN_READY;
      default -> LightsState.SCORE_ALIGN_NOT_READY;
    };
  }
}
