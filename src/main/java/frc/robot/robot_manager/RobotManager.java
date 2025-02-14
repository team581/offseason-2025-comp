package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefAlignState;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.climber.ClimberState;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.controller.RumbleControllerSubsystem;
import frc.robot.elevator.ElevatorState;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake_assist.IntakeAssistUtil;
import frc.robot.lights.LightsState;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.purple.Purple;
import frc.robot.robot_manager.collision_avoidance.CollisionAvoidance;
import frc.robot.roll.RollState;
import frc.robot.roll.RollSubsystem;
import frc.robot.swerve.SnapUtil;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.MathHelpers;
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
  public final RumbleControllerSubsystem rumbleController;

  private final Limelight elevatorPurpleLimelight;
  private final Limelight frontCoralLimelight;
  private final Limelight backTagLimelight;
  private final Limelight baseTagLimelight;

  private final LightsSubsystem lights;

  public final Purple purple;

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
      Limelight baseTagLimelight,
      LightsSubsystem lights,
      Purple purple,
      ClimberSubsystem climber,
      RumbleControllerSubsystem rumbleController) {
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
    this.baseTagLimelight = baseTagLimelight;
    this.lights = lights;
    this.purple = purple;
    this.climber = climber;
    this.rumbleController = rumbleController;
  }

  private double reefSnapAngle = 0.0;
  private Pose2d nearestReefSidePose = Pose2d.kZero;
  private ReefPipeLevel scoringLevel = ReefPipeLevel.BASE;
  private ChassisSpeeds purpleSpeeds = new ChassisSpeeds();
  private boolean confirmScoreActive = false;

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case IDLE_NO_GP,
              IDLE_ALGAE,
              IDLE_CORAL,
              PROCESSOR_WAITING,
              NET_BACK_WAITING,
              NET_FORWARD_WAITING,
              CORAL_L1_3_PLACE,
              CORAL_CENTERED_L2_3_PLACE,
              CORAL_CENTERED_L3_3_PLACE,
              CORAL_CENTERED_L4_3_PLACE,
              CORAL_DISPLACED_L2_3_PLACE,
              CORAL_DISPLACED_L3_3_PLACE,
              CORAL_DISPLACED_L4_3_PLACE,
              CLIMBING_1_LINEUP,
              CLIMBING_2_HANGING,
              DISLODGE_ALGAE_L2_WAIT,
              DISLODGE_ALGAE_L3_WAIT,
              UNJAM ->
          currentState;

      // in auto: keep transition the same
      // in teleop: if right trigger is pressed, go 2 -> 3. else, stay in 2

      case REHOME_ELEVATOR ->
          elevator.getState() == ElevatorState.STOWED ? RobotState.IDLE_NO_GP : currentState;
      case REHOME_WRIST ->
          wrist.getState() == WristState.CORAL_STOWED ? RobotState.IDLE_NO_GP : currentState;
      case REHOME_ROLL ->
          roll.getState() == RollState.STOWED ? RobotState.IDLE_NO_GP : currentState;
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
          AutoAlign.isCloseToReefSide(
                  localization.getPose(), nearestReefSidePose, swerve.getFieldRelativeSpeeds())
              ? RobotState.CORAL_L1_3_PLACE
              : currentState;
      case CORAL_L2_1_APPROACH -> {
        var isClose =
            AutoAlign.isCloseToReefSide(
                localization.getPose(), nearestReefSidePose, swerve.getFieldRelativeSpeeds());

        if (!isClose) {
          yield currentState;
        }

        yield intake.isCoralCentered()
            ? RobotState.CORAL_CENTERED_L2_2_LINEUP
            : RobotState.CORAL_DISPLACED_L2_2_LINEUP;
      }
      case CORAL_CENTERED_L2_2_LINEUP ->
          shouldProgressTeleopScore() ? RobotState.CORAL_CENTERED_L2_3_PLACE : currentState;
      case CORAL_CENTERED_L3_2_LINEUP ->
          shouldProgressTeleopScore() ? RobotState.CORAL_CENTERED_L3_3_PLACE : currentState;
      case CORAL_CENTERED_L4_2_LINEUP ->
          shouldProgressTeleopScore() ? RobotState.CORAL_CENTERED_L4_3_PLACE : currentState;

      case CORAL_DISPLACED_L2_2_LINEUP ->
          shouldProgressTeleopScore() ? RobotState.CORAL_DISPLACED_L2_3_PLACE : currentState;
      case CORAL_DISPLACED_L3_2_LINEUP ->
          shouldProgressTeleopScore() ? RobotState.CORAL_DISPLACED_L3_3_PLACE : currentState;
      case CORAL_DISPLACED_L4_2_LINEUP ->
          shouldProgressTeleopScore() ? RobotState.CORAL_DISPLACED_L4_3_PLACE : currentState;

      case CORAL_L3_1_APPROACH -> {
        var isClose =
            AutoAlign.isCloseToReefSide(
                localization.getPose(), nearestReefSidePose, swerve.getFieldRelativeSpeeds());

        if (!isClose) {
          yield currentState;
        }

        yield intake.isCoralCentered()
            ? RobotState.CORAL_CENTERED_L3_2_LINEUP
            : RobotState.CORAL_DISPLACED_L3_2_LINEUP;
      }
      case CORAL_L4_1_APPROACH -> {
        var isClose =
            AutoAlign.isCloseToReefSide(
                localization.getPose(), nearestReefSidePose, swerve.getFieldRelativeSpeeds());

        if (!isClose) {
          yield currentState;
        }

        yield intake.isCoralCentered()
            ? RobotState.CORAL_CENTERED_L4_2_LINEUP
            : RobotState.CORAL_DISPLACED_L4_2_LINEUP;
      }

      // Dislodging
      case DISLODGE_ALGAE_L2_PUSHING -> {
        if (wrist.atGoal() && elevator.atGoal() && roll.atGoal()) {
          if (intake.getHasGP()) {
            yield RobotState.CORAL_CENTERED_L2_2_LINEUP;
          } else if (cameraOnlineAndFarEnoughFromReef()) {
            yield RobotState.IDLE_NO_GP;
          }
        }
        yield currentState;
      }
      case DISLODGE_ALGAE_L3_PUSHING -> {
        if (wrist.atGoal() && elevator.atGoal() && roll.atGoal()) {
          if (intake.getHasGP()) {
            yield RobotState.CORAL_CENTERED_L3_2_LINEUP;
          } else if (cameraOnlineAndFarEnoughFromReef()) {
            yield RobotState.IDLE_NO_GP;
          }
        }
        yield currentState;
      }
      // Scoring
      case PROCESSOR_SCORING, NET_FORWARD_SCORING -> {
        if (intake.getHasGP()) {
          yield currentState;
        }

        rumbleController.rumbleRequest();
        yield RobotState.IDLE_NO_GP;
      }
      case ALGAE_OUTTAKE -> {
        if (timeout(0.5)) {
          yield RobotState.IDLE_NO_GP;
        }
        yield currentState;
      }
      case CORAL_L1_4_RELEASE -> intake.getHasGP() ? currentState : RobotState.IDLE_NO_GP;
      case CORAL_CENTERED_L2_4_RELEASE,
          CORAL_CENTERED_L3_4_RELEASE,
          CORAL_CENTERED_L4_4_RELEASE,
          CORAL_DISPLACED_L2_4_RELEASE,
          CORAL_DISPLACED_L3_4_RELEASE,
          CORAL_DISPLACED_L4_4_RELEASE -> {
        var done =
            wrist.atGoal()
                && elevator.atGoal()
                && (DriverStation.isTeleop() ? cameraOnlineAndFarEnoughFromReef() : timeout(0.5));

        if (done) {
          rumbleController.rumbleRequest();
          yield RobotState.IDLE_NO_GP;
        }

        yield currentState;
      }

      case CORAL_CENTERED_L4_3_PLACE_THEN_RELEASE ->
          wrist.atGoal() && elevator.atGoal() && roll.atGoal()
              ? RobotState.CORAL_CENTERED_L4_4_RELEASE
              : currentState;

      case CORAL_DISPLACED_L4_3_PLACE_THEN_RELEASE ->
          wrist.atGoal() && elevator.atGoal() && roll.atGoal()
              ? RobotState.CORAL_DISPLACED_L4_4_RELEASE
              : currentState;

      // Intaking
      case INTAKE_ALGAE_FLOOR -> {
        if (intake.getHasGP()) {
          rumbleController.rumbleRequest();
          yield RobotState.IDLE_ALGAE;
        }

        yield currentState;
      }
      case INTAKE_ALGAE_L2, INTAKE_ALGAE_L3 -> {
        if (intake.getHasGP()) {
          rumbleController.rumbleRequest();
          if (cameraOnlineAndFarEnoughFromReef()) {
            yield RobotState.IDLE_ALGAE;
          }
        }

        yield currentState;
      }

      case INTAKE_CORAL_FLOOR_HORIZONTAL,
          INTAKE_ASSIST_CORAL_FLOOR_HORIZONTAL,
          INTAKE_CORAL_FLOOR_UPRIGHT -> {
        if (intake.getHasGP()) {
          rumbleController.rumbleRequest();
          yield RobotState.IDLE_CORAL;
        }

        yield currentState;
      }
      case INTAKE_STATION_APPROACH -> {
        if (Units.radiansToDegrees(Math.abs(swerve.getTeleopSpeeds().omegaRadiansPerSecond))
            < 15.0) {
          if (AutoAlign.shouldIntakeStationFront(localization.getPose())) {
            yield RobotState.INTAKE_CORAL_STATION_FRONT;
          }
          yield RobotState.INTAKE_CORAL_STATION_BACK;
        }
        yield currentState;
      }
      case INTAKE_CORAL_STATION_FRONT -> {
        if (intake.getHasGP()) {
          rumbleController.rumbleRequest();
          yield RobotState.IDLE_CORAL;
        }
        yield currentState;
      }
      case INTAKE_CORAL_STATION_BACK -> {
        if (intake.getHasGP()) {
          rumbleController.rumbleRequest();
          yield RobotState.SMART_STOW_1;
        }
        yield currentState;
      }
      case SMART_STOW_1 ->
          elevator.atGoal() && roll.atGoal() ? RobotState.SMART_STOW_2 : currentState;
      case SMART_STOW_2 -> wrist.atGoal() ? RobotState.IDLE_CORAL : currentState;
      case NET_BACK_SCORING -> intake.getHasGP() ? currentState : RobotState.IDLE_NO_GP;
    };
  }

  private boolean shouldProgressTeleopScore() {
    return DriverStation.isTeleop()
        && confirmScoreActive
        && getReefAlignState() == ReefAlignState.HAS_TAGS_IN_POSITION;
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case IDLE_NO_GP -> {
        intake.setState(IntakeState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, WristState.CORAL_STOWED);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(
            gamePieceMode == GamePieceMode.CORAL
                ? LightsState.IDLE_NO_GP_CORAL_MODE
                : LightsState.IDLE_NO_GP_ALGAE_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case IDLE_ALGAE -> {
        intake.setState(IntakeState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.STOWED, WristState.ALGAE_STOWED);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.STOWED);
        frontCoralLimelight.setState(LimelightState.TAGS);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.IDLE_WITH_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case IDLE_CORAL -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.STOWED, WristState.CORAL_STOWED);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.CORAL_SCORE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.IDLE_WITH_CORAL);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_ALGAE_FLOOR -> {
        intake.setState(IntakeState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.GROUND_ALGAE_INTAKE, WristState.GROUND_ALGAE_INTAKE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.STOWED);
        // TODO: This should be in an algae state
        frontCoralLimelight.setState(LimelightState.TAGS);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
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
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
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
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.IDLE_NO_GP_ALGAE_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_STATION_APPROACH -> {
        intake.setState(IntakeState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, WristState.CORAL_STOWED);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_CORAL_STATION_BACK -> {
        intake.setState(IntakeState.INTAKING_CORAL);
        moveSuperstructure(
            ElevatorState.INTAKING_CORAL_STATION_BACK, WristState.INTAKING_CORAL_STATION_BACK);
        roll.setState(RollState.STOWED);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getCoralStationAngle(localization.getPose()));
        frontCoralLimelight.setState(LimelightState.TAGS);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_CORAL_STATION_FRONT -> {
        intake.setState(IntakeState.INTAKING_CORAL);
        moveSuperstructure(
            ElevatorState.INTAKING_CORAL_STATION_FRONT, WristState.INTAKING_CORAL_STATION_FRONT);
        roll.setState(RollState.STOWED);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getCoralStationAngle(localization.getPose()) - 180.0);
        frontCoralLimelight.setState(LimelightState.TAGS);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case SMART_STOW_1 -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.STOWED, WristState.INTAKING_CORAL_STATION_BACK);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.SMART_STOW);
        frontCoralLimelight.setState(LimelightState.TAGS);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case SMART_STOW_2 -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.STOWED, WristState.CORAL_STOWED);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.SMART_STOW);
        frontCoralLimelight.setState(LimelightState.TAGS);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
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
        baseTagLimelight.setState(LimelightState.TAGS);
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
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_ASSIST_CORAL_FLOOR_HORIZONTAL -> {
        intake.setState(IntakeState.INTAKING_CORAL);
        moveSuperstructure(ElevatorState.GROUND_CORAL_INTAKE, WristState.GROUND_CORAL_INTAKE);
        swerve.enableCoralIntakeAssist();
        swerve.setSnapToAngle(0);
        roll.setState(RollState.INTAKING_CORAL_HORIZONTAL);
        frontCoralLimelight.setState(LimelightState.CORAL);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case DISLODGE_ALGAE_L2_WAIT -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.ALGAE_DISLODGE_L2, WristState.DISLODGE_L2_LOW);
        roll.setState(RollState.STOWED);
        swerve.setSnapToAngle(reefSnapAngle);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.IDLE_NO_GP_ALGAE_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case DISLODGE_ALGAE_L2_PUSHING -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.ALGAE_DISLODGE_L2, WristState.DISLODGE_L2_HIGH);
        roll.setState(RollState.STOWED);
        swerve.setSnapToAngle(reefSnapAngle);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.IDLE_NO_GP_ALGAE_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case DISLODGE_ALGAE_L3_WAIT -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.ALGAE_DISLODGE_L3, WristState.DISLODGE_L3_LOW);
        roll.setState(RollState.STOWED);
        swerve.setSnapToAngle(reefSnapAngle);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.IDLE_NO_GP_ALGAE_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case DISLODGE_ALGAE_L3_PUSHING -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.ALGAE_DISLODGE_L3, WristState.DISLODGE_L3_HIGH);
        roll.setState(RollState.STOWED);
        swerve.setSnapToAngle(reefSnapAngle);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.IDLE_NO_GP_ALGAE_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L1_1_APPROACH, CORAL_L2_1_APPROACH, CORAL_L3_1_APPROACH, CORAL_L4_1_APPROACH -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.STOWED, WristState.CORAL_STOWED);
        swerve.setSnapToAngle(reefSnapAngle);
        swerve.setSnapsEnabled(true);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L1_3_PLACE -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L1_PLACE, WristState.CORAL_SCORE_PLACING_L1, true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L1_4_RELEASE -> {
        intake.setState(IntakeState.SCORE_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L1_RELEASE, WristState.CORAL_SCORE_PLACING_L1, true);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.STOWED);
        swerve.setSnapsEnabled(false);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L2_2_LINEUP -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L2_LINEUP, WristState.CORAL_SCORE_CENTERED_LINEUP_L2);
        swerve.enableScoringAlignment();
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L2_3_PLACE -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L2_PLACE,
            WristState.CORAL_SCORE_CENTERED_PLACING_L2,
            true);
        swerve.enableScoringAlignment();
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L2_4_RELEASE -> {
        intake.setState(IntakeState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L2_RELEASE,
            WristState.CORAL_SCORE_CENTERED_PLACING_L2,
            true);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L3_2_LINEUP -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L3_LINEUP, WristState.CORAL_SCORE_CENTERED_LINEUP_L3);
        swerve.enableScoringAlignment();
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L3_3_PLACE -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L3_PLACE,
            WristState.CORAL_SCORE_CENTERED_PLACING_L3,
            true);
        swerve.enableScoringAlignment();
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L3_4_RELEASE -> {
        intake.setState(IntakeState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L3_RELEASE,
            WristState.CORAL_SCORE_CENTERED_PLACING_L3,
            true);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L4_2_LINEUP -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L4_LINEUP, WristState.CORAL_SCORE_CENTERED_LINEUP_L4);
        swerve.enableScoringAlignment();
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L4_3_PLACE_THEN_RELEASE, CORAL_CENTERED_L4_3_PLACE -> {
        intake.setState(IntakeState.IDLE_W_CORAL);

        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L4_PLACE, WristState.CORAL_SCORE_CENTERED_LINEUP_L4, true);
        swerve.enableScoringAlignment();
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L4_4_RELEASE -> {
        intake.setState(IntakeState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L4_RELEASE,
            WristState.CORAL_SCORE_CENTERED_PLACING_L4,
            true);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L2_2_LINEUP -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L2_LINEUP, WristState.CORAL_SCORE_DISPLACED_LINEUP_L2);
        swerve.enableScoringAlignment();
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L2_3_PLACE -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L2_PLACE, WristState.CORAL_SCORE_DISPLACED_PLACING_L2);
        swerve.enableScoringAlignment();
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L2_4_RELEASE -> {
        intake.setState(IntakeState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L2_RELEASE,
            WristState.CORAL_SCORE_DISPLACED_PLACING_L2,
            true);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L3_2_LINEUP -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L3_LINEUP, WristState.CORAL_SCORE_DISPLACED_LINEUP_L3);
        swerve.enableScoringAlignment();
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L3_3_PLACE -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L3_PLACE, WristState.CORAL_SCORE_DISPLACED_PLACING_L3);
        swerve.enableScoringAlignment();
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L3_4_RELEASE -> {
        intake.setState(IntakeState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L3_RELEASE,
            WristState.CORAL_SCORE_DISPLACED_PLACING_L3,
            true);
        swerve.setSnapsEnabled(false);

        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L4_2_LINEUP -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L4_LINEUP, WristState.CORAL_SCORE_DISPLACED_LINEUP_L4);
        swerve.enableScoringAlignment();
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L4_3_PLACE_THEN_RELEASE, CORAL_DISPLACED_L4_3_PLACE -> {
        intake.setState(IntakeState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L4_PLACE, WristState.CORAL_SCORE_DISPLACED_PLACING_L4);
        swerve.enableScoringAlignment();
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L4_4_RELEASE -> {
        intake.setState(IntakeState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L4_RELEASE,
            WristState.CORAL_SCORE_DISPLACED_PLACING_L4,
            true);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(reefSnapAngle);
        roll.setState(RollState.CORAL_SCORE);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.REEF_TAGS);
        backTagLimelight.setState(LimelightState.REEF_TAGS);
        baseTagLimelight.setState(LimelightState.REEF_TAGS);
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
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case NET_BACK_SCORING -> {
        intake.setState(IntakeState.SCORE_ALGAE_NET);
        moveSuperstructure(ElevatorState.NET, WristState.ALGAE_BACKWARD_NET);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getBackwardNetDirection());
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
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
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case NET_FORWARD_SCORING -> {
        intake.setState(IntakeState.SCORE_ALGAE_NET);
        moveSuperstructure(ElevatorState.NET, WristState.ALGAE_FORWARD_NET);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getForwardNetDirection());
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
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
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case PROCESSOR_SCORING -> {
        intake.setState(IntakeState.SCORE_ALGAE_PROCESSOR);
        moveSuperstructure(ElevatorState.PROCESSOR, WristState.ALGAE_PROCESSOR);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getProcessorAngle());
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_OUTTAKE -> {
        intake.setState(IntakeState.SCORE_ALGAE_PROCESSOR);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0.0);
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      // TODO: Create special light states for climbing, unjam, and rehoming
      case CLIMBING_1_LINEUP -> {
        intake.setState(IntakeState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, WristState.CORAL_STOWED);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.LINEUP);
      }
      case CLIMBING_2_HANGING -> {
        intake.setState(IntakeState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, WristState.CORAL_STOWED);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.HANGING);
      }
      case UNJAM -> {
        intake.setState(IntakeState.OUTTAKING);
        moveSuperstructure(ElevatorState.UNJAM, WristState.UNJAM);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.STOWED);
      }
      case REHOME_ELEVATOR -> {
        intake.setState(IntakeState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.MID_MATCH_HOMING, WristState.CORAL_STOWED);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.STOWED);
      }
      case REHOME_WRIST -> {
        intake.setState(IntakeState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, WristState.MID_MATCH_HOMING);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.STOWED);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.STOWED);
      }
      case REHOME_ROLL -> {
        intake.setState(IntakeState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, WristState.CORAL_STOWED);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
        roll.setState(RollState.HOMING);
        elevatorPurpleLimelight.setState(LimelightState.PURPLE);
        frontCoralLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
        baseTagLimelight.setState(LimelightState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.STOWED);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("RobotManager/NearestReefSidePose", nearestReefSidePose);
    DogLog.log(
        "RobotManager/ShouldIntakeForward",
        AutoAlign.shouldIntakeStationFront(localization.getPose()));
    DogLog.log("CollisionAvoidance/latestUnsafe", latestUnsafe);

    // Continuous state actions
    moveSuperstructure(latestElevatorGoal, latestWristGoal, latestUnsafe);

    // Update snaps
    switch (getState()) {
      case DISLODGE_ALGAE_L2_WAIT,
          DISLODGE_ALGAE_L3_WAIT,
          DISLODGE_ALGAE_L2_PUSHING,
          DISLODGE_ALGAE_L3_PUSHING,
          INTAKE_ALGAE_L2,
          INTAKE_ALGAE_L3 -> {
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(reefSnapAngle);
      }
      case CORAL_L1_1_APPROACH,
          CORAL_L2_1_APPROACH,
          CORAL_L3_1_APPROACH,
          CORAL_L4_1_APPROACH,
          CORAL_L1_3_PLACE,
          CORAL_L1_4_RELEASE,
          CORAL_CENTERED_L2_2_LINEUP,
          CORAL_CENTERED_L2_3_PLACE,
          CORAL_CENTERED_L2_4_RELEASE,
          CORAL_CENTERED_L3_2_LINEUP,
          CORAL_CENTERED_L3_3_PLACE,
          CORAL_CENTERED_L3_4_RELEASE,
          CORAL_CENTERED_L4_2_LINEUP,
          CORAL_CENTERED_L4_3_PLACE,
          CORAL_CENTERED_L4_4_RELEASE,
          CORAL_DISPLACED_L2_2_LINEUP,
          CORAL_DISPLACED_L2_3_PLACE,
          CORAL_DISPLACED_L2_4_RELEASE,
          CORAL_DISPLACED_L3_2_LINEUP,
          CORAL_DISPLACED_L3_3_PLACE,
          CORAL_DISPLACED_L3_4_RELEASE,
          CORAL_DISPLACED_L4_2_LINEUP,
          CORAL_DISPLACED_L4_3_PLACE,
          CORAL_DISPLACED_L4_4_RELEASE -> {
        swerve.setSnapToAngle(reefSnapAngle);
      }
      case INTAKE_CORAL_STATION_BACK -> {
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getCoralStationAngle(localization.getPose()));
      }
      case INTAKE_CORAL_STATION_FRONT -> {
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getCoralStationAngle(localization.getPose()) - 180.0);
      }
      default -> {}
    }

    // Update lights
    switch (getState()) {
      case CORAL_L1_1_APPROACH,
          CORAL_L2_1_APPROACH,
          CORAL_L3_1_APPROACH,
          CORAL_L4_1_APPROACH,
          CORAL_L1_3_PLACE,
          CORAL_L1_4_RELEASE,
          CORAL_CENTERED_L2_2_LINEUP,
          CORAL_CENTERED_L2_3_PLACE,
          CORAL_CENTERED_L2_4_RELEASE,
          CORAL_CENTERED_L3_2_LINEUP,
          CORAL_CENTERED_L3_3_PLACE,
          CORAL_CENTERED_L3_4_RELEASE,
          CORAL_CENTERED_L4_2_LINEUP,
          CORAL_CENTERED_L4_3_PLACE,
          CORAL_CENTERED_L4_4_RELEASE,
          CORAL_DISPLACED_L2_2_LINEUP,
          CORAL_DISPLACED_L2_3_PLACE,
          CORAL_DISPLACED_L2_4_RELEASE,
          CORAL_DISPLACED_L3_2_LINEUP,
          CORAL_DISPLACED_L3_3_PLACE,
          CORAL_DISPLACED_L3_4_RELEASE,
          CORAL_DISPLACED_L4_2_LINEUP,
          CORAL_DISPLACED_L4_3_PLACE,
          CORAL_DISPLACED_L4_4_RELEASE -> {
        lights.setState(getLightStateForScoring());
      }
      default -> {}
    }

    // Update purple to finish aligning after elevator is raised
    // switch (getState()) {
    //   case CORAL_CENTERED_L2_2_LINEUP,
    //       CORAL_CENTERED_L2_3_PLACE,
    //       CORAL_CENTERED_L2_4_RELEASE,
    //       CORAL_CENTERED_L3_2_LINEUP,
    //       CORAL_CENTERED_L3_3_PLACE,
    //       CORAL_CENTERED_L3_4_RELEASE,
    //       CORAL_CENTERED_L4_2_LINEUP,
    //       CORAL_CENTERED_L4_3_PLACE,
    //       CORAL_CENTERED_L4_4_RELEASE,
    //       CORAL_DISPLACED_L2_2_LINEUP,
    //       CORAL_DISPLACED_L2_3_PLACE,
    //       CORAL_DISPLACED_L2_4_RELEASE,
    //       CORAL_DISPLACED_L3_2_LINEUP,
    //       CORAL_DISPLACED_L3_3_PLACE,
    //       CORAL_DISPLACED_L3_4_RELEASE,
    //       CORAL_DISPLACED_L4_2_LINEUP,
    //       CORAL_DISPLACED_L4_3_PLACE,
    //       CORAL_DISPLACED_L4_4_RELEASE -> {
    //     if (elevator.atGoal()) {
    //       purple.setBeforeRaisedOffset(false);
    //     }
    //   }
    //   default -> {}
    // }

    if (elevatorPurpleLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || frontCoralLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || backTagLimelight.getCameraHealth() == CameraHealth.OFFLINE) {
      lights.setDisabledState(LightsState.ERROR);
    } else if (wrist.getState() == WristState.PRE_MATCH_HOMING && !wrist.rangeOfMotionGood()) {
      lights.setDisabledState(LightsState.UNHOMED);
    } else {
      lights.setDisabledState(LightsState.HEALTHY);
    }
  }

  @Override
  protected void collectInputs() {
    super.collectInputs();
    var lookaheadRobotPose = MathHelpers.poseLookahead(localization.getPose(), swerve.getFieldRelativeSpeeds(), 0.9);
    DogLog.log("RobotManager/LookaheadPose", lookaheadRobotPose);
    nearestReefSidePose = AutoAlign.getClosestReefSide(lookaheadRobotPose).getPose();
    reefSnapAngle = nearestReefSidePose.getRotation().getDegrees();
    scoringLevel =
        switch (getState()) {
          case CORAL_L1_1_APPROACH, CORAL_L1_3_PLACE, CORAL_L1_4_RELEASE -> ReefPipeLevel.L1;
          case CORAL_L2_1_APPROACH,
                  CORAL_CENTERED_L2_2_LINEUP,
                  CORAL_CENTERED_L2_3_PLACE,
                  CORAL_CENTERED_L2_4_RELEASE,
                  CORAL_DISPLACED_L2_2_LINEUP,
                  CORAL_DISPLACED_L2_3_PLACE,
                  CORAL_DISPLACED_L2_4_RELEASE ->
              ReefPipeLevel.L2;
          case CORAL_L3_1_APPROACH,
                  CORAL_CENTERED_L3_2_LINEUP,
                  CORAL_CENTERED_L3_3_PLACE,
                  CORAL_CENTERED_L3_4_RELEASE,
                  CORAL_DISPLACED_L3_2_LINEUP,
                  CORAL_DISPLACED_L3_3_PLACE,
                  CORAL_DISPLACED_L3_4_RELEASE ->
              ReefPipeLevel.L3;
          case CORAL_L4_1_APPROACH,
                  CORAL_CENTERED_L4_2_LINEUP,
                  CORAL_CENTERED_L4_3_PLACE,
                  CORAL_CENTERED_L4_4_RELEASE,
                  CORAL_CENTERED_L4_3_PLACE_THEN_RELEASE,
                  CORAL_DISPLACED_L4_2_LINEUP,
                  CORAL_DISPLACED_L4_3_PLACE,
                  CORAL_DISPLACED_L4_4_RELEASE,
                  CORAL_DISPLACED_L4_3_PLACE_THEN_RELEASE ->
              ReefPipeLevel.L4;
          default -> ReefPipeLevel.BASE;
        };

    if (getState() == RobotState.INTAKE_ASSIST_CORAL_FLOOR_HORIZONTAL) {
      ChassisSpeeds coralAssistSpeeds =
          IntakeAssistUtil.getCoralAssistSpeeds(
              frontCoralLimelight.getCoralResult(), imu.getRobotHeading(), true);
      swerve.setFieldRelativeCoralAssistSpeedsOffset(coralAssistSpeeds);
    }
    DogLog.log("PurpleAlignment/UsedPose", purple.getUsedScoringPose());
    purple.setLevel(scoringLevel);
    if (vision.isAnyScoringTagLimelightOnline()) {
      swerve.setPurpleSpeeds(purple.getPoseAlignmentChassisSpeeds(false));
    } else {
      swerve.setPurpleSpeeds(new ChassisSpeeds());
    }


    swerve.setElevatorHeight(elevator.getHeight());
  }

  private boolean cameraOnlineAndFarEnoughFromReef() {
    var tagCameraOnline = vision.isAnyScoringTagLimelightOnline();

    if (!tagCameraOnline) {
      return timeout(0.5);
    }

    var isFarEnoughFromReefSide =
        !AutoAlign.isCloseToReefSide(localization.getPose(), nearestReefSidePose, 0.75);

    return isFarEnoughFromReefSide;
  }

  public void setGamePieceMode(GamePieceMode newMode) {
    gamePieceMode = newMode;

    switch (getState()) {
      // If we are actively placing a game piece, or climbing, etc. then don't change mode
      case CORAL_L1_4_RELEASE,
          CORAL_CENTERED_L2_4_RELEASE,
          CORAL_CENTERED_L3_4_RELEASE,
          CORAL_DISPLACED_L2_4_RELEASE,
          CORAL_DISPLACED_L3_4_RELEASE,
          CLIMBING_1_LINEUP,
          NET_BACK_SCORING,
          NET_FORWARD_SCORING,
          PROCESSOR_SCORING,
          ALGAE_OUTTAKE,
          CLIMBING_2_HANGING,
          UNJAM,
          REHOME_ELEVATOR -> {}
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
          l2CoralApproachRequest();
        }
      }
      case INTAKE_ALGAE_L3, DISLODGE_ALGAE_L3_WAIT, DISLODGE_ALGAE_L3_PUSHING -> {
        if (newMode == GamePieceMode.CORAL) {
          l3CoralApproachRequest();
        }
      }
      case INTAKE_CORAL_STATION_BACK, INTAKE_CORAL_STATION_FRONT -> {
        if (newMode == GamePieceMode.ALGAE) {
          stowRequest();
        }
      }
      case INTAKE_CORAL_FLOOR_UPRIGHT,
          INTAKE_CORAL_FLOOR_HORIZONTAL,
          INTAKE_ASSIST_CORAL_FLOOR_HORIZONTAL -> {
        if (newMode == GamePieceMode.ALGAE) {
          intakeFloorAlgaeRequest();
        }
      }
      case CORAL_L1_1_APPROACH, CORAL_L1_3_PLACE -> {
        if (newMode == GamePieceMode.ALGAE) {
          processorWaitingRequest();
        }
      }
      case CORAL_L2_1_APPROACH,
          CORAL_CENTERED_L2_2_LINEUP,
          CORAL_CENTERED_L2_3_PLACE,
          CORAL_DISPLACED_L2_2_LINEUP,
          CORAL_DISPLACED_L2_3_PLACE -> {
        if (newMode == GamePieceMode.ALGAE) {
          intakeAlgaeL2Request();
        }
      }
      case CORAL_L3_1_APPROACH,
          CORAL_CENTERED_L3_2_LINEUP,
          CORAL_CENTERED_L3_3_PLACE,
          CORAL_DISPLACED_L3_2_LINEUP,
          CORAL_DISPLACED_L3_3_PLACE -> {
        if (newMode == GamePieceMode.ALGAE) {
          intakeAlgaeL3Request();
        }
      }
      case CORAL_L4_1_APPROACH,
          CORAL_CENTERED_L4_2_LINEUP,
          CORAL_CENTERED_L4_3_PLACE,
          CORAL_DISPLACED_L4_2_LINEUP,
          CORAL_DISPLACED_L4_3_PLACE -> {
        if (newMode == GamePieceMode.ALGAE) {
          algaeNetRequest();
        }
      }
      case NET_BACK_WAITING,
          NET_BACK_PREPARE_TO_SCORE,
          NET_FORWARD_WAITING,
          NET_FORWARD_PREPARE_TO_SCORE -> {
        if (newMode == GamePieceMode.CORAL) {
          l4CoralApproachRequest();
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
    if (gamePieceMode == GamePieceMode.CORAL) {
      if (intake.getHasGP()) {
        setStateFromRequest(RobotState.IDLE_CORAL);
      } else {
        setStateFromRequest(RobotState.IDLE_NO_GP);
      }
    } else {
      setStateFromRequest(RobotState.IDLE_ALGAE);
    }
  }

  public void intakeFloorRequest() {
    if (gamePieceMode == GamePieceMode.ALGAE) {
      intakeFloorAlgaeRequest();
    } else {
      intakeFloorCoralHorizontalRequest();
    }
  }

  public void intakeAssistFloorRequest() {
    if (gamePieceMode == GamePieceMode.ALGAE) {
      intakeFloorAlgaeRequest();
    } else {
      intakeAssistFloorCoralHorizontalRequest();
    }
  }

  public void intakeFloorAlgaeRequest() {
    gamePieceMode = GamePieceMode.ALGAE;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.INTAKE_ALGAE_FLOOR);
    }
  }

  public void intakeFloorCoralHorizontalRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.INTAKE_CORAL_FLOOR_HORIZONTAL);
    }
  }

  public void intakeAssistFloorCoralHorizontalRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.INTAKE_ASSIST_CORAL_FLOOR_HORIZONTAL);
    }
  }

  public void intakeStationRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.INTAKE_STATION_APPROACH);
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
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.PROCESSOR_WAITING);
    }
  }

  public void l1CoralLineupRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.CORAL_L1_1_APPROACH);
    }
  }

  public void l2LineupRequest() {
    if (gamePieceMode == GamePieceMode.ALGAE) {
      intakeAlgaeL2Request();
    } else {
      l2CoralApproachRequest();
    }
  }

  private void intakeAlgaeL2Request() {
    gamePieceMode = GamePieceMode.ALGAE;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.INTAKE_ALGAE_L2);
    }
  }

  public void l2CoralApproachRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.CORAL_L2_1_APPROACH);
    }
  }

  public void l3LineupRequest() {
    if (gamePieceMode == GamePieceMode.ALGAE) {
      intakeAlgaeL3Request();
    } else {
      l3CoralApproachRequest();
    }
  }

  private void intakeAlgaeL3Request() {
    gamePieceMode = GamePieceMode.ALGAE;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.INTAKE_ALGAE_L3);
    }
  }

  public void l3CoralApproachRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.CORAL_L3_1_APPROACH);
    }
  }

  public void l4CoralScoreRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default ->
          setStateFromRequest(
              intake.isCoralCentered()
                  ? RobotState.CORAL_CENTERED_L4_3_PLACE
                  : RobotState.CORAL_DISPLACED_L4_3_PLACE);
    }
  }

  public void l4coralPlaceAndReleaseRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default ->
          setStateFromRequest(
              intake.isCoralCentered()
                  ? RobotState.CORAL_CENTERED_L4_3_PLACE_THEN_RELEASE
                  : RobotState.CORAL_DISPLACED_L4_3_PLACE_THEN_RELEASE);
    }
  }

  public void l4CoralReleaseRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default ->
          setStateFromRequest(
              intake.isCoralCentered()
                  ? RobotState.CORAL_CENTERED_L4_4_RELEASE
                  : RobotState.CORAL_DISPLACED_L4_4_RELEASE);
    }
  }

  public void highApproachRequest() {
    if (gamePieceMode == GamePieceMode.ALGAE) {
      algaeNetRequest();
    } else {
      l4CoralApproachRequest();
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
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.NET_FORWARD_WAITING);
    }
  }

  private void algaeNetBackRequest() {
    gamePieceMode = GamePieceMode.ALGAE;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.NET_BACK_WAITING);
    }
  }

  public void l4CoralApproachRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.CORAL_L4_1_APPROACH);
    }
  }

  public void l4CoralLineupRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL, REHOME_WRIST -> {}
      default ->
          setStateFromRequest(
              intake.isCoralCentered()
                  ? RobotState.CORAL_CENTERED_L4_2_LINEUP
                  : RobotState.CORAL_DISPLACED_L4_2_LINEUP);
    }
  }

  public void preloadCoralRequest() {
    gamePieceMode = GamePieceMode.CORAL;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.IDLE_CORAL);
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
          INTAKE_ASSIST_CORAL_FLOOR_HORIZONTAL,
          INTAKE_CORAL_FLOOR_UPRIGHT,
          INTAKE_CORAL_STATION_BACK,
          INTAKE_CORAL_STATION_FRONT,
          INTAKE_STATION_APPROACH -> {}

      case IDLE_ALGAE -> {
        setStateFromRequest(RobotState.ALGAE_OUTTAKE);
      }
      case PROCESSOR_WAITING -> setStateFromRequest(RobotState.PROCESSOR_PREPARE_TO_SCORE);
      case NET_BACK_WAITING -> setStateFromRequest(RobotState.NET_BACK_PREPARE_TO_SCORE);
      case NET_FORWARD_WAITING -> setStateFromRequest(RobotState.NET_FORWARD_PREPARE_TO_SCORE);

      case CORAL_L1_1_APPROACH -> setStateFromRequest(RobotState.CORAL_L1_3_PLACE);
      case CORAL_L1_3_PLACE -> setStateFromRequest(RobotState.CORAL_L1_4_RELEASE);

      case CORAL_L2_1_APPROACH ->
          setStateFromRequest(
              intake.isCoralCentered()
                  ? RobotState.CORAL_CENTERED_L2_2_LINEUP
                  : RobotState.CORAL_DISPLACED_L2_2_LINEUP);
      case CORAL_CENTERED_L2_2_LINEUP -> setStateFromRequest(RobotState.CORAL_CENTERED_L2_3_PLACE);
      case CORAL_CENTERED_L2_3_PLACE -> setStateFromRequest(RobotState.CORAL_CENTERED_L2_4_RELEASE);
      case CORAL_DISPLACED_L2_2_LINEUP ->
          setStateFromRequest(RobotState.CORAL_DISPLACED_L2_3_PLACE);
      case CORAL_DISPLACED_L2_3_PLACE ->
          setStateFromRequest(RobotState.CORAL_DISPLACED_L2_4_RELEASE);

      case CORAL_L3_1_APPROACH ->
          setStateFromRequest(
              intake.isCoralCentered()
                  ? RobotState.CORAL_CENTERED_L3_2_LINEUP
                  : RobotState.CORAL_DISPLACED_L3_2_LINEUP);
      case CORAL_CENTERED_L3_2_LINEUP -> setStateFromRequest(RobotState.CORAL_CENTERED_L3_3_PLACE);
      case CORAL_CENTERED_L3_3_PLACE -> setStateFromRequest(RobotState.CORAL_CENTERED_L3_4_RELEASE);
      case CORAL_DISPLACED_L3_2_LINEUP ->
          setStateFromRequest(RobotState.CORAL_DISPLACED_L3_3_PLACE);
      case CORAL_DISPLACED_L3_3_PLACE ->
          setStateFromRequest(RobotState.CORAL_DISPLACED_L3_4_RELEASE);

      case CORAL_L4_1_APPROACH ->
          setStateFromRequest(
              intake.isCoralCentered()
                  ? RobotState.CORAL_CENTERED_L4_2_LINEUP
                  : RobotState.CORAL_DISPLACED_L4_2_LINEUP);
      case CORAL_CENTERED_L4_2_LINEUP -> setStateFromRequest(RobotState.CORAL_CENTERED_L4_3_PLACE);
      case CORAL_CENTERED_L4_3_PLACE -> setStateFromRequest(RobotState.CORAL_CENTERED_L4_4_RELEASE);
      case CORAL_DISPLACED_L4_2_LINEUP ->
          setStateFromRequest(RobotState.CORAL_DISPLACED_L4_3_PLACE);
      case CORAL_DISPLACED_L4_3_PLACE ->
          setStateFromRequest(RobotState.CORAL_DISPLACED_L4_4_RELEASE);

      case CORAL_L1_4_RELEASE,
              CORAL_CENTERED_L2_4_RELEASE,
              CORAL_DISPLACED_L2_4_RELEASE,
              CORAL_CENTERED_L3_4_RELEASE,
              CORAL_DISPLACED_L3_4_RELEASE,
              CORAL_CENTERED_L4_4_RELEASE,
              CORAL_DISPLACED_L4_4_RELEASE ->
          setStateFromRequest(RobotState.IDLE_NO_GP);

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
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.UNJAM);
    }
  }

  public void rehomeElevatorRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ROLL, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.REHOME_ELEVATOR);
    }
  }

  public void rehomeWristRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_ROLL -> {}
      default -> setStateFromRequest(RobotState.REHOME_WRIST);
    }
  }

  public void rehomeRollRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, REHOME_ELEVATOR, REHOME_WRIST -> {}
      default -> setStateFromRequest(RobotState.REHOME_ROLL);
    }
  }

  private ElevatorState latestElevatorGoal = ElevatorState.STOWED;
  private WristState latestWristGoal = WristState.PRE_MATCH_HOMING;
  private boolean latestUnsafe = false;

  private void moveSuperstructure(ElevatorState elevatorGoal, WristState wristGoal) {
    moveSuperstructure(elevatorGoal, wristGoal, false);
  }

  private void moveSuperstructure(
      ElevatorState elevatorGoal, WristState wristGoal, boolean unsafe) {
    latestElevatorGoal = elevatorGoal;
    latestWristGoal = wristGoal;
    latestUnsafe = unsafe;

    var maybeIntermediaryPosition =
        CollisionAvoidance.plan(
            new SuperstructurePosition(elevator.getHeight(), wrist.getAngle()),
            new SuperstructurePosition(elevatorGoal.height, wristGoal.angle));

    if (!unsafe && maybeIntermediaryPosition.isPresent()) {
      var intermediaryPosition = maybeIntermediaryPosition.get();

      // A collision was detected, so we need to go to an intermediary point
      elevator.setCollisionAvoidanceGoal(intermediaryPosition.elevatorHeight());
      elevator.setState(ElevatorState.COLLISION_AVOIDANCE);

      wrist.setCollisionAvoidanceGoal(intermediaryPosition.wristAngle());
      wrist.setState(WristState.COLLISION_AVOIDANCE);
    } else {
      // No collision, go straight to goal state or unsafe mode
      elevator.setState(elevatorGoal);
      wrist.setState(wristGoal);
    }
  }

  private ReefAlignState getReefAlignState() {
    return AutoAlign.getReefAlignState(
        localization.getPose(),
        purple.getPurpleState(),
        scoringLevel,
        baseTagLimelight
            .getTagResult()
            .or(frontCoralLimelight::getTagResult)
            .or(backTagLimelight::getTagResult),
        CameraHealth.combine(
            baseTagLimelight.getCameraHealth(),
            frontCoralLimelight.getCameraHealth(),
            backTagLimelight.getCameraHealth()));
  }

  private LightsState getLightStateForScoring() {
    return switch (getReefAlignState()) {
      case CAMERA_DEAD -> LightsState.ERROR;
      // TODO: Once purple is implemented, only say we're ready once purple is aligned
      case HAS_TAGS_IN_POSITION, HAS_PURPLE_ALIGNED -> LightsState.SCORE_ALIGN_READY;
      default -> LightsState.SCORE_ALIGN_NOT_READY;
    };
  }

  public void setConfirmScoreActive(boolean newValue) {
    confirmScoreActive = newValue;
  }
}
