package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.ArmState;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefSide;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.claw.ClawState;
import frc.robot.claw.ClawSubsystem;
import frc.robot.climber.ClimberState;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.config.FeatureFlags;
import frc.robot.controller.RumbleControllerSubsystem;
import frc.robot.elevator.ElevatorState;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake_assist.IntakeAssistUtil;
import frc.robot.intake_deploy.DeploySubsystem;
import frc.robot.lights.LightsState;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SnapUtil;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionState;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.game_piece_detection.CoralMap;
import java.util.List;
import java.util.Optional;

public class RobotManager extends StateMachine<RobotState> {
  public final LocalizationSubsystem localization;

  public final VisionSubsystem vision;
  public final ImuSubsystem imu;

  public final CoralMap coralMap;
  private final SwerveSubsystem swerve;
  public final ClawSubsystem claw;
  public final IntakeSubsystem intake;
  public final ArmSubsystem arm;
  public final ElevatorSubsystem elevator;
  public final ClimberSubsystem climber;
  public final RumbleControllerSubsystem rumbleController;
  public final DeploySubsystem deploy;

  private final LightsSubsystem lights;

  public final AutoAlign autoAlign;

  public RobotManager(
      DeploySubsystem deploy,
      IntakeSubsystem intake,
      ClawSubsystem claw,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      VisionSubsystem vision,
      ImuSubsystem imu,
      SwerveSubsystem swerve,
      LocalizationSubsystem localization,
      CoralMap coralMap,
      LightsSubsystem lights,
      AutoAlign autoAlign,
      ClimberSubsystem climber,
      RumbleControllerSubsystem rumbleController) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.CLAW_EMPTY_DEPLOY_EMPTY);
    this.deploy = deploy;
    this.intake = intake;
    this.claw = claw;
    this.arm = arm;
    this.elevator = elevator;
    this.vision = vision;
    this.imu = imu;
    this.swerve = swerve;
    this.localization = localization;
    this.coralMap = coralMap;
    this.lights = lights;
    this.climber = climber;
    this.autoAlign = autoAlign;
    this.rumbleController = rumbleController;

    var stateCount = RobotState.values().length;

    if (stateCount > 50) {
      DogLog.log("RobotManager/StateCount", stateCount);
    }
  }

  private final List<RobotState> algaeStates =
      List.of(
          RobotState.CLAW_ALGAE_DEPLOY_CORAL,
          RobotState.CLAW_ALGAE_DEPLOY_EMPTY,
          RobotState.CORAL_INTAKE_FLOOR_CLAW_ALGAE);

  private double reefSnapAngle = 0.0;
  private RobotScoringSide robotScoringSide = RobotScoringSide.RIGHT;
  private double coralIntakeAssistAngle = 0.0;
  private Optional<Pose2d> maybeBestCoralMapTranslation = Optional.empty();
  private ReefSide nearestReefSide = ReefSide.SIDE_GH;
  private ReefPipeLevel scoringLevel = ReefPipeLevel.BASE;
  private boolean isRollHomed = false;
  private boolean confirmScoreActive = false;

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case CLAW_EMPTY_DEPLOY_EMPTY,
              CLAW_ALGAE_DEPLOY_EMPTY,
              CLAW_ALGAE_DEPLOY_CORAL,
              CLAW_EMPTY_DEPLOY_CORAL,
              ALGAE_PROCESSOR_WAITING,
              ALGAE_NET_LEFT_WAITING_DEPLOY_CORAL,
              ALGAE_NET_LEFT_WAITING_DEPLOY_EMPTY,
              ALGAE_NET_RIGHT_WAITING_DEPLOY_EMPTY,
              ALGAE_NET_RIGHT_WAITING_DEPLOY_CORAL,
              CLIMBING_2_HANGING,
              CLIMBING_3_HANGING_2,
              CLIMBING_4_HANGING_3,
              UNJAM ->
          currentState;

      case REHOME_ELEVATOR ->
          elevator.getState() == ElevatorState.STOWED
              ? RobotState.CLAW_EMPTY_DEPLOY_EMPTY
              : currentState;

      // Reef lineup states
      case CORAL_L2_LEFT_LINEUP -> {
        var isClose =
            AutoAlign.isCloseToReefSide(
                localization.getPose(), nearestReefSide.getPose(), swerve.getFieldRelativeSpeeds());

        if (!isClose) {
          yield currentState;
        }

        yield RobotState.CORAL_L2_LEFT_RELEASE;
      }

      case CORAL_L3_LEFT_LINEUP -> {
        var isClose =
            AutoAlign.isCloseToReefSide(
                localization.getPose(), nearestReefSide.getPose(), swerve.getFieldRelativeSpeeds());

        if (!isClose) {
          yield currentState;
        }

        yield RobotState.CORAL_L3_LEFT_RELEASE;
      }
      case CORAL_L4_LEFT_LINEUP -> {
        // Require explicit transition in auto
        if (FeatureFlags.EXPLICIT_L4_LINEUP.getAsBoolean() && DriverStation.isAutonomous()) {
          yield currentState;
        }
        var isClose =
            AutoAlign.isCloseToReefSide(
                localization.getPose(), nearestReefSide.getPose(), swerve.getFieldRelativeSpeeds());

        if (!isClose) {
          yield currentState;
        }

        yield RobotState.CORAL_L4_LEFT_RELEASE;
      }

      // Scoring
      case ALGAE_PROCESSOR_RELEASE,
          ALGAE_NET_LEFT_RELEASE_DEPLOY_CORAL,
          ALGAE_NET_RIGHT_RELEASE_DEPLOY_CORAL -> {
        if (timeout(0.5)) {

          yield RobotState.CLAW_EMPTY_DEPLOY_CORAL;
        }
        yield !claw.getHasGP() ? RobotState.CLAW_EMPTY_DEPLOY_EMPTY : currentState;
      }
      case ALGAE_NET_LEFT_RELEASE_DEPLOY_EMPTY, ALGAE_NET_RIGHT_RELEASE_DEPLOY_EMPTY -> {
        if (timeout(0.5)) {

          yield RobotState.CLAW_EMPTY_DEPLOY_EMPTY;
        }
        yield !claw.getHasGP() ? RobotState.CLAW_EMPTY_DEPLOY_EMPTY : currentState;
      }
      case CORAL_L1_LEFT_RELEASE,
          CORAL_L2_LEFT_RELEASE,
          CORAL_L3_LEFT_RELEASE,
          CORAL_L4_LEFT_RELEASE,
          CORAL_L1_RIGHT_RELEASE,
          CORAL_L2_RIGHT_RELEASE,
          CORAL_L3_RIGHT_RELEASE,
          CORAL_L4_RIGHT_RELEASE -> {
        var done =
            arm.atGoal()
                && elevator.atGoal()
                && !claw.getHasGP()
                && (DriverStation.isTeleop() ? cameraOnlineAndFarEnoughFromReef() : timeout(0.5));

        if (done) {
          rumbleController.rumbleRequest();
          yield RobotState.CLAW_EMPTY_DEPLOY_EMPTY;
        }

        yield currentState;
      }

      // Intaking
      case ALGAE_INTAKE_FLOOR_DEPLOY_EMPTY -> {
        if (claw.getHasGP()) {
          rumbleController.rumbleRequest();
          yield RobotState.CLAW_ALGAE_DEPLOY_EMPTY;
        }

        yield currentState;
      }
      case ALGAE_INTAKE_L2_DEPLOY_CORAL, ALGAE_INTAKE_L3_DEPLOY_CORAL -> {
        if (claw.getHasGP()) {
          rumbleController.rumbleRequest();
          if (cameraOnlineAndFarEnoughFromReef()) {
            yield RobotState.CLAW_ALGAE_DEPLOY_CORAL;
          }
        }

        yield currentState;
      }
      case ALGAE_INTAKE_L2_DEPLOY_EMPTY, ALGAE_INTAKE_L3_DEPLOY_EMPTY -> {
        if (claw.getHasGP()) {
          rumbleController.rumbleRequest();
          if (cameraOnlineAndFarEnoughFromReef()) {
            yield RobotState.CLAW_ALGAE_DEPLOY_EMPTY;
          }
        }

        yield currentState;
      }
      case CORAL_INTAKE_FLOOR_CLAW_EMPTY,
          CORAL_INTAKE_UPRIGHT_CLAW_EMPTY,
          CORAL_INTAKE_FLOOR_CLAW_ALGAE -> {
        if (claw.getHasGP()) {
          rumbleController.rumbleRequest();
          yield RobotState.CLAW_EMPTY_DEPLOY_CORAL;
        }

        yield currentState;
      }
      case CLIMBING_1_LINEUP ->
          climber.holdingCage() ? RobotState.CLIMBING_2_HANGING : currentState;
      default -> throw new IllegalArgumentException("Unexpected value: " + currentState);
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case CLAW_EMPTY_DEPLOY_EMPTY -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, ArmState.CORAL_STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.STOWED);
      }
      case CLAW_ALGAE_DEPLOY_EMPTY -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.STOWED, ArmState.ALGAE_STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case CLAW_EMPTY_DEPLOY_CORAL -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.STOWED, ArmState.CORAL_STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_CORAL);
        climber.setState(ClimberState.STOWED);
      }
      case CLAW_ALGAE_DEPLOY_CORAL -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.STOWED, ArmState.CORAL_STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_CORAL);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_INTAKE_FLOOR_DEPLOY_EMPTY -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        intake.setState(IntakeState.IDLE_NO_GP);
        moveSuperstructure(
            ElevatorState.GROUND_ALGAE_INTAKE, ArmState.ALGAE_INTAKE_FLOOR_DEPLOY_EMPTY);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_INTAKE_L2_DEPLOY_EMPTY -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L2, ArmState.ALGAE_INTAKE_L2);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_INTAKE_L2_DEPLOY_CORAL -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L2, ArmState.ALGAE_INTAKE_L2);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_INTAKE_L3_DEPLOY_EMPTY -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L3, ArmState.ALGAE_INTAKE_L3);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_INTAKE_L3_DEPLOY_CORAL -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L3, ArmState.ALGAE_INTAKE_L3);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_INTAKE_UPRIGHT_CLAW_EMPTY -> {
        claw.setState(ClawState.CORAL_HANDOFF);
        moveSuperstructure(ElevatorState.GROUND_CORAL_INTAKE, ArmState.GROUND_CORAL_INTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.CORAL_DETECTION);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_INTAKE_FLOOR_CLAW_EMPTY -> {
        claw.setState(ClawState.CORAL_HANDOFF);
        moveSuperstructure(ElevatorState.GROUND_CORAL_INTAKE, ArmState.GROUND_CORAL_INTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.CORAL_DETECTION);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_INTAKE_FLOOR_CLAW_ALGAE -> {
        claw.setState(ClawState.CORAL_HANDOFF);
        moveSuperstructure(ElevatorState.GROUND_CORAL_INTAKE, ArmState.GROUND_CORAL_INTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.CORAL_DETECTION);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_INTAKE_ASSIST_FLOOR_CLAW_EMPTY -> {
        claw.setState(ClawState.CORAL_HANDOFF);
        moveSuperstructure(ElevatorState.GROUND_CORAL_INTAKE, ArmState.GROUND_CORAL_INTAKE);
        // Enable assist in periodic if there's coral in map
        swerve.normalDriveRequest();
        vision.setState(VisionState.CORAL_DETECTION);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_NET_LEFT_WAITING_DEPLOY_EMPTY -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.NET, ArmState.ALGAE_BACKWARD_NET);
        swerve.snapsDriveRequest(SnapUtil.getLeftNetDirection(localization.getPose()));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_NET_RIGHT_WAITING_DEPLOY_EMPTY -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.NET, ArmState.ALGAE_BACKWARD_NET);
        swerve.snapsDriveRequest(SnapUtil.getRightNetDirection(localization.getPose()));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_NET_LEFT_RELEASE_DEPLOY_EMPTY -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.NET, ArmState.ALGAE_BACKWARD_NET);
        swerve.snapsDriveRequest(SnapUtil.getLeftNetDirection(localization.getPose()));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_NET_RIGHT_RELEASE_DEPLOY_EMPTY -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.NET, ArmState.ALGAE_BACKWARD_NET);
        swerve.snapsDriveRequest(SnapUtil.getRightNetDirection(localization.getPose()));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_NET_LEFT_WAITING_DEPLOY_CORAL -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.NET, ArmState.ALGAE_BACKWARD_NET);
        swerve.snapsDriveRequest(SnapUtil.getLeftNetDirection(localization.getPose()));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_NET_RIGHT_WAITING_DEPLOY_CORAL -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.NET, ArmState.ALGAE_BACKWARD_NET);
        swerve.snapsDriveRequest(SnapUtil.getRightNetDirection(localization.getPose()));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_NET_LEFT_RELEASE_DEPLOY_CORAL -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.NET, ArmState.ALGAE_BACKWARD_NET);
        swerve.snapsDriveRequest(SnapUtil.getLeftNetDirection(localization.getPose()));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_NET_RIGHT_RELEASE_DEPLOY_CORAL -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.NET, ArmState.ALGAE_BACKWARD_NET);
        swerve.snapsDriveRequest(SnapUtil.getRightNetDirection(localization.getPose()));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_PROCESSOR_WAITING -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.PROCESSOR, ArmState.ALGAE_PROCESSOR);
        swerve.snapsDriveRequest(SnapUtil.getProcessorAngle());
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_PROCESSOR_RELEASE -> {
        claw.setState(ClawState.SCORE_ALGAE_PROCESSOR);
        moveSuperstructure(ElevatorState.PROCESSOR, ArmState.ALGAE_PROCESSOR);
        swerve.snapsDriveRequest(SnapUtil.getProcessorAngle());
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      // TODO: Create special light states for climbing, unjam, and rehoming
      case CLIMBING_1_LINEUP -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, ArmState.CORAL_STOWED);
        swerve.climbRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.LINEUP);
      }
      case CLIMBING_2_HANGING -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, ArmState.CLIMBING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.HANGING);
      }
      case CLIMBING_3_HANGING_2 -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, ArmState.CLIMBING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.HANGING_2);
      }
      case CLIMBING_4_HANGING_3 -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, ArmState.CLIMBING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.HANGING_3);
      }
      case UNJAM -> {
        claw.setState(ClawState.OUTTAKING);
        moveSuperstructure(ElevatorState.UNJAM, ArmState.UNJAM);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.STOWED);
      }
      case REHOME_ELEVATOR -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.MID_MATCH_HOMING, ArmState.CORAL_STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.STOWED);
      }
      case REHOME_ARM -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, ArmState.MID_MATCH_HOMING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.STOWED);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("RobotManager/NearestReefSidePose", nearestReefSide.getPose());
    DogLog.log(
        "RobotManager/ShouldIntakeForward",
        AutoAlign.shouldIntakeStationFront(localization.getPose()));
    DogLog.log("CollisionAvoidance/latestUnsafe", latestUnsafe);

    // Continuous state actions
    moveSuperstructure(latestElevatorGoal, latestArmGoal, latestUnsafe);

    // Update snaps
    switch (getState()) {
      case ALGAE_INTAKE_L2_DEPLOY_CORAL,
          ALGAE_INTAKE_L2_DEPLOY_EMPTY,
          ALGAE_INTAKE_L3_DEPLOY_CORAL,
          ALGAE_INTAKE_L3_DEPLOY_EMPTY -> {
        swerve.scoringAlignmentRequest(reefSnapAngle);
      }
      case CORAL_L1_LEFT_LINEUP,
          CORAL_L1_LEFT_RELEASE,
          CORAL_L2_LEFT_LINEUP,
          CORAL_L2_LEFT_RELEASE,
          CORAL_L3_LEFT_LINEUP,
          CORAL_L3_LEFT_RELEASE,
          CORAL_L4_LEFT_LINEUP,
          CORAL_L4_LEFT_RELEASE,
          CORAL_L1_RIGHT_LINEUP,
          CORAL_L1_RIGHT_RELEASE,
          CORAL_L2_RIGHT_LINEUP,
          CORAL_L2_RIGHT_RELEASE,
          CORAL_L3_RIGHT_LINEUP,
          CORAL_L3_RIGHT_RELEASE,
          CORAL_L4_RIGHT_LINEUP,
          CORAL_L4_RIGHT_RELEASE -> {
        swerve.scoringAlignmentRequest(reefSnapAngle);
      }
      case CORAL_INTAKE_ASSIST_FLOOR_CLAW_EMPTY -> {
        if (FeatureFlags.CORAL_DETECTION.getAsBoolean()) {
          if (maybeBestCoralMapTranslation.isPresent()) {
            var bestCoralMapTranslation = maybeBestCoralMapTranslation.orElseThrow();

            if (DriverStation.isTeleop()) {
              Pose2d lookaheadPose = localization.getLookaheadPose(0.5);
              DogLog.log("IntakeAssist/LookaheadPose", lookaheadPose);
              swerve.setFieldRelativeCoralAssistSpeedsOffset(
                  IntakeAssistUtil.getAssistSpeedsFromPose(bestCoralMapTranslation, lookaheadPose));
              swerve.coralAlignmentDriveRequest();
            } else {
              var coralSnapAngle =
                  IntakeAssistUtil.getIntakeAssistAngle(
                      bestCoralMapTranslation.getTranslation(), localization.getPose());
              swerve.snapsDriveRequest(coralSnapAngle);
            }
          } else {
            swerve.normalDriveRequest();
          }
        }
      }
      default -> {}
    }

    // Update lights
    switch (getState()) {
      case CORAL_INTAKE_ASSIST_FLOOR_CLAW_EMPTY -> {
        lights.setState(getLightStateForScoring());
      }
      default -> {}
    }

    // Prevent this from interfering with the lights for field calibration
    if (!FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      if (vision.isAnyCameraOffline()) {
        lights.setDisabledState(LightsState.ERROR);
      } else if (arm.getState() == ArmState.PRE_MATCH_HOMING && !arm.rangeOfMotionGood()) {
        lights.setDisabledState(LightsState.UNHOMED);
      } else {
        lights.setDisabledState(LightsState.HEALTHY);
      }
    }

    autoAlign.setDriverPoseOffset(swerve.getPoseOffset());
    switch (swerve.getState()) {
      case REEF_ALIGN_TELEOP -> {
        if (autoAlign.isTagAlignedDebounced()) {
          swerve.reefAlignTeleopFineAdjustRequest();
        }
      }
      default -> {}
    }
  }

  @Override
  protected void collectInputs() {
    super.collectInputs();
    nearestReefSide = autoAlign.getClosestReefSide();
    maybeBestCoralMapTranslation = coralMap.getBestCoral();
    robotScoringSide = AutoAlign.getScoringSideFromRobotPose(localization.getPose());
    autoAlign.setScoringLevel(scoringLevel, robotScoringSide);

    reefSnapAngle = autoAlign.getUsedScoringPose().getRotation().getDegrees();
    scoringLevel =
        switch (getState()) {
          case CORAL_L1_LEFT_LINEUP,
                  CORAL_L1_RIGHT_LINEUP,
                  CORAL_L1_LEFT_RELEASE,
                  CORAL_L1_RIGHT_RELEASE ->
              ReefPipeLevel.L1;
          case CORAL_L2_LEFT_LINEUP,
                  CORAL_L2_RIGHT_LINEUP,
                  CORAL_L2_LEFT_RELEASE,
                  CORAL_L2_RIGHT_RELEASE ->
              ReefPipeLevel.L2;
          case CORAL_L3_LEFT_LINEUP,
                  CORAL_L3_RIGHT_LINEUP,
                  CORAL_L3_LEFT_RELEASE,
                  CORAL_L3_RIGHT_RELEASE ->
              ReefPipeLevel.L3;
          case CORAL_L4_LEFT_LINEUP,
                  CORAL_L4_RIGHT_LINEUP,
                  CORAL_L4_LEFT_RELEASE,
                  CORAL_L4_RIGHT_RELEASE ->
              ReefPipeLevel.L4;
          default -> ReefPipeLevel.BASE;
        };

    DogLog.log("AutoAlign/UsedPose", autoAlign.getUsedScoringPose());

    vision.setClosestScoringReefTag(nearestReefSide.getTagID());

    autoAlign.setTeleopSpeeds(swerve.getTeleopSpeeds());
    if (vision.isAnyTagLimelightOnline() || DriverStation.isAutonomous()) {
      var idealAlignSpeeds =
          switch (getState()) {
            case ALGAE_INTAKE_L2_DEPLOY_CORAL,
                    ALGAE_INTAKE_L2_DEPLOY_EMPTY,
                    ALGAE_INTAKE_L3_DEPLOY_CORAL,
                    ALGAE_INTAKE_L3_DEPLOY_EMPTY ->
                autoAlign.getAlgaeAlignSpeeds();
            default -> autoAlign.getTagAlignSpeeds();
          };
      swerve.setAutoAlignAutoSpeeds(idealAlignSpeeds);
      swerve.setAutoAlignSpeeds(autoAlign.calculateConstrainedAndWeightedSpeeds(idealAlignSpeeds));
    } else {
      swerve.setAutoAlignAutoSpeeds(new ChassisSpeeds());
      swerve.setAutoAlignSpeeds(swerve.getTeleopSpeeds());
    }

    swerve.setElevatorHeight(elevator.getHeight());
  }

  private boolean cameraOnlineAndFarEnoughFromReef() {
    var tagCameraOnline = vision.isAnyTagLimelightOnline();

    if (!tagCameraOnline) {
      return timeout(0.5);
    }

    var isFarEnoughFromReefSide =
        !AutoAlign.isCloseToReefSide(localization.getPose(), nearestReefSide.getPose(), 0.75);

    return isFarEnoughFromReefSide;
  }

  public void forceIdleNoGp() {
    setStateFromRequest(RobotState.CLAW_EMPTY_DEPLOY_EMPTY);
    lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
  }

  public void stowRequest() {
    if (claw.getHasGP()) {
      // Claw is maybe algae or coral

      if (intake.getHasGP()) {
        // Intake is holding coral
        // Technically claw could be holding coral but that shouldn't happen
        setStateFromRequest(RobotState.CLAW_ALGAE_DEPLOY_CORAL);
      } else {
        switch (getState()) {
          // TODO: Finish implementing
          // case algae states -> setStateFromRequest(RobotState.CLAW_ALGAE_DEPLOY_EMPTY);
          default -> setStateFromRequest(RobotState.CLAW_CORAL_DEPLOY_EMPTY);
        }
      }
    }

    // if (claw.getHasGP() && intake.getHasGP()) {
    //   setStateFromRequest(RobotState.CLAW_ALGAE_DEPLOY_CORAL);
    // } else if (claw.getHasGP() && !intake.getHasGP()) {
    //   setStateFromRequest(RobotState.CLAW_ALGAE_DEPLOY_EMPTY);
    // } else if (!claw.getHasGP() && intake.getHasGP()) {
    //   setStateFromRequest(RobotState.CLAW_EMPTY_DEPLOY_CORAL);
    // } else {
    //   setStateFromRequest(RobotState.CLAW_EMPTY_DEPLOY_EMPTY);
    // }
  }

  public void intakeFloorAlgaeRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ARM -> {}
      case CLAW_EMPTY_DEPLOY_CORAL, CLAW_ALGAE_DEPLOY_CORAL ->
          setStateFromRequest(RobotState.ALGAE_INTAKE_FLOOR_DEPLOY_CORAL);
      default -> setStateFromRequest(RobotState.ALGAE_INTAKE_FLOOR_DEPLOY_EMPTY);
    }
  }

  public void intakeFloorCoralHorizontalRequest() {

    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.CORAL_INTAKE_FLOOR_CLAW_EMPTY);
    }
  }

  public void intakeAssistFloorCoralHorizontalRequest() {

    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.CORAL_INTAKE_ASSIST_FLOOR_CLAW_EMPTY);
    }
  }

  public void processorWaitingRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.ALGAE_PROCESSOR_WAITING);
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

    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.ALGAE_NET_RIGHT_WAITING_DEPLOY_EMPTY);
    }
  }

  private void algaeNetBackRequest() {

    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.ALGAE_NET_LEFT_WAITING_DEPLOY_EMPTY);
    }
  }

  public void preloadCoralRequest() {

    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.CLAW_EMPTY_DEPLOY_CORAL);
    }
  }

  public void confirmScoreRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          CORAL_L1_LEFT_LINEUP,
          CORAL_L1_LEFT_RELEASE,
          CORAL_L2_LEFT_LINEUP,
          CORAL_L2_LEFT_RELEASE,
          CORAL_L3_LEFT_LINEUP,
          CORAL_L3_LEFT_RELEASE,
          CORAL_L4_LEFT_LINEUP,
          CORAL_L4_LEFT_RELEASE,
          CORAL_L1_RIGHT_LINEUP,
          CORAL_L1_RIGHT_RELEASE,
          CORAL_L2_RIGHT_LINEUP,
          CORAL_L2_RIGHT_RELEASE,
          CORAL_L3_RIGHT_LINEUP,
          CORAL_L3_RIGHT_RELEASE,
          CORAL_L4_RIGHT_LINEUP,
          CORAL_L4_RIGHT_RELEASE -> {}

      case CLAW_ALGAE_DEPLOY_EMPTY -> {
        setStateFromRequest(RobotState.UNJAM);
      }
      case ALGAE_PROCESSOR_WAITING -> setStateFromRequest(RobotState.ALGAE_PROCESSOR_RELEASE);

      case ALGAE_NET_LEFT_WAITING_DEPLOY_EMPTY ->
          setStateFromRequest(RobotState.ALGAE_NET_LEFT_WAITING_DEPLOY_EMPTY);
      case ALGAE_NET_LEFT_WAITING_DEPLOY_CORAL ->
          setStateFromRequest(RobotState.ALGAE_NET_LEFT_WAITING_DEPLOY_CORAL);

      case ALGAE_NET_RIGHT_WAITING_DEPLOY_EMPTY ->
          setStateFromRequest(RobotState.ALGAE_NET_RIGHT_WAITING_DEPLOY_EMPTY);
      case ALGAE_NET_RIGHT_WAITING_DEPLOY_CORAL ->
          setStateFromRequest(RobotState.ALGAE_NET_RIGHT_WAITING_DEPLOY_CORAL);

      default -> {}
    }
  }

  public void nextClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP -> setStateFromRequest(RobotState.CLIMBING_2_HANGING);
      case CLIMBING_2_HANGING -> setStateFromRequest(RobotState.CLIMBING_3_HANGING_2);
      case CLIMBING_3_HANGING_2 -> setStateFromRequest(RobotState.CLIMBING_4_HANGING_3);
      case CLIMBING_4_HANGING_3 -> {}
      default -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
    }
  }

  public void previousClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_4_HANGING_3 -> setStateFromRequest(RobotState.CLIMBING_3_HANGING_2);
      case CLIMBING_3_HANGING_2 -> setStateFromRequest(RobotState.CLIMBING_2_HANGING);
      case CLIMBING_2_HANGING -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
      case CLIMBING_1_LINEUP -> {
        setStateFromRequest(RobotState.CLAW_EMPTY_DEPLOY_EMPTY);
      }
      default -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
    }
  }

  public void unjamRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.UNJAM);
    }
  }

  public void rehomeElevatorRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.REHOME_ELEVATOR);
    }
  }

  public void rehomeArmRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR -> {}
      default -> setStateFromRequest(RobotState.REHOME_ARM);
    }
  }

  public Command waitForRollHomedCommand() {
    return Commands.waitUntil(() -> isRollHomed);
  }

  private ElevatorState latestElevatorGoal = ElevatorState.STOWED;
  private ArmState latestArmGoal = ArmState.PRE_MATCH_HOMING;
  private boolean latestUnsafe = false;

  private void moveSuperstructure(ElevatorState elevatorGoal, ArmState armGoal) {
    moveSuperstructure(elevatorGoal, armGoal, false);
  }

  private void moveSuperstructure(ElevatorState elevatorGoal, ArmState armGoal, boolean unsafe) {
    elevator.setState(elevatorGoal);
    arm.setState(armGoal);
  }

  private LightsState getLightStateForScoring() {
    return switch (autoAlign.getReefAlignState()) {
      case ALL_CAMERAS_DEAD -> LightsState.ERROR;
      case HAS_TAGS_IN_POSITION -> LightsState.SCORE_ALIGN_READY;
      default -> LightsState.SCORE_ALIGN_NOT_READY;
    };
  }

  public void setConfirmScoreActive(boolean newValue) {
    confirmScoreActive = newValue;
  }
}
