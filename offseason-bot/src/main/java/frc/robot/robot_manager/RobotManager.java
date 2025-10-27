package frc.robot.robot_manager;

import com.team581.controller.RumbleControllerSubsystem;
import com.team581.math.MathHelpers;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.arm.ArmState;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.poses.ReefPipe;
import frc.robot.auto_align.poses.ReefPipeLevel;
import frc.robot.auto_align.poses.ReefSide;
import frc.robot.claw.ClawState;
import frc.robot.claw.ClawSubsystem;
import frc.robot.climber.ClimberState;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.config.FeatureFlags;
import frc.robot.elevator.ElevatorState;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.lights.LightsState;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.ground_manager.GroundManager;
import frc.robot.robot_manager.ground_manager.GroundState;
import frc.robot.swerve.SnapUtil;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionState;
import frc.robot.vision.VisionSubsystem;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  public final LocalizationSubsystem localization;

  public final VisionSubsystem vision;
  public final ImuSubsystem imu;

  public final SwerveSubsystem swerve;
  public final ClawSubsystem claw;
  public final ArmSubsystem arm;
  public final ElevatorSubsystem elevator;
  public final ClimberSubsystem climber;
  public final RumbleControllerSubsystem rumbleController;

  public final GroundManager groundManager;

  private final LightsSubsystem lights;

  public final AutoAlign autoAlign;

  public RobotManager(
      GroundManager groundManager,
      ClawSubsystem claw,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      VisionSubsystem vision,
      ImuSubsystem imu,
      SwerveSubsystem swerve,
      LocalizationSubsystem localization,
      LightsSubsystem lights,
      AutoAlign autoAlign,
      ClimberSubsystem climber,
      RumbleControllerSubsystem rumbleController) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.CLAW_EMPTY);
    this.groundManager = groundManager;
    this.arm = arm;
    this.claw = claw;
    this.elevator = elevator;
    this.vision = vision;
    this.imu = imu;
    this.swerve = swerve;
    this.localization = localization;
    this.lights = lights;
    this.climber = climber;
    this.autoAlign = autoAlign;
    this.rumbleController = rumbleController;

    var stateCount = RobotState.values().length;

    DogLog.log("RobotManager/StateCount", stateCount);
  }

  private ReefSide nearestReefSide = ReefSide.SIDE_GH;
  private ReefPipeLevel scoringLevel = ReefPipeLevel.L4;
  private Pose2d robotPose = new Pose2d();
  private Pose2d lastNetReleasePose = new Pose2d();
  private boolean scoringAlignActive = false;
  private double rawRightControllerYValue = 0.0;
  private boolean reachedCenterSinceLastBumpRequest = false;

  private ArmState latestArmGoal = ArmState.STOWED;
  private ElevatorState latestElevatorGoal = ElevatorState.STOWED;

  @Override
  protected RobotState getNextState(RobotState currentState) {
    if (RobotState.missingGP(currentState, claw.getHasGP())) {
      lights.blinkError();
      if (DriverStation.isEnabled()) {
        DogLog.logFault("MISSING_GAME_PIECE", AlertType.kError);
      }
      return RobotState.CLAW_EMPTY;
    }

    return switch (currentState) {
      case CLAW_ALGAE,
          CLAW_CORAL,
          CLAW_ALGAE_STOW_INWARD,
          ALGAE_PROCESSOR_WAITING,
          ALGAE_NET_WAITING,
          CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBER_STOP,
          UNJAM,
          FORCED_HANDOFF ->
          currentState;

      case CLAW_EMPTY -> {
        if (DriverStation.isDisabled()) {
          if (claw.getHasGP()) {
            DogLog.clearFault("CLAW_MISSING_PRELOAD");
            lights.blink();
            yield RobotState.CLAW_CORAL;
          } else {
            DogLog.logFault("CLAW_MISSING_PRELOAD");
          }
        }
        yield currentState;
      }

      case CORAL_L2_PLACE, CORAL_L3_PLACE, CORAL_L4_PLACE -> {
        if (((FeatureFlags.AUTO_ALIGN_AUTO_SCORE.getAsBoolean() && scoringAlignActive)
                || DriverStation.isAutonomous())
            && ((arm.atGoal() && elevator.atGoal()) || timeout(0.15))) {
          autoAlign.markPipeScored();
          yield currentState.getNextScoreState();
        }
        yield currentState;
      }
      case CORAL_L1_LINEUP, CORAL_L2_LINEUP, CORAL_L3_LINEUP, CORAL_L4_LINEUP -> {
        if (((FeatureFlags.AUTO_ALIGN_AUTO_SCORE.getAsBoolean() && scoringAlignActive)
                || DriverStation.isAutonomous())
            && (autoAlign.isAligned() || (DriverStation.isAutonomous() && timeout(3.0)))
            && arm.atGoal()
            && elevator.atGoal()) {
          yield currentState.getNextScoreState();
        }
        yield currentState;
      }
      case CORAL_L1_PREPARE_HANDOFF,
          CORAL_L2_PREPARE_HANDOFF,
          CORAL_L3_PREPARE_HANDOFF,
          CORAL_L4_PREPARE_HANDOFF ->
          elevator.atGoal() && arm.atGoal() && (groundManager.getTopHasGP() || claw.getHasGP())
              ? currentState.getNextHandoffState()
              : currentState;

      case CORAL_L1_RELEASE_HANDOFF,
          CORAL_L2_RELEASE_HANDOFF,
          CORAL_L3_RELEASE_HANDOFF,
          CORAL_L4_RELEASE_HANDOFF ->
          claw.getHasGP() ? currentState.getNextHandoffState() : currentState;
      case CORAL_L1_AFTER_RELEASE_HANDOFF,
          CORAL_L2_AFTER_RELEASE_HANDOFF,
          CORAL_L3_AFTER_RELEASE_HANDOFF,
          CORAL_L4_AFTER_RELEASE_HANDOFF ->
          elevator.atGoal() && arm.atGoal() ? currentState.getNextHandoffState() : currentState;

      // Approach
      case CORAL_L1_APPROACH ->
          elevator.atGoal() && arm.atGoal() && autoAlign.isCentered()
              ? currentState.getNextScoreState()
              : currentState;

      case CORAL_L2_APPROACH -> {
        yield elevator.nearGoal()
                && arm.nearGoal()
                && (!FeatureFlags.APPROACH_TAG_CHECK.getAsBoolean() || vision.seeingTag())
                && autoAlign.isNearRotationGoal()
            ? currentState.getNextScoreState()
            : currentState;
      }
      case CORAL_L3_APPROACH -> {
        yield elevator.nearGoal()
                && arm.nearGoal()
                && (!FeatureFlags.APPROACH_TAG_CHECK.getAsBoolean() || vision.seeingTag())
                && autoAlign.isNearRotationGoal()
            ? currentState.getNextScoreState()
            : currentState;
      }
      case CORAL_L4_APPROACH -> {
        yield elevator.nearGoal()
                && arm.nearGoal()
                && (!FeatureFlags.APPROACH_TAG_CHECK.getAsBoolean() || vision.seeingTag())
                && autoAlign.isNearRotationGoal()
            ? currentState.getNextScoreState()
            : currentState;
      }

      case CORAL_L1_RELEASE -> {
        if (!claw.getHasGP() && timeout(0.5)) {
          yield currentState.getNextScoreState();
        }

        yield currentState;
      }
      case CORAL_L1_BACKAWAY -> {
        if (DriverStation.isTeleop()) {
          // In teleop, we go to CLAW_EMPTY when you drive away or if we know the score succeeded
          if (drivingAwayFromReef()) {
            yield RobotState.CLAW_EMPTY;
          }
        }
        yield currentState;
      }

      case CORAL_L2_RELEASE, CORAL_L3_RELEASE, CORAL_L4_RELEASE -> {
        if (DriverStation.isTeleop()) {
          // In teleop, we go to CLAW_EMPTY when you drive away or if we know the score succeeded
          if (drivingAwayFromReef()
              || ((autoAlign.isAlgaeRemoved()
                      || groundManager.getState().equals(GroundState.INTAKING))
                  && farEnoughFromReef())) {
            yield RobotState.CLAW_EMPTY;
          }
        }
        yield currentState;
      }

      // Algae scoring
      case ALGAE_PROCESSOR_RELEASE -> {
        if (FeatureFlags.AUTO_STOW_ALGAE.getAsBoolean()) {
          yield timeout(0.5) || !claw.getHasGP() ? RobotState.CLAW_EMPTY : currentState;
        }

        yield currentState;
      }

      case ALGAE_NET_RELEASE -> {
        yield backedAwayFromNetEnough() ? RobotState.CLAW_EMPTY : currentState;
      }

      case ALGAE_OUTTAKE -> timeout(0.5) || !claw.getHasGP() ? RobotState.CLAW_EMPTY : currentState;

      // Intaking

      case ALGAE_INTAKE_L2_APPROACH, ALGAE_INTAKE_L3_APPROACH ->
          arm.nearGoal() && elevator.nearGoal()
              ? currentState.getNextAlgaeIntakeState()
              : currentState;

      case ALGAE_INTAKE_L2, ALGAE_INTAKE_L3 -> {
        if (claw.getHasGP()) {
          rumbleController.rumbleRequest();
          yield currentState.getNextAlgaeIntakeState();
        }

        yield currentState;
      }

      case ALGAE_INTAKE_L2_HOLDING, ALGAE_INTAKE_L3_HOLDING -> {
        if (farEnoughFromReef()) {
          // yield DriverStation.isAutonomous() ? RobotState.CLAW_ALGAE_STOW_INWARD :
          // RobotState.CLAW_ALGAE;
          // TODO: implement stow inward
          yield RobotState.CLAW_ALGAE;
        }

        yield currentState;
      }
      case ALGAE_INTAKE_FLOOR -> {
        if (claw.getHasGP() && swerve.getFieldRelativeSpeeds().vxMetersPerSecond > 0.1) {
          rumbleController.rumbleRequest();
          yield RobotState.CLAW_ALGAE;
        }
        yield currentState;
      }
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case CLAW_EMPTY -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_EMPTY);
        climber.setState(ClimberState.STOPPED);
      }
      case CLAW_ALGAE -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.STOWED_ALGAE, ArmState.STOWED_ALGAE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case CLAW_CORAL -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_CORAL);
        climber.setState(ClimberState.STOPPED);
      }
      case FORCED_HANDOFF -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.OTHER);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_FLOOR -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_GROUND, ArmState.ALGAE_INTAKE_FLOOR);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L2_APPROACH -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L2, ArmState.ALGAE_INTAKE_L2);
        autoAlign.approachAlgaeRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L2 -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L2, ArmState.ALGAE_INTAKE_L2);
        autoAlign.intakeAlgaeRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L2_HOLDING -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L2, ArmState.ALGAE_INTAKE_L2);
        autoAlign.backAwayFromAlgaeRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L3_APPROACH -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L3, ArmState.ALGAE_INTAKE_L3);
        autoAlign.approachAlgaeRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L3 -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L3, ArmState.ALGAE_INTAKE_L3);
        autoAlign.intakeAlgaeRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L3_HOLDING -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L3, ArmState.ALGAE_INTAKE_L3);
        autoAlign.backAwayFromAlgaeRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_NET_WAITING -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET);
        swerve.snapsDriveRequest(SnapUtil.getNetScoringAngle(robotPose));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_NET_RELEASE -> {
        claw.setState(ClawState.SCORE_ALGAE_NET);
        moveSuperstructure(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING_ALGAE);
        climber.setState(ClimberState.STOPPED);
        lastNetReleasePose = robotPose;
      }
      case ALGAE_PROCESSOR_WAITING -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.PROCESSOR, ArmState.ALGAE_PROCESSOR);
        swerve.snapsDriveRequest(SnapUtil.getProcessorAngle());
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_PROCESSOR_RELEASE -> {
        claw.setState(ClawState.SCORE_ALGAE_PROCESSOR);
        moveSuperstructure(ElevatorState.PROCESSOR, ArmState.ALGAE_PROCESSOR);
        swerve.snapsDriveRequest(SnapUtil.getProcessorAngle());
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      // Handoff states
      case CORAL_L1_PREPARE_HANDOFF,
          CORAL_L2_PREPARE_HANDOFF,
          CORAL_L3_PREPARE_HANDOFF,
          CORAL_L4_PREPARE_HANDOFF -> {
        claw.setState(ClawState.CORAL_HANDOFF);
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.CORAL_HANDOFF);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L1_RELEASE_HANDOFF,
          CORAL_L2_RELEASE_HANDOFF,
          CORAL_L3_RELEASE_HANDOFF,
          CORAL_L4_RELEASE_HANDOFF -> {
        claw.setState(ClawState.CORAL_HANDOFF);
        moveSuperstructure(ElevatorState.HANDOFF, ArmState.CORAL_HANDOFF);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.CORAL_HANDOFF);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L1_AFTER_RELEASE_HANDOFF,
          CORAL_L2_AFTER_RELEASE_HANDOFF,
          CORAL_L3_AFTER_RELEASE_HANDOFF,
          CORAL_L4_AFTER_RELEASE_HANDOFF -> {
        claw.setState(ClawState.CORAL_HANDOFF);
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.CORAL_HANDOFF);
        climber.setState(ClimberState.STOPPED);
      }

      // L1
      case CORAL_L1_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.L1_SCORE_LINEUP, ArmState.CORAL_SCORE_LINEUP_L1);
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L1_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.L1_SCORE_LINEUP, ArmState.CORAL_SCORE_LINEUP_L1);
        autoAlign.lineupL1Request();
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L1_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL_L1);
        moveSuperstructure(ElevatorState.L1_SCORE_RELEASE, ArmState.CORAL_SCORE_RELEASE_L1);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING_CORAL);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L1_BACKAWAY -> {
        claw.setState(ClawState.SCORE_CORAL_L1);
        moveSuperstructure(ElevatorState.L1_SCORE_RELEASE, ArmState.CORAL_SCORE_RELEASE_L1);
        autoAlign.backAwayFromL1Request();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING_CORAL);
        climber.setState(ClimberState.STOPPED);
      }

      // L2
      case CORAL_L2_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.L2_SCORE_LINEUP, ArmState.CORAL_SCORE_LINEUP_L2);
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L2_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.L2_SCORE_LINEUP, ArmState.CORAL_SCORE_LINEUP_L2);
        vision.setState(VisionState.TAGS);
        autoAlign.lineupPipeRequest();
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L2_PLACE -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.L2_SCORE_RELEASE, ArmState.CORAL_SCORE_RELEASE_L2);
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L2_RELEASE -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.L2_SCORE_RELEASE, ArmState.CORAL_SCORE_RELEASE_L2);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING_CORAL);
        autoAlign.backAwayFromPipeRequest();
        climber.setState(ClimberState.STOPPED);
      }
      // L3
      case CORAL_L3_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.L3_SCORE_LINEUP, ArmState.CORAL_SCORE_LINEUP_L3);
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L3_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.L3_SCORE_LINEUP, ArmState.CORAL_SCORE_LINEUP_L3);
        autoAlign.lineupPipeRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L3_PLACE -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.L3_SCORE_RELEASE, ArmState.CORAL_SCORE_RELEASE_L3);
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L3_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(ElevatorState.L3_SCORE_RELEASE, ArmState.CORAL_SCORE_RELEASE_L3);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING_CORAL);
        autoAlign.backAwayFromPipeRequest();

        climber.setState(ClimberState.STOPPED);
      }
      // L4
      case CORAL_L4_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.L4_SCORE_LINEUP, ArmState.CORAL_SCORE_LINEUP_L4);
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());

        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L4_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.L4_SCORE_LINEUP, ArmState.CORAL_SCORE_LINEUP_L4);
        autoAlign.lineupPipeRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());

        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L4_PLACE -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.L4_SCORE_RELEASE, ArmState.CORAL_SCORE_RELEASE_L4);
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L4_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(ElevatorState.L4_SCORE_RELEASE, ArmState.CORAL_SCORE_RELEASE_L4);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING_CORAL);
        autoAlign.backAwayFromPipeRequest();

        climber.setState(ClimberState.STOPPED);
      }
      // Climb
      case CLIMBING_1_LINEUP -> {
        claw.setState(ClawState.IDLE_NO_GP);
        groundManager.climbRequest();
        moveSuperstructure(ElevatorState.CLIMBING, ArmState.CLIMBING);
        swerve.climbRequest(SnapUtil.getCageAngle(robotPose));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.CLIMB_LINEUP);
        climber.setState(ClimberState.LINEUP_FORWARD);
      }
      case CLIMBING_2_HANGING -> {
        claw.setState(ClawState.IDLE_NO_GP);
        groundManager.climbRequest();
        moveSuperstructure(ElevatorState.CLIMBING, ArmState.CLIMBING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.CLIMB_HANG);
        climber.setState(ClimberState.HANGING);
      }
      case CLIMBER_STOP -> {
        claw.setState(ClawState.IDLE_NO_GP);
        groundManager.climbRequest();
        moveSuperstructure(ElevatorState.CLIMBING, ArmState.CLIMBING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.CLIMB_STOP);
        climber.setState(ClimberState.STOPPED);
      }
      case UNJAM -> {
        claw.setState(ClawState.OUTTAKING);
        moveSuperstructure(ElevatorState.UNJAM, ArmState.UNJAM);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.OTHER);
        groundManager.outtakeRequest();
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_OUTTAKE -> {
        claw.setState(ClawState.OUTTAKING);
        moveSuperstructure(ElevatorState.ALGAE_OUTTAKE, ArmState.ALGAE_OUTTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case CLAW_ALGAE_STOW_INWARD -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.STOWED_ALGAE, ArmState.STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
    }
  }

  @Override
  public void whileInState(RobotState currentState) {
    moveSuperstructure(latestElevatorGoal, latestArmGoal);

    DogLog.log("RobotManager/NearestReefSidePose", nearestReefSide.getPose(robotPose));
    MechanismVisualizer.log(elevator.getHeight(), arm.getAngle(), groundManager.deploy.getAngle());

    switch (getState()) {
      case ALGAE_INTAKE_L2_APPROACH,
          ALGAE_INTAKE_L3_APPROACH,
          ALGAE_INTAKE_L2,
          ALGAE_INTAKE_L3,
          ALGAE_INTAKE_L2_HOLDING,
          ALGAE_INTAKE_L3_HOLDING -> {
        if (scoringAlignActive && vision.isAnyCameraOnlineForTags()) {
          swerve.driveToPoseRequest(autoAlign.getCurrentTargetPose(), autoAlign.useAngleBisector());
        } else {
          swerve.normalDriveRequest();
        }
        lights.setState(getLightStateFoAlgaeIntaking());
      }

      case CORAL_L1_PREPARE_HANDOFF,
          CORAL_L1_RELEASE_HANDOFF,
          CORAL_L1_AFTER_RELEASE_HANDOFF,
          CORAL_L1_APPROACH,
          CORAL_L1_LINEUP,
          CORAL_L1_RELEASE,
          CORAL_L1_BACKAWAY,
          CORAL_L2_PREPARE_HANDOFF,
          CORAL_L2_RELEASE_HANDOFF,
          CORAL_L2_AFTER_RELEASE_HANDOFF,
          CORAL_L3_PREPARE_HANDOFF,
          CORAL_L3_RELEASE_HANDOFF,
          CORAL_L3_AFTER_RELEASE_HANDOFF,
          CORAL_L4_PREPARE_HANDOFF,
          CORAL_L4_RELEASE_HANDOFF,
          CORAL_L4_AFTER_RELEASE_HANDOFF,
          CORAL_L2_APPROACH,
          CORAL_L3_APPROACH,
          CORAL_L4_APPROACH,
          CORAL_L2_LINEUP,
          CORAL_L3_LINEUP,
          CORAL_L4_LINEUP,
          CORAL_L2_PLACE,
          CORAL_L3_PLACE,
          CORAL_L4_PLACE,
          CORAL_L2_RELEASE,
          CORAL_L3_RELEASE,
          CORAL_L4_RELEASE -> {
        if (scoringAlignActive && vision.isAnyCameraOnlineForTags()) {
          swerve.driveToPoseRequest(
              autoAlign.getCurrentTargetPose(),
              autoAlign.useAngleBisector(),
              autoAlign.getVelocityLimit());
        } else {
          swerve.normalDriveRequest();
        }

        if (reachedCenterSinceLastBumpRequest) {
          if (rawRightControllerYValue > 0.5) {
            reachedCenterSinceLastBumpRequest = false;
            bumpUpLevelRequest();
          } else if (rawRightControllerYValue < -0.5) {
            reachedCenterSinceLastBumpRequest = false;
            bumpDownLevelRequest();
          }
        }

        lights.setState(getLightStateForScoring());
      }
      case CLAW_EMPTY -> {
        lights.setState(LightsState.IDLE_EMPTY);
        if (groundManager.getBottomHasGP() && vision.isAnyCameraOnlineForTags()) {
          swerve.snapsDriveRequest(
              MathHelpers.getDriveDirection(AutoAlign.getAllianceCenterOfReef(robotPose), robotPose)
                  .getDegrees());

        } else {
          swerve.normalDriveRequest();
        }
      }
      case CLAW_ALGAE -> {
        if (AutoAlign.shouldScoreInNet(robotPose) && AutoAlign.isCloseToNet(robotPose)) {
          swerve.snapsDriveRequest(SnapUtil.getNetScoringAngle(robotPose));
        } else if (!AutoAlign.shouldScoreInNet(robotPose)) {
          swerve.snapsDriveRequest(SnapUtil.getProcessorAngle());
        } else {
          swerve.normalDriveRequest();
        }
        lights.setState(LightsState.HOLDING_ALGAE);
      }
      case CLAW_CORAL -> {
        if (vision.isAnyCameraOnlineForTags()) {
          swerve.snapsDriveRequest(
              MathHelpers.getDriveDirection(AutoAlign.getAllianceCenterOfReef(robotPose), robotPose)
                  .getDegrees());
        } else {
          swerve.normalDriveRequest();
        }
      }

      case ALGAE_NET_WAITING -> {
        if (reachedCenterSinceLastBumpRequest) {
          if (rawRightControllerYValue > 0.5) {
            reachedCenterSinceLastBumpRequest = false;

          } else if (rawRightControllerYValue < -0.5) {
            reachedCenterSinceLastBumpRequest = false;
            processorWaitingRequest();
          }
        }
      }

      case ALGAE_PROCESSOR_WAITING -> {
        if (reachedCenterSinceLastBumpRequest) {
          if (rawRightControllerYValue > 0.5) {
            reachedCenterSinceLastBumpRequest = false;
            algaeNetRequest();
          } else if (rawRightControllerYValue < -0.5) {
            reachedCenterSinceLastBumpRequest = false;
          }
        }
      }
      default -> {}
    }

    if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      // Do nothing, field calibration will set the lights manually
    } else if (vision.isAnyCameraOffline()) {
      lights.setDisabledState(LightsState.ERROR);
    } else if (arm.getState() == ArmState.PRE_MATCH_HOMING && !arm.rangeOfMotionGood()) {
      lights.setDisabledState(LightsState.UNHOMED);
    } else if (vision.seeingTagDebounced()) {
      lights.setDisabledState(LightsState.HOMED_SEES_TAGS);
    } else {
      lights.setDisabledState(LightsState.HOMED_NO_TAGS);
    }

    switch (getState()) {
      case ALGAE_INTAKE_FLOOR -> {
        if (claw.getHasGP()) {
          rumbleController.rumbleRequest();
        }
      }
      default -> {}
    }
  }

  @Override
  protected void collectInputs() {
    if (DriverStation.isAutonomous()) {
      scoringAlignActive = true;
    }

    vision.setEstimatedPoseAngle(localization.getPose().getRotation().getDegrees());
    nearestReefSide = autoAlign.getClosestReefSide();
    robotPose = localization.getPose();
    if (Math.abs(rawRightControllerYValue) < 0.5) {
      reachedCenterSinceLastBumpRequest = true;
    }

    scoringLevel =
        switch (getState()) {
          case CORAL_L1_PREPARE_HANDOFF,
              CORAL_L1_RELEASE_HANDOFF,
              CORAL_L1_AFTER_RELEASE_HANDOFF,
              CORAL_L1_APPROACH,
              CORAL_L1_LINEUP,
              CORAL_L1_RELEASE ->
              ReefPipeLevel.L1;

          case CORAL_L2_PREPARE_HANDOFF,
              CORAL_L2_RELEASE_HANDOFF,
              CORAL_L2_AFTER_RELEASE_HANDOFF,
              CORAL_L2_APPROACH,
              CORAL_L2_RELEASE ->
              ReefPipeLevel.L2;

          case CORAL_L3_PREPARE_HANDOFF,
              CORAL_L3_RELEASE_HANDOFF,
              CORAL_L3_AFTER_RELEASE_HANDOFF,
              CORAL_L3_APPROACH,
              CORAL_L3_RELEASE ->
              ReefPipeLevel.L3;

          case CORAL_L4_PREPARE_HANDOFF,
              CORAL_L4_RELEASE_HANDOFF,
              CORAL_L4_AFTER_RELEASE_HANDOFF,
              CORAL_L4_APPROACH,
              CORAL_L4_RELEASE ->
              ReefPipeLevel.L4;

          case CORAL_L2_LINEUP, CORAL_L2_PLACE -> ReefPipeLevel.L2;
          case CORAL_L3_LINEUP, CORAL_L3_PLACE -> ReefPipeLevel.L3;
          case CORAL_L4_LINEUP, CORAL_L4_PLACE -> ReefPipeLevel.L4;
          default -> ReefPipeLevel.RAISING;
        };

    autoAlign.setScoringLevel(scoringLevel);

    swerve.setElevatorHeight(elevator.getHeight());
  }

  private boolean backedAwayFromNetEnough() {
    var rotation = lastNetReleasePose.getRotation().getDegrees();
    var redSide = MathUtil.isNear(180, rotation, 10);
    var farEnoughFromReleasePose =
        redSide
            ? robotPose.getX() >= lastNetReleasePose.getX() + Units.inchesToMeters(5.0)
            : robotPose.getX() < lastNetReleasePose.getX() - Units.inchesToMeters(5.0);

    var xMetersPerSecond = swerve.getTeleopSpeeds().vxMetersPerSecond;
    var swerveMovingFastEnough = redSide ? xMetersPerSecond > 0.1 : xMetersPerSecond < -0.1;

    return farEnoughFromReleasePose && swerveMovingFastEnough;
  }

  private boolean farEnoughFromReef() {
    var tagCameraOnline = vision.isAnyCameraOnlineForTags();

    if (!tagCameraOnline) {
      return timeout(0.5);
    }

    var isFarEnoughFromReefSide = !autoAlign.isCloseToReefSide(0.8);

    return isFarEnoughFromReefSide;
  }

  private boolean drivingAwayFromReef() {
    var tagCameraOnline = vision.isAnyCameraOnlineForTags();

    if (!tagCameraOnline) {
      return timeout(1.0);
    }

    var isFarEnoughFromReefSide = !autoAlign.isCloseToReefSide(0.8);

    var speeds = swerve.getTeleopSpeeds();
    var isDrivingAway = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > 0.3;
    return isFarEnoughFromReefSide && isDrivingAway;
  }

  public void intakeRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }
    groundManager.intakeRequest();
  }

  public void forceIdleNoGp() {
    setStateFromRequest(RobotState.CLAW_EMPTY);
  }

  public void stowRequest() {
    groundManager.stowRequest();
    switch (getState()) {
      case ALGAE_INTAKE_L2, ALGAE_INTAKE_L3 -> setStateFromRequest(RobotState.CLAW_EMPTY);
      default -> {
        if (claw.getHasGP()) {
          // Claw is maybe algae or coral
          if (getState().clawGp == ClawGamePiece.ALGAE) {
            setStateFromRequest(RobotState.CLAW_ALGAE);
          } else {
            setStateFromRequest(RobotState.CLAW_CORAL);
          }
        } else {
          setStateFromRequest(RobotState.CLAW_EMPTY);
        }
      }
    }
  }

  public void forcedHandoffRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    setStateFromRequest(RobotState.FORCED_HANDOFF);
  }

  public void intakeFloorAlgaeRequest() {
    if (!getState().climbingOrRehoming && !RobotState.isHandoffReleaseState(getState())) {
      if (groundManager.getState().equals(GroundState.INTAKING)) {
        groundManager.outtakeRequest();
      } else {
        setStateFromRequest(RobotState.ALGAE_INTAKE_FLOOR);
      }
    }
  }

  public void stopOuttakeRequest() {
    if (groundManager.getState().equals(GroundState.OUTTAKING)) {
      groundManager.intakeRequest();
    }
  }

  public void processorWaitingRequest() {
    if (!getState().climbingOrRehoming && !RobotState.isHandoffReleaseState(getState())) {
      setStateFromRequest(RobotState.ALGAE_PROCESSOR_WAITING);
    }
  }

  public void scoringAlignOffRequest() {
    scoringAlignActive = false;
  }

  public void l4CoralAutoApproachRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(RobotState.CORAL_L4_APPROACH);
    }
  }

  public void l3CoralAutoApproachRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(RobotState.CORAL_L3_APPROACH);
    }
  }

  public void l2CoralAutoApproachRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(RobotState.CORAL_L2_APPROACH);
    }
  }

  public void l2CoralAutoLineupRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(RobotState.CORAL_L2_LINEUP);
    }
  }

  public void l4CoralApproachRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }
    autoAlign.approachPipeRequest();
    if (claw.getHasGP() || RobotState.isLineupOrApproachState(getState())) {
      setStateFromRequest(RobotState.CORAL_L4_APPROACH);
    } else {
      setStateFromRequest(RobotState.CORAL_L4_PREPARE_HANDOFF);
    }
  }

  public void l3CoralApproachRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    autoAlign.approachPipeRequest();

    if (claw.getHasGP() || RobotState.isLineupOrApproachState(getState())) {
      setStateFromRequest(RobotState.CORAL_L3_APPROACH);
    } else {
      setStateFromRequest(RobotState.CORAL_L3_PREPARE_HANDOFF);
    }
  }

  public void l2CoralApproachRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    autoAlign.approachPipeRequest();

    if (claw.getHasGP() || RobotState.isLineupOrApproachState(getState())) {
      setStateFromRequest(RobotState.CORAL_L2_APPROACH);
    } else {
      setStateFromRequest(RobotState.CORAL_L2_PREPARE_HANDOFF);
    }
  }

  public void l1CoralApproachRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }
    autoAlign.approachL1Request();
    if (claw.getHasGP() || RobotState.isLineupOrApproachState(getState())) {
      setStateFromRequest(RobotState.CORAL_L1_APPROACH);
    } else {
      setStateFromRequest(RobotState.CORAL_L1_PREPARE_HANDOFF);
    }
  }

  public void highLineupRequest() {
    if (!getState().climbingOrRehoming) {
      if (getState().clawGp == ClawGamePiece.ALGAE) {
        algaeNetRequest();
      } else {
        l4CoralApproachRequest();
      }
    }
  }

  public void lowLineupRequest() {
    if (!getState().climbingOrRehoming) {
      if (getState().clawGp == ClawGamePiece.ALGAE) {
        processorWaitingRequest();
      } else {
        l1CoralApproachRequest();
      }
    }
  }

  public void l4CoralReleaseRequest() {
    if (!getState().climbingOrRehoming) {
      if (claw.getHasGP()) {
        setStateFromRequest(RobotState.CORAL_L4_RELEASE);
      } else {
        setStateFromRequest(RobotState.CORAL_L4_PREPARE_HANDOFF);
      }
    }
  }

  public void l3CoralReleaseRequest() {
    if (!getState().climbingOrRehoming) {
      if (claw.getHasGP()) {
        setStateFromRequest(RobotState.CORAL_L3_RELEASE);
      } else {
        setStateFromRequest(RobotState.CORAL_L3_PREPARE_HANDOFF);
      }
    }
  }

  public void algaeReefIntakeRequest() {
    if (!getState().climbingOrRehoming && !RobotState.isHandoffReleaseState(getState())) {
      scoringAlignActive = true;
      autoAlign.approachAlgaeRequest();
      if (nearestReefSide.algaeHeight == ReefPipeLevel.L3) {
        setStateFromRequest(RobotState.ALGAE_INTAKE_L3_APPROACH);
      } else {
        setStateFromRequest(RobotState.ALGAE_INTAKE_L2_APPROACH);
      }
    }
  }

  private void algaeNetRequest() {
    if (!getState().climbingOrRehoming && !RobotState.isHandoffReleaseState(getState())) {
      setStateFromRequest(RobotState.ALGAE_NET_WAITING);
    }
  }

  public void scoreRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }
    switch (getState()) {
      case CLAW_ALGAE, ALGAE_INTAKE_FLOOR, ALGAE_INTAKE_L2_HOLDING, ALGAE_INTAKE_L3_HOLDING -> {
        if (AutoAlign.shouldScoreInNet(robotPose)) {
          setStateFromRequest(RobotState.ALGAE_NET_WAITING);

        } else {
          setStateFromRequest(RobotState.ALGAE_PROCESSOR_WAITING);
        }
      }

      case ALGAE_PROCESSOR_WAITING -> setStateFromRequest(RobotState.ALGAE_PROCESSOR_RELEASE);

      case ALGAE_NET_WAITING -> setStateFromRequest(RobotState.ALGAE_NET_RELEASE);

      default -> {
        scoringAlignActive = true;
        autoAlign.approachPipeRequest();
        var bestLevel = autoAlign.getBestLevel();
        DogLog.log("Debug/BestLevel", bestLevel);
        switch (bestLevel) {
          case L4 -> l4CoralApproachRequest();
          case L3 -> l3CoralApproachRequest();
          case L2 -> l2CoralApproachRequest();
          case L1 -> l1CoralApproachRequest();
          default -> {}
        }
      }
    }
  }

  public void scoreRequest(ReefPipe pipe, ReefPipeLevel level) {
    autoAlign.setAutoPipeOverride(pipe, level);
    if (getState().climbingOrRehoming) {
      return;
    }
    switch (getState()) {
      case CLAW_ALGAE, ALGAE_INTAKE_FLOOR, ALGAE_INTAKE_L2_HOLDING, ALGAE_INTAKE_L3_HOLDING -> {
        if (AutoAlign.shouldScoreInNet(robotPose)) {
          setStateFromRequest(RobotState.ALGAE_NET_WAITING);

        } else {
          setStateFromRequest(RobotState.ALGAE_PROCESSOR_WAITING);
        }
      }

      case ALGAE_PROCESSOR_WAITING -> setStateFromRequest(RobotState.ALGAE_PROCESSOR_RELEASE);

      case ALGAE_NET_WAITING -> setStateFromRequest(RobotState.ALGAE_NET_RELEASE);

      default -> {
        scoringAlignActive = true;
        autoAlign.approachPipeRequest();
        var bestLevel = autoAlign.getBestLevel();
        DogLog.log("Debug/BestLevel", bestLevel);
        switch (bestLevel) {
          case L4 -> l4CoralApproachRequest();
          case L3 -> l3CoralApproachRequest();
          case L2 -> l2CoralApproachRequest();
          case L1 -> l1CoralApproachRequest();
          default -> {}
        }
      }
    }
  }

  public void forceNextScoringStateRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }
    setStateFromRequest(getState().getNextScoreState());
  }

  private void bumpDownLevelRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    switch (scoringLevel) {
      case L4 -> {
        autoAlign.markLevelScored(scoringLevel);
        autoAlign.bumpRequest(ReefPipeLevel.L3);
        l3CoralApproachRequest();
      }
      case L3 -> {
        autoAlign.markLevelScored(scoringLevel);
        autoAlign.bumpRequest(ReefPipeLevel.L2);
        l2CoralApproachRequest();
      }
      case L2 -> {
        autoAlign.markLevelScored(scoringLevel);
        autoAlign.bumpRequest(ReefPipeLevel.L1);
        l1CoralApproachRequest();
      }

      default -> {}
    }
  }

  public void bumpUpLevelRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    switch (getState()) {
      case ALGAE_PROCESSOR_WAITING -> {
        algaeNetRequest();
      }
      default -> {}
    }

    switch (scoringLevel) {
      case L3 -> {
        autoAlign.bumpRequest(ReefPipeLevel.L4);

        l4CoralApproachRequest();
      }
      case L2 -> {
        autoAlign.bumpRequest(ReefPipeLevel.L3);

        l3CoralApproachRequest();
      }
      case L1 -> {
        autoAlign.bumpRequest(ReefPipeLevel.L3);

        l2CoralApproachRequest();
      }
      default -> {}
    }
  }

  public void setRawRightControllerYValue(double value) {
    rawRightControllerYValue = value;
  }

  public void nextClimbStateRequest() {
    switch (getState()) {
      case CLIMBER_STOP -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
      case CLAW_EMPTY, CLAW_CORAL, CLAW_ALGAE -> {
        if (arm.atGoal() && elevator.atGoal()) {
          setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
        }
      }
      case CLIMBING_1_LINEUP -> setStateFromRequest(RobotState.CLIMBING_2_HANGING);
      case CLIMBING_2_HANGING -> {}
      default -> {
        // Do nothing
      }
    }
  }

  public void stopClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> setStateFromRequest(RobotState.CLIMBER_STOP);
      default -> {}
    }
  }

  public void unjamRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.UNJAM);
    }
  }

  private void moveSuperstructure(ElevatorState elevatorGoal, ArmState armGoal) {
    latestArmGoal = armGoal;
    latestElevatorGoal = elevatorGoal;
    elevator.setState(elevatorGoal);
    arm.setState(armGoal);
  }

  private LightsState getLightStateFoAlgaeIntaking() {
    if (!vision.isAnyCameraOnlineForTags()) {
      return LightsState.ERROR;
    }

    if (claw.getHasGP()) {
      return LightsState.SCORE_ALIGN_TAGS;
    }
    return LightsState.SCORE_NO_ALIGN_TAGS;
  }

  private LightsState getLightStateForScoring() {
    return switch (autoAlign.getTagAlignState()) {
      case ALL_CAMERAS_DEAD -> LightsState.ERROR;
      case HAS_TAGS_IN_POSITION, HAS_TAGS_WRONG_POSITION ->
          scoringAlignActive ? LightsState.SCORE_ALIGN_TAGS : LightsState.SCORE_NO_ALIGN_TAGS;
      case NO_TAGS_IN_POSITION, NO_TAGS_WRONG_POSITION ->
          scoringAlignActive ? LightsState.SCORE_ALIGN_NO_TAGS : LightsState.SCORE_NO_ALIGN_NO_TAGS;
    };
  }
}
