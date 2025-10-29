package frc.robot.auto_align;

import static java.util.Comparator.comparingDouble;

import com.google.common.collect.ImmutableList;
import com.team581.auto_align.TagAlignState;
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto_align.tag_align.AlignmentCostUtil;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveState;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionSubsystem;
import java.util.Collections;

public class AutoAlign extends StateMachineSubsystem<AutoAlignState> {
  private static final ImmutableList<ReefSide> ALL_REEF_SIDES =
      ImmutableList.copyOf(ReefSide.values());
  public static final ImmutableList<ReefPipe> ALL_REEF_PIPES =
      ImmutableList.copyOf(ReefPipe.values());

  private static final Translation2d CENTER_OF_REEF_RED =
      new Translation2d(Units.inchesToMeters(514.13), Units.inchesToMeters(158.5));
  private static final Translation2d CENTER_OF_REEF_BLUE =
      new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.5));

  private static final DoubleSubscriber TRANSLATION_GOOD_THRESHOLD =
      DogLog.tunable("AutoAlign/IsAlignedTranslation", Units.inchesToMeters(1.0));
  private static final DoubleSubscriber ROTATION_GOOD_THRESHOLD =
      DogLog.tunable("AutoAlign/IsAlignedRotation", 3.0);

  public static Translation2d getAllianceCenterOfReef() {
    return FmsUtil.isRedAlliance() ? CENTER_OF_REEF_RED : CENTER_OF_REEF_BLUE;
  }

  public static Translation2d getAllianceCenterOfReef(Pose2d robotPose) {
    return robotPose.getX() > 17.55 / 2 ? CENTER_OF_REEF_RED : CENTER_OF_REEF_BLUE;
  }

  public static boolean isCloseToReefSide(
      Pose2d robotPose, Pose2d nearestReefSide, double thresholdMeters) {
    return robotPose.getTranslation().getDistance(nearestReefSide.getTranslation())
        < thresholdMeters;
  }

  private final VisionSubsystem vision;
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final AlignmentCostUtil alignmentCostUtil;
  private final boolean explicitSelection;
  private final Debouncer isAlignedDebouncer = new Debouncer(0.1, DebounceType.kRising);
  private final ReefState reefState = new ReefState();

  private boolean isAligned = false;
  private boolean isAlignedDebounced = false;
  private double rawControllerXValue = 0.0;
  private double rawControllerYValue = 0.0;
  private ReefSide bestAlgaeSide = ReefSide.SIDE_AB;
  private ReefPipe bestPipe = ReefPipe.PIPE_A;
  private ReefSide closestReefSide = ReefSide.SIDE_AB;
  private ReefPipeLevel currentReefPipeLevel = ReefPipeLevel.L1;
  private Pose2d currentPose = Pose2d.kZero;
  private Pose2d currentTargetPose = Pose2d.kZero;
  private Pose2d autoTargetPoseOverride = new Pose2d();
  private boolean useAngleBisector = true;
  private boolean driverJoystickReachedCenter = false;
  private boolean bestPipeSelected = false;

  public AutoAlign(
      VisionSubsystem vision,
      LocalizationSubsystem localization,
      SwerveSubsystem swerve,
      boolean explicitSelection) {
    super(SubsystemPriority.AUTO_ALIGN, AutoAlignState.EXPLICIT_SAFE_WAITING);
    alignmentCostUtil = new AlignmentCostUtil(localization, swerve, reefState);
    this.vision = vision;
    this.localization = localization;
    this.swerve = swerve;
    this.explicitSelection = explicitSelection;
  }

  @Override
  protected AutoAlignState getNextState(AutoAlignState currentState) {
    return switch (getState()) {
      case EXPLICIT_SAFE_WAITING -> {
        var wantedState = getWantedPipeSideState(closestReefSide);
        yield wantedState == AutoAlignState.LEFT_PIPE
            ? AutoAlignState.EXPLICIT_LEFT_CENTER
            : wantedState == AutoAlignState.RIGHT_PIPE
                ? AutoAlignState.EXPLICIT_RIGHT_CENTER
                : currentState;
      }
      case EXPLICIT_SAFE -> {
        var wantedState = getWantedPipeSideState(closestReefSide);
        yield wantedState == AutoAlignState.LEFT_PIPE
            ? AutoAlignState.LEFT_PIPE
            : wantedState == AutoAlignState.RIGHT_PIPE ? AutoAlignState.RIGHT_PIPE : currentState;
      }
      case EXPLICIT_LEFT_CENTER -> {
        if (currentPose.getTranslation().getDistance(currentTargetPose.getTranslation())
                < Units.inchesToMeters(15.0)
            && MathUtil.isNear(
                currentTargetPose.getRotation().getDegrees(),
                currentPose.getRotation().getDegrees(),
                25.0,
                -180.0,
                180.0)) {
          yield AutoAlignState.EXPLICIT_LEFT_WAITING;
        } else if (getWantedPipeSideState(closestReefSide) == AutoAlignState.RIGHT_PIPE) {
          yield AutoAlignState.EXPLICIT_RIGHT_CENTER;
        }
        yield currentState;
      }
      case EXPLICIT_RIGHT_CENTER -> {
        if (currentPose.getTranslation().getDistance(currentTargetPose.getTranslation())
                < Units.inchesToMeters(15.0)
            && MathUtil.isNear(
                currentTargetPose.getRotation().getDegrees(),
                currentPose.getRotation().getDegrees(),
                25.0,
                -180.0,
                180.0)) {
          yield AutoAlignState.EXPLICIT_RIGHT_WAITING;
        } else if (getWantedPipeSideState(closestReefSide) == AutoAlignState.LEFT_PIPE) {
          yield AutoAlignState.EXPLICIT_LEFT_CENTER;
        }
        yield currentState;
      }
      case BEST_PIPE_CENTER -> {
        if (currentPose.getTranslation().getDistance(currentTargetPose.getTranslation())
                < Units.inchesToMeters(15.0)
            && MathUtil.isNear(
                currentTargetPose.getRotation().getDegrees(),
                currentPose.getRotation().getDegrees(),
                25.0,
                -180.0,
                180.0)) {
          yield AutoAlignState.BEST_PIPE_WAITING;
        } else if (getWantedPipeSideState(closestReefSide) == AutoAlignState.LEFT_PIPE) {
          yield AutoAlignState.EXPLICIT_LEFT_CENTER;
        } else if (getWantedPipeSideState(closestReefSide) == AutoAlignState.RIGHT_PIPE) {
          yield AutoAlignState.EXPLICIT_RIGHT_CENTER;
        }
        yield currentState;
      }
      case BEST_PIPE_WAITING, EXPLICIT_LEFT_WAITING, EXPLICIT_RIGHT_WAITING -> {
        if (getWantedPipeSideState(closestReefSide) == AutoAlignState.LEFT_PIPE) {
          yield AutoAlignState.EXPLICIT_LEFT_WAITING;
        } else if (getWantedPipeSideState(closestReefSide) == AutoAlignState.RIGHT_PIPE) {
          yield AutoAlignState.EXPLICIT_RIGHT_WAITING;
        }
        yield currentState;
      }
      case LEFT_PIPE, RIGHT_PIPE, BEST_PIPE -> {
        if (getWantedPipeSideState(closestReefSide) == AutoAlignState.LEFT_PIPE) {
          yield AutoAlignState.LEFT_PIPE;
        } else if (getWantedPipeSideState(closestReefSide) == AutoAlignState.RIGHT_PIPE) {
          yield AutoAlignState.RIGHT_PIPE;
        }
        yield currentState;
      }
      default -> currentState;
    };
  }

  @Override
  protected void beforeTransition(AutoAlignState oldState, AutoAlignState newState) {
    if (newState == AutoAlignState.BEST_PIPE_CENTER
        || newState == AutoAlignState.EXPLICIT_SAFE_WAITING) {
      driverJoystickReachedCenter = false;
    }
  }

  @Override
  protected void collectInputs() {
    currentPose = localization.getPose();
    bestAlgaeSide = getBestAlgaeSide();
    closestReefSide = getClosestReefSide();
    currentTargetPose = findTargetPose();
    isAligned = isRobotPoseAlignedWithTargetPose();
    isAlignedDebounced = isAlignedDebouncer.calculate(isAligned);
    if (!explicitSelection && !bestPipeSelected && currentReefPipeLevel != ReefPipeLevel.L1) {
      bestPipe = getBestPipeForScoring();
      DogLog.log("AutoAlign/BestPipe", bestPipe);
      bestPipeSelected = true;
    }

    DogLog.log("AutoAlign/CurrentLevel", currentReefPipeLevel);
    DogLog.log("AutoAlign/PoleSelectioin/JoystickReachedCenter", driverJoystickReachedCenter);
    var controllerValues = swerve.getControllerValues();
    rawControllerXValue = controllerValues.getX();
    rawControllerYValue = controllerValues.getY();
    DogLog.log("AutoAlign/PoleSelectioin/RawControllerX", rawControllerXValue);
    DogLog.log("AutoAlign/PoleSelectioin/RawControllerY", rawControllerYValue);
    DogLog.log(
        "AutoAlign/PoleSelectioin/Hypot", Math.hypot(rawControllerXValue, rawControllerYValue));

    // Only allow switching pipe sides if the driver has let the joystick return to center
    // Resets when entering BEST_PIPE_CENTER or EXPLICIT_SAFE_WAITING
    if (Math.hypot(rawControllerXValue, rawControllerYValue) < 0.3) {
      driverJoystickReachedCenter = true;
    }

    switch (getState()) {
      case LEFT_PIPE, RIGHT_PIPE, BEST_PIPE, PIPE_BACKUP, ALGAE_BACKUP -> {
        useAngleBisector = false;
      }

      default -> {
        useAngleBisector = true;
      }
    }
  }

  private Pose2d findTargetPose() {
    return switch (getState()) {
      case EXPLICIT_SAFE, EXPLICIT_SAFE_WAITING ->
          getClosestReefSide().getPose(ReefSideOffset.SAFE, currentPose);
      case EXPLICIT_LEFT_CENTER ->
          getCenterPipePoseFromRobotDistance(getClosestReefSide().leftPipe);
      case EXPLICIT_RIGHT_CENTER ->
          getCenterPipePoseFromRobotDistance(getClosestReefSide().rightPipe);
      case EXPLICIT_LEFT_WAITING ->
          getClosestReefSide().leftPipe.getPose(ReefPipeLevel.RAISING, currentPose);
      case EXPLICIT_RIGHT_WAITING ->
          getClosestReefSide().rightPipe.getPose(ReefPipeLevel.RAISING, currentPose);
      case LEFT_PIPE -> getClosestReefSide().leftPipe.getPose(currentReefPipeLevel, currentPose);
      case RIGHT_PIPE -> getClosestReefSide().rightPipe.getPose(currentReefPipeLevel, currentPose);
      case BEST_PIPE_CENTER -> getCenterPipePoseFromRobotDistance(bestPipe);
      case BEST_PIPE_WAITING -> bestPipe.getPose(ReefPipeLevel.RAISING, currentPose);
      case BEST_PIPE -> bestPipe.getPose(currentReefPipeLevel, currentPose);
      case PIPE_BACKUP -> getClosestReefPipe().getPose(ReefPipeLevel.BACK_AWAY, currentPose);
      case ALGAE_CENTER -> getCenterSidePoseFromRobotDistance(bestAlgaeSide);
      case ALGAE_WAITING -> bestAlgaeSide.getPose(ReefSideOffset.SAFE, currentPose);
      case ALGAE_INTAKE -> bestAlgaeSide.getPose(ReefSideOffset.ALGAE_INTAKING, currentPose);
      case ALGAE_BACKUP -> closestReefSide.getPose(ReefSideOffset.SAFE, currentPose);
    };
  }

  public boolean useAngleBisector() {
    return useAngleBisector;
  }

  /**
   * Sets an override target pose for auto period. If set to Pose2d.kZero, the normal logic will be
   * used.
   *
   * @param target The target pose to use during auto.
   */
  public void setAutoTargetPoseOverride(Pose2d target) {
    autoTargetPoseOverride = target;
  }

  /**
   * Determines if the driver is commanding left or right pipe selection based on controller
   * joystick input.
   *
   * @param closestSide The closest reef side to the robot.
   * @return The desired AutoAlignState based on controller input.
   */
  private AutoAlignState getWantedPipeSideState(ReefSide closestSide) {
    if (!DriverStation.isTeleop() || swerve.getState() != SwerveState.DRIVE_TO_POSE) {
      return getState();
    }

    if (driverJoystickReachedCenter
        && (Math.hypot(rawControllerXValue, rawControllerYValue) > 0.3)) {
      var inputVector = new Translation2d(rawControllerXValue, -rawControllerYValue);
      var viewOffset = 0;
      if (FmsUtil.isRedAlliance()) {
        viewOffset = 180;
      }

      var sideAngle = closestSide.getPose(currentPose);

      var rotatedVector =
          inputVector.rotateBy(
              Rotation2d.fromDegrees((viewOffset - sideAngle.getRotation().getDegrees())));
      var isLeft = rotatedVector.getX() < 0;
      if (isLeft) {
        return AutoAlignState.LEFT_PIPE;
      } else {
        return AutoAlignState.RIGHT_PIPE;
      }
    }
    return getState();
  }

  /**
   * Calculates a target pose that is centered with the specified pipe, based on the forward
   * distance of the current pose.
   *
   * @param pipe The reef pipe to center on.
   * @return The target pose centered with the specified pipe.
   */
  private Pose2d getCenterPipePoseFromRobotDistance(ReefPipe pipe) {
    var pipePose = pipe.getPose(ReefPipeLevel.RAISING, currentPose);
    var robotRelativePipeTranslation =
        new Pose2d(
                currentPose.getTranslation().minus(pipePose.getTranslation()),
                pipePose.getRotation())
            .rotateBy(pipePose.getRotation().unaryMinus());

    var forwardDistanceToPipe = -robotRelativePipeTranslation.getX();
    var lookaheadDistance = Math.copySign(0.3, forwardDistanceToPipe);
    var lookaheadDistanceToPipe = forwardDistanceToPipe - lookaheadDistance;

    // When going around to a different side of the reef, we want to approach from further away
    var minDist = 0.2;
    if (!explicitSelection && ReefSide.fromPipe(bestPipe) != closestReefSide) {
      minDist = 0.5;
    }

    // Clamp the distance to make it faster to approach if we're far away
    var clampedDistance = MathUtil.clamp(lookaheadDistanceToPipe, minDist, 1.0);
    var poseTransform = new Transform2d(-clampedDistance, 0.0, Rotation2d.fromDegrees(0));
    return pipePose.plus(poseTransform);
  }

  /**
   * Calculates a target pose that is centered with the specified side, based on the forward
   * distance of the current pose.
   *
   * @param side The reef side to center on.
   * @return The target pose centered with the specified pipe.
   */
  private Pose2d getCenterSidePoseFromRobotDistance(ReefSide side) {
    var sidePose = side.getPose(ReefSideOffset.SAFE, currentPose);
    var robotRelativeSideTranslation =
        new Pose2d(
                currentPose.getTranslation().minus(sidePose.getTranslation()),
                sidePose.getRotation())
            .rotateBy(sidePose.getRotation().unaryMinus());

    var forwardDistanceToSide = robotRelativeSideTranslation.getY();

    var lookaheadDistance = Math.copySign(0.3, forwardDistanceToSide);
    var lookaheadDistanceToSide = forwardDistanceToSide - lookaheadDistance;

    // Clamp the distance to make it faster to approach if we're far away
    var clampedDistance = MathUtil.clamp(lookaheadDistanceToSide, 0.2, 1.0);
    var poseTransform = new Transform2d(0, clampedDistance, Rotation2d.fromDegrees(0));
    return sidePose.plus(poseTransform);
  }

  /**
   * Checks if the robot's current pose is aligned with the target pose within thresholds.
   *
   * <p>In teleop, alignment is only checked when in LEFT_PIPE, RIGHT_PIPE, or BEST_PIPE states to
   * ensure no accidental placing.
   *
   * @return true if the robot's pose is aligned with the target pose
   */
  private boolean isRobotPoseAlignedWithTargetPose() {
    if (DriverStation.isTeleop()
        && (getState() != AutoAlignState.LEFT_PIPE && getState() != AutoAlignState.RIGHT_PIPE)
        && getState() != AutoAlignState.BEST_PIPE) {
      return false;
    }

    var correctTargetPose =
        DriverStation.isTeleop()
            ? currentTargetPose
            : autoTargetPoseOverride.equals(Pose2d.kZero)
                ? currentTargetPose
                : autoTargetPoseOverride;

    var translationGood =
        (currentPose.getTranslation().getDistance(correctTargetPose.getTranslation())
            <= TRANSLATION_GOOD_THRESHOLD.get());
    var rotationGood =
        MathUtil.isNear(
            correctTargetPose.getRotation().getDegrees(),
            currentPose.getRotation().getDegrees(),
            ROTATION_GOOD_THRESHOLD.get(),
            -180.0,
            180.0);

    return translationGood && rotationGood;
  }

  /** Returns true if the robot's rotation is within 10 degrees of the target pose's rotation. */
  public boolean isNearRotationGoal() {
    return MathUtil.isNear(
        currentTargetPose.getRotation().getDegrees(),
        currentPose.getRotation().getDegrees(),
        10.0,
        -180.0,
        180.0);
  }

  /**
   * Checks if algae has been removed from the specified reef side.
   *
   * @param side The reef side to check.
   * @return true if algae has been removed from the specified side
   */
  public boolean isAlgaeRemoved(ReefSide side) {
    return reefState.isAlgaeRemoved(side);
  }

  /**
   * Checks if algae has been removed from the closest reef side.
   *
   * @return true if algae has been removed from the closest reef side
   */
  public boolean isAlgaeRemoved() {
    return isAlgaeRemoved(closestReefSide);
  }

  /** Clears the reef state, marking all algae as present and all pipes as unscored. */
  public void clearReefState() {
    reefState.clear();
  }

  /** Marks algae as removed from the closest reef side. */
  public void markAlgaeRemoved() {
    reefState.markAlgaeRemoved(closestReefSide);
  }

  /** Marks the closest pipe as having been scored on at the current pipe level. */
  public void markPipeScored() {
    reefState.markCoralScored(getClosestReefPipe(), currentReefPipeLevel);
  }

  /** Finds the closest reef side to the robot's current position. */
  public ReefSide getClosestReefSide() {
    return getClosestReefSide(currentPose);
  }

  public static ReefSide getClosestReefSide(Pose2d currentPose) {
    return Collections.min(
        ALL_REEF_SIDES,
        comparingDouble(
            side ->
                currentPose
                    .getTranslation()
                    .getDistance(side.getPose(ReefSideOffset.SAFE, currentPose).getTranslation())));
  }

  /** Finds the best pipe to score on based on alignment cost and reef state. */
  public ReefPipe getBestPipeForScoring() {
    if (currentReefPipeLevel == ReefPipeLevel.L1) {
      // TODO: Update for L1 auto align
      return getClosestReefPipe();
    }
    return Collections.min(
        ALL_REEF_PIPES, alignmentCostUtil.getReefPipeComparator(currentReefPipeLevel));
  }

  /** Finds the closest reef pipe to the robot's current position and scoring side. */
  public ReefPipe getClosestReefPipe() {
    return Collections.min(
        ALL_REEF_PIPES,
        comparingDouble(
            pipe ->
                currentPose
                    .getTranslation()
                    .getDistance(
                        pipe.getPose(ReefPipeLevel.BACK_AWAY, currentPose).getTranslation())));
  }

  /** Returns true once the best pipe to align to is calculated. */
  public boolean isReadyToAlign() {
    return explicitSelection
        || bestPipeSelected
        || (getState() != AutoAlignState.BEST_PIPE_CENTER
            && getState() != AutoAlignState.BEST_PIPE_WAITING
            && getState() != AutoAlignState.BEST_PIPE);
  }

  public Pose2d getCurrentTargetPose() {
    return currentTargetPose;
  }

  /** Finds the best side to intake algae from based on algae state in reef */
  public ReefSide getBestAlgaeSide() {
    return Collections.min(ALL_REEF_SIDES, alignmentCostUtil.getAlgaeComparator());
  }

  /** Switches into algae center state */
  public void approachAlgaeRequest() {
    setStateFromRequest(AutoAlignState.ALGAE_CENTER);
  }

  /** Switches into algae intaking state to drive in to intake */
  public void intakeAlgaeRequest() {
    setStateFromRequest(AutoAlignState.ALGAE_INTAKE);
  }

  /** Switches into algae backup state for after intaking algae */
  public void backAwayFromAlgaeRequest() {
    setStateFromRequest(AutoAlignState.ALGAE_BACKUP);
  }

  /** Switches into correct approach state based on explicitSelection value */
  public void approachPipeRequest() {
    driverJoystickReachedCenter = false;
    if (explicitSelection) {
      setStateFromRequest(AutoAlignState.EXPLICIT_SAFE_WAITING);
    } else {
      bestPipeSelected = false;
      setStateFromRequest(AutoAlignState.BEST_PIPE_CENTER);
    }
  }

  /** Switches into correct pipe side state based on current approach state */
  public void lineupPipeRequest() {
    switch (getState()) {
      case EXPLICIT_LEFT_CENTER, EXPLICIT_LEFT_WAITING -> {
        setStateFromRequest(AutoAlignState.LEFT_PIPE);
      }
      case EXPLICIT_RIGHT_CENTER, EXPLICIT_RIGHT_WAITING -> {
        setStateFromRequest(AutoAlignState.RIGHT_PIPE);
      }
      case EXPLICIT_SAFE_WAITING -> {
        setStateFromRequest(AutoAlignState.EXPLICIT_SAFE);
      }
      case BEST_PIPE_CENTER, BEST_PIPE_WAITING -> {
        setStateFromRequest(AutoAlignState.BEST_PIPE);
      }
      default -> {}
    }
  }

  /** Switches into pipe backup state for after placing coral */
  public void backAwayFromPipeRequest() {
    bestPipeSelected = false;
    setStateFromRequest(AutoAlignState.PIPE_BACKUP);
  }

  @Override
  public void whileInState(AutoAlignState currentState) {
    DogLog.log("AutoAlign/IsAligned", isAligned);
    DogLog.log("AutoAlign/IsAlignedDebounced", isAlignedDebounced);
  }

  /** Sets the current scoring level and side for alignment calculations. */
  public void setScoringLevel(ReefPipeLevel level) {
    currentReefPipeLevel = level;
  }

  /** Returns true if the robot is aligned with the target pose, debounced over 0.1 seconds. */
  public boolean isAligned() {
    return isAlignedDebounced;
  }

  public TagAlignState getTagAlignState() {
    if (vision.isCameraOffline()) {
      return TagAlignState.ALL_CAMERAS_DEAD;
    }

    if (vision.getTagResult().isPresent()) {
      if (isAligned) {
        return TagAlignState.HAS_TAGS_IN_POSITION;
      }

      return TagAlignState.HAS_TAGS_WRONG_POSITION;
    }

    if (isAligned) {
      return TagAlignState.NO_TAGS_IN_POSITION;
    }
    return TagAlignState.NO_TAGS_WRONG_POSITION;
  }
}
