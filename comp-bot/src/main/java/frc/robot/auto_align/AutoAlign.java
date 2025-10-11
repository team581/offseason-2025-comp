package frc.robot.auto_align;

import com.google.common.collect.ImmutableList;
import com.team581.auto_align.TagAlignState;
import com.team581.math.MathHelpers;
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto_align.tag_align.AlignmentCostUtil;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.collision_avoidance.ObstructionKind;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionSubsystem;
import java.util.Comparator;

public class AutoAlign extends StateMachine<AutoAlignState> {

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

  private static final DoubleSubscriber OBSTRUCTION_DISTANCE =
      DogLog.tunable("AutoAlign/ObstructionDistance", 0.75);

  /**
   * Determines which side of the robot to score algae in net based on the robot's position on the
   * field
   */
  public static RobotScoringSide getNetScoringSideFromRobotPose(Pose2d robotPose) {
    double robotX = robotPose.getX();
    double theta = MathHelpers.angleModulus(robotPose.getRotation().getDegrees());

    // entire field length is 17.55m
    double halfFieldLength = 17.55 / 2.0;

    // Robot is on blue side
    if (robotX < halfFieldLength) {
      if (theta > 0.0) {
        return RobotScoringSide.RIGHT;
      }
      return RobotScoringSide.LEFT;
    }

    // Robot is on red side
    if (theta > 0.0) {
      return RobotScoringSide.LEFT;
    }
    return RobotScoringSide.RIGHT;
  }

  private static Translation2d getAllianceCenterOfReef(boolean isRedAliance) {
    return isRedAliance ? CENTER_OF_REEF_RED : CENTER_OF_REEF_BLUE;
  }

  public static Translation2d getAllianceCenterOfReef() {
    return FmsUtil.isRedAlliance() ? CENTER_OF_REEF_RED : CENTER_OF_REEF_BLUE;
  }

  public static Translation2d getAllianceCenterOfReef(Pose2d robotPose) {
    return robotPose.getX() > 17.55 / 2 ? CENTER_OF_REEF_RED : CENTER_OF_REEF_BLUE;
  }

  /**
   * Determines which side of the robot to score coral on based on the robot's position on the field
   * and which limelights are online
   */
  public static RobotScoringSide getScoringSideFromRobotPose(
      Pose2d robotPose, boolean leftLimelightsOnline, boolean rightLimelightOnline) {
    if (!leftLimelightsOnline) {
      return RobotScoringSide.RIGHT;
    }
    if (!rightLimelightOnline) {
      return RobotScoringSide.LEFT;
    }
    var centerOfReef = getAllianceCenterOfReef(robotPose.getX() > (17.5 / 2));
    var angleToAim =
        MathUtil.angleModulus(
            Math.atan2(
                centerOfReef.getY() - robotPose.getY(), centerOfReef.getX() - robotPose.getX()));
    var errorRight =
        Math.abs(
            MathUtil.angleModulus(
                angleToAim - (robotPose.getRotation().getRadians() - (Math.PI / 2.0))));
    var errorLeft =
        Math.abs(
            MathUtil.angleModulus(
                angleToAim - (robotPose.getRotation().getRadians() + (Math.PI / 2.0))));

    if (errorRight < errorLeft) {
      return RobotScoringSide.RIGHT;
    }
    return RobotScoringSide.LEFT;
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
  private final Debouncer isAlignedDebouncer = new Debouncer(0.1, DebounceType.kRising);
  private final ReefState reefState = new ReefState();

  private boolean isAligned = false;
  private boolean isAlignedDebounced = false;
  private double rawControllerXValue = 0.0;
  private double rawControllerYValue = 0.0;
  private ReefSide bestAlgaeSide = ReefSide.SIDE_AB;
  private ReefSide closestReefSide = ReefSide.SIDE_AB;
  private RobotScoringSide currentScoringSide = RobotScoringSide.RIGHT;
  private ReefSideOffset currentAlgaeIntakingReefSideOffset = ReefSideOffset.BASE;
  private ReefPipeLevel currentReefPipeLevel = ReefPipeLevel.L1;
  private Pose2d currentPose = Pose2d.kZero;
  private Pose2d currentTargetPose = Pose2d.kZero;
  private Pose2d autoTargetPoseOverride = new Pose2d();

  public AutoAlign(
      VisionSubsystem vision, LocalizationSubsystem localization, SwerveSubsystem swerve) {
    super(SubsystemPriority.AUTO_ALIGN, AutoAlignState.SAFE_WAITING);
    alignmentCostUtil = new AlignmentCostUtil(localization, swerve, reefState, currentScoringSide);

    this.vision = vision;
    this.localization = localization;
    this.swerve = swerve;
  }

  @Override
  protected AutoAlignState getNextState(AutoAlignState currentState) {
    return switch (getState()) {
      case SAFE_WAITING -> {
        var wantedState = getWantedPipeSideState(closestReefSide);
        yield wantedState.equals(AutoAlignState.LEFT_PIPE)
            ? AutoAlignState.SAFE_WAITING_LEFT_CENTER
            : wantedState.equals(AutoAlignState.RIGHT_PIPE)
                ? AutoAlignState.SAFE_WAITING_RIGHT_CENTER
                : currentState;
      }
      case SAFE_PREPARE -> {
        var wantedState = getWantedPipeSideState(closestReefSide);
        yield wantedState.equals(AutoAlignState.LEFT_PIPE)
            ? AutoAlignState.LEFT_PIPE
            : wantedState.equals(AutoAlignState.RIGHT_PIPE)
                ? AutoAlignState.RIGHT_PIPE
                : currentState;
      }
      case SAFE_WAITING_LEFT_CENTER -> {
        if (currentPose.getTranslation().getDistance(currentTargetPose.getTranslation())
                < Units.inchesToMeters(8.0)
            && MathUtil.isNear(
                currentTargetPose.getRotation().getDegrees(),
                currentPose.getRotation().getDegrees(),
                20.0)) {
          yield AutoAlignState.SAFE_WAITING_LEFT_RAISING;
        } else if (getWantedPipeSideState(closestReefSide) == AutoAlignState.RIGHT_PIPE) {
          yield AutoAlignState.SAFE_WAITING_RIGHT_CENTER;
        }
        yield currentState;
      }
      case SAFE_WAITING_RIGHT_CENTER -> {
        if (currentPose.getTranslation().getDistance(currentTargetPose.getTranslation())
                < Units.inchesToMeters(8.0)
            && MathUtil.isNear(
                currentTargetPose.getRotation().getDegrees(),
                currentPose.getRotation().getDegrees(),
                20.0)) {
          yield AutoAlignState.SAFE_WAITING_RIGHT_RAISING;
        } else if (getWantedPipeSideState(closestReefSide) == AutoAlignState.LEFT_PIPE) {
          yield AutoAlignState.SAFE_WAITING_LEFT_CENTER;
        }
        yield currentState;
      }
      default -> currentState;
    };
  }

  @Override
  protected void collectInputs() {
    currentPose = localization.getPose();
    bestAlgaeSide = getBestAlgaeSide();
    closestReefSide = getClosestReefSide();
    currentTargetPose = findTargetPose();
    isAligned = isRobotPoseAlignedWithTargetPose();
    isAlignedDebounced = isAlignedDebouncer.calculate(isAligned);

    var controllerValues = swerve.getControllerValues();
    rawControllerXValue = controllerValues.getX();
    rawControllerYValue = controllerValues.getY();
  }

  private Pose2d findTargetPose() {
    return switch (getState()) {
      case SAFE_PREPARE, SAFE_WAITING ->
          getClosestReefSide().getPose(ReefSideOffset.SAFE, currentScoringSide, currentPose);
      case SAFE_WAITING_LEFT_CENTER ->
          getClosestReefSide().leftPipe.getPose(ReefPipeLevel.CENTER, currentScoringSide);
      case SAFE_WAITING_RIGHT_CENTER ->
          getClosestReefSide().rightPipe.getPose(ReefPipeLevel.CENTER, currentScoringSide);
      case SAFE_WAITING_LEFT_RAISING ->
          getClosestReefSide().leftPipe.getPose(ReefPipeLevel.RAISING, currentScoringSide);
      case SAFE_WAITING_RIGHT_RAISING ->
          getClosestReefSide().rightPipe.getPose(ReefPipeLevel.RAISING, currentScoringSide);
      case LEFT_PIPE ->
          getClosestReefSide().leftPipe.getPose(currentReefPipeLevel, currentScoringSide);
      case RIGHT_PIPE ->
          getClosestReefSide().rightPipe.getPose(currentReefPipeLevel, currentScoringSide);
      case ALGAE ->
          bestAlgaeSide.getPose(
              currentAlgaeIntakingReefSideOffset, currentScoringSide, currentPose);
    };
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
    if (!DriverStation.isTeleop()) {
      return getState();
    }

    if ((Math.hypot(rawControllerXValue, rawControllerYValue) > 0.3)) {
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

  private boolean isRobotPoseAlignedWithTargetPose() {
    if (DriverStation.isTeleop()
        && (!getState().equals(AutoAlignState.LEFT_PIPE)
            && !getState().equals(AutoAlignState.RIGHT_PIPE))) {
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
    var rotationGood =
        MathUtil.isNear(
            currentTargetPose.getRotation().getDegrees(),
            currentPose.getRotation().getDegrees(),
            10.0,
            -180.0,
            180.0);
    return rotationGood;
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

  public boolean isAlgaeRemoved() {
    return isAlgaeRemoved(closestReefSide);
  }

  public void clearReefState() {
    reefState.clear();
  }

  public void markAlgaeRemoved() {
    reefState.markAlgaeRemoved(closestReefSide);
  }

  /**
   * Marks the specified pipe as having been scored on at the current pipe level.
   *
   * @param pipe The reef pipe that has been scored on.
   */
  public void markScored(ReefPipe pipe) {
    reefState.markCoralScored(pipe, currentReefPipeLevel);
  }

  public ReefSide getClosestReefSide() {
    return ALL_REEF_SIDES.stream()
        .min(
            Comparator.comparingDouble(
                side ->
                    currentPose
                        .getTranslation()
                        .getDistance(
                            side.getPose(ReefSideOffset.SAFE, currentScoringSide, currentPose)
                                .getTranslation())))
        .orElseThrow();
  }

  public Pose2d getCurrentTargetPose() {
    return currentTargetPose;
  }

  /** Finds the best side to intake algae from based on algae state in reef */
  public ReefSide getBestAlgaeSide() {
    return ALL_REEF_SIDES.stream().min(alignmentCostUtil.getAlgaeComparator()).orElseThrow();
  }

  public void setState(AutoAlignState newState) {
    switch (getState()) {
      case SAFE_WAITING_LEFT_CENTER, SAFE_WAITING_LEFT_RAISING -> {
        if (newState == AutoAlignState.SAFE_PREPARE) {
          setStateFromRequest(AutoAlignState.LEFT_PIPE);
        }
      }
      case SAFE_WAITING_RIGHT_CENTER, SAFE_WAITING_RIGHT_RAISING -> {
        if (newState == AutoAlignState.SAFE_PREPARE) {
          setStateFromRequest(AutoAlignState.RIGHT_PIPE);
        }
      }

      default -> setStateFromRequest(newState);
    }
  }

  @Override
  public void whileInState(AutoAlignState currentState) {
    DogLog.log("AutoAlign/IsAligned", isAligned);
    DogLog.log("AutoAlign/IsAlignedDebounced", isAlignedDebounced);
  }

  public ObstructionKind getObstruction() {
    // Account for distance we'll be at once we finish forward motion
    var lookaheadPose = localization.getPose();
    var lookaheadPoseDistance =
        lookaheadPose
            .getTranslation()
            .getDistance(closestReefSide.getPose(currentPose).getTranslation());
    if (lookaheadPoseDistance < OBSTRUCTION_DISTANCE.get()) {
      return currentScoringSide == RobotScoringSide.RIGHT
          ? ObstructionKind.RIGHT_OBSTRUCTED
          : ObstructionKind.LEFT_OBSTRUCTED;
    }
    return ObstructionKind.NONE;
  }

  public void setScoringLevel(ReefPipeLevel level, RobotScoringSide side) {
    currentScoringSide = side;
    currentReefPipeLevel = level;
  }

  public void setReefAlgaeIntakingOffset(ReefSideOffset offset) {
    currentAlgaeIntakingReefSideOffset = offset;
  }

  public boolean isAligned() {
    return isAlignedDebounced;
  }

  
  public TagAlignState getTagAlignState() {
    if (!vision.isAnyLeftScoringTagLimelightOnline()
        && !vision.isAnyRightScoringTagLimelightOnline()) {
      return TagAlignState.ALL_CAMERAS_DEAD;
    }

    if (vision.getLeftBackTagResult().isPresent()
        || vision.getLeftFrontTagResult().isPresent()
        || vision.getRightTagResult().isPresent()
        || vision.getGamePieceTagResult().isPresent()) {
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
