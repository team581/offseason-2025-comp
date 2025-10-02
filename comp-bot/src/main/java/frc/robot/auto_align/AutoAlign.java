package frc.robot.auto_align;

import com.team581.auto_align.TagAlignState;
import com.google.common.collect.ImmutableList;
import com.team581.math.MathHelpers;
import com.team581.math.PoseErrorTolerance;
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  private final PoseErrorTolerance positionTolerance = new PoseErrorTolerance(0.2, 4);

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

  public static Translation2d getAllianceCenterOfReef(boolean isRedAliance) {
    return isRedAliance ? CENTER_OF_REEF_RED : CENTER_OF_REEF_BLUE;
  }

  public static Translation2d getAllianceCenterOfReef(Pose2d robotPose) {
    return robotPose.getX() > 17.55 / 2 ? CENTER_OF_REEF_RED : CENTER_OF_REEF_BLUE;
  }

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

  private final Debouncer isAlignedDebouncer = new Debouncer(0.1, DebounceType.kRising);
  private final VisionSubsystem vision;
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final AlignmentCostUtil alignmentCostUtil;

  private Pose2d robotPose = Pose2d.kZero;
  private final ChassisSpeeds tagAlignSpeeds = new ChassisSpeeds();
  private boolean isAligned = false;
  private boolean isNearRotation = false;
  private boolean isAlignedDebounced = false;
  private RobotScoringSide robotScoringSide = RobotScoringSide.RIGHT;
  private final ReefPipe bestReefPipe = ReefPipe.PIPE_A;
  private Pose2d targetPose = Pose2d.kZero;
  private ReefSideOffset reefSideOffset = ReefSideOffset.BASE;
  private ReefSide bestAlgaeSide = ReefSide.SIDE_AB;
  private ReefSide closestSide = ReefSide.SIDE_AB;
  private ReefPipeLevel pipeLevel = ReefPipeLevel.L1;
  private AutoAlignState wantedLeftRightState = AutoAlignState.SAFE_PREPARE_SELECTION;
  private final ReefState reefState = new ReefState();
  private Pose2d autoTargetPoseOverride = new Pose2d();
  private double rawControllerXValue = 0.0;
  private double rawControllerYValue = 0.0;
  private boolean pipeSelected = false;

  public AutoAlign(
      VisionSubsystem vision, LocalizationSubsystem localization, SwerveSubsystem swerve) {
    super(SubsystemPriority.AUTO_ALIGN, AutoAlignState.SAFE_PREPARE_SELECTION);
    alignmentCostUtil = new AlignmentCostUtil(localization, swerve, reefState, robotScoringSide);

    this.vision = vision;
    this.localization = localization;
    this.swerve = swerve;
  }

  @Override
  protected AutoAlignState getNextState(AutoAlignState currentState) {
    return switch (getState()) {
      case SAFE_PREPARE_SELECTION -> {
        if (pipeSelected) {
          pipeSelected = false;
          yield wantedLeftRightState;
        }
        yield currentState;
      }
      default -> currentState;
    };
  }

  public void setAutoTargetPoseOverride(Pose2d target) {
    autoTargetPoseOverride = target;
  }

  private AutoAlignState getLeftRightState(ReefSide closestSide) {
    if (!DriverStation.isTeleop()) {
      return AutoAlignState.SAFE_WAITING;
    }

    if ((Math.hypot(rawControllerXValue, rawControllerYValue) > 0.5)) {
      var inputVector = new Translation2d(rawControllerXValue, -rawControllerYValue);
      var viewOffset = 0;
      if (FmsUtil.isRedAlliance()) {
        viewOffset = 180;
      }

      var sideAngle = closestSide.getPose(robotPose);

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
    return AutoAlignState.SAFE_WAITING;
  }

  public void setControllerValues(double controllerXValue, double controllerYValue) {
    rawControllerXValue = controllerXValue;
    rawControllerYValue = controllerYValue;
  }

  private boolean isRobotPoseAlignedWithTargetPose() {
    if (DriverStation.isTeleop()
        && (getState().equals(AutoAlignState.SAFE_PREPARE_SELECTION)
            || getState().equals(AutoAlignState.SAFE_WAITING))) {
      return false;
    }

    var correctTargetPose =
        DriverStation.isTeleop()
            ? targetPose
            : autoTargetPoseOverride.equals(Pose2d.kZero) ? targetPose : autoTargetPoseOverride;

    var translationGood =
        (robotPose.getTranslation().getDistance(correctTargetPose.getTranslation())
            <= TRANSLATION_GOOD_THRESHOLD.get());
    var rotationGood =
        MathUtil.isNear(
            correctTargetPose.getRotation().getDegrees(),
            robotPose.getRotation().getDegrees(),
            ROTATION_GOOD_THRESHOLD.get(),
            -180.0,
            180.0);

    return translationGood && rotationGood;
  }

  private boolean isNearRotationGoal() {
    var rotationGood =
        MathUtil.isNear(
            targetPose.getRotation().getDegrees(),
            robotPose.getRotation().getDegrees(),
            10.0,
            -180.0,
            180.0);
    return rotationGood;
  }

  public void markAlgaeRemoved() {
    reefState.markAlgaeRemoved(closestSide);
  }

  public boolean isAlgaeRemoved(ReefSide side) {
    return reefState.isAlgaeRemoved(side);
  }

  public boolean isAlgaeRemoved() {
    return isAlgaeRemoved(closestSide);
  }

  public void clearReefState() {
    reefState.clear();
  }

  public void markScored(ReefPipe pipe) {
    reefState.markCoralScored(pipe, pipeLevel);
  }

  public ReefSide getClosestReefSide() {
    return ALL_REEF_SIDES.stream()
        .min(
            Comparator.comparingDouble(
                side ->
                    robotPose
                        .getTranslation()
                        .getDistance(
                            side.getPose(ReefSideOffset.SAFE, robotScoringSide, robotPose)
                                .getTranslation())))
        .orElseThrow();
  }

  private Pose2d findTargetPose() {
    return switch (getState()) {
      case SAFE_PREPARE_SELECTION, SAFE_WAITING ->
          getClosestReefSide().getPose(ReefSideOffset.SAFE, robotScoringSide, robotPose);
      case LEFT_PIPE -> getClosestReefSide().leftPipe.getPose(pipeLevel, robotScoringSide);
      case RIGHT_PIPE -> getClosestReefSide().rightPipe.getPose(pipeLevel, robotScoringSide);
      case ALGAE -> bestAlgaeSide.getPose(reefSideOffset, robotScoringSide, robotPose);
    };
  }

  public Pose2d getTargetPose() {
    return targetPose;
  }

  public ReefSide getBestAlgaeSide() {
    return ALL_REEF_SIDES.stream().min(alignmentCostUtil.getAlgaeComparator()).orElseThrow();
  }

  @Override
  protected void collectInputs() {
    robotPose = localization.getPose();
    bestAlgaeSide = getBestAlgaeSide();
    closestSide = getClosestReefSide();
    targetPose = findTargetPose();
    isAligned = isRobotPoseAlignedWithTargetPose();
    isNearRotation = isNearRotationGoal();
    isAlignedDebounced = isAlignedDebouncer.calculate(isAligned);

    var controllerValues = swerve.getControllerValues();
    rawControllerXValue = controllerValues.getX();
    rawControllerYValue = controllerValues.getY();
    DogLog.log("AutoAlign/PipeSelected", pipeSelected);
    switch (getState()) {
      case SAFE_WAITING, SAFE_PREPARE_SELECTION -> {
        if (!pipeSelected) {
          var wantedSide = getLeftRightState(closestSide);
          var controllerFlicked =
              wantedSide.equals(AutoAlignState.LEFT_PIPE)
                  || wantedSide.equals(AutoAlignState.RIGHT_PIPE);
          if (controllerFlicked) {
            wantedLeftRightState = wantedSide;
            pipeSelected = true;
          } else {
            wantedLeftRightState = AutoAlignState.SAFE_PREPARE_SELECTION;
          }
        }
      }
    }
    DogLog.log("AutoAlign/Wanted", wantedLeftRightState);
  }

  public void setState(AutoAlignState newState) {
    setStateFromRequest(newState);
  }

  public boolean isNearRaisingPoint() {
    return positionTolerance.atPose(
        bestReefPipe.getPose(ReefPipeLevel.RAISING, robotScoringSide, robotPose), robotPose);
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
            .getDistance(
                bestReefPipe
                    .getPose(ReefPipeLevel.BASE, robotScoringSide, robotPose)
                    .getTranslation());
    if (lookaheadPoseDistance < OBSTRUCTION_DISTANCE.get()) {
      return robotScoringSide == RobotScoringSide.RIGHT
          ? ObstructionKind.RIGHT_OBSTRUCTED
          : ObstructionKind.LEFT_OBSTRUCTED;
    }
    return ObstructionKind.NONE;
  }

  public ChassisSpeeds getTagAlignSpeeds() {
    return tagAlignSpeeds;
  }

  public ReefPipe getBestReefPipe() {
    return bestReefPipe;
  }

  public void setScoringLevel(ReefPipeLevel level, RobotScoringSide side) {
    robotScoringSide = side;
    pipeLevel = level;
  }

  public void setReefAlgaeIntakingOffset(ReefSideOffset offset) {
    reefSideOffset = offset;
  }

  public boolean isAligned() {
    return isAlignedDebounced;
  }

  public TagAlignState getReefAlignState() {
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
