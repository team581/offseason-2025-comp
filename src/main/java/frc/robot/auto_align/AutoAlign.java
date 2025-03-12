package frc.robot.auto_align;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.auto_align.tag_align.TagAlign;
import frc.robot.autos.constraints.AutoConstraintCalculator;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SnapUtil;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.CameraHealth;
import frc.robot.vision.limelight.Limelight;

public class AutoAlign extends StateMachine<AutoAlignState> {
  private static final double REEF_FINAL_SPEEDS_DISTANCE_THRESHOLD = 1.5;
  private static final double LOWEST_TELEOP_SPEED_SCALAR = 0.5;
  private static final double MAX_CONSTRAINT = 1.5;

  public static boolean shouldNetScoreForwards(Pose2d robotPose) {
    double robotX = robotPose.getX();
    double theta = robotPose.getRotation().getDegrees();

    // entire field length is 17.55m
    double halfFieldLength = 17.55 / 2.0;

    // Robot is on blue side
    if (robotX < halfFieldLength) {
      return Math.abs(theta) < 90;
    }

    // Robot is on red side
    return Math.abs(theta) > 90;
  }

  public static boolean shouldIntakeStationFront(Pose2d robotPose) {
    double theta = robotPose.getRotation().getDegrees();
    var coralStationBackwardAngle = SnapUtil.getCoralStationAngle(robotPose);

    return !MathUtil.isNear(coralStationBackwardAngle, theta, 90, -180, 180);
  }

  public static boolean isStationIntakeProcessorSide(Pose2d robotPose) {
    if (robotPose.getY() > 4.025) {
      if (FmsSubsystem.isRedAlliance()) {
        // Coral station red, processor side
        return true;
      }

      // Coral station blue, non processor side
      return false;
    } else {
      if (FmsSubsystem.isRedAlliance()) {
        // Coral station red, non processor side
        return false;
      }
      // Coral station blue, processor side
      return true;
    }
  }

  public static boolean isCloseToReefSide(
      Pose2d robotPose, Pose2d nearestReefSide, double thresholdMeters) {
    return robotPose.getTranslation().getDistance(nearestReefSide.getTranslation())
        < thresholdMeters;
  }

  private static final double LINEAR_VELOCITY_TO_REEF_SIDE_DISTANCE_KS = 1.5;
  private static final double LINEAR_VELOCITY_TO_REEF_SIDE_DISTANCE_KP = 0.625;

  public static boolean isCloseToReefSide(
      Pose2d robotPose, Pose2d nearestReefSide, ChassisSpeeds robotSpeeds) {
    var linearVelocity = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
    DogLog.log("Swerve/LinearVelocity", linearVelocity);
    return isCloseToReefSide(
        robotPose,
        nearestReefSide,
        LINEAR_VELOCITY_TO_REEF_SIDE_DISTANCE_KS
            + LINEAR_VELOCITY_TO_REEF_SIDE_DISTANCE_KP * linearVelocity);
  }

  private final Debouncer isAlignedDebouncer = new Debouncer(0.25, DebounceType.kRising);
  private final Limelight frontLeftLimelight;
  private final Limelight frontRightLimelight;
  private final LocalizationSubsystem localization;
  private final TagAlign tagAlign;
  private final SwerveSubsystem swerve;

  private ChassisSpeeds teleopSpeeds = new ChassisSpeeds();
  private ChassisSpeeds tagAlignSpeeds = new ChassisSpeeds();
  private ChassisSpeeds algaeAlignSpeeds = new ChassisSpeeds();
  private boolean isAligned = false;
  private boolean isAlignedDebounced = false;
  private ReefPipe bestReefPipe = ReefPipe.PIPE_A;
  private Pose2d usedScoringPose = Pose2d.kZero;

  public AutoAlign(
      Limelight frontLeftLimelight,
      Limelight frontRightLimelight,
      LocalizationSubsystem localization,
      SwerveSubsystem swerve) {
    super(SubsystemPriority.AUTO_ALIGN, AutoAlignState.DEFAULT_STATE);

    this.tagAlign = new TagAlign(swerve, localization);

    this.frontLeftLimelight = frontLeftLimelight;
    this.frontRightLimelight = frontRightLimelight;
    this.localization = localization;
    this.swerve = swerve;
  }

  public void setAutoReefPipeOverride(ReefPipe override) {
    tagAlign.setPipeOveride(override);
  }

  public ReefSide getClosestReefSide() {
    return ReefSide.fromPipe(bestReefPipe);
  }

  public void setTeleopSpeeds(ChassisSpeeds speeds) {
    teleopSpeeds = speeds;
  }

  private ChassisSpeeds constrainLinearVelocity(ChassisSpeeds speeds, double maxSpeed) {
    var options =
        new AutoConstraintOptions()
            .withMaxAngularAcceleration(0)
            .withMaxAngularVelocity(0)
            .withMaxLinearAcceleration(0)
            .withMaxLinearVelocity(maxSpeed);
    return AutoConstraintCalculator.constrainLinearVelocity(speeds, options);
  }

  public ChassisSpeeds calculateConstrainedAndWeightedSpeeds(ChassisSpeeds alignSpeeds) {
    var addedSpeeds = teleopSpeeds.plus(alignSpeeds);
    var constrainedSpeeds = constrainLinearVelocity(addedSpeeds, MAX_CONSTRAINT);

    var robotPose = localization.getPose();
    var distanceToReef = robotPose.getTranslation().getDistance(usedScoringPose.getTranslation());

    if (distanceToReef > REEF_FINAL_SPEEDS_DISTANCE_THRESHOLD) {
      return constrainedSpeeds;
    }

    var progress =
        MathUtil.clamp(
            distanceToReef / REEF_FINAL_SPEEDS_DISTANCE_THRESHOLD, LOWEST_TELEOP_SPEED_SCALAR, 1.0);
    DogLog.log("Debug/Progress", progress);
    var newTeleopSpeeds = teleopSpeeds.times(progress);
    if (progress == LOWEST_TELEOP_SPEED_SCALAR) {
      progress = 0.0;
    }
    var newAlignSpeeds = alignSpeeds.times(1.0 - progress);
    var newAddedSpeeds = newTeleopSpeeds.plus(newAlignSpeeds);
    var newConstrainedSpeeds = constrainLinearVelocity(newAddedSpeeds, MAX_CONSTRAINT);
    DogLog.log("Debug/NewConstrainedSpeeds", newConstrainedSpeeds);
    return newConstrainedSpeeds;
  }

  @Override
  protected void collectInputs() {
    bestReefPipe = tagAlign.getBestPipe();
    usedScoringPose = tagAlign.getUsedScoringPose(bestReefPipe);
    isAligned = tagAlign.isAligned(bestReefPipe);
    isAlignedDebounced = isAlignedDebouncer.calculate(isAligned);
    tagAlignSpeeds = tagAlign.getPoseAlignmentChassisSpeeds(usedScoringPose, false);
    algaeAlignSpeeds = tagAlign.getAlgaeAlignmentSpeeds(ReefSide.fromPipe(bestReefPipe).getPose());
  }

  public ChassisSpeeds getTagAlignSpeeds() {
    return tagAlignSpeeds;
  }

  public ChassisSpeeds getAlgaeAlignSpeeds() {
    return algaeAlignSpeeds;
  }

  /**
   * @deprecated Use {@link #isTagAlignedDebounced()} instead.
   */
  @Deprecated
  public boolean isTagAligned() {
    return isAligned;
  }

  public ReefPipe getBestReefPipe() {
    return bestReefPipe;
  }

  public void markPipeScored() {
    tagAlign.markScored(bestReefPipe);
  }

  public void setScoringLevel(ReefPipeLevel level) {
    tagAlign.setLevel(level);
  }

  public void clearReefState() {
    tagAlign.clearReefState();
  }

  public boolean isTagAlignedDebounced() {
    return isAlignedDebounced;
  }

  public Pose2d getUsedScoringPose() {
    return usedScoringPose;
  }

  public Pose2d getUsedScoringPose(ReefPipe pipe) {
    return tagAlign.getUsedScoringPose(pipe);
  }

  public Pose2d getUsedScoringPose(ReefPipe pipe, ReefPipeLevel level) {
    setScoringLevel(level);
    return getUsedScoringPose(pipe);
  }

  public void setDriverPoseOffset(Translation2d offset) {
    tagAlign.setDriverPoseOffset(offset);
  }

  public ReefAlignState getReefAlignState() {

    var tagResult = frontLeftLimelight.getTagResult().or(frontRightLimelight::getTagResult);

    var combinedTagHealth =
        CameraHealth.combine(
            frontLeftLimelight.getCameraHealth(), frontRightLimelight.getCameraHealth());

    if (combinedTagHealth == CameraHealth.OFFLINE) {
      return ReefAlignState.ALL_CAMERAS_DEAD;
    }

    if (tagResult.isEmpty()) {
      if (isAligned) {
        return ReefAlignState.NO_TAGS_IN_POSITION;
      }
      return ReefAlignState.NO_TAGS_WRONG_POSITION;
    }

    if (isAligned) {
      return ReefAlignState.HAS_TAGS_IN_POSITION;
    }

    return ReefAlignState.HAS_TAGS_WRONG_POSITION;
  }
}
