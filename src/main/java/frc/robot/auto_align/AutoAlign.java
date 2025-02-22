package frc.robot.auto_align;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.auto_align.purple_align.PurpleAlign;
import frc.robot.auto_align.purple_align.PurpleAlignState;
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
  private final PurpleAlign purple;
  private final Limelight purpleLimelight;
  private final Limelight frontLimelight;
  private final Limelight baseLimelight;
  private final LocalizationSubsystem localization;
  private final TagAlign tagAlign;
  private final SwerveSubsystem swerve;

  private ChassisSpeeds teleopSpeeds = new ChassisSpeeds();
  private ChassisSpeeds tagAlignSpeeds = new ChassisSpeeds();
  private ChassisSpeeds tagAlignSpeedsForwardForPurple = new ChassisSpeeds();
  private boolean seenPurple = false;
  private boolean isAligned = false;
  private boolean isAlignedDebounced = false;
  private ReefPipe bestReefPipe = ReefPipe.PIPE_A;
  private Pose2d usedScoringPose = Pose2d.kZero;

  public AutoAlign(
      Limelight purpleLimelight,
      Limelight frontLimelight,
      Limelight baseLimelight,
      LocalizationSubsystem localization,
      SwerveSubsystem swerve) {
    super(SubsystemPriority.AUTO_ALIGN, AutoAlignState.DEFAULT_STATE);

    this.tagAlign = new TagAlign(swerve, localization);
    this.purple = new PurpleAlign(purpleLimelight);

    this.purpleLimelight = purpleLimelight;
    this.frontLimelight = frontLimelight;
    this.baseLimelight = baseLimelight;
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
            .withCollisionAvoidance(false)
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

  public ChassisSpeeds getCombinedTagAndPurpleChassisSpeeds() {
    var purpleState = purple.getPurpleState();
    DogLog.log("PurpleAlignment/SeenPurple", seenPurple);
    DogLog.log("PurpleAlignment/PurpleState", purpleState);
    if (!seenPurple && !isAligned) {
      DogLog.log("PurpleAlignment/TagAligned", false);
      return tagAlignSpeedsForwardForPurple;
    }
    DogLog.log("PurpleAlignment/TagAligned", true);

    var speeds =
        switch (purpleState) {
          case NO_PURPLE, CENTERED -> tagAlignSpeedsForwardForPurple;
          case VISIBLE_NOT_CENTERED ->
              tagAlignSpeedsForwardForPurple.plus(
                  purple.getPurpleAlignChassisSpeeds(
                      localization.getPose().getRotation().getDegrees()));
        };
    DogLog.log("PurpleAlignment/CombinedSpeeds", speeds);
    return speeds;
  }

  @Override
  protected void collectInputs() {
    seenPurple = purple.seenPurple();
    bestReefPipe = tagAlign.getBestPipe();
    usedScoringPose = tagAlign.getUsedScoringPose(bestReefPipe);
    isAligned = tagAlign.isAligned(bestReefPipe);
    isAlignedDebounced = isAlignedDebouncer.calculate(isAligned);
    tagAlignSpeeds = tagAlign.getPoseAlignmentChassisSpeeds(usedScoringPose, false);
    tagAlignSpeedsForwardForPurple =
        tagAlign.getPoseAlignmentChassisSpeeds(usedScoringPose, seenPurple);
  }

  public ChassisSpeeds getTagAlignSpeeds() {
    return tagAlignSpeeds;
  }

  public boolean isTagAligned() {
    return isAligned;
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

  public void setDriverPoseOffset(Translation2d offset) {
    tagAlign.setDriverPoseOffset(offset);
  }

  public ReefAlignState getReefAlignState() {

    var tagResult = frontLimelight.getTagResult().or(baseLimelight::getTagResult);
    var purpleState = purple.getPurpleState();
    var purpleHealth = purpleLimelight.getCameraHealth();
    var combinedTagHealth =
        CameraHealth.combine(frontLimelight.getCameraHealth(), baseLimelight.getCameraHealth());

    if (combinedTagHealth == CameraHealth.OFFLINE) {
      if (purpleHealth == CameraHealth.OFFLINE) {
        return ReefAlignState.ALL_CAMERAS_DEAD;
      }
      return ReefAlignState.TAG_CAMERAS_DEAD;
    }

    if (purpleHealth == CameraHealth.OFFLINE) {
      return ReefAlignState.PURPLE_CAMERA_DEAD;
    }

    if (purple.canUsePurple()) {
      // We can't trust purple unless we are near the reef, to avoid false positives
      if (purpleState == PurpleAlignState.CENTERED) {
        return ReefAlignState.HAS_PURPLE_ALIGNED;
      }
      if (purpleState == PurpleAlignState.VISIBLE_NOT_CENTERED) {
        return ReefAlignState.HAS_PURPLE_NOT_ALIGNED;
      }
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
