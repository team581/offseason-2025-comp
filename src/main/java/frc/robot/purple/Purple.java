package frc.robot.purple;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.vision.limelight.Limelight;

public class Purple {
  private final Limelight purpleCamera;
  private final LocalizationSubsystem localization;

  private static final double PURPLE_SIDEWAYS_KP = 1.0;
  private static final double TAG_KP = 2.5;
  private static final double BEFORE_RAISED_INITIAL_DISTANCE_OFFSET = 0.35;
  private static final double TAG_ALIGNMENT_FINISHED_DISTANCE_THRESHOLD = 0.05;
  private static final double SEEN_PURPLE_TIMEOUT = 3.0;

  private static boolean beforeRaisedOffset = false;

  private double lastTimeSeen = 0.0;
  private boolean seenPurple = false;
  private ReefPipeLevel level = ReefPipeLevel.L1;

  public Purple(Limelight purpleCamera, LocalizationSubsystem localization) {
    this.purpleCamera = purpleCamera;
    this.localization = localization;
  }

  public PurpleState getPurpleState() {
    var maybeResult = purpleCamera.getPurpleResult();
    if (maybeResult.isEmpty()) {
      return PurpleState.NO_PURPLE;
    }

    var result = maybeResult.get();

    if (MathUtil.isNear(0, result.ty(), 0.5)) {
      return PurpleState.CENTERED;
    }

    return PurpleState.VISIBLE_NOT_CENTERED;
  }

  public void setBeforeRaisedOffset(boolean offsetOn) {
    beforeRaisedOffset = offsetOn;
  }

  public void setLevel(ReefPipeLevel level) {
    this.level = level;
  }

  public boolean isTagAligned() {
    var robotPose = localization.getPose();
    var scoringPoseFieldRelative = AutoAlign.getClosestReefPipe(robotPose, level);
    return robotPose.getTranslation().getDistance(scoringPoseFieldRelative.getTranslation())
        <= TAG_ALIGNMENT_FINISHED_DISTANCE_THRESHOLD;
  }

  public Pose2d getUsedScoringPose() {
    var rawPose = AutoAlign.getClosestReefPipe(localization.getPose(), level);
    if (beforeRaisedOffset) {
      return new Pose2d(
          rawPose.getX() - BEFORE_RAISED_INITIAL_DISTANCE_OFFSET,
          rawPose.getY(),
          rawPose.getRotation());
    }
    return rawPose;
  }

  public ChassisSpeeds getPoseAlignmentChassisSpeeds(boolean forwardOnly) {
    var robotPose = localization.getPose();
    var scoringTranslationFieldRelative = getUsedScoringPose();

    var scoringTranslationRobotRelative =
        scoringTranslationFieldRelative
            .getTranslation()
            .minus(robotPose.getTranslation())
            .rotateBy(Rotation2d.fromDegrees(360 - robotPose.getRotation().getDegrees()));

    var goalTranslationUnrotated = new Translation2d();
    if (forwardOnly) {
      goalTranslationUnrotated = new Translation2d(scoringTranslationRobotRelative.getX(), 0.0);
    } else {
      goalTranslationUnrotated = scoringTranslationRobotRelative;
    }

    var goalTranslation = goalTranslationUnrotated.rotateBy(robotPose.getRotation());

    var xEffort = goalTranslation.getX() * TAG_KP;
    var yEffort = goalTranslation.getY() * TAG_KP;

    DogLog.log("PurpleAlignment/Tag/XEffort", xEffort);
    DogLog.log("PurpleAlignment/Tag/YEffort", yEffort);
    DogLog.log("PurpleAlignment/Tag/ForwardOnly", forwardOnly);

    return new ChassisSpeeds(xEffort, yEffort, 0.0);
  }

  public ChassisSpeeds getPurpleAlignChassisSpeeds() {
    var robotHeading = localization.getPose().getRotation().getDegrees();
    var maybeResult = purpleCamera.getPurpleResult();
    if (maybeResult.isEmpty()) {
      return new ChassisSpeeds();
    }
    var rawAngle = maybeResult.get().ty();
    DogLog.log("PurpleAlignment/Purple/RawAngleTY", rawAngle);
    var rawAngleRadians = Units.degreesToRadians(rawAngle);
    var rawAngleTranslation = new Translation2d(0, rawAngleRadians);
    var rotatedAngleTranslation =
        rawAngleTranslation.rotateBy(Rotation2d.fromDegrees(robotHeading));
    var xError = rotatedAngleTranslation.getX();
    var yError = rotatedAngleTranslation.getY();

    var xEffort = xError * PURPLE_SIDEWAYS_KP;
    var yEffort = yError * PURPLE_SIDEWAYS_KP;
    DogLog.log("PurpleAlignment/Purple/XEffort", xEffort);
    DogLog.log("PurpleAlignment/Purple/YEffort", yEffort);
    return new ChassisSpeeds(xEffort, yEffort, 0.0);
  }

  public ChassisSpeeds getCombinedTagAndPurpleChassisSpeeds() {
    var robotPose = localization.getPose();
    DogLog.log("PurpleAlignment/SeenPurple", seenPurple);
    DogLog.log("PurpleAlignment/LastSeenTimestamp", lastTimeSeen);
    DogLog.log("PurpleAlignment/PurpleState", getPurpleState());
    if (Timer.getFPGATimestamp() - lastTimeSeen >= SEEN_PURPLE_TIMEOUT) {
      seenPurple = false;
    }
    if (!seenPurple && !isTagAligned()) {
      DogLog.log("PurpleAlignment/TagAligned", false);
      return getPoseAlignmentChassisSpeeds(seenPurple);
    }
    DogLog.log("PurpleAlignment/TagAligned", true);

    var speeds =
        switch (getPurpleState()) {
          case NO_PURPLE -> {
            yield getPoseAlignmentChassisSpeeds(seenPurple);
          }
          case VISIBLE_NOT_CENTERED -> {
            seenPurple = true;
            lastTimeSeen = Timer.getFPGATimestamp();
            yield getPoseAlignmentChassisSpeeds(seenPurple).plus(getPurpleAlignChassisSpeeds());
          }
          case CENTERED -> {
            seenPurple = true;
            lastTimeSeen = Timer.getFPGATimestamp();
            yield getPoseAlignmentChassisSpeeds(seenPurple);
          }
        };
    DogLog.log("PurpleAlignment/CombinedSpeeds/x", speeds.vxMetersPerSecond);
    DogLog.log("PurpleAlignment/CombinedSpeeds/y", speeds.vyMetersPerSecond);
    DogLog.log("PurpleAlignment/CombinedSpeeds/omega", speeds.omegaRadiansPerSecond);
    return speeds;
  }
}
