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
import frc.robot.vision.limelight.Limelight;

public class Purple {
  private final Limelight purpleCamera;
  private static final double PURPLE_SIDEWAYS_KP = 1.0;
  private static final double TAG_KP = 0.5;

  private static final double SEEN_PURPLE_TIMEOUT = 3.0;
  private double lastTimeSeen = 0.0;
  private boolean seenPurple = false;

  public Purple(Limelight purpleCamera) {
    this.purpleCamera = purpleCamera;
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

  public static ChassisSpeeds getPoseAlignmentChassisSpeeds(
      Pose2d robotPose, ReefPipeLevel level, boolean forwardOnly) {
    var scoringTranslationFieldRelative =
        AutoAlign.getClosestReefPipe(robotPose, level).getTranslation();
    if (robotPose.getTranslation().getDistance(scoringTranslationFieldRelative) > 0.8) {
      return new ChassisSpeeds();
    }
    DogLog.log(
        "PurpleAlignment/Tag/TargetPose",
        new Pose2d(scoringTranslationFieldRelative, new Rotation2d()));
    var scoringTranslationRobotRelative =
        scoringTranslationFieldRelative
            .minus(robotPose.getTranslation())
            .rotateBy(Rotation2d.fromDegrees(360 - robotPose.getRotation().getDegrees()));
    var goalTranslationUnrotated = new Translation2d();
    if (forwardOnly) {
      goalTranslationUnrotated = new Translation2d(0, scoringTranslationRobotRelative.getX());
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

  public ChassisSpeeds getPurpleAlignChassisSpeeds(double robotHeading) {
    var maybeResult = purpleCamera.getPurpleResult();
    if (maybeResult.isEmpty()) {
      return new ChassisSpeeds();
    }
    var rawAngle = purpleCamera.getPurpleResult().get().ty();
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

  public ChassisSpeeds getCombinedTagAndPurpleChassisSpeeds(
      Pose2d robotPose, ReefPipeLevel reefPipeLevel) {
    DogLog.log("PurpleAlignment/SeenPurple", seenPurple);
    DogLog.log("PurpleAlignment/LastSeenTimestamp", lastTimeSeen);
    DogLog.log("PurpleAlignment/PurpleState", getPurpleState());

    var speeds =
        switch (getPurpleState()) {
          case NO_PURPLE -> {
            if (Timer.getFPGATimestamp() - lastTimeSeen >= SEEN_PURPLE_TIMEOUT) {
              seenPurple = false;
            }
            yield getPoseAlignmentChassisSpeeds(robotPose, reefPipeLevel, seenPurple);
          }
          case VISIBLE_NOT_CENTERED -> {
            seenPurple = true;
            lastTimeSeen = Timer.getFPGATimestamp();
            yield getPoseAlignmentChassisSpeeds(robotPose, reefPipeLevel, true)
                .plus(getPurpleAlignChassisSpeeds(robotPose.getRotation().getDegrees()));
          }
          case CENTERED -> {
            seenPurple = true;
            lastTimeSeen = Timer.getFPGATimestamp();
            yield getPoseAlignmentChassisSpeeds(robotPose, reefPipeLevel, true);
          }
        };
    DogLog.log("PurpleAlignment/CombinedSpeeds/x", speeds.vxMetersPerSecond);
    DogLog.log("PurpleAlignment/CombinedSpeeds/y", speeds.vyMetersPerSecond);
    DogLog.log("PurpleAlignment/CombinedSpeeds/omega", speeds.omegaRadiansPerSecond);
    return speeds;
  }
}
