package frc.robot.auto_align;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.MathHelpers;

public class MagnetismUtil {
  private static final double IDEAL_MAGNITUDE = 1.0;
  private static final double ASSIST_RADIUS = 2.0;
  private static final Rotation2d MAX_ASSIST_ANGLE = Rotation2d.fromDegrees(30.0);
  private static final Rotation2d MIN_ASSIST_ANGLE = Rotation2d.kZero;

  private static Rotation2d clampAngle(Rotation2d angle) {
    return MathHelpers.angleMax(
        MathHelpers.angleMin(MathHelpers.angleAbs(angle), MAX_ASSIST_ANGLE), MIN_ASSIST_ANGLE);
  }

  public static ChassisSpeeds getMagnetizedChassisSpeeds(
      ChassisSpeeds fieldRelativeSpeeds, Pose2d robotPose, Pose2d goalPose) {
    var robotRelativeToGoal = goalPose.minus(robotPose).getTranslation();

    if (robotRelativeToGoal.getNorm() > ASSIST_RADIUS) {
      return fieldRelativeSpeeds;
    }
    var robotSpeeds = MathHelpers.chassisSpeedsToTranslation2d(fieldRelativeSpeeds).unaryMinus();
    var idealSpeeds =
        new Translation2d(
            IDEAL_MAGNITUDE,
            robotSpeeds
                .getAngle()
                .plus(clampAngle(robotRelativeToGoal.getAngle().minus(robotSpeeds.getAngle()))));

    var magnetismWeight =
        (1 - MathHelpers.nonZeroDivide(robotRelativeToGoal.getNorm(), ASSIST_RADIUS));

    var unnormalizedTransform =
        idealSpeeds.times(magnetismWeight).plus(robotSpeeds.times(1 - magnetismWeight));

    var normalizedTransform =
        new Translation2d(robotSpeeds.getNorm(), unnormalizedTransform.getAngle());

    ChassisSpeeds magnetizedSpeeds =
        new ChassisSpeeds(
            normalizedTransform.getX(),
            normalizedTransform.getY(),
            fieldRelativeSpeeds.omegaRadiansPerSecond);
    DogLog.log("Debug/RobotVector", fieldRelativeSpeeds);
    DogLog.log("Debug/MagnetismWeight", magnetismWeight);
    DogLog.log(
        "Debug/UnnormalizedVector",
        MathHelpers.translation2dToChassisSpeeds(unnormalizedTransform));
    DogLog.log("Debug/IdealVector", MathHelpers.translation2dToChassisSpeeds(idealSpeeds));
    DogLog.log("Debug/RobotPose", robotPose);
    DogLog.log("Debug/RobotPoseRelativeToGoal", robotRelativeToGoal);
    DogLog.log("Debug/GoalPose", goalPose);
    DogLog.log("Debug/MagnetizedSpeeds", magnetizedSpeeds);
    return magnetizedSpeeds;
  }

  public static ChassisSpeeds getReefMagnetizedChassisSpeeds(
      ChassisSpeeds fieldRelativeSpeeds, Pose2d robotPose) {
    Pose2d closestReefPipe =
        robotPose.nearest(AutoAlign.getClosestReefSide(robotPose).getPipes(ReefPipeLevel.L1));

    return getMagnetizedChassisSpeeds(fieldRelativeSpeeds, robotPose, closestReefPipe);
  }
}
