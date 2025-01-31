package frc.robot.auto_align;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.MathHelpers;

public class MagnetismUtil {
  private static final double IDEAL_MAGNITUDE = 1.0;
  private static final double ASSIST_RADIUS = 2.0;

  public static ChassisSpeeds getMagnetizedChassisSpeeds(
      ChassisSpeeds fieldRelativeSpeeds, Pose2d robotPose, Pose2d goalPose) {
    var robotRelativeToGoal = goalPose.minus(robotPose).getTranslation();

    if (robotRelativeToGoal.getNorm() > ASSIST_RADIUS) {
      return fieldRelativeSpeeds;
    }
    var robotSpeeds = MathHelpers.chassisSpeedsToTranslation2d(fieldRelativeSpeeds).unaryMinus();
    var idealSpeeds = new Translation2d(IDEAL_MAGNITUDE, robotRelativeToGoal.getAngle());

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
    DogLog.log("Debug/RobotVectorAngle", robotSpeeds.plus(new Translation2d(0.001, 0)).getAngle());
    DogLog.log("Debug/IdealVectorAngle", idealSpeeds.getAngle());
    DogLog.log("Debug/IdealSpeeds", MathHelpers.translation2dToChassisSpeeds(idealSpeeds));
    DogLog.log("Debug/RobotSpeeds", MathHelpers.translation2dToChassisSpeeds(robotSpeeds));
    DogLog.log("Debug/RobotPose", robotPose);
    DogLog.log("Debug/GoalPose", goalPose);
    DogLog.log("Debug/OutputVectorAngle", normalizedTransform.getAngle());
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
