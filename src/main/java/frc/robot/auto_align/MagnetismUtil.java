package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.fms.FmsSubsystem;

public class MagnetismUtil {
  private static final double kP = 2.0;
  private static final double MAX_ASSIST = 2.0;
  private static final double MIN_ASSIST = 1.0;
  private static final double ASSIST_RADIUS = 1.5;

  private static double clamp(double val) {
    return Math.min(Math.max(val, MIN_ASSIST), MAX_ASSIST);
  }

  private static Pose2d[] getPipePoses() {
    ReefPipe[] values = ReefPipe.values();
    Pose2d[] reefPipes = new Pose2d[12];
    int i = 0;
    for (ReefPipe pipe : values) {
      reefPipes[i] = FmsSubsystem.isRedAlliance() ? pipe.redPose : pipe.bluePose;
      i++;
    }
    return reefPipes;
  }

  public static ChassisSpeeds getMagnetizedChassisSpeeds(
      ChassisSpeeds fieldRelativeRobotSpeeds, Pose2d robotPose) {
    double accumulateAngle = 0.0;
    double accumulateMagnitude = 0.0;
    double timesRan = 0.0;
    for (Pose2d pipe : getPipePoses()) {
      double dist = pipe.getTranslation().getDistance(robotPose.getTranslation());
      if (dist > ASSIST_RADIUS) {
        continue;
      }
      accumulateAngle +=
          pipe.getTranslation().minus(robotPose.getTranslation()).getAngle().getRadians();
      accumulateMagnitude += clamp(kP / dist);

      timesRan += 1.0;
    }
    double averageHeading =
        (Math.atan2(
                    fieldRelativeRobotSpeeds.vxMetersPerSecond,
                    fieldRelativeRobotSpeeds.vyMetersPerSecond)
                + (accumulateAngle / timesRan))
            / 2;
    double robotVectorMagnitude =
        Math.hypot(
                fieldRelativeRobotSpeeds.vxMetersPerSecond,
                fieldRelativeRobotSpeeds.vyMetersPerSecond)
            * (accumulateMagnitude / timesRan);

    return new ChassisSpeeds(
        robotVectorMagnitude * Math.cos(averageHeading),
        robotVectorMagnitude * Math.sin(averageHeading),
        fieldRelativeRobotSpeeds.omegaRadiansPerSecond);
  }
}
