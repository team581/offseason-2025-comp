package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.fms.FmsSubsystem;
import frc.robot.util.MathHelpers;

public class MagnetismUtil {
  private static final double kP = 1.0;
  private static final double MAX_ASSIST_SPEED = 1.5;
  private static final double MIN_ASSIST_SPEED = 0.3;
  private static final double ASSIST_RADIUS = 2.5;

  private static double clampX(double val, double rad) {
    return Math.min(
        Math.max(val, MIN_ASSIST_SPEED * Math.cos(rad)), MAX_ASSIST_SPEED * Math.cos(rad));
  }

  private static double clampY(double val, double rad) {
    return Math.min(
        Math.max(val, MIN_ASSIST_SPEED * Math.sin(rad)), MAX_ASSIST_SPEED * Math.sin(rad));
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
    boolean withinRadius = false;
    ChassisSpeeds accumulateSpeeds = new ChassisSpeeds();

    double timesRan = 0.0;
    for (Pose2d pipe : getPipePoses()) {
      Translation2d dXdY = pipe.getTranslation().minus(robotPose.getTranslation());
      boolean tempInRadius = (Math.hypot(dXdY.getX(), dXdY.getY()) < ASSIST_RADIUS);
      if (!tempInRadius) {
        continue;
      }
      withinRadius = true;
      accumulateSpeeds =
          accumulateSpeeds.plus(new ChassisSpeeds(dXdY.getX() * kP, dXdY.getY() * kP, 0.0));
      timesRan += 1.0;
    }
    if (!withinRadius) {
      return fieldRelativeRobotSpeeds;
    }
    double robotDirection =
        Math.atan2(
            fieldRelativeRobotSpeeds.vyMetersPerSecond, fieldRelativeRobotSpeeds.vxMetersPerSecond);
    double vxSign = Math.copySign(1.0, fieldRelativeRobotSpeeds.vxMetersPerSecond);
    double vySign = Math.copySign(1.0, fieldRelativeRobotSpeeds.vyMetersPerSecond);

    accumulateSpeeds.vxMetersPerSecond =
        vxSign
            * clampX(
                (Math.abs(accumulateSpeeds.vxMetersPerSecond)
                    / (timesRan * MathHelpers.sec(robotDirection))),
                robotDirection);
    accumulateSpeeds.vyMetersPerSecond =
        vySign
            * clampY(
                (Math.abs(accumulateSpeeds.vyMetersPerSecond)
                    / (timesRan * MathHelpers.csc(robotDirection))),
                robotDirection);

    return fieldRelativeRobotSpeeds.plus(accumulateSpeeds);
  }
}
