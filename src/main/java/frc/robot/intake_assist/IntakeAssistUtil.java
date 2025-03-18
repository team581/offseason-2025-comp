package frc.robot.intake_assist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class IntakeAssistUtil {
  private static final double CORAL_ASSIST_KP = 3.0;

  public static ChassisSpeeds getAssistSpeedsFromPose(Pose2d target, Pose2d robotPose) {
    var robotRelativePose =
        target
            .getTranslation()
            .minus(robotPose.getTranslation())
            .rotateBy(Rotation2d.fromDegrees(360 - robotPose.getRotation().getDegrees()));
    var sidewaysError = robotRelativePose.getY();
    var robotRelativeError = new Translation2d(0.0, sidewaysError);
    var fieldRelativeError = robotRelativeError.rotateBy(robotPose.getRotation());
    return new ChassisSpeeds(
        fieldRelativeError.getX() * CORAL_ASSIST_KP,
        fieldRelativeError.getY() * CORAL_ASSIST_KP,
        0.0);
  }

  public static double getIntakeAssistAngle(Translation2d target, Pose2d robotPose) {
    return Units.radiansToDegrees(
        Math.atan2(target.getY() - robotPose.getY(), target.getX() - robotPose.getX()));
  }
}
