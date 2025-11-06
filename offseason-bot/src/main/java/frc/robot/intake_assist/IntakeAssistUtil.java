package frc.robot.intake_assist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class IntakeAssistUtil {

  private static final double ASSIST_KP = 3.7;

  public static ChassisSpeeds getAssistSpeedsFromPose(
      Pose2d target, Pose2d robotPose, ChassisSpeeds teleopSpeeds) {
    var robotRelativePose =
        target
            .getTranslation()
            .minus(robotPose.getTranslation())
            .rotateBy(Rotation2d.fromDegrees(360 - robotPose.getRotation().getDegrees()));
    var sidewaysSpeed = MathUtil.clamp(robotRelativePose.getY() * ASSIST_KP, -1.0, 1.0);
    var robotRelativeError = new Translation2d(0, sidewaysSpeed);
    var fieldRelativeError = robotRelativeError.rotateBy(robotPose.getRotation());
    var assistSpeeds = new ChassisSpeeds(fieldRelativeError.getX(), fieldRelativeError.getY(), 0.0);

    return assistSpeeds.plus(teleopSpeeds);
  }

  private IntakeAssistUtil() {}
}
