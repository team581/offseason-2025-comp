package frc.robot.intake_assist;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.vision.game_piece_detection.GamePieceDetectionUtil;
import frc.robot.vision.results.GamePieceResult;

public class IntakeAssistUtil {
  private static final double CORAL_ASSIST_KP = 3.0;
  private static final double INTAKE_OFFSET = Units.inchesToMeters(18);

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

  public static Optional<Pose2d> getLollipopIntakePoseFromVisionResult(Optional<GamePieceResult> result, Pose2d robotPose) {
    if (result.isEmpty()) {
      return Optional.empty();
    }
   var translation = GamePieceDetectionUtil.calculateRobotRelativeLollipopTranslationFromCamera(robotPose, result.get());
   var offset = new Translation2d(translation.getX()-INTAKE_OFFSET, translation.getY());
   var fieldRelativeOffsetTranslation = GamePieceDetectionUtil.robotRelativeToFieldRelativeGamePiecePose(robotPose, offset);
   var rotation = getIntakeAssistAngle(fieldRelativeOffsetTranslation, robotPose);
   return Optional.of(new Pose2d(fieldRelativeOffsetTranslation, Rotation2d.fromDegrees(rotation)));
  }
}
