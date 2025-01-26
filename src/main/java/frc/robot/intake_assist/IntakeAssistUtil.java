package frc.robot.intake_assist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.vision.game_piece_detection.GamePieceDetectionUtil;
import frc.robot.vision.results.GamePieceResult;
import java.util.Optional;

public class IntakeAssistUtil {
  private static final double CORAL_ASSIST_KP = 3.0;
  private static final double ALGAE_ASSIST_KP = 3.0;
  private static final double MAX_ANGLE_CHANGE = 35.0;

  public static ChassisSpeeds getCoralAssistSpeeds(
      Optional<GamePieceResult> coral, ChassisSpeeds fieldRelativeInputSpeeds) {
    return getRobotRelativeAssistSpeeds(coral, fieldRelativeInputSpeeds, CORAL_ASSIST_KP);
  }

  public static ChassisSpeeds getAlgaeAssistSpeeds(
      Optional<GamePieceResult> algae, ChassisSpeeds fieldRelativeInputSpeeds) {
    return getRobotRelativeAssistSpeeds(algae, fieldRelativeInputSpeeds, ALGAE_ASSIST_KP);
  }

  private static ChassisSpeeds getRobotRelativeAssistSpeeds(
      Optional<GamePieceResult> visionResult, ChassisSpeeds fieldRelativeInputSpeeds, double kP) {
    if (visionResult.isEmpty()) {
      return fieldRelativeInputSpeeds;
    }

    var angleError = GamePieceDetectionUtil.getRobotRelativeAngleToGamePiece(visionResult.get());
    var modifiedAngleError = MathUtil.clamp(-MAX_ANGLE_CHANGE, MAX_ANGLE_CHANGE, angleError * kP);

    Translation2d inputSpeedsTranslation =
        new Translation2d(
            fieldRelativeInputSpeeds.vxMetersPerSecond, fieldRelativeInputSpeeds.vyMetersPerSecond);

    Translation2d newSpeedsTranslation =
        inputSpeedsTranslation.rotateBy(Rotation2d.fromDegrees(modifiedAngleError));

    return new ChassisSpeeds(
        newSpeedsTranslation.getX(),
        newSpeedsTranslation.getY(),
        fieldRelativeInputSpeeds.omegaRadiansPerSecond);
  }
}
