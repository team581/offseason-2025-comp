package frc.robot.intake_assist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.vision.game_piece_detection.GamePieceDetectionUtil;
import frc.robot.vision.results.GamePieceResult;
import java.util.Optional;

public class IntakeAssistUtil {
  private static final double CORAL_ASSIST_KP = 0.1;
  private static final double ALGAE_ASSIST_KP = 0.1;

  public static ChassisSpeeds getCoralAssistSpeeds(
      Optional<GamePieceResult> coral,
      double robotHeading,
      ChassisSpeeds fieldRelativeInputSpeeds) {
    return getRobotRelativeAssistSpeeds(
        coral, robotHeading, fieldRelativeInputSpeeds, CORAL_ASSIST_KP);
  }

  public static ChassisSpeeds getAlgaeAssistSpeeds(
      Optional<GamePieceResult> algae,
      double robotHeading,
      ChassisSpeeds fieldRelativeInputSpeeds) {
    return getRobotRelativeAssistSpeeds(
        algae, robotHeading, fieldRelativeInputSpeeds, ALGAE_ASSIST_KP);
  }

  private static ChassisSpeeds getRobotRelativeAssistSpeeds(
      Optional<GamePieceResult> visionResult,
      double robotHeading,
      ChassisSpeeds fieldRelativeInputSpeeds,
      double kP) {
    if (visionResult.isEmpty()) {
      return fieldRelativeInputSpeeds;
    }

    var gamePiecePoseRobotRelative =
        GamePieceDetectionUtil.calculateRobotRelativeTranslationFromCamera(visionResult.get());
    var gamePiecePoseRotatedRobot =
        gamePiecePoseRobotRelative.rotateBy(Rotation2d.fromDegrees(robotHeading));
    var xError = gamePiecePoseRotatedRobot.getX();
    var yError = gamePiecePoseRotatedRobot.getY();

    var xEffort = xError * kP;
    var yEffort = yError * kP;

    return new ChassisSpeeds(
        fieldRelativeInputSpeeds.vxMetersPerSecond + xEffort,
        fieldRelativeInputSpeeds.vyMetersPerSecond + yEffort,
        fieldRelativeInputSpeeds.omegaRadiansPerSecond);
  }
}
