package frc.robot.vision.game_piece_detection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.vision.results.GamePieceResult;

public class GamePieceDetectionUtil {
  public static final Pose3d LIMELIGHT_POSE_TO_ROBOT =
      new Pose3d(
          // Positive-Forward
          Units.inchesToMeters(11.41),
          // Positive-Left
          Units.inchesToMeters(10.22),

          // Positive-Up
          Units.inchesToMeters(36.09),
          new Rotation3d(
              Units.degreesToRadians(-5),
              Units.degreesToRadians(32.5),
              Units.degreesToRadians(-15)));

  public static Translation2d calculateFieldRelativeTranslationFromCamera(
      Pose2d robotPoseAtCapture, GamePieceResult visionResult) {
    var robotRelative =
        calculateRobotRelativeTranslationFromCamera(visionResult, LIMELIGHT_POSE_TO_ROBOT);
    return robotRelativeToFieldRelativeGamePiecePose(robotPoseAtCapture, robotRelative);
  }

  private static Translation2d calculateRobotRelativeTranslationFromCamera(
      GamePieceResult visionResult, Pose3d limelightToRobotOffset) {

    double thetaX = Units.degreesToRadians(visionResult.tx());
    double thetaY = Units.degreesToRadians(visionResult.ty());
    double hypot = Math.copySign(Math.hypot(thetaX, thetaY), thetaX);
    double thetaRelativeToCenter = Math.atan(thetaY / thetaX);
    double adjustedRelativeToCenter =
        thetaRelativeToCenter + LIMELIGHT_POSE_TO_ROBOT.getRotation().getX();
    double newThetaX = -1 * (hypot * Math.cos(adjustedRelativeToCenter));
    double newThetaY = hypot * Math.sin(adjustedRelativeToCenter);

    double adjustedThetaY = limelightToRobotOffset.getRotation().getY() - newThetaY;

    double yOffset = 0;
    if (adjustedThetaY == 0) {
      yOffset = Math.abs(limelightToRobotOffset.getY());
    } else {
      yOffset =
          // .getZ() represents height from floor
          (limelightToRobotOffset.getZ() / Math.tan(adjustedThetaY))
              // .getX() is supposed to represent forward and backward distance from center of robot
              + Math.abs(limelightToRobotOffset.getX());
    }

    double xOffset = yOffset * Math.tan(newThetaX);

    var cameraRelativeTranslation = new Translation2d(yOffset, xOffset);
    var robotRelativeTranslation =
        cameraRelativeTranslation
            .rotateBy(new Rotation2d(limelightToRobotOffset.getRotation().getZ()))
            .plus(limelightToRobotOffset.getTranslation().toTranslation2d());
    return robotRelativeTranslation;
  }

  public static Translation2d calculateRobotRelativeTranslationFromCamera(
      GamePieceResult visionResult) {
    return calculateRobotRelativeTranslationFromCamera(visionResult, LIMELIGHT_POSE_TO_ROBOT);
  }

  public static double getRobotRelativeAngleToGamePiece(GamePieceResult visionResult) {
    var gamePiecePose = calculateRobotRelativeTranslationFromCamera(visionResult);
    return LocalizationSubsystem.distanceAngleToTarget(
            new Pose2d(gamePiecePose, Rotation2d.kZero), Pose2d.kZero)
        .targetAngle();
  }

  private static Translation2d robotRelativeToFieldRelativeGamePiecePose(
      Pose2d robotPose, Translation2d robotRelativeGamePiecePose) {
    return robotRelativeGamePiecePose
        .rotateBy(robotPose.getRotation())
        .plus(robotPose.getTranslation());
  }

  public static Translation2d calculateFieldRelativePoseToIntake(
      GamePieceResult visionResult, Pose2d robotPose) {
    var robotRelativeGamePiecePose = calculateRobotRelativeTranslationFromCamera(visionResult);
    var adjustedTranslationForIntake =
        new Translation2d(
            robotRelativeGamePiecePose.getX() - 0.762, robotRelativeGamePiecePose.getY());
    return robotRelativeToFieldRelativeGamePiecePose(robotPose, adjustedTranslationForIntake);
  }

  public static Translation2d calculateRobotRelativePoseToIntake(
      GamePieceResult visionResult, double offset) {
    var robotRelativeGamePiecePose = calculateRobotRelativeTranslationFromCamera(visionResult);
    return new Translation2d(
        robotRelativeGamePiecePose.getX() - offset, robotRelativeGamePiecePose.getY());
  }
}
