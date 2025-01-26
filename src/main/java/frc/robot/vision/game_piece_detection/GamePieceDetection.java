package frc.robot.vision.game_piece_detection;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.vision.results.GamePieceResult;

public class GamePieceDetection {
  private static final Pose3d LIMELIGHT_POSE_TO_ROBOT =
      new Pose3d(0.5, 0.0, 0.5, new Rotation3d(0, Units.degreesToRadians(20), 0));

  private static Translation2d calculateFieldRelativeTranslationFromCamera(
      double tx, double ty, Pose2d robotPoseAtCapture, Pose3d limelightToRobotOffset) {

    // Convert tx and ty to angles, tx on the limelight does not follow RHR
    // convention, so we need to negate by one
    DogLog.log("GamePieceDetection/limelightPose3d", LIMELIGHT_POSE_TO_ROBOT);
    double thetaX = -1 * Units.degreesToRadians(tx);
    double thetaY = Units.degreesToRadians(ty);
    double adjustedThetaY = limelightToRobotOffset.getRotation().getY() - thetaY;

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

    double xOffset = yOffset * Math.tan(thetaX);

    var cameraRelativeTranslation = new Translation2d(yOffset, xOffset);
    var fieldRelativeTranslation =
        cameraRelativeTranslation
            .rotateBy(
                Rotation2d.fromRadians(
                    robotPoseAtCapture.getRotation().getRadians()
                        + limelightToRobotOffset.getRotation().getZ()))
            .plus(robotPoseAtCapture.getTranslation())
            .plus(limelightToRobotOffset.getTranslation().toTranslation2d());
    return fieldRelativeTranslation;
  }

  public static Translation2d calculateFieldRelativeTranslationFromCamera(
      Pose2d robotPoseAtCapture, GamePieceResult visionResult) {
    return calculateFieldRelativeTranslationFromCamera(
        visionResult.tx(), visionResult.ty(), robotPoseAtCapture, LIMELIGHT_POSE_TO_ROBOT);
  }

  public static double getFieldRelativeAngleToGamePiece(
      Pose2d robotPoseAtCapture, GamePieceResult visionResult) {
    var gamePiecePose =
        calculateFieldRelativeTranslationFromCamera(robotPoseAtCapture, visionResult);
    DogLog.log("GamePieceDetection/pose", new Pose2d(gamePiecePose, new Rotation2d()));
    return LocalizationSubsystem.distanceAngleToTarget(
            new Pose2d(gamePiecePose, new Rotation2d()), robotPoseAtCapture)
        .targetAngle();
  }
}
