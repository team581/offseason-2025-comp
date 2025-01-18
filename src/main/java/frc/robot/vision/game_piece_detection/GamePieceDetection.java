package frc.robot.vision.game_piece_detection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.vision.results.GamePieceResult;

public class GamePieceDetection {
  private final LocalizationSubsystem localization;
  private static final Pose3d LIMELIGHT_POSE_TO_ROBOT = new Pose3d();

  public GamePieceDetection(LocalizationSubsystem localization) {
    this.localization = localization;
  }

  private static Translation2d calculateFieldRelativeTranslationFromCamera(
      double tx, double ty, Pose2d robotPoseAtCapture, Pose3d limelightToRobotOffset) {

    // Convert tx and ty to angles, tx on the limelight does not follow RHR
    // convention, so we need to negate by one

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
              // .getY() is supposed to represent forward and backward distance from center of robot
              + Math.abs(limelightToRobotOffset.getY());
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

  public Translation2d calculateFieldRelativeTranslationFromCamera(GamePieceResult visionResult) {
    return calculateFieldRelativeTranslationFromCamera(
        visionResult.tx(),
        visionResult.ty(),
        localization.getPose(visionResult.timestamp()),
        LIMELIGHT_POSE_TO_ROBOT);
  }
}
