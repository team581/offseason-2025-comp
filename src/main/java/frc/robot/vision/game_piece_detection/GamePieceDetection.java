package frc.robot.vision.game_piece_detection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class GamePieceDetection {

  public static Translation2d calculateFieldRelativeTranslationFromCamera(
      double tx, double ty, Pose2d robotPoseAtCapture, Pose3d limelightPoseRobotSpace) {

    // Convert tx and ty to angles, tx on the limelight does not follow RHR
    // convention, so we need to negate by one

    double thetaX = -1 * Units.degreesToRadians(tx);
    double thetaY = Units.degreesToRadians(ty);
    double adjustedThetaY = limelightPoseRobotSpace.getRotation().getY() - thetaY;
    double yOffset =
        // .getZ() represents height from floor
        (limelightPoseRobotSpace.getZ() / Math.tan(adjustedThetaY))
            // .getY() is supposed to represent forward and backward distance from center of robot
            + Math.abs(limelightPoseRobotSpace.getY());

    double xOffset = yOffset * Math.tan(thetaX);

    var cameraRelativeTranslation = new Translation2d(yOffset, xOffset);
    var fieldRelativeTranslation =
        cameraRelativeTranslation
            .rotateBy(
                Rotation2d.fromRadians(
                    robotPoseAtCapture.getRotation().getRadians()
                        + limelightPoseRobotSpace.getRotation().getZ()))
            .plus(robotPoseAtCapture.getTranslation())
            .plus(limelightPoseRobotSpace.getTranslation().toTranslation2d());
    return fieldRelativeTranslation;
  }
}
