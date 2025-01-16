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
    double adjustedThetaX = limelightPoseRobotSpace.getRotation().getZ() - thetaX;

    double thetaY = Units.degreesToRadians(ty);
    double adjustedThetaY = limelightPoseRobotSpace.getRotation().getY() - thetaY;
    double yOffset =
        // .getY() is supposed to represent up and down distance from center of robot
        (limelightPoseRobotSpace.getZ() / Math.tan(adjustedThetaY));

    double xOffset = yOffset * Math.tan(adjustedThetaX);

    var robotRelativeTranslation = new Translation2d(yOffset, xOffset);
    var fieldRelativeTranslation =
        robotRelativeTranslation
            .rotateBy(new Rotation2d(robotPoseAtCapture.getRotation().getRadians()))
            .plus(robotPoseAtCapture.getTranslation())
            .plus(limelightPoseRobotSpace.getTranslation().toTranslation2d());
    return fieldRelativeTranslation;
  }
}
