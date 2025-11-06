package frc.robot.vision.game_piece_detection;

import com.team581.math.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig;
import frc.robot.vision.results.GamePieceResult;

public final class GamePieceDetectionUtil {
  private static final double CORAL_RADIUS = 2.25;

  private static final Transform3d HORIZONTAL_CORAL_OFFSET =
      new Transform3d(0, 0, Units.inchesToMeters(-CORAL_RADIUS), Rotation3d.kZero);

  private static final Pose3d LIMELIGHT_POSE_TO_ROBOT_WITH_CORAL_OFFSET =
      RobotConfig.get().vision().gamePieceToRobotPose().transformBy(HORIZONTAL_CORAL_OFFSET);

  public static Translation2d calculateFieldRelativeCoralTranslationFromCamera(
      Pose2d robotPoseAtCapture, GamePieceResult visionResult) {
    var robotRelative =
        calculateRobotRelativeTranslationFromCamera(
            visionResult, LIMELIGHT_POSE_TO_ROBOT_WITH_CORAL_OFFSET);
    return robotRelativeToFieldRelativeGamePiecePose(robotPoseAtCapture, robotRelative);
  }

  public static double getFieldRelativeAngleToCoral(
      Pose2d robotPoseAtCapture, GamePieceResult visionResult) {
    var gamePiecePose =
        calculateFieldRelativeCoralTranslationFromCamera(robotPoseAtCapture, visionResult);
    return MathHelpers.getDriveDirection(gamePiecePose, robotPoseAtCapture).getDegrees();
  }

  public static Translation2d calculateRobotRelativeTranslationFromCamera(
      GamePieceResult visionResult, Pose3d limelightToRobotOffset) {
    double thetaX = Units.degreesToRadians(visionResult.tx());
    double thetaY = Units.degreesToRadians(visionResult.ty());
    double hypot = Math.copySign(Math.hypot(thetaX, thetaY), thetaX);
    double thetaRelativeToCenter = Math.atan(thetaY / thetaX);
    double adjustedRelativeToCenter =
        thetaRelativeToCenter + limelightToRobotOffset.getRotation().getX();
    double newThetaX = -1 * (hypot * Math.cos(adjustedRelativeToCenter));
    double newThetaY = hypot * Math.sin(adjustedRelativeToCenter);

    double adjustedThetaY = limelightToRobotOffset.getRotation().getY() - newThetaY;

    double forwardOffset = 0;
    if (adjustedThetaY == 0) {
      forwardOffset = Math.abs(limelightToRobotOffset.getY());
    } else {
      forwardOffset =
          // .getZ() represents height from floor
          (limelightToRobotOffset.getZ() / Math.tan(adjustedThetaY));
    }

    double sidewaysOffset = forwardOffset * Math.tan(newThetaX);

    var cameraRelativeTranslation = new Translation2d(forwardOffset, sidewaysOffset);
    return cameraRelativeTranslation
        .rotateBy(new Rotation2d(limelightToRobotOffset.getRotation().getZ()))
        .plus(limelightToRobotOffset.getTranslation().toTranslation2d());
  }

  public static Translation2d robotRelativeToFieldRelativeGamePiecePose(
      Pose2d robotPose, Translation2d robotRelativeGamePiecePose) {
    return robotRelativeGamePiecePose
        .rotateBy(robotPose.getRotation())
        .plus(robotPose.getTranslation());
  }

  private GamePieceDetectionUtil() {}
}
