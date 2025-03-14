package frc.robot.vision.game_piece_detection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.intake_assist.IntakeAssistUtil;
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
              Units.degreesToRadians(-7),
              Units.degreesToRadians(40.0),
              Units.degreesToRadians(-15)));

  private static final double CORAL_LENGTH = 11.875;
  private static final double CORAL_RADIUS = 2.25;
  private static final double ALGAE_DIAMETER = 16.25;
  private static final double algaeToGroundOffset =
      CORAL_LENGTH + (ALGAE_DIAMETER / 2.0); // length of coral + half of diameter of algae

  private static final Transform3d ALGAE_OFFSET =
      new Transform3d(0, 0, Units.inchesToMeters(-algaeToGroundOffset), Rotation3d.kZero);

  private static final Pose3d LIMELIGHT_POSE_TO_ROBOT_WITH_ALGAE_OFFSET =
      LIMELIGHT_POSE_TO_ROBOT.transformBy(ALGAE_OFFSET);

  public static Translation2d calculateFieldRelativeTranslationFromCamera(
      Pose2d robotPoseAtCapture, GamePieceResult visionResult) {
    var robotRelative =
        calculateRobotRelativeTranslationFromCamera(visionResult, LIMELIGHT_POSE_TO_ROBOT);
    return robotRelativeToFieldRelativeGamePiecePose(robotPoseAtCapture, robotRelative);
  }

  public static Translation2d calculateFieldRelativeLollipopTranslationFromCamera(
      Pose2d robotPoseAtCapture, GamePieceResult visionResult) {
    var robotRelative =
        calculateRobotRelativeLollipopTranslationFromCamera(
            visionResult, LIMELIGHT_POSE_TO_ROBOT_WITH_ALGAE_OFFSET);
    return robotRelativeToFieldRelativeGamePiecePose(robotPoseAtCapture, robotRelative);
  }

  public static double getFieldRelativeAngleToGamePiece(
      Pose2d robotPoseAtCapture, GamePieceResult visionResult) {
    var gamePiecePose =
        calculateFieldRelativeTranslationFromCamera(robotPoseAtCapture, visionResult);
    return IntakeAssistUtil.getIntakeAssistAngle(gamePiecePose, robotPoseAtCapture);
  }

  private static Translation2d calculateRobotRelativeLollipopTranslationFromCamera(
      GamePieceResult visionResult, Pose3d limelightToRobotOffset) {

    double thetaX = Units.degreesToRadians(visionResult.tx());
    double thetaY = Units.degreesToRadians(visionResult.ty());
    double hypot = Math.copySign(Math.hypot(thetaX, thetaY), thetaX);
    double thetaRelativeToCenter = Math.atan(thetaY / thetaX);
    double adjustedRelativeToCenter =
        thetaRelativeToCenter + LIMELIGHT_POSE_TO_ROBOT_WITH_ALGAE_OFFSET.getRotation().getX();
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

    double forwardOffset = 0;
    if (adjustedThetaY == 0) {
      forwardOffset = Math.abs(limelightToRobotOffset.getY());
    } else {
      forwardOffset =
          // .getZ() represents height from floor
          ((limelightToRobotOffset.getZ() - Units.inchesToMeters(CORAL_RADIUS))
              / Math.tan(adjustedThetaY));
    }

    double sidewaysOffset = forwardOffset * Math.tan(newThetaX);

    var cameraRelativeTranslation = new Translation2d(forwardOffset, sidewaysOffset);
    var robotRelativeTranslation =
        cameraRelativeTranslation
            .rotateBy(new Rotation2d(limelightToRobotOffset.getRotation().getZ()))
            .plus(limelightToRobotOffset.getTranslation().toTranslation2d());
    return robotRelativeTranslation;
  }

  private static Translation2d calculateRobotRelativeTranslationFromCamera(
      GamePieceResult visionResult) {
    return calculateRobotRelativeTranslationFromCamera(visionResult, LIMELIGHT_POSE_TO_ROBOT);
  }

  private static Translation2d robotRelativeToFieldRelativeGamePiecePose(
      Pose2d robotPose, Translation2d robotRelativeGamePiecePose) {
    return robotRelativeGamePiecePose
        .rotateBy(robotPose.getRotation())
        .plus(robotPose.getTranslation());
  }

  public static Translation2d calculateRobotRelativePoseToIntake(
      GamePieceResult visionResult, double offset) {
    var robotRelativeGamePiecePose = calculateRobotRelativeTranslationFromCamera(visionResult);
    return new Translation2d(
        robotRelativeGamePiecePose.getX() - offset, robotRelativeGamePiecePose.getY());
  }
}
