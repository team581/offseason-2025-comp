package frc.robot.intake_assist;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.vision.game_piece_detection.GamePieceDetection;
import frc.robot.vision.limelight.Limelight;

public class IntakeAssistManager {
  private static final double ASSIST_KP = 3.0;
  private static final double MAX_ANGLE_CHANGE = -35.0;
  private static final double MIN_ANGLE_CHANGE = 35.0;

  /// tune angle change

  public static ChassisSpeeds getRobotRelativeAssistSpeeds(
      LocalizationSubsystem localization,
      Limelight limelight,
      ChassisSpeeds fieldRelativeInputSpeeds) {
    var visionResult = limelight.getCoralResult();
    if (visionResult.isEmpty()) {
      return fieldRelativeInputSpeeds;
    }

    Pose2d robotPose = localization.getPose(visionResult.get().timestamp());
    double fieldRelativeAngle =
        GamePieceDetection.getFieldRelativeAngleToGamePiece(robotPose, visionResult.get());
    DogLog.log("IntakeAssist/TX", fieldRelativeAngle);
    double angleError = fieldRelativeAngle - robotPose.getRotation().getDegrees();
    double angleChange = MathUtil.clamp(MIN_ANGLE_CHANGE, MAX_ANGLE_CHANGE, angleError * ASSIST_KP);

    Translation2d requestedFieldRelativeDrive =
        new Translation2d(
            fieldRelativeInputSpeeds.vxMetersPerSecond, fieldRelativeInputSpeeds.vyMetersPerSecond);

    Translation2d newDriveRequest =
        requestedFieldRelativeDrive.rotateBy(Rotation2d.fromDegrees(angleChange));

    return new ChassisSpeeds(
        newDriveRequest.getX(),
        newDriveRequest.getY(),
        fieldRelativeInputSpeeds.omegaRadiansPerSecond);
  }
}
