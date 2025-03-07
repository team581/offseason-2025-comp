package frc.robot.intake_assist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.vision.game_piece_detection.GamePieceDetectionUtil;
import frc.robot.vision.results.GamePieceResult;
import java.util.Optional;

public class IntakeAssistUtil {
  private static final double INITIAL_LINEUP_DISTANCE_THRESHOLD = 0.05;
  private static final double INITIAL_LINEUP_DISTANCE_FROM_CORAL = 0.9;
  private static final double FINAL_SHOVE_DISTANCE_FROM_CORAL = 0.5;
  private static final double CORAL_ASSIST_KP = 4.0;
  private static final double ALGAE_ASSIST_KP = 2.0;

  public static ChassisSpeeds getCoralAssistSpeeds(
      Optional<GamePieceResult> coral, double robotHeading, boolean greedyIntake) {
    return getRobotRelativeAssistSpeeds(coral, robotHeading, CORAL_ASSIST_KP, greedyIntake);
  }

  public static ChassisSpeeds getAlgaeAssistSpeeds(
      Optional<GamePieceResult> algae, double robotHeading, boolean greedyIntake) {
    return getRobotRelativeAssistSpeeds(algae, robotHeading, ALGAE_ASSIST_KP, greedyIntake);
  }

  private static ChassisSpeeds getRobotRelativeAssistSpeeds(
      Optional<GamePieceResult> visionResult,
      double robotHeading,
      double kP,
      boolean greedyIntake) {
    if (visionResult.isEmpty()) {
      return new ChassisSpeeds();
    }

    var gamePiecePoseRobotRelative =
        GamePieceDetectionUtil.calculateRobotRelativePoseToIntake(
            visionResult.get(), INITIAL_LINEUP_DISTANCE_FROM_CORAL);

    if (greedyIntake
        && gamePiecePoseRobotRelative.getDistance(new Translation2d(0, 0))
            < INITIAL_LINEUP_DISTANCE_THRESHOLD) {
      var gamePiecePoseForwardRobotRelative =
          GamePieceDetectionUtil.calculateRobotRelativePoseToIntake(
              visionResult.get(), FINAL_SHOVE_DISTANCE_FROM_CORAL);

      var gamePiecePoseForwardRotatedRobot =
          gamePiecePoseForwardRobotRelative.rotateBy(Rotation2d.fromDegrees(robotHeading));
      var forwardXError = gamePiecePoseForwardRotatedRobot.getX();
      var forwardYError = gamePiecePoseForwardRotatedRobot.getY();

      var forwardXEffort = forwardXError * kP;
      var forwardYEffort = forwardYError * kP;

      return new ChassisSpeeds(forwardXEffort, forwardYEffort, 0.0);
    }

    var gamePiecePoseRotatedRobot =
        gamePiecePoseRobotRelative.rotateBy(Rotation2d.fromDegrees(robotHeading));
    var xError = gamePiecePoseRotatedRobot.getX();
    var yError = gamePiecePoseRotatedRobot.getY();

    var xEffort = xError * kP;
    var yEffort = yError * kP;

    return new ChassisSpeeds(xEffort, yEffort, 0.0);
  }

  public static ChassisSpeeds getAssistSpeedsFromPose(Pose2d target, Pose2d robotPose) {
    var robotRelativePose = target.getTranslation().minus(robotPose.getTranslation()).rotateBy(Rotation2d.fromDegrees(360-robotPose.getRotation().getDegrees()));
    var sidewaysError = robotRelativePose.getY();
    var robotRelativeError = new Translation2d(0.0, sidewaysError);
    var fieldRelativeError= robotRelativeError.rotateBy(robotPose.getRotation());
    return new ChassisSpeeds(fieldRelativeError.getX()*CORAL_ASSIST_KP, fieldRelativeError.getY()*CORAL_ASSIST_KP, 0.0);

  }

  public static double getIntakeAssistAngle(Translation2d target, Pose2d robotPose) {
    return Units.radiansToDegrees(
        Math.atan2(target.getY() - robotPose.getY(), target.getX() - robotPose.getX()));
  }
}
