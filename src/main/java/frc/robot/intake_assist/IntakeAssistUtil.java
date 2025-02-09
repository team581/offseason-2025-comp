package frc.robot.intake_assist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.vision.game_piece_detection.GamePieceDetectionUtil;
import frc.robot.vision.results.GamePieceResult;
import java.util.Optional;

public class IntakeAssistUtil {
  private static final double INITIAL_LINEUP_DISTANCE_THRESHOLD = 0.1;
  private static final double INITIAL_LINEUP_DISTANCE_FROM_CORAL = 0.9;
  private static final double FINAL_SHOVE_DISTANCE_FROM_CORAL = 0.5;
  private static final double CORAL_ASSIST_KP = 3.5;
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
}
