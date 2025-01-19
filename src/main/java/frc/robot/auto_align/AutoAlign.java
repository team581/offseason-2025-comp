package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.fms.FmsSubsystem;
import frc.robot.purple.PurpleState;
import frc.robot.vision.CameraHealth;
import frc.robot.vision.results.TagResult;
import java.util.List;
import java.util.Optional;

public class AutoAlign {
  private static final List<ReefSide> ALL_REEF_SIDES = List.of(ReefSide.values());

  public static Pose2d getClosestReefSide(Pose2d robotPose, boolean isRedAlliance) {
    var reefSide =
        ALL_REEF_SIDES.stream()
            .min(
                (a, b) ->
                    Double.compare(
                        robotPose
                            .getTranslation()
                            .getDistance(a.getPose(isRedAlliance).getTranslation()),
                        robotPose
                            .getTranslation()
                            .getDistance(b.getPose(isRedAlliance).getTranslation())))
            .get();
    return reefSide.getPose(isRedAlliance);
  }

  public static Pose2d getClosestReefSide(Pose2d robotPose) {
    return getClosestReefSide(robotPose, FmsSubsystem.isRedAlliance());
  }

  public static boolean shouldNetScoreForwards(Pose2d robotPose) {
    double robotX = robotPose.getX();
    double theta = robotPose.getRotation().getDegrees();

    // entire field length is 17.55m
    double halfFieldLength = 17.55 / 2.0;

    // Robot is on blue side
    if (robotX < halfFieldLength) {
      return theta < 90 || theta > 270;
    }

    // Robot is on red side
    return theta > 90 && theta < 270;
  }

  public static boolean isCloseToReefSide(Pose2d robotPose, Pose2d nearestReefSide) {
    return robotPose.getTranslation().getDistance(nearestReefSide.getTranslation())
        < Units.feetToMeters(3);
  }

  public static ReefAlignState getReefAlignState(
      Pose2d robotPose,
      PurpleState purpleState,
      Optional<TagResult> tagResult,
      CameraHealth tagCameraHealth) {
    var reefPose = getClosestReefSide(robotPose);
    var closeToReefSide = isCloseToReefSide(robotPose, reefPose);

    if (closeToReefSide) {
      // We can't trust purple unless we are near the reef, to avoid false positives
      if (tagCameraHealth == CameraHealth.OFFLINE) {
        return ReefAlignState.CAMERA_DEAD;
      }
      if (purpleState == PurpleState.CENTERED) {
        return ReefAlignState.HAS_PURPLE_ALIGNED;
      }
      if (purpleState == PurpleState.VISIBLE_NOT_CENTERED) {
        return ReefAlignState.HAS_PURPLE_NOT_ALIGNED;
      }
    }

    if (tagResult.isEmpty()) {
      if (closeToReefSide) {
        return ReefAlignState.NO_TAGS_IN_POSITION;
      }
      return ReefAlignState.NO_TAGS_WRONG_POSITION;
    }

    if (closeToReefSide) {
      return ReefAlignState.HAS_TAGS_IN_POSITION;
    }

    return ReefAlignState.HAS_TAGS_WRONG_POSITION;
  }
}
