package frc.robot.auto_align;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.fms.FmsSubsystem;
import frc.robot.purple.PurpleState;
import frc.robot.swerve.SnapUtil;
import frc.robot.vision.CameraHealth;
import frc.robot.vision.results.TagResult;
import java.util.List;
import java.util.Optional;

public class AutoAlign {
  private static Optional<ReefPipe> autoReefPipeOverride = Optional.empty();

  private static final List<ReefSide> ALL_REEF_SIDES = List.of(ReefSide.values());
  private static final List<ReefPipe> ALL_REEF_PIPES = List.of(ReefPipe.values());

  public static void setAutoReefPipeOverride(ReefPipe override) {
    autoReefPipeOverride = Optional.of(override);
  }

  public static ReefSide getClosestReefSide(Pose2d robotPose, boolean isRedAlliance) {
    if (DriverStation.isAutonomous() && autoReefPipeOverride.isPresent()) {
      return ReefSide.fromPipe(autoReefPipeOverride.orElseThrow());
    }

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
    return reefSide;
  }

  public static ReefSide getClosestReefSide(Pose2d robotPose) {
    return getClosestReefSide(robotPose, FmsSubsystem.isRedAlliance());
  }

  public static Pose2d getClosestReefPipe(
      Pose2d robotPose, ReefPipeLevel level, boolean isRedAlliance) {
    if (DriverStation.isAutonomous() && autoReefPipeOverride.isPresent()) {
      return autoReefPipeOverride.orElseThrow().getPose(level);
    }

    var reefPipe =
        ALL_REEF_PIPES.stream()
            .min(
                (a, b) ->
                    Double.compare(
                        robotPose
                            .getTranslation()
                            .getDistance(a.getPose(level, isRedAlliance).getTranslation()),
                        robotPose
                            .getTranslation()
                            .getDistance(b.getPose(level, isRedAlliance).getTranslation())))
            .get();

    return reefPipe.getPose(level, isRedAlliance);
  }

  public static Pose2d getClosestReefPipe(Pose2d robotPose, ReefPipeLevel level) {
    return getClosestReefPipe(robotPose, level, FmsSubsystem.isRedAlliance());
  }

  public static boolean shouldNetScoreForwards(Pose2d robotPose) {
    double robotX = robotPose.getX();
    double theta = robotPose.getRotation().getDegrees();

    // entire field length is 17.55m
    double halfFieldLength = 17.55 / 2.0;

    // Robot is on blue side
    if (robotX < halfFieldLength) {
      return Math.abs(theta) < 90;
    }

    // Robot is on red side
    return Math.abs(theta) > 90;
  }

  public static boolean shouldIntakeStationFront(Pose2d robotPose) {
    double theta = robotPose.getRotation().getDegrees();
    var coralStationBackwardAngle = SnapUtil.getCoralStationAngle(robotPose);

    return !MathUtil.isNear(coralStationBackwardAngle, theta, 90, -180, 180);
  }

  public static boolean isCloseToReefSide(
      Pose2d robotPose, Pose2d nearestReefSide, double thresholdMeters) {
    return robotPose.getTranslation().getDistance(nearestReefSide.getTranslation())
        < thresholdMeters;
  }

  public static boolean isCloseToReefSide(Pose2d robotPose, Pose2d nearestReefSide) {
    return isCloseToReefSide(robotPose, nearestReefSide, Units.feetToMeters(5));
  }

  public static boolean isCloseToReefPipe(
      Pose2d robotPose, Pose2d nearestReefPipe, double thresholdMeters) {
    return robotPose.getTranslation().getDistance(nearestReefPipe.getTranslation())
        < thresholdMeters;
  }

  public static boolean isCloseToReefPipe(Pose2d robotPose, Pose2d nearestReefPipe) {
    return isCloseToReefPipe(robotPose, nearestReefPipe, Units.feetToMeters(1.5));
  }

  public static ReefAlignState getReefAlignState(
      Pose2d robotPose,
      PurpleState purpleState,
      ReefPipeLevel scoringLevel,
      Optional<TagResult> tagResult,
      CameraHealth tagCameraHealth) {
    var reefPipe = getClosestReefPipe(robotPose, scoringLevel);
    var closeToReefPipe = isCloseToReefPipe(robotPose, reefPipe);

    if (closeToReefPipe) {
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
      if (closeToReefPipe) {
        return ReefAlignState.NO_TAGS_IN_POSITION;
      }
      return ReefAlignState.NO_TAGS_WRONG_POSITION;
    }

    if (closeToReefPipe) {
      return ReefAlignState.HAS_TAGS_IN_POSITION;
    }

    return ReefAlignState.HAS_TAGS_WRONG_POSITION;
  }
}
