package frc.robot.auto_align;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.autos.constraints.AutoConstraintCalculator;
import frc.robot.autos.constraints.AutoConstraintOptions;
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

  private static final double REEF_FINAL_SPEEDS_DISTANCE_THRESHOLD = 1.5;

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

  private static final double LINEAR_VELOCITY_TO_REEF_SIDE_DISTANCE_KS = 1.5;
  private static final double LINEAR_VELOCITY_TO_REEF_SIDE_DISTANCE_KP = 0.625;

  public static boolean isCloseToReefSide(
      Pose2d robotPose, Pose2d nearestReefSide, ChassisSpeeds robotSpeeds) {
    var linearVelocity = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
    DogLog.log("Swerve/LinearVelocity", linearVelocity);
    return isCloseToReefSide(
        robotPose,
        nearestReefSide,
        LINEAR_VELOCITY_TO_REEF_SIDE_DISTANCE_KS
            + LINEAR_VELOCITY_TO_REEF_SIDE_DISTANCE_KP * linearVelocity);
  }

  public static boolean isCloseToReefPipe(
      Pose2d robotPose, Pose2d nearestReefPipe, double thresholdMeters) {
    return robotPose.getTranslation().getDistance(nearestReefPipe.getTranslation())
        < thresholdMeters;
  }

  public static boolean isCloseToReefPipe(Pose2d robotPose, Pose2d nearestReefPipe) {
    return isCloseToReefPipe(robotPose, nearestReefPipe, Units.feetToMeters(1.5));
  }

  public static ChassisSpeeds calculateTeleopAndAlignSpeeds(
      ChassisSpeeds teleopSpeeds,
      ChassisSpeeds alignSpeeds,
      double baseTeleopSpeed,
      double minConstraint) {
    double teleopVelocity =
        Math.hypot(teleopSpeeds.vxMetersPerSecond, teleopSpeeds.vyMetersPerSecond);
    double alignVelocity = Math.hypot(alignSpeeds.vxMetersPerSecond, alignSpeeds.vyMetersPerSecond);
    var wantedSpeeds = teleopSpeeds.plus(alignSpeeds);

    var teleopVelocityMax = Math.max(baseTeleopSpeed, teleopVelocity);

    var minSpeed = Math.min(alignVelocity, teleopVelocityMax);
    if (alignVelocity < minConstraint) {
      minSpeed = teleopVelocityMax;
    }
    DogLog.log("PurpleAlignment/Constraint", minSpeed);
    var options =
        new AutoConstraintOptions()
            .withCollisionAvoidance(false)
            .withMaxAngularAcceleration(0)
            .withMaxAngularVelocity(0)
            .withMaxLinearAcceleration(0)
            .withMaxLinearVelocity(minSpeed);
    return AutoConstraintCalculator.constrainLinearVelocity(wantedSpeeds, options);
  }

  public static ChassisSpeeds calculateConstrainedAndWeightedSpeeds(
      Pose2d robotPose,
      ChassisSpeeds teleopSpeeds,
      ChassisSpeeds alignSpeeds,
      double baseTeleopSpeed,
      double minConstraint) {
    var constrainedSpeeds =
        calculateTeleopAndAlignSpeeds(teleopSpeeds, alignSpeeds, baseTeleopSpeed, minConstraint);
    var distanceToReef =
        robotPose
            .getTranslation()
            .getDistance(getClosestReefPipe(robotPose, ReefPipeLevel.L1).getTranslation());
    if (distanceToReef > REEF_FINAL_SPEEDS_DISTANCE_THRESHOLD) {
      return constrainedSpeeds;
    }

    var progress = MathUtil.clamp(distanceToReef / REEF_FINAL_SPEEDS_DISTANCE_THRESHOLD, 0.1, 1.0);
    DogLog.log("Debug/Progress", progress);
    var newTeleopSpeeds = teleopSpeeds.times(progress);
    var newAlignSpeeds = alignSpeeds.times(1 - progress);
    var newConstrainedSpeeds =
        calculateTeleopAndAlignSpeeds(
            newTeleopSpeeds, newAlignSpeeds, baseTeleopSpeed, minConstraint);
    DogLog.log("Debug/NewConstrainedSpeeds", newConstrainedSpeeds);
    return newConstrainedSpeeds;
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
