package frc.robot.auto_align.tag_align;

import com.google.common.collect.ImmutableList;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefState;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.config.FeatureFlags;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import java.util.Optional;

public class TagAlign {
  private static final ImmutableList<ReefPipe> ALL_REEF_PIPES =
      ImmutableList.copyOf(ReefPipe.values());

  private static final PIDController TAG_SIDEWAYS_PID = new PIDController(6.0, 0.0, 0.0);
  private static final PIDController TAG_FORWARD_PID = new PIDController(3.0, 0.0, 0.0);
  private static final DoubleSubscriber TRANSLATION_GOOD_THRESHOLD =
      DogLog.tunable("AutoAlign/IsAlignedTranslation", 0.05);
  private static final DoubleSubscriber ROTATION_GOOD_THRESHOLD =
      DogLog.tunable("AutoAlign/IsAlignedRotation", 5.0);

  private final AlignmentCostUtil alignmentCostUtil;
  private final LocalizationSubsystem localization;
  private ReefPipeLevel level = ReefPipeLevel.BASE;
  private RobotScoringSide robotScoringSide = RobotScoringSide.RIGHT;
  private Translation2d driverPoseOffset = Translation2d.kZero;
  private Optional<ReefPipe> reefPipeOverride = Optional.empty();

  public ReefState reefState = new ReefState();

  public TagAlign(SwerveSubsystem swerve, LocalizationSubsystem localization) {
    this.localization = localization;
    alignmentCostUtil = new AlignmentCostUtil(localization, swerve, reefState, robotScoringSide);
  }

  public void setLevel(ReefPipeLevel level, RobotScoringSide side) {
    this.level = level;
    this.robotScoringSide = side;
  }

  public void setPipeOveride(ReefPipe pipe) {
    this.reefPipeOverride = Optional.of(pipe);
  }

  public void setDriverPoseOffset(Translation2d offset) {
    driverPoseOffset = offset;
  }

  public boolean isAligned(ReefPipe pipe) {
    var robotPose = localization.getPose();
    var scoringPoseFieldRelative = getUsedScoringPose(pipe);
    var translationGood =
        (robotPose.getTranslation().getDistance(scoringPoseFieldRelative.getTranslation())
            <= TRANSLATION_GOOD_THRESHOLD.get());
    var rotationGood =
        MathUtil.isNear(
            scoringPoseFieldRelative.getRotation().getDegrees(),
            robotPose.getRotation().getDegrees(),
            ROTATION_GOOD_THRESHOLD.get(),
            -180.0,
            180.0);
    return translationGood && rotationGood;
  }

  public void markScored(ReefPipe pipe) {
    reefState.markScored(pipe, level);
  }

  public void clearReefState() {
    reefState.clear();
  }

  public Pose2d getUsedScoringPose(ReefPipe pipe) {
    var theoreticalScoringPose = pipe.getPose(level, robotScoringSide);

    if (DriverStation.isTeleop()) {
      var offsetPose =
          new Pose2d(
              theoreticalScoringPose.getTranslation().plus(driverPoseOffset),
              theoreticalScoringPose.getRotation());
      return offsetPose;
    }

    return theoreticalScoringPose;
  }

  /** Returns the best reef pipe for scoring, based on the robot's current state. */
  public ReefPipe getBestPipe() {
    if (DriverStation.isAutonomous() && reefPipeOverride.isPresent()) {
      return reefPipeOverride.orElseThrow();
    }

    return ALL_REEF_PIPES.stream()
        .min(alignmentCostUtil.getReefPipeComparator(level))
        .orElseThrow();
  }

  public ChassisSpeeds getAlgaeAlignmentSpeeds(Pose2d usedScoringPose) {
    var robotPose = localization.getPose();
    var scoringTranslationRobotRelative =
        usedScoringPose
            .getTranslation()
            .minus(robotPose.getTranslation())
            .rotateBy(Rotation2d.fromDegrees(360 - robotPose.getRotation().getDegrees()));

    var goalTranslationWithP =
        new Translation2d(
            TAG_SIDEWAYS_PID.calculate(scoringTranslationRobotRelative.getX()),
            TAG_FORWARD_PID.calculate(0));
    var goalTranslation = goalTranslationWithP.rotateBy(robotPose.getRotation());
    var goalSpeeds = new ChassisSpeeds(goalTranslation.getX(), goalTranslation.getY(), 0.0);
    DogLog.log("AutoAlign/AlgaeAlign/GoalSpeeds", goalSpeeds);
    return goalSpeeds;
  }

  public ChassisSpeeds getPoseAlignmentChassisSpeeds(Pose2d usedScoringPose) {
   if (FeatureFlags.AUTO_ALIGN_FIX_ROTATION_CAUSING_OVERSHOOT.getAsBoolean()) {
     var robotPose = localization.getPose();

     var scoringTranslationRobotRelative =
         usedScoringPose
             .getTranslation()
             .minus(robotPose.getTranslation())
             .rotateBy(Rotation2d.fromDegrees(360 - usedScoringPose.getRotation().getDegrees()));

     var goalTranslationWithP =
         new Translation2d(
             TAG_SIDEWAYS_PID.calculate(scoringTranslationRobotRelative.getX()),
             TAG_FORWARD_PID.calculate(scoringTranslationRobotRelative.getY()));
     var goalTranslation = goalTranslationWithP.rotateBy(usedScoringPose.getRotation());

     var goalSpeeds = new ChassisSpeeds(-goalTranslation.getX(), -goalTranslation.getY(), 0.0);
     DogLog.log("AutoAlign/GoalSpeeds", goalSpeeds);
     return goalSpeeds;
   } else {
    var robotPose = localization.getPose();
    var scoringTranslationRobotRelative =
        usedScoringPose
            .getTranslation()
            .minus(robotPose.getTranslation())
            .rotateBy(Rotation2d.fromDegrees(360 - robotPose.getRotation().getDegrees()));

    var goalTranslationWithP =
        new Translation2d(
            TAG_SIDEWAYS_PID.calculate(scoringTranslationRobotRelative.getX()),
            TAG_FORWARD_PID.calculate(scoringTranslationRobotRelative.getY()));
    var goalTranslation = goalTranslationWithP.rotateBy(robotPose.getRotation());

    var goalSpeeds = new ChassisSpeeds(-goalTranslation.getX(), -goalTranslation.getY(), 0.0);
    DogLog.log("AutoAlign/GoalSpeeds", goalSpeeds);
    return goalSpeeds;

   }
  }
}
