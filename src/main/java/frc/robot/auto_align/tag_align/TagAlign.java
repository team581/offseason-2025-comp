package frc.robot.auto_align.tag_align;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefState;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.MathHelpers;
import java.util.List;

public class TagAlign {
  private static final List<ReefPipe> ALL_REEF_PIPES = List.of(ReefPipe.values());

  private static final PIDController TAG_PID = new PIDController(3.2, 0.0, 0.03);
  private static final double BEFORE_RAISED_INITIAL_DISTANCE_OFFSET = 0.35;
  private static final double TAG_ALIGNMENT_FINISHED_DISTANCE_THRESHOLD = 0.05;

  private final AlignmentCostUtil alignmentCostUtil;
  private final LocalizationSubsystem localization;
  private ReefPipeLevel level = ReefPipeLevel.L1;
  private ChassisSpeeds rawTeleopSpeeds = new ChassisSpeeds();
  private Translation2d driverPoseOffset = Translation2d.kZero;

  public ReefState reefState = new ReefState();

  public TagAlign(SwerveSubsystem swerve, LocalizationSubsystem localization) {
    this.localization = localization;
    alignmentCostUtil = new AlignmentCostUtil(localization, swerve, reefState);
  }

  public void setLevel(ReefPipeLevel level) {
    this.level = level;
  }

  public void setRawTeleopSpeeds(ChassisSpeeds speeds) {
    rawTeleopSpeeds = speeds;
  }

  public void setDriverPoseOffset(Translation2d offset) {
    driverPoseOffset = offset;
  }

  public boolean isAligned() {
    var robotPose = localization.getPose();
    var scoringPoseFieldRelative = getBestPipe().getPose(level);
    return robotPose.getTranslation().getDistance(scoringPoseFieldRelative.getTranslation())
        <= TAG_ALIGNMENT_FINISHED_DISTANCE_THRESHOLD;
  }

  public void markScored() {
    reefState.markScored(getBestPipe(), level);
  }

  public Pose2d getUsedScoringPose() {
    var rawRobotPose = localization.getPose();
    var lookaheadPose = MathHelpers.poseLookahead(rawRobotPose, rawTeleopSpeeds, 0.4);
    DogLog.log("PurpleAlignment/LookaheadPose", lookaheadPose);
    var rawPose = getBestPipe().getPose(level);
    var offsetPose =
        new Pose2d(rawPose.getTranslation().plus(driverPoseOffset), rawPose.getRotation());
    return offsetPose;
  }

  /** Returns the best reef pipe for scoring, based on the robot's current state. */
  public ReefPipe getBestPipe() {
    return ALL_REEF_PIPES.stream()
        .min(alignmentCostUtil.getReefPipeComparator(level))
        .orElseThrow();
  }

  public ChassisSpeeds getPoseAlignmentChassisSpeeds(boolean forwardOnly) {
    var robotPose = localization.getPose();
    var scoringTranslationFieldRelative = getUsedScoringPose();

    var scoringTranslationRobotRelative =
        scoringTranslationFieldRelative
            .getTranslation()
            .minus(robotPose.getTranslation())
            .rotateBy(Rotation2d.fromDegrees(360 - robotPose.getRotation().getDegrees()));

    var goalTranslationUnrotated = new Translation2d();
    if (forwardOnly) {
      goalTranslationUnrotated = new Translation2d(scoringTranslationRobotRelative.getX(), 0.0);
    } else {
      goalTranslationUnrotated = scoringTranslationRobotRelative;
    }

    var goalTranslation = goalTranslationUnrotated.rotateBy(robotPose.getRotation());

    var xEffort = TAG_PID.calculate(-goalTranslation.getX());
    var yEffort = TAG_PID.calculate(-goalTranslation.getY());

    DogLog.log("PurpleAlignment/Tag/XEffort", xEffort);
    DogLog.log("PurpleAlignment/Tag/YEffort", yEffort);
    DogLog.log("PurpleAlignment/Tag/ForwardOnly", forwardOnly);

    return new ChassisSpeeds(xEffort, yEffort, 0.0);
  }
}
