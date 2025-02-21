package frc.robot.auto_align.tag_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefState;
import frc.robot.config.FeatureFlags;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.MathHelpers;
import java.util.Comparator;

public class AlignmentCostUtil {
  private static final double LOOKAHEAD = 0.5;
  private static final double ANGLE_DIFFERENCE_SCALAR = 0.02;

  /**
   * Returns the "cost" (a dimensionless number) of aligning to a given pose based on the robot's
   * current state.
   *
   * @param target Pose to align to
   * @param robotPose The robot's current pose
   * @param robotVelocity The robot's current velocity (field relative)
   */
  private static double getAlignCost(Pose2d target, Pose2d robotPose, ChassisSpeeds robotVelocity) {
    if (FeatureFlags.REEF_ALIGN_LOOKAHEAD_DISTANCE_COST_FN.getAsBoolean()) {
      var lookahead = MathHelpers.poseLookahead(robotPose, robotVelocity, LOOKAHEAD);
      return lookahead.getTranslation().getDistance(target.getTranslation());
    }

    var distanceCost = target.getTranslation().getDistance(robotPose.getTranslation());
    if (target.getTranslation().equals(Translation2d.kZero)
        || robotPose.getTranslation().equals(Translation2d.kZero)) {
      return distanceCost;
    }

    if (Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond) == 0.0) {
      return distanceCost;
    }

    var targetRobotRelative = target.getTranslation().minus(robotPose.getTranslation());
    var targetDirection = targetRobotRelative.getAngle();

    var driveAngleCost =
        ANGLE_DIFFERENCE_SCALAR
            * Math.abs(
                targetDirection.minus(MathHelpers.vectorDirection(robotVelocity)).getRadians());
    return distanceCost + driveAngleCost;
  }

  public final Comparator<Pose2d> ALIGN_COST_COMPARATOR;

  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final ReefState reefState;

  private final Comparator<ReefPipe> pipeL4Comparator = createReefPipeComparator(ReefPipeLevel.L4);
  private final Comparator<ReefPipe> pipeL3Comparator = createReefPipeComparator(ReefPipeLevel.L3);
  private final Comparator<ReefPipe> pipeL2Comparator = createReefPipeComparator(ReefPipeLevel.L2);

  public AlignmentCostUtil(
      LocalizationSubsystem localization, SwerveSubsystem swerve, ReefState reefState) {
    this.localization = localization;
    this.swerve = swerve;
    this.reefState = reefState;

    ALIGN_COST_COMPARATOR =
        Comparator.comparingDouble(
            pose -> getAlignCost(pose, localization.getPose(), swerve.getFieldRelativeSpeeds()));
  }

  public Comparator<ReefPipe> getReefPipeComparator(ReefPipeLevel level) {
    return switch (level) {
      case L4 -> pipeL4Comparator;
      case L3 -> pipeL3Comparator;
      case L2 -> pipeL2Comparator;
      // Shouldn't ever happen
      default -> pipeL2Comparator;
    };
  }

  /** Helper function to create a singleton comparator for each level. */
  private Comparator<ReefPipe> createReefPipeComparator(ReefPipeLevel level) {
    return Comparator.comparingDouble(
        pipe ->
            getAlignCost(pipe.getPose(level), localization.getPose(), swerve.getTeleopSpeeds())
                + (reefState.isScored(pipe, level) ? 0.2 : 0));
  }
}
