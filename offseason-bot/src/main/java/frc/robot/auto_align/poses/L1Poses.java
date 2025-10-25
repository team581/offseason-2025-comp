package frc.robot.auto_align.poses;

import edu.wpi.first.math.geometry.Pose2d;

// Compute the offset poses for scoring on the reef once on boot to reduce GC pressure from dynamic
// computation
record L1Poses(Pose2d base, Pose2d l1, Pose2d raising, Pose2d backaway) {
  public L1Poses(Pose2d base) {
    this(
        base,
        base.transformBy(ReefPipeLevel.L1.transform),
        base.transformBy(ReefPipeLevel.RAISING.transform),
        base.transformBy(ReefPipeLevel.BACK_AWAY.transform));
  }

  public Pose2d getPose(ReefPipeLevel level) {
    return switch (level) {
      case BASE -> base;
      case L1 -> l1;
      case RAISING -> raising;
      case BACK_AWAY -> backaway;
      default -> base;
    };
  }
}
