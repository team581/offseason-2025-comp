package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;

// Compute the offset poses for scoring on the reef once on boot to reduce GC pressure from dynamic
// computation
record ReefPipePoses(
    Pose2d base,
    Pose2d l1,
    Pose2d l2,
    Pose2d l3,
    Pose2d l4,
    Pose2d raising,
    Pose2d backaway) {
  public ReefPipePoses(Pose2d base) {
    this(
        base,
        base.transformBy(ReefPipeLevel.L1.transform),
        base.transformBy(ReefPipeLevel.L2.transform),
        base.transformBy(ReefPipeLevel.L3.transform),
        base.transformBy(ReefPipeLevel.L4.transform),
        base.transformBy(ReefPipeLevel.RAISING.transform),
        base.transformBy(ReefPipeLevel.BACK_AWAY.transform));
  }

  public Pose2d getPose(ReefPipeLevel level) {
    return switch (level) {
      case BASE -> base;
      case L1 -> l1;
      case L2 -> l2;
      case L3 -> l3;
      case L4 -> l4;
      case RAISING -> raising;
      case BACK_AWAY -> backaway;
    };
  }
}
