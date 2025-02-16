package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;

// Compute the offset poses for scoring on the reef once on boot to reduce GC pressure from dynamic
// computation
record ReefPipePoses(Pose2d base, Pose2d L1, Pose2d L2, Pose2d L3, Pose2d L4) {
  private static Pose2d offset(Pose2d base, ReefPipeLevel level) {
    return new Pose2d(
        base.getTranslation().plus(level.offset.getTranslation().rotateBy(base.getRotation())),
        base.getRotation());
  }

  public ReefPipePoses(Pose2d base) {
    this(
        base,
        offset(base, ReefPipeLevel.L1),
        offset(base, ReefPipeLevel.L2),
        offset(base, ReefPipeLevel.L3),
        offset(base, ReefPipeLevel.L4));
  }

  public Pose2d getPose(ReefPipeLevel level) {
    return switch (level) {
      case BASE -> base;
      case L1 -> L1;
      case L2 -> L2;
      case L3 -> L3;
      case L4 -> L4;
    };
  }
}
