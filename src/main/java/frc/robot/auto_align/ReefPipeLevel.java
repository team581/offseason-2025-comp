package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum ReefPipeLevel {
  BASE(Pose2d.kZero),
  L1(new Pose2d(-0.62, 0, Rotation2d.kZero)),
  L2(new Pose2d(-0.62, 0, Rotation2d.kZero)),
  L3(new Pose2d(-0.62, 0, Rotation2d.kZero)),
  L4(new Pose2d(-0.62, 0, Rotation2d.kZero));

  public final Pose2d offset;

  private ReefPipeLevel(Pose2d offset) {
    this.offset = offset;
  }
}
