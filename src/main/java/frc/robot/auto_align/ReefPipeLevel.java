package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public enum ReefPipeLevel {
  BASE(Transform2d.kZero),
  L1(new Transform2d(-0.60, 0, Rotation2d.kZero)),
  L2(new Transform2d(-0.60, 0, Rotation2d.kZero)),
  L3(new Transform2d(-0.60, 0, Rotation2d.kZero)),
  L4(new Transform2d(-0.64635, 0, Rotation2d.kZero));

  public final Transform2d offset;

  private ReefPipeLevel(Transform2d offset) {
    this.offset = offset;
  }
}
