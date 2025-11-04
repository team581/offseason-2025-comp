package frc.robot.auto_align.poses;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public enum ReefPipeLevel {
  BASE(Transform2d.kZero),
  L1(new Transform2d(-Units.inchesToMeters(14.5 + 5.0 + 2.0), 0.0, Rotation2d.kZero)),
  L2(new Transform2d(-Units.inchesToMeters(14.5 + 8.25 + 2.0), 0.0, Rotation2d.kZero)),
  L3(new Transform2d(-Units.inchesToMeters(14.5 + 8.25 + 2.0), 0.0, Rotation2d.kZero)),
  L4(new Transform2d(-Units.inchesToMeters(14.5 + 8.25 + 2.0), 0.0, Rotation2d.kZero)),

  RAISING(new Transform2d(-Units.inchesToMeters(14.5 + 4.0 + 2.0 + 15), 0.0, Rotation2d.kZero)),
  BACK_AWAY(new Transform2d(-Units.inchesToMeters(14.5 + 4.0 + 2.0 + 15), 0.0, Rotation2d.kZero));

  public final Transform2d transform;

  private ReefPipeLevel(Transform2d transform) {
    this.transform = transform;
  }
}
