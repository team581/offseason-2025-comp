package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public enum ReefSideOffset {
  BASE(
      new Transform2d(0, 0, Rotation2d.fromDegrees(0))),
  SAFE(
      new Transform2d(
          -Units.inchesToMeters(14.5 + 4.0 + 20.0),
          0.0,
          Rotation2d.kZero)),
  ALGAE_INTAKING(
      new Transform2d(
          // Half of drivebase + bumper side width + reef side to bumper distance
          -Units.inchesToMeters(14.5 + 4.0 + 5.0),
          0.0,
          Rotation2d.kZero));

  public final Transform2d offset;

  private ReefSideOffset(Transform2d offset) {
    this.offset = offset;
  }
}
