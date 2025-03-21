package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public enum ReefPipeLevel {
  BASE(new Transform2d(0,0,Rotation2d.fromDegrees(90)),new Transform2d(0,0,Rotation2d.fromDegrees(270))),
  L1(new Transform2d(-0.60, 0, Rotation2d.fromDegrees(90)), new Transform2d(-0.60, 0, Rotation2d.fromDegrees(270))),
  L2(new Transform2d(-0.60, 0, Rotation2d.fromDegrees(90)),new Transform2d(-0.60, 0, Rotation2d.fromDegrees(270))),
  L3(new Transform2d(-0.60, 0, Rotation2d.fromDegrees(90)),new Transform2d(-0.60, 0, Rotation2d.fromDegrees(270))),
  L4(new Transform2d(-0.64635, 0, Rotation2d.fromDegrees(90)),new Transform2d(-0.64635, 0, Rotation2d.fromDegrees(270)));

  public final Transform2d leftOffset;
  public final Transform2d rightOffset;

  private ReefPipeLevel(Transform2d leftOffset, Transform2d rightOffset) {
  this.leftOffset = leftOffset;
  this.rightOffset = rightOffset;
  }

}
