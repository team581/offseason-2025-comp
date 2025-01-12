package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;

public enum ReefSide {
  // TODO:set poses
  SIDE_AB(new Pose2d(), new Pose2d()),
  SIDE_CD(new Pose2d(), new Pose2d()),
  SIDE_EF(new Pose2d(), new Pose2d()),
  SIDE_GH(new Pose2d(), new Pose2d()),
  SIDE_IJ(new Pose2d(), new Pose2d()),
  SIDE_KL(new Pose2d(), new Pose2d());

  // add a pose of where this side is considered to be
  // used to compare with robot pose to choose closest side
  public final Pose2d bluePose;
  public final Pose2d redPose;

  private ReefSide(Pose2d bluePose, Pose2d redPose) {
    this.bluePose = bluePose;
    this.redPose = redPose;
  }
}
