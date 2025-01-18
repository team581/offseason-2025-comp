package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.fms.FmsSubsystem;

public enum ReefPipe {
  PIPE_A(
      new Pose2d(3.6982, 3.8616, new Rotation2d(Math.PI)),
      new Pose2d(13.8254, 4.1901, new Rotation2d(0.0))),
  PIPE_B(
      new Pose2d(3.6982, 4.1902, new Rotation2d(Math.PI)),
      new Pose2d(13.8254, 3.8617, new Rotation2d(0.0))),
  PIPE_C(
      new Pose2d(3.9463, 4.6199, new Rotation2d(2.0944)),
      new Pose2d(13.5774, 3.4320, new Rotation2d(-1.0472))),
  PIPE_D(
      new Pose2d(4.2309, 4.7842, new Rotation2d(2.0944)),
      new Pose2d(13.2928, 3.2677, new Rotation2d(-1.0472))),
  PIPE_E(
      new Pose2d(4.7270, 4.7842, new Rotation2d(1.0472)),
      new Pose2d(12.7966, 3.2677, new Rotation2d(-2.0944))),
  PIPE_F(
      new Pose2d(5.0116, 4.6199, new Rotation2d(1.0472)),
      new Pose2d(12.5120, 3.4320, new Rotation2d(-2.0944))),
  PIPE_G(
      new Pose2d(5.2597, 4.1902, new Rotation2d(0.0)),
      new Pose2d(12.2640, 3.8617, new Rotation2d(Math.PI))),
  PIPE_H(
      new Pose2d(5.2597, 3.8616, new Rotation2d(0.0)),
      new Pose2d(12.2640, 4.1903, new Rotation2d(Math.PI))),
  PIPE_I(
      new Pose2d(5.0116, 3.4319, new Rotation2d(-1.0472)),
      new Pose2d(12.5120, 4.6200, new Rotation2d(2.0944))),
  PIPE_J(
      new Pose2d(4.7270, 3.2676, new Rotation2d(-1.0472)),
      new Pose2d(12.7966, 4.7843, new Rotation2d(2.0944))),
  PIPE_K(
      new Pose2d(4.2309, 3.2676, new Rotation2d(-2.0944)),
      new Pose2d(13.2928, 4.7843, new Rotation2d(1.0472))),
  PIPE_L(
      new Pose2d(3.9463, 3.4319, new Rotation2d(-2.0944)),
      new Pose2d(13.5774, 4.62008, new Rotation2d(1.0472)));

  // These are the poses for the tip of each vertical pipe on the reef
  // PIPE_A is on the reef side closest to alliance area, it is the leftmost bottom pipe
  // The following pipes are in order clockwise, with PIPE_B being above PIPE_A, and PIPE_C on the
  // following reef side
  // For each pipe there has been 2 poses inputted, first one is the pipes on the reef of the blue
  // side of the field and the red side

  public final Pose2d redPose;
  public final Pose2d bluePose;

  ReefPipe(Pose2d bluePose, Pose2d redPose) {
    this.redPose = redPose;
    this.bluePose = bluePose;
  }

  public Pose2d getPose() {
    return (FmsSubsystem.isRedAlliance() ? redPose : bluePose);
  }
}
