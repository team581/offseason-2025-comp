package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.fms.FmsSubsystem;

public enum ReefSide {
  SIDE_AB(
      new Pose2d(3.658, 4.026, Rotation2d.fromRadians(0.0)),
      new Pose2d(13.89, 4.026, Rotation2d.fromRadians(-Math.PI))),
  SIDE_CD(
      new Pose2d(4.073, 4.746, Rotation2d.fromRadians(-1.0472)),
      new Pose2d(13.475, 3.306, Rotation2d.fromRadians(2.0944))),
  SIDE_EF(
      new Pose2d(4.905, 4.746, Rotation2d.fromRadians(-2.0944)),
      new Pose2d(12.643, 3.306, Rotation2d.fromRadians(1.0472))),
  SIDE_GH(
      new Pose2d(5.321, 4.026, Rotation2d.fromRadians(Math.PI)),
      new Pose2d(12.227, 4.026, Rotation2d.fromRadians(0.0))),
  SIDE_IJ(
      new Pose2d(4.905, 3.306, Rotation2d.fromRadians(2.0944)),
      new Pose2d(12.643, 4.746, Rotation2d.fromRadians(-1.0472))),
  SIDE_KL(
      new Pose2d(4.074, 3.306, Rotation2d.fromRadians(1.0472)),
      new Pose2d(13.474, 4.746, Rotation2d.fromRadians(-2.0944)));

  // pose of where this side is considered to be
  // used to compare with robot pose to choose closest side
  public final Pose2d bluePose;
  public final Pose2d redPose;

  private ReefSide(Pose2d bluePose, Pose2d redPose) {
    this.bluePose = bluePose;
    this.redPose = redPose;
  }

  public Pose2d getPose(boolean isRedAlliance) {
    return isRedAlliance ? redPose : bluePose;
  }

  public Pose2d getPose() {
    return getPose(FmsSubsystem.isRedAlliance());
  }
}
