package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.fms.FmsSubsystem;

public enum ReefPipe {
  PIPE_A(
      new Pose2d(3.6982, 3.8616, Rotation2d.fromDegrees(0.0)),
      new Pose2d(13.8254, 3.8617, Rotation2d.fromDegrees(180.0))),
  PIPE_B(
      new Pose2d(3.6982, 4.1902, Rotation2d.fromDegrees(0.0)),
      new Pose2d(13.8254, 4.1901, Rotation2d.fromDegrees(180.0))),
  PIPE_C(
      new Pose2d(3.9463, 4.6199, Rotation2d.fromDegrees(300.000281)),
      new Pose2d(13.5774, 4.62008, Rotation2d.fromDegrees((240.00014)))),
  PIPE_D(
      new Pose2d(4.2309, 4.7842, Rotation2d.fromDegrees(300.000281)),
      new Pose2d(13.2928, 4.7843, Rotation2d.fromDegrees(240.00014))),
  PIPE_E(
      new Pose2d(4.7270, 4.7842, Rotation2d.fromDegrees(240.00014)),
      new Pose2d(12.7966, 4.7843, Rotation2d.fromDegrees(300.000281))),
  PIPE_F(
      new Pose2d(5.0116, 4.6199, Rotation2d.fromDegrees(240.00014)),
      new Pose2d(12.5120, 4.6200, Rotation2d.fromDegrees(300.000281))),
  PIPE_G(
      new Pose2d(5.2597, 4.1902, Rotation2d.fromDegrees(180.0)),
      new Pose2d(12.2640, 4.1903, Rotation2d.fromDegrees(0.0))),
  PIPE_H(
      new Pose2d(5.2597, 3.8616, Rotation2d.fromDegrees(180.0)),
      new Pose2d(12.2640, 3.8617, Rotation2d.fromDegrees(180.0))),
  PIPE_I(
      new Pose2d(5.0116, 3.4319, Rotation2d.fromDegrees(119.99986)),
      new Pose2d(12.5120, 3.4320, Rotation2d.fromDegrees(59.999719))),
  PIPE_J(
      new Pose2d(4.7270, 3.2676, Rotation2d.fromDegrees(119.99986)),
      new Pose2d(12.7966, 3.2677, Rotation2d.fromDegrees(59.999719))),
  PIPE_K(
      new Pose2d(4.2309, 3.2676, Rotation2d.fromDegrees(59.999719)),
      new Pose2d(13.2928, 3.2677, Rotation2d.fromDegrees(119.99986))),
  PIPE_L(
      new Pose2d(3.9463, 3.4319, Rotation2d.fromDegrees(59.999719)),
      new Pose2d(13.5774, 3.4320, Rotation2d.fromDegrees(119.99986)));

  private static final Pose2d L1Offset = new Pose2d(-0.6, 0, Rotation2d.kZero);
  private static final Pose2d L2Offset = new Pose2d(-0.6, 0, Rotation2d.kZero); // -0.8382 might be a good idea
  private static final Pose2d L3Offset = new Pose2d(-0.6, 0, Rotation2d.kZero);
  private static final Pose2d L4Offset = new Pose2d(-0.6, 0, Rotation2d.kZero);

  public final Pose2d redPose;
  public final Pose2d bluePose;

  ReefPipe(Pose2d bluePose, Pose2d redPose) {
    this.redPose = redPose;
    this.bluePose = bluePose;
  }

  public Pose2d getPose(ReefPipeLevel level, boolean isRedAlliance) {
    var basePipePose = isRedAlliance ? redPose : bluePose;

    var offset =
        switch (level) {
          case BASE -> new Pose2d();
          case L1 -> L1Offset;
          case L2 -> L2Offset;
          case L3 -> L3Offset;
          case L4 -> L4Offset;
        };

    return new Pose2d(
        basePipePose
            .getTranslation()
            .plus(offset.getTranslation().rotateBy(basePipePose.getRotation())),
        basePipePose.getRotation());
  }

  public Pose2d getPose(ReefPipeLevel level) {
    return getPose(level, FmsSubsystem.isRedAlliance());
  }
}
