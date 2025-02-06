package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.fms.FmsSubsystem;
import java.util.List;

public enum ReefSide {
  SIDE_AB(
      new Pose2d(Units.inchesToMeters(144.003), Units.inchesToMeters(158.500), Rotation2d.kZero),
      new Pose2d(Units.inchesToMeters(546.873), Units.inchesToMeters(158.5), Rotation2d.k180deg),
      ReefPipe.PIPE_A,
      ReefPipe.PIPE_B),
  SIDE_CD(
      new Pose2d(
          Units.inchesToMeters(160.375), Units.inchesToMeters(130.144), Rotation2d.fromDegrees(60)),
      new Pose2d(
          Units.inchesToMeters(530.501),
          Units.inchesToMeters(186.856),
          Rotation2d.fromDegrees(240)),
      ReefPipe.PIPE_C,
      ReefPipe.PIPE_D),
  SIDE_EF(
      new Pose2d(
          Units.inchesToMeters(193.118),
          Units.inchesToMeters(130.145),
          Rotation2d.fromDegrees(120)),
      new Pose2d(
          Units.inchesToMeters(497.758),
          Units.inchesToMeters(186.855),
          Rotation2d.fromDegrees(300)),
      ReefPipe.PIPE_E,
      ReefPipe.PIPE_F),
  SIDE_GH(
      new Pose2d(Units.inchesToMeters(209.489), Units.inchesToMeters(158.502), Rotation2d.k180deg),
      new Pose2d(Units.inchesToMeters(481.387), Units.inchesToMeters(158.498), Rotation2d.kZero),
      ReefPipe.PIPE_G,
      ReefPipe.PIPE_H),
  SIDE_IJ(
      new Pose2d(
          Units.inchesToMeters(193.116),
          Units.inchesToMeters(186.858),
          Rotation2d.fromDegrees(240)),
      new Pose2d(
          Units.inchesToMeters(497.76), Units.inchesToMeters(130.142), Rotation2d.fromDegrees(60)),
      ReefPipe.PIPE_I,
      ReefPipe.PIPE_J),
  SIDE_KL(
      new Pose2d(
          Units.inchesToMeters(160.373),
          Units.inchesToMeters(186.857),
          Rotation2d.fromDegrees(300)),
      new Pose2d(
          Units.inchesToMeters(530.503),
          Units.inchesToMeters(130.143),
          Rotation2d.fromDegrees(120)),
      ReefPipe.PIPE_K,
      ReefPipe.PIPE_L);

  public final Pose2d bluePose;
  public final Pose2d redPose;
  public final ReefPipe pipe1;
  public final ReefPipe pipe2;

  private ReefSide(Pose2d bluePose, Pose2d redPose, ReefPipe pipe1, ReefPipe pipe2) {
    this.bluePose = bluePose;
    this.redPose = redPose;
    this.pipe1 = pipe1;
    this.pipe2 = pipe2;
  }

  public Pose2d getPose(boolean isRedAlliance) {
    return isRedAlliance ? redPose : bluePose;
  }

  public Pose2d getPose() {
    return getPose(FmsSubsystem.isRedAlliance());
  }

  public List<Pose2d> getPipes(ReefPipeLevel level) {
    return List.of(pipe1.getPose(level), pipe2.getPose(level));
  }
}
