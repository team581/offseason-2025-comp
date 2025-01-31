package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.fms.FmsSubsystem;
import java.util.List;

public enum ReefSide {
  SIDE_AB(
      new Pose2d(3.658, 4.026, Rotation2d.fromRadians(0.0)),
      new Pose2d(13.89, 4.026, Rotation2d.fromRadians(-Math.PI)),
      ReefPipe.PIPE_A,
      ReefPipe.PIPE_B),
  SIDE_CD(
      new Pose2d(4.073, 4.746, Rotation2d.fromRadians(-1.0472)),
      new Pose2d(13.475, 3.306, Rotation2d.fromRadians(2.0944)),
      ReefPipe.PIPE_C,
      ReefPipe.PIPE_D),
  SIDE_EF(
      new Pose2d(4.905, 4.746, Rotation2d.fromRadians(-2.0944)),
      new Pose2d(12.643, 3.306, Rotation2d.fromRadians(1.0472)),
      ReefPipe.PIPE_E,
      ReefPipe.PIPE_F),
  SIDE_GH(
      new Pose2d(5.321, 4.026, Rotation2d.fromRadians(Math.PI)),
      new Pose2d(12.227, 4.026, Rotation2d.fromRadians(0.0)),
      ReefPipe.PIPE_G,
      ReefPipe.PIPE_H),
  SIDE_IJ(
      new Pose2d(4.905, 3.306, Rotation2d.fromRadians(2.0944)),
      new Pose2d(12.643, 4.746, Rotation2d.fromRadians(-1.0472)),
      ReefPipe.PIPE_I,
      ReefPipe.PIPE_J),
  SIDE_KL(
      new Pose2d(4.074, 3.306, Rotation2d.fromRadians(1.0472)),
      new Pose2d(13.474, 4.746, Rotation2d.fromRadians(-2.0944)),
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
