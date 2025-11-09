package frc.robot.auto_align.poses;

import com.team581.util.FmsUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public enum L1Location {
  // 0.165 is half width between pipes
  AB_LEFT(
      new Pose2d(3.71, 4.19, Rotation2d.kZero)
          .transformBy(new Transform2d(0, 0.195, Rotation2d.kZero)),
      new Pose2d(13.84, 3.86, Rotation2d.k180deg)
          .transformBy(new Transform2d(0, 0.195, Rotation2d.kZero))),
  AB_MIDDLE(
      new Pose2d(3.71, 4.19, Rotation2d.kZero)
          .transformBy(new Transform2d(0, -0.165, Rotation2d.kZero)),
      new Pose2d(13.84, 3.86, Rotation2d.k180deg)
          .transformBy(new Transform2d(0, -0.165, Rotation2d.kZero))),
  AB_RIGHT(
      new Pose2d(3.71, 3.86, Rotation2d.kZero)
          .transformBy(new Transform2d(0, -0.195, Rotation2d.kZero)),
      new Pose2d(13.84, 4.19, Rotation2d.k180deg)
          .transformBy(new Transform2d(0, -0.195, Rotation2d.kZero))),
  CD_LEFT(
      new Pose2d(3.96, 3.43, Rotation2d.fromDegrees(60))
          .transformBy(new Transform2d(0, 0.195, Rotation2d.kZero)),
      new Pose2d(13.59, 4.62, Rotation2d.fromDegrees(240))
          .transformBy(new Transform2d(0, 0.195, Rotation2d.kZero))),
  CD_MIDDLE(
      new Pose2d(3.96, 3.43, Rotation2d.fromDegrees(60))
          .transformBy(new Transform2d(0, -0.165, Rotation2d.kZero)),
      new Pose2d(13.59, 4.62, Rotation2d.fromDegrees(240))
          .transformBy(new Transform2d(0, -0.165, Rotation2d.kZero))),
  CD_RIGHT(
      new Pose2d(4.24, 3.27, Rotation2d.fromDegrees(60))
          .transformBy(new Transform2d(0, -0.195, Rotation2d.kZero)),
      new Pose2d(13.31, 4.78, Rotation2d.fromDegrees(240))
          .transformBy(new Transform2d(0, -0.195, Rotation2d.kZero))),
  EF_LEFT(
      new Pose2d(4.74, 3.27, Rotation2d.fromDegrees(120))
          .transformBy(new Transform2d(0, 0.195, Rotation2d.kZero)),
      new Pose2d(12.81, 4.78, Rotation2d.fromDegrees(300))
          .transformBy(new Transform2d(0, 0.195, Rotation2d.kZero))),
  EF_MIDDLE(
      new Pose2d(4.74, 3.27, Rotation2d.fromDegrees(120))
          .transformBy(new Transform2d(0, -0.165, Rotation2d.kZero)),
      new Pose2d(12.81, 4.78, Rotation2d.fromDegrees(300))
          .transformBy(new Transform2d(0, -0.165, Rotation2d.kZero))),
  EF_RIGHT(
      new Pose2d(5.02, 3.43, Rotation2d.fromDegrees(120))
          .transformBy(new Transform2d(0, -0.195, Rotation2d.kZero)),
      new Pose2d(12.53, 4.62, Rotation2d.fromDegrees(300))
          .transformBy(new Transform2d(0, -0.195, Rotation2d.kZero))),
  GH_LEFT(
      new Pose2d(5.27, 3.86, Rotation2d.k180deg)
          .transformBy(new Transform2d(0, 0.195, Rotation2d.kZero)),
      new Pose2d(12.29, 4.19, Rotation2d.kZero)
          .transformBy(new Transform2d(0, 0.195, Rotation2d.kZero))),
  GH_MIDDLE(
      new Pose2d(5.27, 3.86, Rotation2d.k180deg)
          .transformBy(new Transform2d(0, -0.165, Rotation2d.kZero)),
      new Pose2d(12.29, 4.19, Rotation2d.kZero)
          .transformBy(new Transform2d(0, -0.165, Rotation2d.kZero))),
  GH_RIGHT(
      new Pose2d(5.27, 4.19, Rotation2d.k180deg)
          .transformBy(new Transform2d(0, -0.195, Rotation2d.kZero)),
      new Pose2d(12.29, 3.86, Rotation2d.kZero)
          .transformBy(new Transform2d(0, -0.195, Rotation2d.kZero))),
  IJ_LEFT(
      new Pose2d(5.02, 4.62, Rotation2d.fromDegrees(240))
          .transformBy(new Transform2d(0, 0.195, Rotation2d.kZero)),
      new Pose2d(12.53, 3.43, Rotation2d.fromDegrees(60))
          .transformBy(new Transform2d(0, 0.195, Rotation2d.kZero))),
  IJ_MIDDLE(
      new Pose2d(5.02, 4.62, Rotation2d.fromDegrees(240))
          .transformBy(new Transform2d(0, -0.165, Rotation2d.kZero)),
      new Pose2d(12.53, 3.43, Rotation2d.fromDegrees(60))
          .transformBy(new Transform2d(0, -0.165, Rotation2d.kZero))),
  IJ_RIGHT(
      new Pose2d(4.74, 4.78, Rotation2d.fromDegrees(240))
          .transformBy(new Transform2d(0, -0.195, Rotation2d.kZero)),
      new Pose2d(12.81, 3.27, Rotation2d.fromDegrees(60))
          .transformBy(new Transform2d(0, -0.195, Rotation2d.kZero))),
  KL_LEFT(
      new Pose2d(4.24, 4.78, Rotation2d.fromDegrees(300))
          .transformBy(new Transform2d(0, 0.195, Rotation2d.kZero)),
      new Pose2d(13.31, 3.27, Rotation2d.fromDegrees(120))
          .transformBy(new Transform2d(0, 0.195, Rotation2d.kZero))),
  KL_MIDDLE(
      new Pose2d(4.24, 4.78, Rotation2d.fromDegrees(300))
          .transformBy(new Transform2d(0, -0.165, Rotation2d.kZero)),
      new Pose2d(13.31, 3.27, Rotation2d.fromDegrees(120))
          .transformBy(new Transform2d(0, -0.165, Rotation2d.kZero))),
  KL_RIGHT(
      new Pose2d(3.96, 4.62, Rotation2d.fromDegrees(300))
          .transformBy(new Transform2d(0, -0.195, Rotation2d.kZero)),
      new Pose2d(13.59, 3.43, Rotation2d.fromDegrees(120))
          .transformBy(new Transform2d(0, -0.195, Rotation2d.kZero)));

  private final L1Poses redPoses;
  private final L1Poses bluePoses;

  L1Location(Pose2d blueBase, Pose2d redBase) {
    this.redPoses = new L1Poses(redBase);
    this.bluePoses = new L1Poses(blueBase);
  }

  public Pose2d getPose(ReefPipeLevel level) {
    return FmsUtil.isRedAlliance() ? redPoses.getPose(level) : bluePoses.getPose(level);
  }

  public Pose2d getPose(ReefPipeLevel level, boolean isRedAlliance) {
    return isRedAlliance ? redPoses.getPose(level) : bluePoses.getPose(level);
  }

  public Pose2d getPose(ReefPipeLevel level, Pose2d robotPose) {
    return getPose(level, robotPose.getX() > (17.5 / 2));
  }
}
