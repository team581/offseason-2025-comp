package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.fms.FmsSubsystem;

public enum ReefSide {
  SIDE_AB(reefSidePoses(0), fieldFlip(reefSidePoses(0))),
  SIDE_CD(reefSidePoses(1), fieldFlip(reefSidePoses(1))),
  SIDE_EF(reefSidePoses(2), fieldFlip(reefSidePoses(2))),
  SIDE_GH(reefSidePoses(3), fieldFlip(reefSidePoses(3))),
  SIDE_IJ(reefSidePoses(4), fieldFlip(reefSidePoses(4))),
  SIDE_KL(reefSidePoses(5), fieldFlip(reefSidePoses(5)));

  // add a pose of where this side is considered to be
  // used to compare with robot pose to choose closest side
  public final Pose2d bluePose;
  public final Pose2d redPose;

  public static Pose2d getFieldDims() {
    return new Pose2d(17.5482504, 8.0518, new Rotation2d());
  }

  public static Pose2d reefSidePoses(int i) {
    Pose2d[] poses = {
      new Pose2d(3.658, 4.026, Rotation2d.fromRadians(3.142)),
      new Pose2d(4.073, 4.746, Rotation2d.fromRadians(2.094)),
      new Pose2d(4.905, 4.746, Rotation2d.fromRadians(1.047)),
      new Pose2d(5.321, 4.026, Rotation2d.fromRadians(0)),
      new Pose2d(4.905, 3.306, Rotation2d.fromRadians(-1.047)),
      new Pose2d(4.074, 3.306, Rotation2d.fromRadians(-2.094))
    };
    return poses[i];
  }

  public static Pose2d fieldFlip(Pose2d flipPose) {
    return new Pose2d(
        getFieldDims().getX() - flipPose.getX(),
        getFieldDims().getY() - flipPose.getY(),
        flipPose.getRotation().plus(Rotation2d.fromRotations(0.5)));
  }

  private ReefSide(Pose2d bluePose, Pose2d redPose) {
    this.bluePose = bluePose;
    this.redPose = redPose;
  }

  public Pose2d getPose() {
    return FmsSubsystem.isRedAlliance() ? redPose : bluePose;
  }
}
