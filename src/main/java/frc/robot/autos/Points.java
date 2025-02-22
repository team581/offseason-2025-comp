package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.MathHelpers;

public enum Points {
  START_1_AND_6(Pose2d.kZero),
  START_2_AND_5(new Pose2d(10.289, 1.903, Rotation2d.kZero)),
  START_3_AND_4(Pose2d.kZero),
  START_4_AND_3(Pose2d.kZero),
  START_5_AND_2(Pose2d.kZero),
  START_6_AND_1(Pose2d.kZero),

  LEFT_CORAL_STATION(new Pose2d(16.1, 0.7, Rotation2d.fromDegrees(127.71))),
  RIGHT_CORAL_STATION(Pose2d.kZero);

  public Pose2d redPose;
  public Pose2d bluePose;

  Points(Pose2d redPose, Pose2d bluePose) {
    this.redPose = redPose;
    this.bluePose = bluePose;
  }

  Points(Pose2d redPose) {
    this(redPose, MathHelpers.pathflip(redPose));
  }
}
