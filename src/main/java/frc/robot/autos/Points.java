package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.MathHelpers;

public enum Points {
  AUTO_START_1(Pose2d.kZero),
  AUTO_START_2(new Pose2d(10.289, 1.903, Rotation2d.kZero)),
  AUTO_START_3(Pose2d.kZero),

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
