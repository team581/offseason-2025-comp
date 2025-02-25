package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.MathHelpers;

public enum Points {
  START_1_AND_6(new Pose2d(10.229, 0.758, Rotation2d.kZero)),
  START_2_AND_5(new Pose2d(10.289, 1.903, Rotation2d.kZero)),
  START_3_AND_4(new Pose2d(10.289, 3.054, Rotation2d.kZero)),
  START_4_AND_3(new Pose2d(10.289, 5.069, Rotation2d.kZero)),
  START_5_AND_2(new Pose2d(10.289, 6.127, Rotation2d.kZero)),
  START_6_AND_1(new Pose2d(10.289, 7.292, Rotation2d.kZero)),

  LEFT_CORAL_STATION(new Pose2d(16.194, 0.861, Rotation2d.fromDegrees(125.309))),
  RIGHT_CORAL_STATION(new Pose2d(16.1, 7.35, Rotation2d.fromDegrees(-131.807)));

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
