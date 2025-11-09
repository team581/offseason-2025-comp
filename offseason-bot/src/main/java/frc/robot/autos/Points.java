package frc.robot.autos;

import com.team581.math.MathHelpers;
import com.team581.util.FmsUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum Points {
  NON_PROCESSOR_SIDE_START_ANGLED(
      new Pose2d(10.546, 1.782, Rotation2d.fromDegrees(60.0)),
      new Pose2d(7.042, 5.976, Rotation2d.fromDegrees(240.0))),

  PROCESSOR_SIDE_START_ANGLED(
      new Pose2d(10.546, 6.218, Rotation2d.fromDegrees(300.0)),
      new Pose2d(7.042, 2.024, Rotation2d.fromDegrees(120.0))),

  START_R1_AND_B1(new Pose2d(10.289, 0.47, Rotation2d.fromDegrees(90.0))),
  START_R1_AND_B1_FORWARD(new Pose2d(10.289, 0.758, Rotation2d.kZero)),

  START_R2_AND_B2(new Pose2d(10.289, 1.903, Rotation2d.kZero)),
  START_R3_AND_B3(new Pose2d(10.289, 3.054, Rotation2d.kZero)),
  START_R3_AND_B3_LEFT_FORWARD(new Pose2d(10.289, 3.054, Rotation2d.kCW_90deg)),

  START_R4_AND_B4(new Pose2d(10.289, 5.069, Rotation2d.kZero)),
  START_R5_AND_B5(new Pose2d(10.289, 6.127, Rotation2d.kZero)),
  START_R6_AND_B6_FORWARD(new Pose2d(10.289, 7.292, Rotation2d.kZero)),
  START_R6_AND_B6(new Pose2d(10.289, 7.58, Rotation2d.fromDegrees(90.0))),

  GROUND_INTAKE_NON_PROCESSOR_SIDE_STATION(new Pose2d(16.117, 0.922, Rotation2d.fromDegrees(126.0))),
  PRE_GROUND_INTAKE_NON_PROCESSOR_SIDE_STATION(
      new Pose2d(14.9, 1.7, Rotation2d.fromDegrees(126.0))),

  GROUND_INTAKE_PROCESSOR_SIDE_STATION(new Pose2d(16.117, 7.078, Rotation2d.fromDegrees(234.0))),
  PRE_GROUND_INTAKE_PROCESSOR_SIDE_STATION(new Pose2d(14.9, 6.3, Rotation2d.fromDegrees(234.0))),

  LOLLIPOP_2(new Pose2d(16.125, 4.0, Rotation2d.fromDegrees(180.0)));
  // -LP 2 pose, intaking state

  public final Pose2d redPose;
  public final Pose2d bluePose;

  Points(Pose2d redPose, Pose2d bluePose) {
    this.redPose = redPose;
    this.bluePose = bluePose;
  }

  Points(Pose2d redPose) {
    this(redPose, MathHelpers.pathflip(redPose));
  }

  public Pose2d getPose() {
    return FmsUtil.isRedAlliance() ? redPose : bluePose;
  }
}
