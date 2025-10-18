package frc.robot.autos;

import com.team581.math.MathHelpers;
import com.team581.util.FmsUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum Points {
  START_R1_AND_B1(new Pose2d(10.289, 0.47, Rotation2d.fromDegrees(90))),
  START_R1_AND_B1_FORWARD(new Pose2d(10.289, 0.758, Rotation2d.kZero)),

  START_R2_AND_B2(new Pose2d(10.289, 1.903, Rotation2d.kZero)),
  START_R3_AND_B3(new Pose2d(10.289, 3.054, Rotation2d.kZero)),
  START_R3_AND_B3_LEFT_FORWARD(new Pose2d(10.289, 3.054, Rotation2d.kCW_90deg)),

  START_R4_AND_B4(new Pose2d(10.289, 5.069, Rotation2d.kZero)),
  START_R5_AND_B5(new Pose2d(10.289, 6.127, Rotation2d.kZero)),
  START_R6_AND_B6_FORWARD(new Pose2d(10.289, 7.292, Rotation2d.kZero)),
  START_R6_AND_B6(new Pose2d(10.289, 7.58, Rotation2d.fromDegrees(90))),

  LEFT_CORAL_STATION(new Pose2d(16.194, 0.861, Rotation2d.fromDegrees(125.309))),
  RIGHT_CORAL_STATION(new Pose2d(16.1, 7.35, Rotation2d.fromDegrees(-131.807))),

  GROUND_INTAKE_LEFT_STATION(new Pose2d(16.124, 1.148, Rotation2d.fromDegrees(0))),

  INTAKING(new Pose2d(1.710, 7.079, Rotation2d.fromDegrees(-48.447))),
  // -Pose by station, robot state intaking
  PRE_GROUND_INTAKE_LEFT_STATION(new Pose2d(2.565, 6.265, Rotation2d.fromDegrees(-48.447))),
  // -Pose before getting to station, robot state idle

  LOLLIPOP_2(new Pose2d(1.547, 3.964, Rotation2d.fromDegrees(0))),
  // -LP 2 pose, intaking state
  PRE_LOLLIPOP_2(new Pose2d(2.28, 3.964, Rotation2d.fromDegrees(0))),
  // -away from LP 2 pose, idle state

  PRE_I_L4(new Pose2d(6.0, 5.5, Rotation2d.fromDegrees(240))),
  PRE_J_L4(new Pose2d(5.5, 5.83, Rotation2d.fromDegrees(240))),
  // -robot idle state, a little before I/J pose
  PRE_K_L4(new Pose2d(3.75, 5.75, Rotation2d.fromDegrees(300))),
  PRE_L_L4(new Pose2d(3.441, 5.471, Rotation2d.fromDegrees(300))),
  // -robot intaking state (until singulate coral), a little before K/L pose
  PRE_A_L4(new Pose2d(2.9, 4.19, Rotation2d.kZero)),
  PRE_B_L4(new Pose2d(2.9, 3.86, Rotation2d.kZero)),
  // -robot intaking state (until singulate coral), a little before A/B pose



  I_L4_POSE(new Pose2d(5.02, 4.62, Rotation2d.fromDegrees(240)),new Pose2d(12.53, 3.43, Rotation2d.fromDegrees(60))),
  J_L4_POSE(new Pose2d(4.74, 4.78, Rotation2d.fromDegrees(240)),new Pose2d(12.81, 3.27, Rotation2d.fromDegrees(60))),
  K_L_POSE(new Pose2d(4.24, 4.78, Rotation2d.fromDegrees(300)),new Pose2d(13.31, 3.27, Rotation2d.fromDegrees(120))),
  L_L4_POSE(new Pose2d(3.96, 4.62, Rotation2d.fromDegrees(300)),new Pose2d(13.59, 3.43, Rotation2d.fromDegrees(120))),
  A_L4_POSE(new Pose2d(3.71, 4.19, Rotation2d.kZero),new Pose2d(13.84, 3.86, Rotation2d.k180deg)),
  B_L4_POSE(new Pose2d(3.71, 3.86, Rotation2d.kZero),new Pose2d(13.84, 4.19, Rotation2d.k180deg));
//first is red second is blue
  // PIPE_A(new Pose2d(3.71, 4.19, Rotation2d.kZero), new Pose2d(13.84, 3.86, Rotation2d.k180deg)),
  // PIPE_B(new Pose2d(3.71, 3.86, Rotation2d.kZero), new Pose2d(13.84, 4.19, Rotation2d.k180deg)),
  // PIPE_C(
  //     new Pose2d(3.96, 3.43, Rotation2d.fromDegrees(60)),
  //     new Pose2d(13.59, 4.62, Rotation2d.fromDegrees(240))),
  // PIPE_D(
  //     new Pose2d(4.24, 3.27, Rotation2d.fromDegrees(60)),
  //     new Pose2d(13.31, 4.78, Rotation2d.fromDegrees(240))),
  // PIPE_E(
  //     new Pose2d(4.74, 3.27, Rotation2d.fromDegrees(120)),
  //     new Pose2d(12.81, 4.78, Rotation2d.fromDegrees(300))),
  // PIPE_F(
  //     new Pose2d(5.02, 3.43, Rotation2d.fromDegrees(120)),
  //     new Pose2d(12.53, 4.62, Rotation2d.fromDegrees(300))),
  // PIPE_G(new Pose2d(5.27, 3.86, Rotation2d.k180deg), new Pose2d(12.29, 4.19, Rotation2d.kZero)),
  // PIPE_H(new Pose2d(5.27, 4.19, Rotation2d.k180deg), new Pose2d(12.29, 3.86, Rotation2d.kZero)),
  // PIPE_I(
  //     new Pose2d(5.02, 4.62, Rotation2d.fromDegrees(240)),
  //     new Pose2d(12.53, 3.43, Rotation2d.fromDegrees(60))),
  // PIPE_J(
  //     new Pose2d(4.74, 4.78, Rotation2d.fromDegrees(240)),
  //     new Pose2d(12.81, 3.27, Rotation2d.fromDegrees(60))),
  // PIPE_K(
  //     new Pose2d(4.24, 4.78, Rotation2d.fromDegrees(300)),
  //     new Pose2d(13.31, 3.27, Rotation2d.fromDegrees(120))),
  // PIPE_L(
  //     new Pose2d(3.96, 4.62, Rotation2d.fromDegrees(300)),
  //     new Pose2d(13.59, 3.43, Rotation2d.fromDegrees(120)));

  public Pose2d redPose;
  public Pose2d bluePose;

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
