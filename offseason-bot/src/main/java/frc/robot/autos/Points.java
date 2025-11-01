package frc.robot.autos;

import com.team581.math.MathHelpers;
import com.team581.util.FmsUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.auto_align.poses.ReefPipe;
import frc.robot.auto_align.poses.ReefPipeLevel;

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

  GROUND_INTAKE_LEFT_STATION(
      new Pose2d(16.124, 1.148, Rotation2d.fromDegrees(131.807)),
      new Pose2d(16.1, 7.35, Rotation2d.fromDegrees(131.807))),

  INTAKING(new Pose2d(1.710, 7.079, Rotation2d.fromDegrees(-48.447))),
  // -Pose by station, robot state intaking
  PRE_GROUND_INTAKE_LEFT_STATION(
      new Pose2d(15.1, 2.0, Rotation2d.fromDegrees(131.807)),
      new Pose2d(2.565, 6.265, Rotation2d.fromDegrees(-48.447))),
  // -Pose before getting to station, robot state idle

  LOLLIPOP_2(new Pose2d(16.0, 3.964, Rotation2d.fromDegrees(180))),
  // -LP 2 pose, intaking state
  PRE_LOLLIPOP_2(new Pose2d(16.0, 3.964, Rotation2d.fromDegrees(180))),
  // -away from LP 2 pose, idle state

  PRE_I_L4(ReefPipe.PIPE_I.getPose(ReefPipeLevel.RAISING)),
  PRE_J_L4(ReefPipe.PIPE_J.getPose(ReefPipeLevel.RAISING)),
  // -robot idle state, a little before I/J pose
  PRE_K_L4(ReefPipe.PIPE_K.getPose(ReefPipeLevel.RAISING)),
  PRE_L_L4(ReefPipe.PIPE_L.getPose(ReefPipeLevel.RAISING)),
  // -robot intaking state (until singulate coral), a little before K/L pose
  PRE_A_L4(ReefPipe.PIPE_A.getPose(ReefPipeLevel.RAISING)),
  PRE_B_L4(ReefPipe.PIPE_B.getPose(ReefPipeLevel.RAISING)),
  // -robot intaking state (until singulate coral), a little before A/B pose

  I_L4_POSE(ReefPipe.PIPE_I.getPose(ReefPipeLevel.L4)),
  J_L4_POSE(ReefPipe.PIPE_J.getPose(ReefPipeLevel.L4)),
  K_L_POSE(ReefPipe.PIPE_K.getPose(ReefPipeLevel.L4)),
  L_L4_POSE(ReefPipe.PIPE_L.getPose(ReefPipeLevel.L4)),
  A_L4_POSE(ReefPipe.PIPE_A.getPose(ReefPipeLevel.L4)),
  B_L4_POSE(ReefPipe.PIPE_B.getPose(ReefPipeLevel.L4));

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
