package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.robot_manager.RobotManager;

public class RedFivePiece2JKLABAuto extends BaseAuto {
  public RedFivePiece2JKLABAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R2_AND_B2.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        blocks.scoreL4(ReefPipe.PIPE_J, RobotScoringSide.LEFT),
        blocks.intakeGroundForL4(new Pose2d(12.978, 2.132, Rotation2d.fromDegrees(-18))),
        blocks.scoreL4(ReefPipe.PIPE_K, RobotScoringSide.LEFT),
        blocks.intakeGroundForL4(new Pose2d(13.925, 2.286, Rotation2d.fromDegrees(-30))),
        blocks.scoreL4(ReefPipe.PIPE_L, RobotScoringSide.LEFT),
        blocks.intakeGroundForL4(new Pose2d(14.672, 2.760, Rotation2d.fromDegrees(-100))),
        blocks.scoreL4(ReefPipe.PIPE_A, RobotScoringSide.RIGHT),
        blocks.intakeGroundForL4(new Pose2d(14.931, 3.236, Rotation2d.fromDegrees(-100))),
        blocks.scoreL4(ReefPipe.PIPE_B, RobotScoringSide.RIGHT));
  }
}
