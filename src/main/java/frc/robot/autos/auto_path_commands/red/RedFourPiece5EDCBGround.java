package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.robot_manager.RobotManager;

public class RedFourPiece5EDCBGround extends BaseAuto {
  public RedFourPiece5EDCBGround(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R5_AND_B5.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        // TODO: Add approach points
        blocks.scorePreloadL4(Points.START_R5_AND_B5.redPose, ReefPipe.PIPE_E),
        blocks.intakeGround(new Pose2d(), new Pose2d(14.644, 6.019, Rotation2d.fromDegrees(35))),
        blocks.scoreL4(ReefPipe.PIPE_D),
        blocks.intakeGround(new Pose2d(), new Pose2d(14.644, 6.019, Rotation2d.fromDegrees(35))),
        blocks.scoreL4(ReefPipe.PIPE_C),
        blocks.intakeGround(new Pose2d(), new Pose2d(14.644, 6.019, Rotation2d.fromDegrees(35))),
        blocks.scoreL4(ReefPipe.PIPE_B));
  }
}
