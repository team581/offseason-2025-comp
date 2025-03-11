package frc.robot.autos.auto_path_commands.blue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.MathHelpers;

public class BlueFourPiece5EDCBGround extends BaseAuto {
  public BlueFourPiece5EDCBGround(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R5_AND_B5.bluePose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
      // TODO: Add approach points
        blocks.scorePreloadL4(Points.START_R5_AND_B5.bluePose, ReefPipe.PIPE_E),
        blocks.intakeGround(MathHelpers.pathflip(new Pose2d()),new Pose2d(2.906, 2.031, Rotation2d.fromDegrees(-145.0))),
        blocks.scoreL4(ReefPipe.PIPE_D),
        blocks.intakeGround(MathHelpers.pathflip(new Pose2d()),new Pose2d(2.906, 2.031, Rotation2d.fromDegrees(-145.0))),
        blocks.scoreL4(ReefPipe.PIPE_C),
        blocks.intakeGround(new Pose2d(), new Pose2d(2.906, 2.031, Rotation2d.fromDegrees(-145.0))),
        blocks.scoreL4(ReefPipe.PIPE_B));
  }
}
