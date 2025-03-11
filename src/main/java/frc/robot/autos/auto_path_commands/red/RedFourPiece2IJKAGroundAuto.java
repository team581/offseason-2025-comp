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

public class RedFourPiece2IJKAGroundAuto extends BaseAuto {
  public RedFourPiece2IJKAGroundAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R2_AND_B2.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        blocks.scorePreloadL4(Points.START_R2_AND_B2.redPose, ReefPipe.PIPE_I),
        blocks.intakeStationGround(new Pose2d(15.151, 1.244, Rotation2d.fromDegrees(-20.0))),
        blocks.scoreL4(ReefPipe.PIPE_K),
        blocks.intakeStationGround(new Pose2d(15.151, 1.244, Rotation2d.fromDegrees(-20.0))),
        blocks.scoreL4(ReefPipe.PIPE_L),
        blocks.intakeStationGround(new Pose2d(15.151, 1.244, Rotation2d.fromDegrees(-20.0))),
        blocks.scoreL4(ReefPipe.PIPE_A));
  }
}
