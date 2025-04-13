package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.autos.AutoBlocks;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.robot_manager.RobotManager;

public class RedThreePiece2ILKAuto extends BaseAuto {
  public RedThreePiece2ILKAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R2_AND_B2.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        autoCommands.preloadCoralCommand(),
        autoCommands.homeDeployCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS_FOR_GROUND_AUTOS,
                AutoBlocks.APPROACH_REEF_TOLERANCE,
                new AutoPoint(new Pose2d(10.7, 1.903, Rotation2d.fromDegrees(0))),
                new AutoPoint(
                    new Pose2d(11.473, 2.076, Rotation2d.fromDegrees(-30)),
                    autoCommands.l4ApproachCommand(ReefPipe.PIPE_I, RobotScoringSide.LEFT)))),
        blocks.scoreL4(ReefPipe.PIPE_I, RobotScoringSide.LEFT),
        autoCommands.groundIntakeToL4Command(),
        blocks.intakeCoralPath(
            new Pose2d(12.813, 1.693, Rotation2d.fromDegrees(-45)),
            new Pose2d(13.967, 0.640, Rotation2d.fromDegrees(0)),
            new Pose2d(16.124, 1.148, Rotation2d.fromDegrees(0))),
        blocks.scoreL4(ReefPipe.PIPE_L, RobotScoringSide.LEFT),
        autoCommands.groundIntakeToL4Command(),
        blocks.intakeCoralPath(
            new Pose2d(14.149, 2.075, Rotation2d.fromDegrees(0)),
            new Pose2d(14.608, 0.746, Rotation2d.fromDegrees(0)),
            new Pose2d(16.124, 1.148, Rotation2d.fromDegrees(0))),
        blocks.scoreL4(ReefPipe.PIPE_K, RobotScoringSide.LEFT),
        autoCommands.moveToStartingPositionCommand());
  }
}
