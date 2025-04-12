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

public class RedThreePiece3GHJ extends BaseAuto {
  public RedThreePiece3GHJ(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R3_AND_B3_LEFT_FORWARD.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        autoCommands.preloadCoralCommand(),
        autoCommands.homeDeployCommand(),
        blocks.scoreL4(
            ReefPipe.PIPE_G, RobotScoringSide.LEFT, autoCommands.groundIntakeToL4Command()),
        blocks.intakeGroundForL4(new Pose2d(11.374, 2.2, Rotation2d.fromDegrees(-40))),
        blocks.scoreL4(
            ReefPipe.PIPE_H, RobotScoringSide.LEFT, autoCommands.groundIntakeToL4Command()),
        blocks.intakeGroundForL4(new Pose2d(11.374, 2.25, Rotation2d.fromDegrees(-40))),
        blocks.scoreL4(
            ReefPipe.PIPE_J, RobotScoringSide.LEFT, autoCommands.moveToStartingPositionCommand()));
  }
}
