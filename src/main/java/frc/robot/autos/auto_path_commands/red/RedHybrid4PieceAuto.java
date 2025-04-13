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

public class RedHybrid4PieceAuto extends BaseAuto {
  public RedHybrid4PieceAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R1_AND_B1.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        autoCommands.preloadCoralCommand(),
        autoCommands.homeDeployCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.MAX_CONSTRAINTS,
                AutoBlocks.APPROACH_REEF_TOLERANCE,
                new AutoPoint(new Pose2d(10.289, 0.47, Rotation2d.fromDegrees(-30))),
                new AutoPoint(
                    new Pose2d(10.916, 1.423, Rotation2d.fromDegrees(-30)),
                    autoCommands.l4ApproachCommand(ReefPipe.PIPE_I, RobotScoringSide.LEFT)),
                new AutoPoint(new Pose2d(11.57, 2.342, Rotation2d.fromDegrees(-30))))),
        blocks.scoreL4(
            ReefPipe.PIPE_I, RobotScoringSide.LEFT, autoCommands.intakeLollipopCommand()),
        blocks.intakeCoralGroundPoints(
            new Pose2d(13.886, 1.616, Rotation2d.fromDegrees(0)),
            new Pose2d(13.925, 0.66, Rotation2d.fromDegrees(0)),
            Points.GROUND_INTAKE_LEFT_STATION),
        blocks.scoreL4(
            ReefPipe.PIPE_K, RobotScoringSide.LEFT, autoCommands.groundIntakeToL4Command()),
        blocks.intakeCoralGroundPoints(
            new Pose2d(13.708, 1.615, Rotation2d.fromDegrees(0)),
            new Pose2d(13.925, 0.66, Rotation2d.fromDegrees(0)),
            Points.GROUND_INTAKE_LEFT_STATION),
        blocks.scoreL4(ReefPipe.PIPE_L, RobotScoringSide.LEFT, autoCommands.stowRequest()));
  }
}
