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

public class RedLollipopDescoreKL extends BaseAuto {
  public RedLollipopDescoreKL(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R1_AND_B1.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        timing.time(
            "Preload",
            autoCommands.preloadCoralCommand(),
            timing.time(
                "Preload race",
                trailblazer.followSegment(
                    new AutoSegment(
                        AutoBlocks.MAX_CONSTRAINTS,
                        AutoBlocks.APPROACH_REEF_TOLERANCE,
                        new AutoPoint(new Pose2d(10.289, 0.47, Rotation2d.fromDegrees(30))),
                        new AutoPoint(
                            new Pose2d(12.242, 1.278, Rotation2d.fromDegrees(30)),
                            Commands.runOnce(
                                    () ->
                                        robotManager.autoAlign.setAutoReefPipeOverride(
                                            ReefPipe.PIPE_L))
                                .andThen(
                                    autoCommands.l4ApproachCommand(
                                        ReefPipe.PIPE_L, RobotScoringSide.LEFT))),
                        new AutoPoint(new Pose2d(13.672, 2.019, Rotation2d.fromDegrees(30)))))),
            blocks.scoreL4(
                ReefPipe.PIPE_L, RobotScoringSide.LEFT, autoCommands.intakeLollipopCommand())),
        // LOLLIPOP 2
        timing.time(
            "Piece 1",
            blocks.intakeLollipop(
                new Pose2d(15.7, 4.0, Rotation2d.fromDegrees(0))
                    .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
            blocks.scoreL3(
                ReefPipe.PIPE_L, RobotScoringSide.LEFT, autoCommands.intakeLollipopCommand())),
        // LOLLIPOP 1
        timing.time(
            "Piece 2",
            blocks.intakeLollipop(
                new Pose2d(15.799, 2.496, Rotation2d.fromDegrees(-30))
                    .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
            blocks.scoreL3(
                ReefPipe.PIPE_K,
                RobotScoringSide.LEFT,
                autoCommands.moveToStartingPositionCommand())));
  }
}
