package frc.robot.autos.auto_path_commands.red;

import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.AutoSegment;
import com.team581.trailblazer.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.autos.AutoBlocks;
import frc.robot.autos.BaseCommandAuto;
import frc.robot.autos.Points;
import frc.robot.robot_manager.RobotManager;

public class RedLollipop4PieceR6AB extends BaseCommandAuto {
  public RedLollipop4PieceR6AB(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  public Pose2d getStartingPose() {
    return Points.START_R6_AND_B6.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        Commands.parallel(
            autoCommands.homeDeployCommand(),
            autoCommands.preloadCoralCommand(),
            timing.time(
                "Preload",
                timing.time(
                    "Preload race",
                    trailblazer.followSegment(
                        new AutoSegment(
                            AutoBlocks.LOLLIPOP_RACE_CONSTRAINTS,
                            AutoBlocks.APPROACH_REEF_TOLERANCE,
                            new AutoPoint(new Pose2d(12.836, 6.965, Rotation2d.fromDegrees(90))),
                            new AutoPoint(new Pose2d(14.644, 5.81, Rotation2d.fromDegrees(90))),
                            new AutoPoint(
                                new Pose2d(15.034, 4.421, Rotation2d.fromDegrees(90)),
                                autoCommands.l4ApproachCommand(
                                    ReefPipe.PIPE_A, RobotScoringSide.LEFT),
                                AutoBlocks.BASE_CONSTRAINTS)))),
                blocks.scoreL4(ReefPipe.PIPE_A, RobotScoringSide.LEFT),
                autoCommands.intakeLollipopCommand())),
        // LOLLIPOP 3
        timing.time(
            "Piece 1",
            blocks.intakeLollipop(
                new Pose2d(15.967, 5.392, Rotation2d.fromDegrees(51.95))
                    .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
            blocks.scoreL2(
                new Pose2d(15.967, 5.392, Rotation2d.fromDegrees(90)),
                ReefPipe.PIPE_B,
                RobotScoringSide.LEFT,
                new Pose2d(15.7, 4.0, Rotation2d.fromDegrees(0))
                    .transformBy(AutoBlocks.LOLLIPOP_OFFSET)
                    .transformBy(AutoBlocks.APPROACH_LOLLIPOP_OFFSET))),
        autoCommands.intakeLollipopCommand(),
        // LOLLIPOP 2 (MIDDLE)
        timing.time(
            "Piece 2",
            blocks.intakeLollipop(
                new Pose2d(15.7, 4.0, Rotation2d.fromDegrees(0))
                    .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
            blocks.scoreL2(
                new Pose2d(15.7, 4.0, Rotation2d.fromDegrees(90)),
                ReefPipe.PIPE_A,
                RobotScoringSide.LEFT,
                new Pose2d(15.967, 2.658, Rotation2d.fromDegrees(-47.95))
                    .transformBy(AutoBlocks.LOLLIPOP_OFFSET)
                    .transformBy(AutoBlocks.APPROACH_LOLLIPOP_OFFSET)),
            autoCommands.intakeLollipopCommand()),
        // LOLLIPOP 1
        timing.time(
            "Piece 3",
            blocks.intakeLollipop(
                new Pose2d(15.967, 2.658, Rotation2d.fromDegrees(-47.95))
                    .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
            blocks.scoreL4(
                new Pose2d(15.967, 2.658, Rotation2d.fromDegrees(90)),
                ReefPipe.PIPE_B,
                RobotScoringSide.LEFT),
            Commands.none()));
  }
}
