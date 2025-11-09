package frc.robot.autos.auto_path_commands.blue;

import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.AutoSegment;
import com.team581.trailblazer.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.autos.AutoBlocks;
import frc.robot.autos.BaseCommandAuto;
import frc.robot.autos.Points;
import frc.robot.robot_manager.RobotManager;

public class BlueMiddle2PieceAuto extends BaseCommandAuto {
  public BlueMiddle2PieceAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  public Pose2d getStartingPose() {
    return Points.START_MIDDLE_BARGE.bluePose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        autoCommands.preloadCoralCommand(),
        // Score preload coral
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                // add approach point
                new AutoPoint(new Pose2d(6.7, 4.077, Rotation2d.fromDegrees(180.0))),
                new AutoPoint(new Pose2d(5.85, 4.077, Rotation2d.fromDegrees(180.0))))),
        autoCommands.l1ReleaseCommand(ReefPipe.PIPE_G),
        Commands.waitSeconds(0.3),
        // Intake first algae
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                new AutoPoint(new Pose2d(6.197, 4.077, Rotation2d.fromDegrees(180.0))))),
        autoCommands.intakeAlgaeL2Command(),
        Commands.waitSeconds(0.5),
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                new AutoPoint(new Pose2d(5.897, 4.077, Rotation2d.fromDegrees(180.0))))),
        Commands.waitSeconds(1.0),
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                new AutoPoint(new Pose2d(6.432, 4.05, Rotation2d.fromDegrees(180.0))))),
        // Score first algae
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                new AutoPoint(new Pose2d(6.35, 4.05, Rotation2d.fromDegrees(-90.0))),
                new AutoPoint(new Pose2d(6.35, 4.95, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(new Pose2d(6.15, 6.1, Rotation2d.fromDegrees(0.0))))),
        autoCommands.netWaitCommand(),
        Commands.waitSeconds(1.2),
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                new AutoPoint(new Pose2d(7.15, 6.1, Rotation2d.fromDegrees(0.0))))),
        autoCommands.netReleaseCommand(),
        Commands.waitSeconds(0.8),
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                new AutoPoint(new Pose2d(7.05, 6.1, Rotation2d.fromDegrees(0.0))))),
        Commands.waitSeconds(0.5),
        autoCommands.stowCommand(),
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                new AutoPoint(new Pose2d(7.15, 4.95, Rotation2d.fromDegrees(120.0))),
                new AutoPoint(new Pose2d(6.95, 3.15, Rotation2d.fromDegrees(120.0))),
                new AutoPoint(new Pose2d(6.08, 1.8, Rotation2d.fromDegrees(120.0))))));
  }
}
