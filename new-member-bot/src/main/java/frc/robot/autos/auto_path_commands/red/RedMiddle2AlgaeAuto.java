package frc.robot.autos.auto_path_commands.red;

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

public class RedMiddle2AlgaeAuto extends BaseCommandAuto {
  public RedMiddle2AlgaeAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  public Pose2d getStartingPose() {
    return Points.START_MIDDLE_BARGE.redPose;
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
                new AutoPoint(new Pose2d(11.105, 3.973, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(new Pose2d(11.553, 3.973, Rotation2d.fromDegrees(0.0))))),
        autoCommands.l1ReleaseCommand(ReefPipe.PIPE_G),
        Commands.waitSeconds(0.3),
        // Intake first algae
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                new AutoPoint(new Pose2d(11.353, 3.973, Rotation2d.fromDegrees(0.0))))),
        autoCommands.intakeAlgaeL2Command(),
        Commands.waitSeconds(0.5),
        trailblazer.followSegment(
            new AutoSegment(new AutoPoint(new Pose2d(11.653, 3.973, Rotation2d.fromDegrees(0.0))))),
        Commands.waitSeconds(0.6),
        trailblazer.followSegment(
            new AutoSegment(new AutoPoint(new Pose2d(11.118, 4.0, Rotation2d.fromDegrees(0.0))))),
        // Score first algae
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                new AutoPoint(new Pose2d(11.2, 3.7, Rotation2d.fromDegrees(90.0))),
                new AutoPoint(new Pose2d(11.3, 3.1, Rotation2d.fromDegrees(180))),
                new AutoPoint(new Pose2d(11.9, 1.95, Rotation2d.fromDegrees(180.0))))),
        autoCommands.netWaitCommand(),
        Commands.waitSeconds(1.5),
        trailblazer.followSegment(
            new AutoSegment(new AutoPoint(new Pose2d(11.5, 1.95, Rotation2d.fromDegrees(180.0))))),
        Commands.waitSeconds(1),
        trailblazer.followSegment(
            new AutoSegment(new AutoPoint(new Pose2d(12.1, 1.95, Rotation2d.fromDegrees(180.0))))),
        Commands.waitSeconds(0.5),
        autoCommands.stowCommand(),
        // Intake second
        // algae--------------------------------------------------------------------------------
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                new AutoPoint(new Pose2d(11.0, 4.1, Rotation2d.fromDegrees(-60.0))),
                new AutoPoint(new Pose2d(11.5, 6.1, Rotation2d.fromDegrees(-60.0))),
                new AutoPoint(new Pose2d(12.4, 5.5, Rotation2d.fromDegrees(-60.0))))),
        autoCommands.intakeAlgaeL3Command(),
        Commands.waitSeconds(0.6),
        trailblazer.followSegment(
            new AutoSegment(new AutoPoint(new Pose2d(12.4, 5.3, Rotation2d.fromDegrees(-60.0))))),
        Commands.waitSeconds(1),
        trailblazer.followSegment(
            new AutoSegment(new AutoPoint(new Pose2d(11.5, 6.1, Rotation2d.fromDegrees(-60.0))))),
        autoCommands.stowCommand(),
        // Score second algae
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                new AutoPoint(new Pose2d(11.837, 5.2, Rotation2d.fromDegrees(-60.0))),
                new AutoPoint(new Pose2d(11.0, 4.1, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(new Pose2d(11.339, 2.77, Rotation2d.fromDegrees(180.0))),
                new AutoPoint(new Pose2d(11.9, 1.95, Rotation2d.fromDegrees(180.0))))),
        autoCommands.netWaitCommand(),
        Commands.waitSeconds(1),
        trailblazer.followSegment(
            new AutoSegment(new AutoPoint(new Pose2d(11.6, 1.95, Rotation2d.fromDegrees(180.0))))),
        trailblazer.followSegment(
            new AutoSegment(new AutoPoint(new Pose2d(12.1, 1.95, Rotation2d.fromDegrees(180.0))))),
        autoCommands.stowCommand());
  }
}
