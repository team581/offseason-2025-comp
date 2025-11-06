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
    return Points.START_R3_AND_B3.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        autoCommands.preloadCoralCommand(),
        // Score preload coral
        trailblazer
            .followSegment(
                new AutoSegment(
                    AutoBlocks.BASE_CONSTRAINTS,
                    AutoBlocks.APPROACH_REEF_TOLERANCE,
                    // add approach point
                    new AutoPoint(new Pose2d(11.464, 2.834, Rotation2d.fromDegrees(0.0))),
                    new AutoPoint(
                        new Pose2d(12.256, 2.668, Rotation2d.fromDegrees(55.0)),
                        autoCommands.l1ReleaseCommand(ReefPipe.PIPE_I))))
            .andThen(autoCommands.stowCommand()),
        // Intake first algae
        trailblazer
            .followSegment(
                new AutoSegment(
                    AutoBlocks.BASE_CONSTRAINTS,
                    AutoBlocks.APPROACH_REEF_TOLERANCE,
                    new AutoPoint(
                        new Pose2d(12.35, 2.873, Rotation2d.fromDegrees(55.0)),
                        autoCommands.intakeAlgaeL3Command()),
                    new AutoPoint(new Pose2d(12.298, 2.627, Rotation2d.fromDegrees(55.0)))))
            .andThen(autoCommands.stowCommand()),
        // Score first algae
        trailblazer
            .followSegment(
                new AutoSegment(
                    AutoBlocks.BASE_CONSTRAINTS,
                    new AutoPoint(new Pose2d(11.809, 2.053, Rotation2d.fromDegrees(150.0))),
                    new AutoPoint(new Pose2d(11.172, 1.819, Rotation2d.fromDegrees(180))),
                    new AutoPoint(
                        new Pose2d(10.542, 1.819, Rotation2d.fromDegrees(180.0)),
                        autoCommands.netReleaseCommand()),
                    new AutoPoint(new Pose2d(10.98, 1.819, Rotation2d.fromDegrees(180.0)))))
            .andThen(autoCommands.stowCommand()),
        // Intake second algae
        trailblazer
            .followSegment(
                new AutoSegment(
                    AutoBlocks.BASE_CONSTRAINTS,
                    AutoBlocks.APPROACH_REEF_TOLERANCE,
                    new AutoPoint(new Pose2d(11.521, 1.819, Rotation2d.fromDegrees(150.0))),
                    new AutoPoint(new Pose2d(11.521, 2.834, Rotation2d.fromDegrees(90.0))),
                    new AutoPoint(
                        new Pose2d(11.714, 4.026, Rotation2d.fromDegrees(0.0)),
                        autoCommands.intakeAlgaeL2Command()),
                    new AutoPoint(new Pose2d(11.714, 4.026, Rotation2d.fromDegrees(0.0)))))
            .andThen(autoCommands.stowCommand()),
        // Score second algae
        trailblazer
            .followSegment(
                new AutoSegment(
                    AutoBlocks.BASE_CONSTRAINTS,
                    new AutoPoint(new Pose2d(11.521, 2.8934, Rotation2d.fromDegrees(90.0))),
                    new AutoPoint(new Pose2d(11.521, 1.819, Rotation2d.fromDegrees(180.0))),
                    new AutoPoint(
                        new Pose2d(10.567, 1.819, Rotation2d.fromDegrees(180.0)),
                        autoCommands.netReleaseCommand()),
                    new AutoPoint(new Pose2d(10.878, 1.819, Rotation2d.fromDegrees(180.0)))))
            .andThen(autoCommands.stowCommand()));
  }
}
