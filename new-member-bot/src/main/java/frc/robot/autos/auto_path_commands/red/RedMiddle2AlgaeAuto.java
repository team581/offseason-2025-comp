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
        autoCommands.rehomeWrist(),
        autoCommands.preloadCoralCommand(),
        // Score preload coral
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                // add approach point
                new AutoPoint(
                    new Pose2d(12.256, 2.668, Rotation2d.fromDegrees(55.0)),
                    autoCommands.l1ReleaseCommand(ReefPipe.PIPE_I)))),
        // Intake first algae
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                // add backaway (after scoring L1))
                new AutoPoint(
                    new Pose2d(12.215, 2.668, Rotation2d.fromDegrees(55.0)),
                    autoCommands.intakeAlgaeL3Command()))),
        // Score first algae
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                // add backaway, add approach, adjust lineup to score
                new AutoPoint(
                    new Pose2d(10.780, 2.558, Rotation2d.fromDegrees(-180.0)),
                    autoCommands.netReleaseCommand()))),
        // Intake second algae
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                // backaway from last point, add approach point to intake
                new AutoPoint(
                    new Pose2d(11.424, 3.896, Rotation2d.fromDegrees(0.0)),
                    autoCommands.intakeAlgaeL2Command()))),
        // Score second algae
        trailblazer.followSegment(
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                // backaway from reef, approach net, lineup to net
                new AutoPoint(
                    new Pose2d(10.78, 3.229, Rotation2d.fromDegrees(-180.0)),
                    autoCommands.netReleaseCommand()))));
  }
}
