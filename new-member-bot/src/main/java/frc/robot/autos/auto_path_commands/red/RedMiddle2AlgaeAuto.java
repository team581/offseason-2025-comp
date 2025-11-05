package frc.robot.autos.auto_path_commands.red;

import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.AutoSegment;
import com.team581.trailblazer.Trailblazer;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
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
      trailblazer.followSegment(
        new AutoSegment(new AutoPoint(new Pose2d(12.256, 2.668, Rotation2d.fromDegrees(55.0)), autoCommands.l1ReleaseCommand(ReefPipe.PIPE_I)))),
      trailblazer.followSegment(
        new AutoSegment(
          new AutoPoint(new Pose2d(12.215, 2.668, Rotation2d.fromDegrees(55.0)), autoCommands.intakeAlgaeL3Command()))),
      autoCommands.stowCommand(),
      trailblazer.followSegment(
        new AutoSegment(
          new AutoPoint(new Pose2d(10.780, 2.558, Rotation2d.fromDegrees(-180.0)), autoCommands.netReleaseCommand()))),
      trailblazer.followSegment(
        new AutoSegment(
          new AutoPoint(new Pose2d(11.424, 3.896, Rotation2d.fromDegrees(0.0)), autoCommands.intakeAlgaeL2Command()))),
      autoCommands.stowCommand(),
      trailblazer.followSegment(
        new AutoSegment(
          new AutoPoint(new Pose2d(10.78, 3.229, Rotation2d.fromDegrees(-180.0)), autoCommands.netReleaseCommand()))),
      autoCommands.stowCommand()
      );
  }
}
