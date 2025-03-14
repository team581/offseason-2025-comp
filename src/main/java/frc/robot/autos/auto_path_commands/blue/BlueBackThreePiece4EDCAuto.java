package frc.robot.autos.auto_path_commands.blue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.elevator.CoralStation;
import frc.robot.robot_manager.RobotManager;

public class BlueBackThreePiece4EDCAuto extends BaseAuto {
  public BlueBackThreePiece4EDCAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R4_AND_B4.bluePose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        blocks.scorePreloadL4(Points.START_R4_AND_B4.bluePose, ReefPipe.PIPE_E),
        blocks.intakeStationBack(CoralStation.PROCESSOR_SIDE_BLUE),
        blocks.scoreL4(ReefPipe.PIPE_D),
        blocks.intakeStationBack(CoralStation.PROCESSOR_SIDE_BLUE),
        blocks.scoreL4(ReefPipe.PIPE_C));
  }
}
