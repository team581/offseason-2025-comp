package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.elevator.CoralStation;
import frc.robot.robot_manager.RobotManager;

public class RedThreePiece4EDCAuto extends BaseAuto {
  public RedThreePiece4EDCAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R4_AND_B4.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        blocks.scorePreloadL4(Points.START_R4_AND_B4.redPose, ReefPipe.PIPE_E),
        blocks.intakeStationFront(CoralStation.PROCESSOR_SIDE_RED),
        blocks.scoreL4(ReefPipe.PIPE_D),
        blocks.intakeStationFront(CoralStation.PROCESSOR_SIDE_RED),
        blocks.scoreL4(ReefPipe.PIPE_C));
  }
}
