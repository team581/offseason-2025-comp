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

public class RedThreePiece3JKLAuto extends BaseAuto {
  public RedThreePiece3JKLAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R3_AND_B3.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        blocks.scorePreloadL4(Points.START_R3_AND_B3.redPose, ReefPipe.PIPE_J),
        blocks.intakeStationFront(CoralStation.NON_PROCESSOR_SIDE_RED),
        blocks.scoreL4(ReefPipe.PIPE_K),
        blocks.intakeStationFront(CoralStation.NON_PROCESSOR_SIDE_RED),
        blocks.scoreL4(ReefPipe.PIPE_L));
  }
}
