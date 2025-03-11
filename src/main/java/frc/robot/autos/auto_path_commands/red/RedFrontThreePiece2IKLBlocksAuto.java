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

public class RedFrontThreePiece2IKLBlocksAuto extends BaseAuto {
  public RedFrontThreePiece2IKLBlocksAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_3_AND_4.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        // SCORE L4 ON I
        blocks.scorePreloadL4(Points.START_3_AND_4.redPose, ReefPipe.PIPE_J),

        // INTAKE STATION
        blocks.intakeStationFront(CoralStation.NON_PROCESSOR_SIDE_RED),

        // SCORE L4 ON K
        blocks.scoreL4(ReefPipe.PIPE_K),

        // INTAKE STATION
        blocks.intakeStationFront(CoralStation.NON_PROCESSOR_SIDE_RED),

        // SCORE L4 ON L
        blocks.scoreL4(ReefPipe.PIPE_L));
  }
}
