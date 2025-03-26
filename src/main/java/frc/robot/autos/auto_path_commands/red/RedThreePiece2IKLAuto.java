package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.robot_manager.RobotManager;

public class RedThreePiece2IKLAuto extends BaseAuto {
  public RedThreePiece2IKLAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R2_AND_B2.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        blocks.scorePreloadL4(Points.START_R2_AND_B2, ReefPipe.PIPE_I),
        blocks.intakeCoralGroundPoints(Points.GROUND_INTAKE_LEFT_STATION),
        blocks.scoreL4(ReefPipe.PIPE_K),
        blocks.intakeCoralGroundPoints(Points.GROUND_INTAKE_LEFT_STATION),
        blocks.scoreL4(ReefPipe.PIPE_L));
  }
}
