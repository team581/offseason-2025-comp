package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.autos.AutoBlocks;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.elevator.CoralStation;
import frc.robot.robot_manager.RobotManager;

public class RedPushPartnerAuto extends BaseAuto {
  public RedPushPartnerAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R3_AND_B3.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        trailblazer.followSegment(
            // PUSH PARTNER
            new AutoSegment(
                AutoBlocks.BASE_CONSTRAINTS,
                new AutoPoint(Points.START_R3_AND_B3.redPose),
                new AutoPoint(new Pose2d(9.963, 1.903, Rotation2d.kZero)))),
        blocks.scorePreloadL4(Points.START_R3_AND_B3.redPose, ReefPipe.PIPE_J),
        blocks.intakeStationBack(CoralStation.NON_PROCESSOR_SIDE_RED),
        blocks.scoreL4(ReefPipe.PIPE_K),
        blocks.intakeStationBack(CoralStation.NON_PROCESSOR_SIDE_RED),
        blocks.scoreL4(ReefPipe.PIPE_L));
  }
}
