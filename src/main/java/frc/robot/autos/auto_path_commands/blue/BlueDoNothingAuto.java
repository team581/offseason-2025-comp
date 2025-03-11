package frc.robot.autos.auto_path_commands.blue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.robot_manager.RobotManager;

public class BlueDoNothingAuto extends BaseAuto {
  private static final AutoConstraintOptions CONSTRAINTS = new AutoConstraintOptions(2, 57, 4, 30);

  public BlueDoNothingAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R1_AND_B1.bluePose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        Commands.runOnce(robotManager::rehomeRollRequest),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(Points.START_R1_AND_B1.bluePose),
                new AutoPoint(new Pose2d(6.92, 7.29, Rotation2d.fromDegrees(180))))));
  }
}
