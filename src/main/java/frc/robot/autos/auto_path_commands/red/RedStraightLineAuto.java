package frc.robot.autos.auto_path_commands.red;

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

public class RedStraightLineAuto extends BaseAuto {
  private static final AutoConstraintOptions CONSTRAINTS = new AutoConstraintOptions(10, 50, 1, 30);

  public RedStraightLineAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_1_AND_6.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(Points.START_1_AND_6.redPose),
                new AutoPoint(new Pose2d(11.924, 0.758, Rotation2d.kZero)),
                new AutoPoint(new Pose2d(12.0, 0.758, Rotation2d.kZero)),
                new AutoPoint(new Pose2d(13.0, 0.758, Rotation2d.kZero)),
                new AutoPoint(new Pose2d(15.0, 0.758, Rotation2d.kZero)))));
  }
}
