package frc.robot.autos.auto_path_commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Trailblazer;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.robot_manager.RobotManager;

public class StraightLineAuto extends BaseAuto {
  private static final AutoConstraintOptions CONSTRAINTS = new AutoConstraintOptions(1, 50, 4, 30);

  public StraightLineAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getBlueStartingPose() {
    return Pose2d.kZero;
  }

  @Override
  protected Command getBlueAutoCommand() {
    return Commands.none();
  }

  @Override
  protected Pose2d getRedStartingPose() {
    return new Pose2d(10.263, 0.748, Rotation2d.kZero);
  }

  @Override
  protected Command getRedAutoCommand() {
    return Commands.sequence(
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(getRedStartingPose()),
                new AutoPoint(new Pose2d(11.924, 0.748, Rotation2d.kZero)))),
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(new Pose2d(13.313, 0.748, Rotation2d.kZero)),
                new AutoPoint(new Pose2d(14.928, 0.748, Rotation2d.kZero)))));
  }
}
