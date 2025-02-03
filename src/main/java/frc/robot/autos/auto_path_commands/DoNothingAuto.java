package frc.robot.autos.auto_path_commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Trailblazer;
import frc.robot.robot_manager.RobotManager;

public class DoNothingAuto extends BaseAuto {
  public DoNothingAuto(RobotManager robotManager, Trailblazer trailblazer) {
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
    return Pose2d.kZero;
  }

  @Override
  protected Command getRedAutoCommand() {
    return Commands.none();
  }
}
