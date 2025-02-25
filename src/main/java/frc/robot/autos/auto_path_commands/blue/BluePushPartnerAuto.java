package frc.robot.autos.auto_path_commands.blue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Trailblazer;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.robot_manager.RobotManager;

public class BluePushPartnerAuto extends BaseAuto {
  private static final AutoConstraintOptions CONSTRAINTS =
      new AutoConstraintOptions(4.75, 71.5, 8.5, 35.2);

  public BluePushPartnerAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Pose2d.kZero;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.none();
  }
}
