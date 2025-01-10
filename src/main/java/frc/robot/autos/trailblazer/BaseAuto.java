package frc.robot.autos.trailblazer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.trailblazer.Trailblazer;
import frc.robot.fms.FmsSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;

public abstract class BaseAuto {
  protected final RobotManager robotManager;
  protected final AutoCommands autoCommands;
  protected final RobotCommands actions;
  protected final Trailblazer trailblazer;

  protected BaseAuto(RobotManager robotManager, Trailblazer trailblazer) {
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;
    this.actions = new RobotCommands(robotManager, trailblazer);
    this.autoCommands = new AutoCommands(actions, robotManager);
  }

  protected abstract Command getRedAutoCommand();

  protected abstract Command getBlueAutoCommand();

  public Command getAutoCommand() {
    return Commands.either(getRedAutoCommand(), getBlueAutoCommand(), FmsSubsystem::isRedAlliance);
  }
}
