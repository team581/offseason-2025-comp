package frc.robot.autos;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;

public class AutoCommands {
  private final RobotCommands robotCommands;
  private final RobotManager robotManager;

  public AutoCommands(RobotCommands robotCommands, RobotManager robotManager) {
    this.robotCommands = robotCommands;
    this.robotManager = robotManager;
  }

  public Command doNothingCommand() {
    return Commands.none();
  }
}