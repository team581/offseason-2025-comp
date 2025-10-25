package frc.robot.autos;

import com.team581.trailblazer.Trailblazer;
import frc.robot.robot_manager.RobotManager;

public class AutoBlocks {
  private final Trailblazer trailblazer;
  private final RobotManager robotManager;
  private final AutoCommands autoCommands;

  public AutoBlocks(Trailblazer trailblazer, RobotManager robotManager, AutoCommands autoCommands) {
    this.trailblazer = trailblazer;
    this.robotManager = robotManager;
    this.autoCommands = autoCommands;
  }
}
