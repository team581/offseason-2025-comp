package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Autos extends LifecycleSubsystem {
  private final AutoChooser autoChooser;

  public Autos(RobotManager robotManager, Trailblazer trailblazer) {
    super(SubsystemPriority.AUTOS);

    autoChooser = new AutoChooser(robotManager, trailblazer);
  }

  public Command getAutoCommand() {
    return autoChooser.getSelectedAuto().getAutoCommand();
  }
}
