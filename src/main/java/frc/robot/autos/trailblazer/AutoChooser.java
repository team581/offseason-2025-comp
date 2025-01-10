package frc.robot.autos.trailblazer;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.robot_manager.RobotManager;
import java.util.EnumSet;

public class AutoChooser {
  private final SendableChooser<BaseAuto> chooser = new SendableChooser<>();

  public AutoChooser(RobotManager robotManager, Trailblazer trailblazer) {
    SmartDashboard.putData("Autos/SelectedAuto", chooser);

    for (AutoSelection selection : EnumSet.allOf(AutoSelection.class)) {
      chooser.addOption(selection.toString(), selection.auto.apply(robotManager, trailblazer));
    }

    chooser.setDefaultOption(
        AutoSelection.OP.toString(), AutoSelection.OP.auto.apply(robotManager, trailblazer));
  }

  public BaseAuto getSelectedAuto() {
    return chooser.getSelected();
  }
}
