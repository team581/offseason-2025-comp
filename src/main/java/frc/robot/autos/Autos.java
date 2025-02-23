package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.fms.FmsSubsystem;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Autos extends LifecycleSubsystem {
  private final AutoChooser autoChooser;
  private final RobotManager robotManager;
  private final Trailblazer trailblazer;
  private boolean hasEnabledAuto = false;
  private Pair<AutoSelection, BaseAuto> selected;

  public Autos(RobotManager robotManager, Trailblazer trailblazer) {
    super(SubsystemPriority.AUTOS);

    autoChooser = new AutoChooser(robotManager, trailblazer);
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;

    selected =
        Pair.of(
            AutoSelection.DO_NOTHING,
            AutoSelection.DO_NOTHING.auto.apply(robotManager, trailblazer));
  }

  public Command getAutoCommand() {
    DogLog.log("Autos/CreatedAutoCommand", selected.getFirst().name());
    return selected.getSecond().getAutoCommand();
  }

  @Override
  public void robotPeriodic() {
    updateSelection();

    if (DriverStation.isAutonomousEnabled()) {
      hasEnabledAuto = true;
    }

    // Continuously reset pose before auto
    if (!hasEnabledAuto && DriverStation.isDisabled() && DriverStation.isAutonomous()) {
      resetPoseForAuto();
    }
  }

  private void resetPoseForAuto() {
    var auto = selected.getSecond();
    var startingPose =
        FmsSubsystem.isRedAlliance() ? auto.getRedStartingPose() : auto.getBlueStartingPose();
    robotManager.localization.resetPose(startingPose);
  }

  private void updateSelection() {
    if (selected.getFirst() != autoChooser.getSelectedAuto()) {
      selected =
          Pair.of(
              autoChooser.getSelectedAuto(),
              autoChooser.getSelectedAuto().auto.apply(robotManager, trailblazer));
    }
  }
}
