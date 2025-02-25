package frc.robot.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Autos extends LifecycleSubsystem {
  private final AutoChooser autoChooser;
  private final RobotManager robotManager;
  private final Trailblazer trailblazer;
  private boolean hasEnabledAuto = false;
  private Pair<AutoSelection, BaseAuto> selectedRed;
  private Pair<AutoSelection, BaseAuto> selectedBlue;

  public Autos(RobotManager robotManager, Trailblazer trailblazer) {
    super(SubsystemPriority.AUTOS);

    autoChooser = new AutoChooser(robotManager, trailblazer);
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;

    selectedRed =
        Pair.of(
            AutoSelection.DO_NOTHING,
            AutoSelection.DO_NOTHING.redAuto.apply(robotManager, trailblazer));
    selectedBlue =
        Pair.of(
            AutoSelection.DO_NOTHING,
            AutoSelection.DO_NOTHING.blueAuto.apply(robotManager, trailblazer));
  }

  public Command getAutoCommand() {
    return Commands.either(
            selectedRed.getSecond().getAutoCommand(),
            selectedBlue.getSecond().getAutoCommand(),
            FmsSubsystem::isRedAlliance)
        .withName(selectedRed.getSecond().name() + "_or_" + selectedBlue.getSecond().name());
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
    var auto = FmsSubsystem.isRedAlliance() ? selectedRed.getSecond() : selectedBlue.getSecond();
    var startingPose = auto.getStartingPose();
    robotManager.localization.resetPose(startingPose);
  }

  private void updateSelection() {
    if (selectedRed.getFirst() != autoChooser.getSelectedAuto()) {
      selectedRed =
          Pair.of(
              autoChooser.getSelectedAuto(),
              autoChooser.getSelectedAuto().redAuto.apply(robotManager, trailblazer));
    }

    if (selectedBlue.getFirst() != autoChooser.getSelectedAuto()) {
      selectedBlue =
          Pair.of(
              autoChooser.getSelectedAuto(),
              autoChooser.getSelectedAuto().blueAuto.apply(robotManager, trailblazer));
    }
  }
}
