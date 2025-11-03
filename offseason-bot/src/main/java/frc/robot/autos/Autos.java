package frc.robot.autos;

import com.team581.autos.AutoChooser;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.SubsystemPriority;

public class Autos extends StateMachineSubsystem<AutoSelection> {
  private final AutoChooser<AutoSelection> autoChooser;
  private final RobotManager robotManager;
  private final Trailblazer trailblazer;
  private boolean hasEnabledAuto = false;
  private BaseImperativeAuto<?> selectedRedAuto;
  private BaseImperativeAuto<?> selectedBlueAuto;

  public Autos(RobotManager robotManager, Trailblazer trailblazer) {
    super(SubsystemPriority.AUTOS, AutoSelection.DO_NOTHING);

    autoChooser = new AutoChooser<>(AutoSelection.values(), AutoSelection.DO_NOTHING);
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;

    selectedRedAuto = AutoSelection.DO_NOTHING.redAuto.apply(robotManager, trailblazer);
    selectedBlueAuto = AutoSelection.DO_NOTHING.blueAuto.apply(robotManager, trailblazer);
  }

  @Override
  protected AutoSelection getNextState(AutoSelection currentState) {
    if (DriverStation.isEnabled()) {
      return currentState;
    }

    return autoChooser.getSelectedAuto();
  }

  @Override
  protected void afterTransition(AutoSelection newState) {
    // Recreate the auto instances when the selection changes
    selectedRedAuto = newState.redAuto.apply(robotManager, trailblazer);
    selectedBlueAuto = newState.blueAuto.apply(robotManager, trailblazer);
  }

  @Override
  protected void whileInState(AutoSelection state) {
    if (DriverStation.isDisabled()) {
      if (!hasEnabledAuto
          && (RobotBase.isSimulation()
              || DriverStation.isAutonomous()
              || DriverStation.isFMSAttached())) {
        // Continuously reset pose
        resetPoseForAuto();
      }
    }

    if (DriverStation.isAutonomousEnabled()) {
      hasEnabledAuto = true;
    }

    if (DriverStation.isAutonomous()) {
      var auto = FmsUtil.isRedAlliance() ? selectedRedAuto : selectedBlueAuto;
      auto.beforePeriodic();
      auto.periodic();
    }
  }

  private void resetPoseForAuto() {
    var auto = FmsUtil.isRedAlliance() ? selectedRedAuto : selectedBlueAuto;
    var startingPose = auto.getStartingPose();
    robotManager.localization.resetPose(startingPose);
  }
}
