package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.fms.FmsSubsystem;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Autos extends LifecycleSubsystem {
  private final AutoChooser autoChooser;
  private final RobotManager robotManager;
  private boolean hasEnabledAuto = false;

  public Autos(RobotManager robotManager, Trailblazer trailblazer) {
    super(SubsystemPriority.AUTOS);

    autoChooser = new AutoChooser(robotManager, trailblazer);
    this.robotManager = robotManager;
  }

  public Command getAutoCommand() {
    DogLog.log("Autos/CreatedAutoCommand", autoChooser.getSelectedAuto().name());
    return autoChooser.getSelectedAuto().getAutoCommand();
  }

  @Override
  public void robotPeriodic() {
    if (DriverStation.isAutonomousEnabled()) {
      hasEnabledAuto = true;
    }

    // Continuously reset pose before auto
    if (!hasEnabledAuto && DriverStation.isDisabled() && DriverStation.isAutonomous()) {
      resetPoseForAuto();
    }
  }

  private void resetPoseForAuto() {
    var auto = autoChooser.getSelectedAuto();
    var startingPose =
        FmsSubsystem.isRedAlliance() ? auto.getRedStartingPose() : auto.getBlueStartingPose();
    robotManager.localization.resetPose(startingPose);
  }
}
