package frc.robot.autos.auto_state_machines;

import com.team581.trailblazer.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.robot_manager.RobotManager;

public class BlueCrossLineAuto extends BaseImperativeAuto {
  public BlueCrossLineAuto(RobotManager robot, Trailblazer trailblazer) {}

  @Override
  public Pose2d getStartingPose() {
    // TODO: Implement
    return Pose2d.kZero;
  }
}
