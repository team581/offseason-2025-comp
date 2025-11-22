package frc.robot.autos.auto_state_machines;

import com.team581.trailblazer.Trailblazer;
import com.team581.util.FmsUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.Points;
import frc.robot.autos.auto_state_machines.auto_states.DoNothingAutoState;
import frc.robot.robot_manager.RobotManager;

public class DoNothingAuto extends BaseImperativeAuto<DoNothingAutoState> {
  public DoNothingAuto(RobotManager robot, Trailblazer trailblazer) {
    super(DoNothingAutoState.DO_NOTHING, robot, trailblazer);
  }

  @Override
  public Pose2d getStartingPose() {
    return FmsUtil.isRedAlliance()
        ? Points.START_R2_AND_B2.point.redPose()
        : Points.START_R5_AND_B5.point.bluePose();
  }

  @Override
  protected DoNothingAutoState getNextState(DoNothingAutoState currentState) {
    return DoNothingAutoState.DO_NOTHING;
  }
}
