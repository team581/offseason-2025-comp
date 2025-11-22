package frc.robot.autos.auto_state_machines;

import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.AutoSegment;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.constraints.AutoConstraintOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.Points;
import frc.robot.autos.auto_state_machines.auto_states.CrossLineAutoState;
import frc.robot.robot_manager.RobotManager;

public class BlueCrossLineAuto extends BaseImperativeAuto<CrossLineAutoState> {
  private static final AutoConstraintOptions CONSTRAINTS = new AutoConstraintOptions(2, 57, 4, 30);

  private final AutoSegment crossLinePath =
      new AutoSegment(
          CONSTRAINTS,
          new AutoPoint(getStartingPose()),
          new AutoPoint(new Pose2d(6.924, 7.292, Rotation2d.kZero)));

  public BlueCrossLineAuto(RobotManager robot, Trailblazer trailblazer) {
    super(CrossLineAutoState.DRIVE_TO_POINT, robot, trailblazer);
  }

  @Override
  public Pose2d getStartingPose() {
    return Points.START_R1_AND_B1.point.bluePose();
  }

  @Override
  protected void afterTransition(CrossLineAutoState newState) {
    switch (newState) {
      case DRIVE_TO_POINT -> trailblazer.followSegmentInit(crossLinePath);
    }
  }

  @Override
  protected void whileInState(CrossLineAutoState state) {
    switch (state) {
      case DRIVE_TO_POINT -> trailblazer.followSegmentPeriodic(crossLinePath);
    }
  }
}
