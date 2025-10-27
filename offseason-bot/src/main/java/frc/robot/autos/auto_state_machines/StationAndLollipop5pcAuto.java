package frc.robot.autos.auto_state_machines;

import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.AutoSegment;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.constraints.AutoConstraintOptions;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.Points;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;

public class StationAndLollipop5pcAuto extends BaseImperativeAuto<StationAndLollipop5pcAutoState> {
  private static final AutoConstraintOptions CONSTRAINTS = new AutoConstraintOptions(2, 57, 4, 45);

  private AutoSegment path =
      new AutoSegment(
          CONSTRAINTS, new AutoPoint(getStartingPose()), new AutoPoint(getStartingPose()));

  private final int scoreCounter = 0;

  public void createPath(Pose2d goalPose) {
    path =
        new AutoSegment(
            CONSTRAINTS,
            new AutoPoint(robotManager.localization.getPose()),
            new AutoPoint(goalPose));
    DogLog.timestamp("StateMachineAuto/newPathGenerated");
    DogLog.log("StateMachineAuto/newPathGoalPose", goalPose);
  }

  public void createPath(Pose2d goalPose, Pose2d intermediaryPose) {

    path =
        new AutoSegment(
            CONSTRAINTS,
            new AutoPoint(robotManager.localization.getPose()),
            new AutoPoint(intermediaryPose),
            new AutoPoint(goalPose));
    DogLog.timestamp("StateMachineAuto/newPathGenerated");
    DogLog.log("StateMachineAuto/newPathGoalPose", goalPose);
  }

  public StationAndLollipop5pcAuto(RobotManager robot, Trailblazer trailblazer) {
    super(StationAndLollipop5pcAutoState.SCORE, robot, trailblazer);
  }

  @Override
  public Pose2d getStartingPose() {
    return Points.START_R1_AND_B1.getPose();
  }

  private boolean superstructureAtGoal() {
    return robotManager.arm.atGoal() && robotManager.elevator.atGoal();
  }

  private StationAndLollipop5pcAutoState getNextIntakeState() {
    DogLog.log("StateMachineAuto/scoreCounter", scoreCounter);
    if (scoreCounter >= 4) {
      return StationAndLollipop5pcAutoState.PRE_LOLLIPOP_2;
    }
    return StationAndLollipop5pcAutoState.INTAKING;
  }

  // private StationAndLollipop5pcAutoState getNextReefPosition() {
  //   StationAndLollipop5pcAutoState nextScoringPosition = nextScoringPositions.pop();
  //   return nextScoringPosition;
  // }

  @Override
  protected StationAndLollipop5pcAutoState getNextState(
      StationAndLollipop5pcAutoState currentState) {
    DogLog.timestamp("StateMachineAuto/gotNewState");
    DogLog.log(
        "StateMachineAuto/trailblazerFollowSegmentIsFinished",
        trailblazer.followSegmentIsFinished(path));
    return switch (currentState) {
      case SCORE ->
          !robotManager.claw.getHasGP()
                  && robotManager.getState().equals(RobotState.CORAL_L4_RELEASE)
              ? getNextIntakeState()
              : currentState;

      case INTAKING ->
          (superstructureAtGoal() && trailblazer.followSegmentIsFinished(path))
                  || robotManager.claw.getHasGP()
              ? StationAndLollipop5pcAutoState.SCORE
              : currentState;

      case PRE_LOLLIPOP_2 ->
          superstructureAtGoal() && trailblazer.followSegmentIsFinished(path)
              ? StationAndLollipop5pcAutoState.LOLLIPOP_2
              : currentState;
      case LOLLIPOP_2 ->
          superstructureAtGoal() && trailblazer.followSegmentIsFinished(path)
              ? StationAndLollipop5pcAutoState.SCORE
              : currentState;
    };
  }

  @Override
  protected void afterTransition(StationAndLollipop5pcAutoState newState) {
    switch (newState) {
      case SCORE -> {
        robotManager.scoreRequest();
        robotManager.groundManager.intakeThenHandoffRequest();
      }

      case INTAKING -> {
        createPath(newState.pose);
        trailblazer.followSegmentInit(path);
        robotManager.groundManager.intakeThenHandoffRequest();
      }

      case PRE_LOLLIPOP_2 -> {
        createPath(newState.pose);
        trailblazer.followSegmentInit(path);
      }

      case LOLLIPOP_2 -> {
        createPath(newState.pose);
        trailblazer.followSegmentInit(path);
        robotManager.groundManager.intakeThenHandoffRequest();
      }
    }
  }

  @Override
  protected void whileInState(StationAndLollipop5pcAutoState state) {
    switch (state) {
      case SCORE -> {}

      case INTAKING -> {
        trailblazer.followSegmentPeriodic(path);
      }

      case PRE_LOLLIPOP_2 -> {
        trailblazer.followSegmentPeriodic(path);
      }

      case LOLLIPOP_2 -> {
        trailblazer.followSegmentPeriodic(path);
      }
    }
  }
}
