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
import java.util.ArrayDeque;
import java.util.List;

public class StationAuto4pc extends BaseImperativeAuto<StationAndLollipop5pcAutoState> {
  private static final AutoConstraintOptions CONSTRAINTS = new AutoConstraintOptions(2, 57, 4, 45);

  private AutoSegment path =
      new AutoSegment(
          CONSTRAINTS,
          new AutoPoint(getStartingPose()),
          new AutoPoint(StationAndLollipop5pcAutoState.J_L4_SCORING.pose));
  private final ArrayDeque<StationAndLollipop5pcAutoState> nextScoringPositions =
      new ArrayDeque<StationAndLollipop5pcAutoState>(
          List.of(
              // StationAndLollipop5pcAutoState.I_L4_LINEUP,
              StationAndLollipop5pcAutoState.J_L4_SCORING,
              StationAndLollipop5pcAutoState.K_L4_SCORING,
              StationAndLollipop5pcAutoState.L_L4_SCORING));

  public void createPath(Pose2d goalPose) {

    path =
        new AutoSegment(
            CONSTRAINTS,
            new AutoPoint(robotManager.localization.getPose()),
            new AutoPoint(goalPose));
    DogLog.timestamp("StateMachineAuto/newPathGenerated");
    DogLog.log("StateMachineAuto/newPathGoalPose", goalPose);
  }

  public StationAuto4pc(RobotManager robot, Trailblazer trailblazer) {
    super(StationAndLollipop5pcAutoState.J_L4_SCORING, robot, trailblazer);
  }

  @Override
  public Pose2d getStartingPose() {
    return Points.START_R1_AND_B1.getPose();
  }

  private boolean superstructureAtGoal() {
    return robotManager.arm.atGoal() && robotManager.elevator.atGoal();
  }

  private StationAndLollipop5pcAutoState getNextReefPosition() {
    StationAndLollipop5pcAutoState nextScoringPosition = nextScoringPositions.pop();
    return nextScoringPosition;
  }

  @Override
  protected StationAndLollipop5pcAutoState getNextState(
      StationAndLollipop5pcAutoState currentState) {
    DogLog.timestamp("StateMachineAuto/gotNewState");
    DogLog.log(
        "StateMachineAuto/trailblazerFollowSegmentIsFinished",
        trailblazer.followSegmentIsFinished(path));
    return switch (currentState) {
      case I_L4_SCORING ->
          !robotManager.claw.getHasGP()
              ? StationAndLollipop5pcAutoState.I_L4_POST_SCORING
              : currentState;
      case I_L4_POST_SCORING ->
          superstructureAtGoal() && trailblazer.followSegmentIsFinished(path)
              ? StationAndLollipop5pcAutoState.PRE_INTAKING
              : currentState;

      case PRE_INTAKING ->
          superstructureAtGoal() && trailblazer.followSegmentIsFinished(path)
              ? StationAndLollipop5pcAutoState.INTAKING
              : currentState;
      case INTAKING ->
          (superstructureAtGoal() && trailblazer.followSegmentIsFinished(path))
                  || robotManager.claw.getHasGP()
              ? StationAndLollipop5pcAutoState.POST_INTAKING
              : currentState;
      case POST_INTAKING ->
          superstructureAtGoal() && trailblazer.followSegmentIsFinished(path)
              ? getNextReefPosition()
              : currentState;
      case J_L4_SCORING ->
          !robotManager.claw.getHasGP()
              ? StationAndLollipop5pcAutoState.J_L4_POST_SCORING
              : currentState;
      case J_L4_POST_SCORING ->
          superstructureAtGoal() && trailblazer.followSegmentIsFinished(path)
              ? StationAndLollipop5pcAutoState.PRE_INTAKING
              : currentState;
      case K_L4_SCORING ->
          !robotManager.claw.getHasGP()
              ? StationAndLollipop5pcAutoState.K_L4_POST_SCORING
              : currentState;

      case K_L4_POST_SCORING ->
          superstructureAtGoal() && trailblazer.followSegmentIsFinished(path)
              ? StationAndLollipop5pcAutoState.PRE_INTAKING
              : currentState;

      case L_L4_SCORING ->
          !robotManager.claw.getHasGP()
              ? StationAndLollipop5pcAutoState.L_L4_POST_SCORING
              : currentState;

      case L_L4_POST_SCORING ->
          superstructureAtGoal() && trailblazer.followSegmentIsFinished(path)
              ? StationAndLollipop5pcAutoState.PRE_INTAKING
              : currentState;
      case A_L4_SCORING ->
          throw new UnsupportedOperationException("Unimplemented case: " + currentState);
      case A_L4_POST_SCORING ->
          throw new UnsupportedOperationException("Unimplemented case: " + currentState);

      case B_L4_SCORING ->
          throw new UnsupportedOperationException("Unimplemented case: " + currentState);
      case B_L4_POST_SCORING ->
          throw new UnsupportedOperationException("Unimplemented case: " + currentState);
      case LOLLIPOP_2 ->
          throw new UnsupportedOperationException("Unimplemented case: " + currentState);
      case POST_LOLLIPOP_2 ->
          throw new UnsupportedOperationException("Unimplemented case: " + currentState);
      case PRE_LOLLIPOP_2 ->
          throw new UnsupportedOperationException("Unimplemented case: " + currentState);
      default -> throw new IllegalArgumentException("Unexpected value: " + currentState);
    };
  }

  @Override
  protected void afterTransition(StationAndLollipop5pcAutoState newState) {
    switch (newState) {
      case A_L4_SCORING, B_L4_SCORING, I_L4_SCORING, J_L4_SCORING, K_L4_SCORING, L_L4_SCORING -> {
        robotManager.scoreRequest();
        robotManager.groundManager.intakeThenHandoffRequest();
      }

      case A_L4_POST_SCORING,
          B_L4_POST_SCORING,
          I_L4_POST_SCORING,
          J_L4_POST_SCORING,
          K_L4_POST_SCORING,
          L_L4_POST_SCORING -> {
        createPath(newState.pose);
        trailblazer.followSegmentInit(path);
        robotManager.stowRequest();
      }
      case PRE_INTAKING -> {
        createPath(newState.pose);
        trailblazer.followSegmentInit(path);
        robotManager.stowRequest();
      }
      case INTAKING -> {
        createPath(newState.pose);
        trailblazer.followSegmentInit(path);
        robotManager.groundManager.intakeThenHandoffRequest();
      }
      case POST_INTAKING -> {
        createPath(newState.pose);
        trailblazer.followSegmentInit(path);
        robotManager.groundManager.intakeThenHandoffRequest();
      }
      case PRE_LOLLIPOP_2 -> {
        createPath(newState.pose);
        trailblazer.followSegmentInit(path);
        robotManager.stowRequest();
      }

      case POST_LOLLIPOP_2 -> {
        createPath(newState.pose);
        trailblazer.followSegmentInit(path);
        robotManager.groundManager.intakeThenHandoffRequest();
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
      case A_L4_SCORING, B_L4_SCORING, I_L4_SCORING, J_L4_SCORING, K_L4_SCORING, L_L4_SCORING -> {
        robotManager.scoreRequest();
        robotManager.groundManager.intakeThenHandoffRequest();
      }

      case A_L4_POST_SCORING,
          B_L4_POST_SCORING,
          I_L4_POST_SCORING,
          J_L4_POST_SCORING,
          K_L4_POST_SCORING,
          L_L4_POST_SCORING -> {
        trailblazer.followSegmentPeriodic(path);
        robotManager.stowRequest();
      }
      case PRE_INTAKING -> {
        trailblazer.followSegmentPeriodic(path);
        robotManager.stowRequest();
      }
      case INTAKING -> {
        trailblazer.followSegmentPeriodic(path);
        robotManager.groundManager.intakeThenHandoffRequest();
      }
      case POST_INTAKING -> {
        trailblazer.followSegmentPeriodic(path);
        robotManager.groundManager.intakeThenHandoffRequest();
      }
      case PRE_LOLLIPOP_2 -> {
        trailblazer.followSegmentPeriodic(path);
        robotManager.stowRequest();
      }

      case LOLLIPOP_2 -> {
        trailblazer.followSegmentPeriodic(path);
        robotManager.groundManager.intakeThenHandoffRequest();
      }

      case POST_LOLLIPOP_2 -> {
        trailblazer.followSegmentPeriodic(path);
        robotManager.groundManager.intakeThenHandoffRequest();
      }
    }
  }
}
