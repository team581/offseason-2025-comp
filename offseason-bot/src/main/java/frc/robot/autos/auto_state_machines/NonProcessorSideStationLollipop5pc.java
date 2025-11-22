package frc.robot.autos.auto_state_machines;

import com.google.common.collect.ImmutableList;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.AutoSegment;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.constraints.AutoConstraintOptions;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto_align.poses.ReefPipe;
import frc.robot.auto_align.poses.ReefPipeLevel;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.Points;
import frc.robot.autos.auto_state_machines.auto_states.NonProcessorSideStationLollipopAutoState;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import java.util.ArrayDeque;

public class NonProcessorSideStationLollipop5pc
    extends BaseImperativeAuto<NonProcessorSideStationLollipopAutoState> {
  private static final AutoConstraintOptions CONSTRAINTS = new AutoConstraintOptions(6, 57, 6, 45);

  private AutoSegment path = new AutoSegment();

  private boolean isFirstScore = true;

  private ReefPipe currentPipe = ReefPipe.PIPE_I;
  private final ArrayDeque<ReefPipe> nextScoringPositions =
      new ArrayDeque<ReefPipe>(
          ImmutableList.of(ReefPipe.PIPE_K, ReefPipe.PIPE_L, ReefPipe.PIPE_A, ReefPipe.PIPE_B));

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
    DogLog.log("StateMachineAuto/IntermediaryPose", intermediaryPose);
  }

  public NonProcessorSideStationLollipop5pc(RobotManager robot, Trailblazer trailblazer) {
    super(NonProcessorSideStationLollipopAutoState.SCORE, robot, trailblazer);
  }

  @Override
  public Pose2d getStartingPose() {
    return Points.NON_PROCESSOR_SIDE_START_ANGLED.getPose();
  }

  private boolean superstructureAtGoal() {
    return robotManager.arm.atGoal() && robotManager.elevator.atGoal();
  }

  private static NonProcessorSideStationLollipopAutoState getNextIntakeState(
      ReefPipe scorePosition) {
    if (scorePosition == ReefPipe.PIPE_A) {
      DogLog.log(
          "StateMachineAuto/nextIntakeState", NonProcessorSideStationLollipopAutoState.LOLLIPOP_2);

      return NonProcessorSideStationLollipopAutoState.LOLLIPOP_2;
    }
    DogLog.log(
        "StateMachineAuto/nextIntakeState", NonProcessorSideStationLollipopAutoState.INTAKING);

    return NonProcessorSideStationLollipopAutoState.INTAKING;
  }

  @Override
  protected NonProcessorSideStationLollipopAutoState getNextState(
      NonProcessorSideStationLollipopAutoState currentState) {
    DogLog.timestamp("StateMachineAuto/gotNewState");
    DogLog.log(
        "StateMachineAuto/trailblazerFollowSegmentIsFinished",
        trailblazer.followSegmentIsFinished(path));
    return switch (currentState) {
      case SCORE ->
          DriverStation.isEnabled()
                  && ((!robotManager.claw.getHasGP()
                          && robotManager.getState() == RobotState.CORAL_L4_RELEASE)
                      || (timeout(1.5)
                          && !robotManager.claw.getHasGP()
                          && !robotManager.groundManager.getTopHasGP()
                          && !robotManager.groundManager.getBottomHasGP()))
              ? getNextIntakeState(currentPipe)
              : currentState;

      case INTAKING ->
          ((superstructureAtGoal() && trailblazer.followSegmentIsFinished(path))
                      || robotManager.claw.getHasGP())
                  && !nextScoringPositions.isEmpty()
              ? NonProcessorSideStationLollipopAutoState.SCORE
              : currentState;

      case LOLLIPOP_2 ->
          ((superstructureAtGoal() && trailblazer.followSegmentIsFinished(path))
                  || robotManager.claw.getHasGP())
              ? NonProcessorSideStationLollipopAutoState.SCORE
              : currentState;
    };
  }

  @Override
  protected void beforeTransition(
      NonProcessorSideStationLollipopAutoState oldState,
      NonProcessorSideStationLollipopAutoState newState) {
    if (oldState == NonProcessorSideStationLollipopAutoState.SCORE
        && (robotManager.getState() == RobotState.CORAL_L4_RELEASE
            || robotManager.getState() == RobotState.CLAW_EMPTY)) {

      if (!nextScoringPositions.isEmpty()) {
        DogLog.log("StateMachineAuto/NextScoringPosition", nextScoringPositions.getFirst());
        currentPipe = nextScoringPositions.pop();
      }
    }
  }

  @Override
  protected void afterTransition(NonProcessorSideStationLollipopAutoState newState) {
    switch (newState) {
      case SCORE -> {
        if (isFirstScore) {
          robotManager.groundManager.rehomeRequest();
          isFirstScore = false;
        }
        robotManager.scoreRequest(currentPipe, ReefPipeLevel.L4);
        robotManager.groundManager.intakeRequest();
      }

      case INTAKING -> {
        createPath(
            newState.getPose(), Points.PRE_GROUND_INTAKE_NON_PROCESSOR_SIDE_STATION.getPose());
        trailblazer.followSegmentInit(path);
        robotManager.stowRequest();
        robotManager.groundManager.intakeRequest();
      }

      case LOLLIPOP_2 -> {
        createPath(newState.getPose());
        trailblazer.followSegmentInit(path);
        robotManager.stowRequest();
        robotManager.groundManager.intakeRequest();
      }
    }
  }

  @Override
  protected void whileInState(NonProcessorSideStationLollipopAutoState state) {
    switch (state) {
      case SCORE -> {}

      case INTAKING -> {
        trailblazer.followSegmentPeriodic(path);
      }

      case LOLLIPOP_2 -> {
        trailblazer.followSegmentPeriodic(path);
      }
    }
  }
}
