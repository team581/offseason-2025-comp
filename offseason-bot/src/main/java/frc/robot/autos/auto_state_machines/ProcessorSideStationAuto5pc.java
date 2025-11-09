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
import frc.robot.autos.auto_state_machines.auto_states.ProcessorSideStationAutoState;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import java.util.ArrayDeque;

public class ProcessorSideStationAuto5pc extends BaseImperativeAuto<ProcessorSideStationAutoState> {
  private static final AutoConstraintOptions CONSTRAINTS = new AutoConstraintOptions(6, 57, 6, 45);

  private AutoSegment path = new AutoSegment();

  private boolean isFirstScore = true;
  private boolean justFinishedScore = true;

  private final ArrayDeque<ReefPipe> nextScoringPositions =
      new ArrayDeque<ReefPipe>(
          ImmutableList.of(
              ReefPipe.PIPE_F, ReefPipe.PIPE_D, ReefPipe.PIPE_C, ReefPipe.PIPE_E, ReefPipe.PIPE_C));
  private final ArrayDeque<ReefPipeLevel> nextScoringLevel =
      new ArrayDeque<ReefPipeLevel>(
          ImmutableList.of(
              ReefPipeLevel.L4,
              ReefPipeLevel.L4,
              ReefPipeLevel.L4,
              ReefPipeLevel.L4,
              ReefPipeLevel.L1));

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
    DogLog.log("StateMachineAuto/intermediate newPathGoalPose", goalPose);
    DogLog.log("StateMachineAuto/IntermediaryPose", intermediaryPose);
  }

  public ProcessorSideStationAuto5pc(RobotManager robot, Trailblazer trailblazer) {
    super(ProcessorSideStationAutoState.SCORE, robot, trailblazer);
  }

  @Override
  public Pose2d getStartingPose() {
    return Points.PROCESSOR_SIDE_START_ANGLED.getPose();
  }

  private boolean superstructureAtGoal() {
    return robotManager.arm.atGoal() && robotManager.elevator.atGoal();
  }

  @Override
  protected ProcessorSideStationAutoState getNextState(ProcessorSideStationAutoState currentState) {
    DogLog.log(
        "StateMachineAuto/trailblazerFollowSegmentIsFinished",
        trailblazer.followSegmentIsFinished(path));
    return switch (currentState) {
      case SCORE -> {
        if (DriverStation.isEnabled()) {
          var missingGp =
              (timeout(1.5)
                  && !robotManager.claw.getHasGP()
                  && !robotManager.groundManager.getTopHasGP()
                  && !robotManager.groundManager.getBottomHasGP());
          var l4Done =
              (!robotManager.claw.getHasGP()
                  && (robotManager.getState() == RobotState.CORAL_L4_RELEASE));
          var l1Done = robotManager.getState() == RobotState.CORAL_L1_BACKAWAY;
          DogLog.log("StateMachineAuto/MissingGP", missingGp);
          DogLog.log("StateMachineAuto/l4Done", l4Done);
          DogLog.log("StateMachineAuto/l1Done", l1Done);
          DogLog.log("StateMachineAuto/nextScoringLevel", nextScoringLevel.peek());

          if (missingGp || l4Done || l1Done) {
            yield ProcessorSideStationAutoState.INTAKING;
          }
        }
        yield currentState;
      }
      case INTAKING ->
          ((superstructureAtGoal() && trailblazer.followSegmentIsFinished(path))
                      || robotManager.claw.getHasGP())
                  && (!nextScoringPositions.isEmpty() || !nextScoringLevel.isEmpty())
              ? ProcessorSideStationAutoState.SCORE
              : currentState;
    };
  }

  @Override
  protected void afterTransition(ProcessorSideStationAutoState newState) {
    switch (newState) {
      case SCORE -> {
        if (isFirstScore) {
          robotManager.groundManager.rehomeRequest();
          isFirstScore = false;
        }
        justFinishedScore = true;
        robotManager.scoreRequest(nextScoringPositions.peek(), nextScoringLevel.peek());
        robotManager.groundManager.intakeRequest();
      }
      case INTAKING -> {
        createPath(newState.getPose(), Points.PRE_GROUND_INTAKE_PROCESSOR_SIDE_STATION.getPose());
        trailblazer.followSegmentInit(path);
        robotManager.stowRequest();
        robotManager.groundManager.intakeRequest();
      }
    }
  }

  @Override
  protected void whileInState(ProcessorSideStationAutoState state) {
    switch (state) {
      case SCORE -> {
        if (!nextScoringPositions.isEmpty() && !nextScoringLevel.isEmpty()) {
          DogLog.log("StateMachineAuto/NextScoringPosition", nextScoringPositions.getFirst());
          DogLog.log("StateMachineAuto/NextScoringLevel", nextScoringLevel.getFirst());
        }

        if ((robotManager.getState() == RobotState.CORAL_L4_RELEASE
                || robotManager.getState() == RobotState.CORAL_L1_BACKAWAY)
            && justFinishedScore) {
          nextScoringPositions.remove();
          nextScoringLevel.remove();
          justFinishedScore = false;
        }
      }

      case INTAKING -> {
        trailblazer.followSegmentPeriodic(path);
      }
    }
  }
}
