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
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import java.util.ArrayDeque;

public class StationAuto4pc extends BaseImperativeAuto<AutoState> {
  private static final AutoConstraintOptions CONSTRAINTS = new AutoConstraintOptions(5, 57, 4, 45);

  private AutoSegment path = new AutoSegment();

  private boolean isFirstScore = true;
  private boolean justFinishedScore = true;

  private final ArrayDeque<ReefPipe> nextScoringPositions =
      new ArrayDeque<ReefPipe>(
          ImmutableList.of(ReefPipe.PIPE_I, ReefPipe.PIPE_K, ReefPipe.PIPE_L, ReefPipe.PIPE_J));

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

  public StationAuto4pc(RobotManager robot, Trailblazer trailblazer) {
    super(AutoState.SCORE, robot, trailblazer);
  }

  @Override
  public Pose2d getStartingPose() {
    return Points.START_ANGLED.getPose();
  }

  private boolean superstructureAtGoal() {
    return robotManager.arm.atGoal() && robotManager.elevator.atGoal();
  }

  @Override
  protected AutoState getNextState(AutoState currentState) {
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
              ? AutoState.INTAKING
              : currentState;

      case INTAKING ->
          ((superstructureAtGoal() && trailblazer.followSegmentIsFinished(path))
                      || robotManager.claw.getHasGP())
                  && !nextScoringPositions.isEmpty()
              ? AutoState.SCORE
              : currentState;
    };
  }

  @Override
  protected void afterTransition(AutoState newState) {
    switch (newState) {
      case SCORE -> {
        if (isFirstScore) {
          robotManager.groundManager.rehomeRequest();
          isFirstScore = false;
        }
        justFinishedScore = true;
        robotManager.scoreRequest(nextScoringPositions.peek(), ReefPipeLevel.L4);
        robotManager.groundManager.intakeRequest();
      }
      case INTAKING -> {
        createPath(newState.getPose(), Points.PRE_GROUND_INTAKE_LEFT_STATION.getPose());
        trailblazer.followSegmentInit(path);
        robotManager.stowRequest();
        robotManager.groundManager.intakeRequest();
      }
    }
  }

  @Override
  protected void whileInState(AutoState state) {
    switch (state) {
      case SCORE -> {
        if (!nextScoringPositions.isEmpty()) {
          DogLog.log("StateMachineAuto/NextScoringPosition", nextScoringPositions.getFirst());
        }

        if (robotManager.getState() == RobotState.CORAL_L4_RELEASE && justFinishedScore) {
          nextScoringPositions.remove();
          justFinishedScore = false;
        }
      }

      case INTAKING -> {
        trailblazer.followSegmentPeriodic(path);
      }
    }
  }
}
