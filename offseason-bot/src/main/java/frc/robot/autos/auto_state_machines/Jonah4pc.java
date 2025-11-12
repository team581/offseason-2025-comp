package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.v2.AutoSegment;
import com.team581.trailblazer.v2.Trailblazer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto_align.poses.ReefPipe;
import frc.robot.auto_align.poses.ReefPipeLevel;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.Points;
import frc.robot.autos.auto_state_machines.auto_states.Jonah4pcAutoState;
import frc.robot.robot_manager.RobotManager;

public class Jonah4pc extends BaseImperativeAuto<Jonah4pcAutoState> {
  private static final AutoSegment INTAKE_LOLLIPOP_PATH =
      Trailblazer.segment(Points.LOLLIPOP_2.point).withLinearConstraints(4, 3).forever();
  private static final AutoSegment INTAKE_PATH =
      Trailblazer.segment(
              Points.GROUND_INTAKE_PROCESSOR_SIDE_STATION.point,
              Points.PRE_GROUND_INTAKE_PROCESSOR_SIDE_STATION.point)
          .withLinearConstraints(4, 3)
          .untilFinished(new PoseErrorTolerance(0.1, 10));

  public Jonah4pc(RobotManager robotManager, Trailblazer trailblazer) {
    super(Jonah4pcAutoState.SCORE_PRELOAD, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Points.PROCESSOR_SIDE_START_ANGLED.point;
  }

  @Override
  protected Jonah4pcAutoState getNextState(Jonah4pcAutoState currentState) {
    if (DriverStation.isDisabled()) {
      return currentState;
    }

    return switch (currentState) {
      case SCORE_PRELOAD ->
          robotManager.claw.getHasGP() ? currentState : Jonah4pcAutoState.INTAKE_SECOND_PIECE;
      case INTAKE_SECOND_PIECE ->
          robotManager.claw.getHasGP() || (superstructureAtGoal() && trailblazer.atGoal())
              ? Jonah4pcAutoState.SCORE_SECOND_PIECE
              : currentState;
      case SCORE_SECOND_PIECE ->
          robotManager.claw.getHasGP() ? currentState : Jonah4pcAutoState.INTAKE_THIRD_PIECE;
      case INTAKE_THIRD_PIECE ->
          robotManager.claw.getHasGP() || (superstructureAtGoal() && trailblazer.atGoal())
              ? Jonah4pcAutoState.SCORE_THIRD_PIECE
              : currentState;
      case SCORE_THIRD_PIECE ->
          robotManager.claw.getHasGP() ? currentState : Jonah4pcAutoState.INTAKE_FOURTH_PIECE;
      case INTAKE_FOURTH_PIECE ->
          robotManager.claw.getHasGP() || (superstructureAtGoal() && trailblazer.atGoal())
              ? Jonah4pcAutoState.SCORE_FOURTH_PIECE
              : currentState;
      case SCORE_FOURTH_PIECE ->
          robotManager.claw.getHasGP() ? currentState : Jonah4pcAutoState.INTAKE_LOLLIPOP;
      case INTAKE_LOLLIPOP -> currentState;
    };
  }

  @Override
  protected void afterTransition(Jonah4pcAutoState newState) {
    switch (newState) {
      case SCORE_PRELOAD -> {
        robotManager.groundManager.rehomeRequest();
        robotManager.scoreRequest(ReefPipe.PIPE_F, ReefPipeLevel.L4);
        robotManager.groundManager.intakeRequest();
      }
      case INTAKE_SECOND_PIECE, INTAKE_THIRD_PIECE, INTAKE_FOURTH_PIECE -> {
        robotManager.groundManager.intakeRequest();
        trailblazer.followSegment(INTAKE_PATH);
      }
      case SCORE_SECOND_PIECE -> {
        robotManager.scoreRequest(ReefPipe.PIPE_D, ReefPipeLevel.L4);
        robotManager.groundManager.intakeRequest();
      }
      case SCORE_THIRD_PIECE -> {
        robotManager.scoreRequest(ReefPipe.PIPE_C, ReefPipeLevel.L4);
        robotManager.groundManager.intakeRequest();
      }
      case SCORE_FOURTH_PIECE -> {
        robotManager.scoreRequest(ReefPipe.PIPE_E, ReefPipeLevel.L4);
        robotManager.groundManager.intakeRequest();
      }
      case INTAKE_LOLLIPOP -> {
        robotManager.groundManager.intakeRequest();
        trailblazer.followSegment(INTAKE_PATH);
      }
    }
  }

  private boolean superstructureAtGoal() {
    return robotManager.arm.atGoal() && robotManager.elevator.atGoal();
  }
}
