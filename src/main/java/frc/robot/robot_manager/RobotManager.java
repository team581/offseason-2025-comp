package frc.robot.robot_manager;

import frc.robot.util.state_machines.StateMachine;

public class RobotManager extends StateMachine<RobotState> {

  public RobotManager() {

  }


  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case IDLE_NO_GP,
            IDLE_ALGAE,
            IDLE_CORAL,
            IDLE_BOTH_GP,
            PROCESSOR_WAITING,
            NET_WAITING,
            CORAL_L1_WAITING,
            CORAL_L2_WAITING,
            CORAL_L3_WAITING,
            CORAL_L4_WAITING,
            CLIMBING_1_LINEUP,
            CLIMBING_2_HANGING ->
          currentState;


        //   TODO: add check for PREPARE_TO_SCORE transitions 
        case PROCESSOR_PREPARE_TO_SCORE -> true ? RobotState.PROCESSOR_SCORING : currentState;

        case NET_PREPARE_TO_SCORE -> true ? RobotState.NET_SCORING : currentState;

        case CORAL_L1_PREPARE_TO_SCORE -> true ? RobotState.CORAL_L1_SCORING : currentState;

        case CORAL_L2_PREPARE_TO_SCORE -> true ? RobotState.CORAL_L2_SCORING : currentState;

        case CORAL_L3_PREPARE_TO_SCORE -> true ? RobotState.CORAL_L3_SCORING : currentState;

        case CORAL_L4_PREPARE_TO_SCORE -> true ? RobotState.CORAL_L4_SCORING : currentState;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    // TODO: Implement
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    // Continuous state actions
    switch (getState()) {
      default -> {}
    }
  }

  public void stowRequest() {
    switch (getState()) {
        // TODO: Intaking and unjam should not be IDLE_WITH_GP
      case INTAKING,
              INTAKE_ASSIST,
              AMP_PREPARE_TO_SCORE,
              SPEAKER_PREPARE_TO_SCORE,
              FEEDING_PREPARE_TO_SHOOT,
              PASS_PREPARE_TO_SHOOT,
              AMP_WAITING,
              SPEAKER_WAITING,
              FEEDING_WAITING,
              AMP_SCORING,
              SPEAKER_SCORING,
              FEEDING_SHOOTING,
              PASS_SHOOTING,
              IDLE_WITH_GP,
              UNJAM
          // INTAKING_BACK,
          // INTAKING_FORWARD_PUSH
          ->
          setStateFromRequest(RobotState.IDLE_WITH_GP);
      default -> setStateFromRequest(RobotState.IDLE_NO_GP);
    }
  }

  public void nextClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP -> setStateFromRequest(RobotState.CLIMBING_2_HANGING);
      case CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
    }
  }

  public void previousClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_2_HANGING -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
      case CLIMBING_1_LINEUP -> {
        setStateFromRequest(RobotState.IDLE_WITH_GP);
      }
      default -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
    }
  }

}
