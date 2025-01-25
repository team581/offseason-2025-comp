package frc.robot.elevator;

public enum ElevatorState {
  STOWED(0),
  PRE_MATCH_HOMING(0),

  INTAKING_CORAL_STATION(18.4),
  GROUND_CORAL_INTAKE(0),
  GROUND_ALGAE_INTAKE(10.2),

  ALGAE_DISLODGE_L2(0),
  ALGAE_DISLODGE_L3(0),
  ALGAE_INTAKE_L2(0),
  ALGAE_INTAKE_L3(0),

  NET(58.0),
  PROCESSOR(0),

  CORAL_L1_LINEUP_OR_PLACE(0.0),
  CORAL_L2_LINEUP_OR_PLACE(11.0),
  CORAL_L3_LINEUP_OR_PLACE(26.5),
  CORAL_L4_LINEUP_OR_PLACE(52.6),

  CORAL_L1_RELEASE(0.0),
  CORAL_L2_RELEASE(9.0),
  CORAL_L3_RELEASE(24.5),
  CORAL_L4_RELEASE(50.6),

  UNJAM(0),
  CLIMBING(0),

  COLLISION_AVOIDANCE(0);
  // collision avoidance is a special state used for collision avoidance (duh)
  // another class will call a function to set the goal for collision avoidance mode
  // so, elevator.setCollisionAvoidanceGoal(double height)
  // this goal is used in the COLLISION_AVOIDANCE state
  // so that we can have very specific control over exact elevator position to avoid bonking the arm

  public final double height;

  private ElevatorState(double height) {
    this.height = height;
  }
}
