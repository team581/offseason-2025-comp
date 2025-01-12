package frc.robot.elevator;

public enum ElevatorState {
  STOWED(0),
  PRE_MATCH_HOMING(0),
  CLIMBING(0),
  PROCESSOR(0),
  INTAKING_CORAL_STATION(0),
  GROUND_CORAL_INTAKE(0),
  GROUND_ALGAE_INTAKE(0),
  UNJAM(0),
  ALGAE_DISLODGE_L2(0),
  ALGAE_DISLODGE_L3(0),
  ALGAE_INTAKE_L2(0),
  ALGAE_INTAKE_L3(0),
  NET(0),
  CORAL_L1(0),
  CORAL_L2(0),
  CORAL_L3(0),
  CORAL_L4(0),
  COLLISION_AVOIDANCE(0);
  // collision avoidance is a special state used for collision avoidance (duh)
  // another class will call a function to set the goal for collision avoidance mode
  // so, elevator.setCollisionAvoidanceGoal(double height)
  // this goal is used in the COLLISION_AVOIDANCE state
  // so that we can have very specific control over exact elevator position to avoid bonking the arm

  public final double value;

  private ElevatorState(double position) {
    this.value = position;
  }
}
