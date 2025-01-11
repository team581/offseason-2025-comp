package frc.robot.elevator;

public enum ElevatorState {
  STOWED(0),
  PRE_MATCH_HOMING(999),
  CLIMBING(999),
  PROCESSOR(999),
  INTAKE_CORAL_STATION(999),
  GROUND_CORAL_INTAKE(999),
  GROUND_ALGAE_INTAKE(999),
  UNJAM(999),
  ALGAE_DISLODGE_L2(999),
  ALGAE_DISLODGE_L3(999),
  ALGAE_INTAKE_L2(999),
  ALGAE_INTAKE_L3(999),
  NET(999),
  CORAL_L1(999),
  CORAL_L2(999),
  CORAL_L3(999),
  CORAL_L4(999),
  COLLISION_AVOIDANCE(0);
  // collision avoidance is a special state used for collision avoidance (duh)
  // another class will call a function to set the goal for collision avoidance mode
  // so, elevator.setCollisionAvoidanceGoal(double height)
  // this goal is used in the COLLISION_AVOIDANCE state
  // so that we can have very specific control over exact elevator position to avoid bonking the arm
  final double value;

  private ElevatorState(double position) {
    this.value = position;
  }
}
