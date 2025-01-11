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
  NET(999),
  CORAL_L1(999),
  CORAL_L2(999),
  CORAL_L3(999),
  CORAL_L4(999);
  final double value;

  private ElevatorState(double position) {
    this.value = position;
  }
}
