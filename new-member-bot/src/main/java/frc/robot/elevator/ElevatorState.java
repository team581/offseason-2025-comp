package frc.robot.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ElevatorState {
  UNTUNED(0),
  UNJAM(UNTUNED),
  PRE_MATCH_HOMING(UNTUNED),
  STOWED(0),
  ALGAE_INTAKE_GROUND(0.5),
  ALGAE_OUTTAKE(UNTUNED),
  ALGAE_INTAKE_L2(19.0),
  ALGAE_INTAKE_L3(40.0),
  ALGAE_NET(99.0),
  PROCESSOR(0),
  CORAL_INTAKE(0),
  CORAL_SCORE_LINEUP_L1(0.0),
  CORAL_SCORE_RELEASE_L1(0.0),
  CLIMBING(0.0);

  private final double defaultHeight;
  private final DoubleSubscriber tunableHeight;

  private ElevatorState(double height) {
    this.defaultHeight = height;
    this.tunableHeight = DogLog.tunable("Elevator/State/" + name(), height);
  }

  ElevatorState(ElevatorState other) {
    this(other.defaultHeight);
  }

  public double getHeight() {
    return tunableHeight.get();
  }
}
