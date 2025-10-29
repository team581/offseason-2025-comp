package frc.robot.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ElevatorState {
  UNTUNED(0.0),

  PRE_MATCH_HOMING(0.0),
  UNJAM(UNTUNED),

  ALGAE_INTAKE_GROUND(10.0),
  ALGAE_INTAKE_L2(10),
  ALGAE_INTAKE_L3(25),

  ALGAE_NET(49.5),
  PROCESSOR(10),
  ALGAE_OUTTAKE(UNTUNED),

  STOWED(10),
  STOWED_ALGAE(10),
  PRE_CORAL_HANDOFF(10),
  HANDOFF(7),

  L1_SCORE_LINEUP(10),
  L1_SCORE_RELEASE(10),

  L2_SCORE_LINEUP(10),
  L2_SCORE_RELEASE(10),

  L3_SCORE_LINEUP(20),
  L3_SCORE_RELEASE(20),

  L4_SCORE_LINEUP(45),
  L4_SCORE_RELEASE(45),

  CLIMBING(UNTUNED);

  public final double defaultHeight;
  private final DoubleSubscriber tunableHeight;

  public double getHeight() {
    return tunableHeight.get();
  }

  private ElevatorState(double height) {
    this.defaultHeight = height;
    this.tunableHeight = DogLog.tunable("Elevator/State/" + name(), height);
  }

  private ElevatorState(ElevatorState other) {
    this(other.defaultHeight);
  }
}
