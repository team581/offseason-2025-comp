package frc.robot.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ElevatorState {
  UNTUNED(0.0),

  PRE_MATCH_HOMING(10.0),
  UNJAM(UNTUNED),

  ALGAE_INTAKE_GROUND(2.0),
  ALGAE_INTAKE_L2(14.5),
  ALGAE_INTAKE_L3(30),

  ALGAE_NET(51.25),
  PROCESSOR(5.0),
  ALGAE_OUTTAKE(15.0),

  STOWED_CORAL(8.5),
  STOWED_ALGAE(3.0),
  PRE_CORAL_HANDOFF(6.0),
  HANDOFF(3.0),

  L1_SCORE_LINEUP(11),
  L1_SCORE_RELEASE(9),

  L2_SCORE_LINEUP(8.5),
  L2_SCORE_RELEASE(8.5),

  L3_SCORE_LINEUP(23.5),
  L3_SCORE_RELEASE(23.5),

  L4_SCORE_LINEUP(46.5),
  L4_SCORE_RELEASE(46.5),

  CLIMBING(0.0);

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
