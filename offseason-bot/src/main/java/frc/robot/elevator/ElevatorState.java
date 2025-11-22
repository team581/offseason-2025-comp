package frc.robot.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ElevatorState {
  UNTUNED(0.0),

  PRE_MATCH_HOMING(10.0),
  UNJAM(20.0),

  ALGAE_INTAKE_GROUND(2.0),
  ALGAE_INTAKE_L2(14.5),
  ALGAE_INTAKE_L3(30),

  ALGAE_NET(51.25),
  PROCESSOR(3.0),
  ALGAE_OUTTAKE(15.0),

  STOWED_CORAL(10.5),
  STOWED_ALGAE(3.0),
  PRE_CORAL_HANDOFF(6.0),
  HANDOFF(3.0),

  L1_SCORE_LINEUP(13.5),
  L1_SCORE_RELEASE(11.5),

  L2_SCORE_LINEUP(8.5),
  L2_SCORE_RELEASE(8.5),

  L3_SCORE_LINEUP(21.5),
  L3_SCORE_RELEASE(21.5),

  L4_SCORE_LINEUP(46.5),
  L4_SCORE_RELEASE(46.5),

  CLIMBING(3.0);

  public final double defaultHeight;
  private final DoubleSubscriber tunableHeight;

  public double getHeight() {
    return tunableHeight.get();
  }

  private ElevatorState(double height) {
    this.defaultHeight = height;
    this.tunableHeight = DogLog.tunable("Elevator/State/" + name(), height);
  }
}
