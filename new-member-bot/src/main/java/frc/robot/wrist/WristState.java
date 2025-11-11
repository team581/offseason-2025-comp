package frc.robot.wrist;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum WristState {
  UNTUNED(0),
  UNJAM(UNTUNED),
  PRE_MATCH_HOMING(UNTUNED),
  STOWED(76.0),
  FULL_STOWED(149.0),
  HOLDING_ALGAE_STOWED(76.0),

  ALGAE_INTAKE_GROUND(-4.0),
  ALGAE_OUTTAKE(UNTUNED),
  ALGAE_INTAKE_L2(19.2),
  ALGAE_INTAKE_L3(19.2),
  ALGAE_NET(55.0),
  ALGAE_PROCESSOR(17.3),

  CORAL_INTAKE(3.0),
  CORAL_SCORE_LINEUP_L1(77.0),
  CORAL_SCORE_RELEASE_L1(77.0),
  CLIMBING(113.0);

  private final double defaultAngle;
  private final DoubleSubscriber tunableAngle;

  WristState(double angle) {
    this.defaultAngle = angle;
    this.tunableAngle = DogLog.tunable("Wrist/State/" + name(), angle);
  }

  WristState(WristState other) {
    this(other.defaultAngle);
  }

  public double getAngle() {
    return tunableAngle.get();
  }
}
