package frc.robot.wrist;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum WristState {
  UNTUNED(0),
  UNJAM(UNTUNED),
  PRE_MATCH_HOMING(UNTUNED),
  MID_MATCH_HOMING(UNTUNED),
  STOWED(76.0),

  ALGAE_INTAKE_GROUND(-4.0),
  ALGAE_OUTTAKE(UNTUNED),
  ALGAE_INTAKE_L2(0),
  ALGAE_INTAKE_L3(0),
  ALGAE_NET(0),
  ALGAE_PROCESSOR(0),

  CORAL_INTAKE(-2.0),
  CORAL_SCORE_LINEUP_L1(0.0),
  CORAL_SCORE_RELEASE_L1(0.0),
  CLIMBING(0.0);

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
