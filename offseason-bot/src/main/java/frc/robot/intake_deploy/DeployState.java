package frc.robot.intake_deploy;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum DeployState {
  UNTUNED(0.0),
  UNHOMED(UNTUNED),
  OUTTAKE(0),

  REHOME(UNTUNED),

  STOWED(120),
  HANDOFF(120),
  CLIMING(10),
  FLOOR_INTAKE(0),
  L1_SCORE(UNTUNED);

  private final double defaultAngle;
  private final DoubleSubscriber tunableAngle;

  DeployState(double angle) {
    this.defaultAngle = angle;
    this.tunableAngle = DogLog.tunable("Deploy/State/" + name(), angle);
  }

  DeployState(DeployState other) {
    this(other.defaultAngle);
  }

  public double getAngle() {
    return tunableAngle.get();
  }
}
