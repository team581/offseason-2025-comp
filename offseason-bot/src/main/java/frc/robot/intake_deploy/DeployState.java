package frc.robot.intake_deploy;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum DeployState {
  UNTUNED(0.0),
  UNHOMED(UNTUNED),
  OUTTAKE(-5),

  REHOME(UNTUNED),

  STOWED(120),
  HANDOFF(120),
  FLOOR_INTAKE(-5),
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
