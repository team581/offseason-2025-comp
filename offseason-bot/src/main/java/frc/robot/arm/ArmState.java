package frc.robot.arm;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ArmState {
  /**
   * @deprecated This is a placeholder state.
   */
  @Deprecated
  UNTUNED(0),

  PRE_MATCH_HOMING(0),

  UNJAM(UNTUNED),

  // Idle positions
  STOWED(-90),
  ALGAE_STOWED(90),

  // coral positions
  CORAL_SCORE_LINEUP_L2(30.0),
  CORAL_SCORE_RELEASE_L2(5.0),

  CORAL_SCORE_LINEUP_L3(30.0),
  CORAL_SCORE_RELEASE_L3(5.0),

  CORAL_SCORE_LINEUP_L4(50.0),
  CORAL_SCORE_RELEASE_L4(10.0),

  CORAL_SCORE_LINEUP_L1(35.0),
  CORAL_SCORE_RELEASE_L1(35.0),

  // Handoffs
  CORAL_HANDOFF(-90.0),

  // Algae positions
  ALGAE_INTAKE_FLOOR(UNTUNED),

  ALGAE_INTAKE_L2(0.0),
  ALGAE_INTAKE_L3(0.0),

  ALGAE_NET(50),

  ALGAE_PROCESSOR(-20.0),

  ALGAE_OUTTAKE(UNTUNED),

  CLIMBING(UNTUNED);

  private final double defaultAngle;
  private final DoubleSubscriber tunableAngle;

  ArmState(double angle) {
    this.defaultAngle = angle;
    this.tunableAngle = DogLog.tunable("Arm/State/" + name(), angle);
  }

  ArmState(ArmState other) {
    this(other.defaultAngle);
  }

  public double getAngle() {
    return tunableAngle.get();
  }
}
