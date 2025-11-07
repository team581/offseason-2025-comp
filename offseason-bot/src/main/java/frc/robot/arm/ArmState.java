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

  UNJAM(-45),

  // Idle positions
  STOWED_CORAL(-70.0),
  STOWED_ALGAE(60),

  // coral positions
  CORAL_SCORE_LINEUP_L2(24.0),
  CORAL_SCORE_RELEASE_L2(-5.0),

  CORAL_SCORE_LINEUP_L3(28.5),
  CORAL_SCORE_RELEASE_L3(-5.0),

  CORAL_SCORE_LINEUP_L4(33.0),
  CORAL_SCORE_RELEASE_L4(0.0),

  CORAL_SCORE_LINEUP_L1(-20.0),
  CORAL_SCORE_RELEASE_L1(-20.0),

  // Handoffs
  CORAL_HANDOFF(-83.0),

  // Algae positions
  ALGAE_INTAKE_FLOOR(-40.0),

  ALGAE_INTAKE_L2(0.0),
  ALGAE_INTAKE_L3(0.0),

  ALGAE_NET(62),
  AFTER_ALGAE_RELEASE_NET(75),

  ALGAE_PROCESSOR(-20.0),

  ALGAE_OUTTAKE(55),

  CLIMBING(89);

  private final DoubleSubscriber tunableAngle;

  ArmState(double angle) {

    this.tunableAngle = DogLog.tunable("Arm/State/" + name(), angle);
  }

  public double getAngle() {
    return tunableAngle.get();
  }
}
