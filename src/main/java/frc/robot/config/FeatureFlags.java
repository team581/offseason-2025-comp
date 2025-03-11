package frc.robot.config;

import frc.robot.util.FeatureFlag;
import java.util.function.BooleanSupplier;

public class FeatureFlags {
  public static final BooleanSupplier CORAL_INTAKE_ASSIST =
      FeatureFlag.of("CoralIntakeAssist", true);

  public static final BooleanSupplier PURE_PURSUIT_USE_DYNAMIC_LOOKAHEAD =
      FeatureFlag.of("PurePursuit/UseDynamicLookahead", true);

  public static final BooleanSupplier COLLISION_AVOIDANCE_BOX_SHORTCUTS =
      FeatureFlag.of("CollisionAvoidance/BoxShortcuts", true);

  public static final BooleanSupplier INTAKE_VELOCITY_CORAL_DETECTION =
      FeatureFlag.of("Intake/CoralVelocityDetection", true);

  public static final BooleanSupplier FIELD_CALIBRATION = FeatureFlag.of("FieldCalibration", false);

  // Experiment to see if we can make auto motion smoother by avoiding snaps, since Trailblazer does
  // a good enough job maintaining heading on its own
  public static final BooleanSupplier DISABLE_AUTO_SNAPS =
      FeatureFlag.of("Auto/DisableSnaps", true);

  private FeatureFlags() {}
}
