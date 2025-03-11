package frc.robot.config;

import frc.robot.util.FeatureFlag;
import java.util.function.BooleanSupplier;

public class FeatureFlags {
  public static final BooleanSupplier CORAL_INTAKE_ASSIST =
      FeatureFlag.of("CoralIntakeAssist", true);

  public static final BooleanSupplier PURE_PURSUIT_USE_DYNAMIC_LOOKAHEAD =
      FeatureFlag.of("PurePursuit/UseDynamicLookahead", true);

  public static final BooleanSupplier PURE_PURSUIT_ROTATE_IMMEDIATELY =
      FeatureFlag.of("PurePursuit/RotateImmediately", false);

  public static final BooleanSupplier COLLISION_AVOIDANCE_BOX_SHORTCUTS =
      FeatureFlag.of("CollisionAvoidance/BoxShortcuts", true);

  public static final BooleanSupplier INTAKE_VELOCITY_CORAL_DETECTION =
      FeatureFlag.of("Intake/CoralVelocityDetection", true);

  public static final BooleanSupplier FIELD_CALIBRATION = FeatureFlag.of("FieldCalibration", false);

  /**
   * Whether L4 approach state should automatically transition to L4 lineup when close enough to the
   * reef during autonomous. Doesn't alter teleop behavior.
   */
  public static final BooleanSupplier EXPLICIT_L4_LINEUP =
      FeatureFlag.of("Auto/ExplicitL4Lineup", true);

  private FeatureFlags() {}
}
