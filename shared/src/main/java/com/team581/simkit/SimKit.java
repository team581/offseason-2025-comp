package com.team581.simkit;

import com.team581.simkit.internal.PositionMechanism;
import com.team581.simkit.internal.PositionMechanismBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;
import org.jspecify.annotations.Nullable;

/** Entry point for creating simple mechanism simulations. */
public final class SimKit {
  private static final Map<String, PositionMechanism> MECHANISMS = new HashMap<>();

  public static @Nullable PositionMechanism positionMechanism(
      String name, Function<PositionMechanismBuilder, PositionMechanismBuilder> factory) {
    if (RobotBase.isSimulation()) {
      return MECHANISMS.computeIfAbsent(
          name, k -> factory.apply(new PositionMechanismBuilder()).build());
    }

    return null;
  }

  private SimKit() {}
}
