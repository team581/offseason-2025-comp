package com.team581.simkit.internal;

import static com.google.common.base.Preconditions.checkState;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.google.errorprone.annotations.CanIgnoreReturnValue;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;

/** Builder for {@link PositionMechanism}. */
public final class PositionMechanismBuilder {
  private final List<SimMotor> motors = new ArrayList<>();
  private OptionalDouble minPosition = OptionalDouble.empty();
  private OptionalDouble maxPosition = OptionalDouble.empty();

  @CanIgnoreReturnValue
  public PositionMechanismBuilder addMotor(TalonFX motor) {
    return addMotor(motor, motor.getSimState().Orientation);
  }

  @CanIgnoreReturnValue
  public PositionMechanismBuilder addMotor(TalonFX motor, ChassisReference orientation) {
    motors.add(SimMotor.of(motor, orientation));
    return this;
  }

  @CanIgnoreReturnValue
  public PositionMechanismBuilder withMinPosition(double minPosition) {
    this.minPosition = OptionalDouble.of(minPosition);
    return this;
  }

  @CanIgnoreReturnValue
  public PositionMechanismBuilder withMaxPosition(double maxPosition) {
    this.maxPosition = OptionalDouble.of(maxPosition);
    return this;
  }

  public PositionMechanism build() {
    checkState(!motors.isEmpty(), "At least one motor is required");

    return new PositionMechanism(motors, minPosition, maxPosition);
  }
}
