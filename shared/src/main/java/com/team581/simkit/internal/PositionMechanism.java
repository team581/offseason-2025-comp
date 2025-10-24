package com.team581.simkit.internal;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import java.util.OptionalDouble;

/** Predicts a simple Motion Magic style profile using TalonFX configuration state. */
public final class PositionMechanism {
  private static TrapezoidProfile.State currentMechanismState(List<SimMotor> devices) {
    var position =
        devices.stream()
            .mapToDouble(device -> device.motor().getPosition().getValueAsDouble())
            .average()
            .orElse(0.0);
    var velocity =
        devices.stream()
            .mapToDouble(device -> device.motor().getVelocity().getValueAsDouble())
            .average()
            .orElse(0.0);
    return new TrapezoidProfile.State(position, velocity);
  }

  private static TrapezoidProfile.Constraints getMechanismConstraints(List<SimMotor> devices) {
    // Collect configs for averaging
    List<TalonFXConfiguration> configs =
        devices.stream()
            .map(
                motor -> {
                  var config = new TalonFXConfiguration();
                  motor.motor().getConfigurator().refresh(config);
                  return config;
                })
            .toList();

    var cruiseVelocity =
        configs.stream()
            .mapToDouble(c -> c.MotionMagic.MotionMagicCruiseVelocity)
            .average()
            .orElse(0.0);
    var acceleration =
        configs.stream()
            .mapToDouble(c -> c.MotionMagic.MotionMagicAcceleration)
            .average()
            .orElse(0.0);

    return new TrapezoidProfile.Constraints(cruiseVelocity, acceleration);
  }

  private static TrapezoidProfile.State desiredMechanismState(List<SimMotor> devices) {
    var reference =
        devices.stream()
            .mapToDouble(device -> device.motor().getClosedLoopReference().getValueAsDouble())
            .average()
            .orElse(0.0);
    return new TrapezoidProfile.State(reference, 0.0);
  }

  private final List<SimMotor> devices;
  private final OptionalDouble minPosition;
  private final OptionalDouble maxPosition;
  private final Timer updateTimer = new Timer();
  private boolean hasRefreshedConstraints = false;
  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0, 0);

  /** Recomputes the predicted state and pushes the result into each motor sim. */
  public void update() {
    if (!hasRefreshedConstraints) {
      hasRefreshedConstraints = true;
      constraints = getMechanismConstraints(devices);
    }

    if (DriverStation.isDisabled()) {
      return;
    }

    var currentState = currentMechanismState(devices);
    var wantedState = desiredMechanismState(devices);
    var predictedState =
        new TrapezoidProfile(constraints).calculate(updateTimer.get(), currentState, wantedState);
    var boundedState = applyBounds(predictedState);

    for (var motor : devices) {
      motor.applyMechanismState(boundedState);
    }

    updateTimer.restart();
  }

  /** Seeds the rotor position of every motor to match the provided mechanism position. */
  public void seedPosition(double mechanismPosition) {
    for (var motor : devices) {
      motor.applyMechanismState(new TrapezoidProfile.State(mechanismPosition, 0.0));
    }
  }

  PositionMechanism(List<SimMotor> motors, OptionalDouble minPosition, OptionalDouble maxPosition) {
    this.devices = motors;
    this.minPosition = minPosition;
    this.maxPosition = maxPosition;
  }

  private TrapezoidProfile.State applyBounds(TrapezoidProfile.State state) {
    var clampedPosition =
        MathUtil.clamp(
            state.position,
            minPosition.orElse(Double.NEGATIVE_INFINITY),
            maxPosition.orElse(Double.POSITIVE_INFINITY));

    if (clampedPosition == state.position) {
      return state;
    }

    return new TrapezoidProfile.State(clampedPosition, 0.0);
  }
}
