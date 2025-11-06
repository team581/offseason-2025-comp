package com.team581.mechanisms;

import com.google.errorprone.annotations.CanIgnoreReturnValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;

public class VelocityDetector {
  private final double minVelocity;
  private final double minVelocityTimeout;
  private final Timer timeout = new Timer();
  private final Debouncer debouncer;
  private boolean hasSeenMinVelocity = false;

  public VelocityDetector(double minVelocity, double minVelocityTimeout, double debounceTime) {
    this.minVelocity = minVelocity;
    this.minVelocityTimeout = minVelocityTimeout;
    this.debouncer = new Debouncer(debounceTime, DebounceType.kRising);
    timeout.start();
  }

  public VelocityDetector(double minVelocity, double minVelocityTimeout) {
    this.minVelocity = minVelocity;
    this.minVelocityTimeout = minVelocityTimeout;
    this.debouncer = new Debouncer(0.0, DebounceType.kRising);
    timeout.start();
  }

  @CanIgnoreReturnValue
  public VelocityDetector withDebounceRising(double time) {
    this.debouncer.setDebounceTime(time);
    this.debouncer.setDebounceType(DebounceType.kRising);
    return this;
  }

  @CanIgnoreReturnValue
  public VelocityDetector withDebounceFalling(double time) {
    this.debouncer.setDebounceTime(time);
    this.debouncer.setDebounceType(DebounceType.kFalling);
    return this;
  }

  @CanIgnoreReturnValue
  public VelocityDetector withDebounceBoth(double time) {
    this.debouncer.setDebounceTime(time);
    this.debouncer.setDebounceType(DebounceType.kBoth);
    return this;
  }

  public void reset() {
    hasSeenMinVelocity = false;
    timeout.reset();
  }

  /**
   * Returns whether the motor is holding a game piece.
   *
   * @param motorVelocity Current motor velocity.
   */
  public boolean hasGamePiece(double motorVelocity, double maxVelocity) {
    hasSeenMinVelocity =
        hasSeenMinVelocity
            || timeout.hasElapsed(minVelocityTimeout)
            || Math.abs(motorVelocity) >= minVelocity;

    return hasSeenMinVelocity && debouncer.calculate(Math.abs(motorVelocity) <= maxVelocity);
  }
}
