package frc.robot.util;

public class VelocityDetector {
  private boolean hasSeenMinVelocity = false;
  private double[] minVelocities = new double[3];
  private double[] maxVelocities = new double[3];
  public int currentSlot = 0;

  /**
   * @param minVelocity Minimum velocity while intaking
   * @param maxVelocity The maximum velocity that is possible if you are holding a game piece.
   * @param slot The slot in which these values will be placed in.
   * @return VelocityDetector class
   */
  public VelocityDetector(double minVelocity, double maxVelocity, int slot) {
    this.currentSlot = slot;
    this.minVelocities[slot] = minVelocity;
    this.maxVelocities[slot] = maxVelocity;
  }

  /**
   * Sets values to a slot
   *
   * @param minVelocity Minimum velocity while intaking
   * @param maxVelocity The maximum velocity that is possible if you are holding a game piece.
   * @param slot The slot in which these values will be placed in.
   */
  public VelocityDetector inSlot(int slot, double minVelocity, double maxVelocity) {
    this.currentSlot = slot;
    this.minVelocities[slot] = minVelocity;
    this.maxVelocities[slot] = maxVelocity;
    return this;
  }

  public double getMax(int slot) {
    return maxVelocities[slot];
  }

  public double getMin(int slot) {
    return minVelocities[slot];
  }

  /**
   * Resets the detection sequence
   *
   * @param slot Set the current used max and min values slot.
   */
  public void reset(int slot) {
    hasSeenMinVelocity = false;
    this.currentSlot = slot;
  }

  /**
   * Returns whether the motor is holding a game piece.
   *
   * @param motorVelocity Current motor velocity.
   */
  public boolean hasGamePiece(double motorVelocity, boolean timeoutPassed) {
    if (hasSeenMinVelocity) {
      return motorVelocity < maxVelocities[currentSlot];
    }

    hasSeenMinVelocity = motorVelocity > minVelocities[currentSlot];
    return motorVelocity < maxVelocities[currentSlot] && timeoutPassed;
  }
}
