package frc.robot.util;

public class VelocityDetector {
  private boolean hasSeenMinVelocity = false;
  private final double minVelocity;
  private final double maxVelocity;

  /**
   * @param minVelocity Minimum velocity while intaking
   * @param maxVelocity The maximum velocity that is possible if you are holding a game piece.
   * @return VelocityDetector class
   */
  public VelocityDetector(double minVelocity, double maxVelocity) {
    this.minVelocity = minVelocity;
    this.maxVelocity = maxVelocity;
  }

  public void reset() {
    hasSeenMinVelocity = false;
  }

  /**
   * Returns whether the motor is holding a game piece.
   *
   * @param motorVelocity Current motor velocity.
   */
  public boolean hasGamePiece(double motorVelocity) {
    if (hasSeenMinVelocity) {
      return motorVelocity < maxVelocity;
    }

    hasSeenMinVelocity = motorVelocity > minVelocity;
    return false;
  }
}
