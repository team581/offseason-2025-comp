package frc.robot.robot_manager;

import frc.robot.util.MathHelpers;
import java.util.Objects;

public record SuperstructurePosition(double elevatorHeight, double wristAngle) {
  // Elevator heights are accurate to 0.1 inches
  private static final double ELEVATOR_PRECISION = 0.1;
  // Wrist angles are accurate to 0.1 degrees
  private static final double WRIST_PRECISION = 0.1;

  @Override
  public final int hashCode() {
    var roundedElevator = MathHelpers.roundTo(elevatorHeight, ELEVATOR_PRECISION);
    var roundedWrist = MathHelpers.roundTo(wristAngle, WRIST_PRECISION);

    return Objects.hash(roundedElevator, roundedWrist);
  }

  @Override
  public final boolean equals(Object other) {
    if (this == other) {
      return true;
    }

    if (!(other instanceof SuperstructurePosition)) {
      return false;
    }

    var otherPosition = (SuperstructurePosition) other;

    return MathHelpers.roundTo(elevatorHeight, ELEVATOR_PRECISION)
            == MathHelpers.roundTo(otherPosition.elevatorHeight, ELEVATOR_PRECISION)
        && MathHelpers.roundTo(wristAngle, WRIST_PRECISION)
            == MathHelpers.roundTo(otherPosition.wristAngle, WRIST_PRECISION);
  }
}
