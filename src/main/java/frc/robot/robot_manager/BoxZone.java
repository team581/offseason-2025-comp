package frc.robot.robot_manager;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.MathHelpers;
import java.util.Objects;

public record BoxZone(
    Translation2d bottomCorner, Translation2d topCorner, SuperstructurePosition safeZone) {
  // Elevator heights are accurate to 0.1 inches
  private static final double ELEVATOR_PRECISION = 0.1;
  // Wrist angles are accurate to 0.1 degrees
  private static final double WRIST_PRECISION = 0.1;

  @Override
  public final int hashCode() {
    var roundedBottomCorner =
        new Translation2d(
            MathHelpers.roundTo(bottomCorner.getX(), ELEVATOR_PRECISION),
            MathHelpers.roundTo(bottomCorner.getY(), WRIST_PRECISION));
    var roundedTopCorner =
        new Translation2d(
            MathHelpers.roundTo(topCorner.getX(), ELEVATOR_PRECISION),
            MathHelpers.roundTo(topCorner.getY(), WRIST_PRECISION));
    var roundedSafeZone =
        new SuperstructurePosition(
            MathHelpers.roundTo(safeZone.elevatorHeight(), ELEVATOR_PRECISION),
            MathHelpers.roundTo(safeZone.wristAngle(), WRIST_PRECISION));

    return Objects.hash(roundedBottomCorner, roundedTopCorner, roundedSafeZone);
  }
}
