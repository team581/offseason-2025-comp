package com.team581;

import edu.wpi.first.wpilibj.RobotController;
import java.util.Optional;

public enum RobotKind {
  COMP_BOT("placeholder3"),
  OFFSEASON_BOT("placeholder2"),
  NEW_MEMBER_BOT("placeholder");

  /**
   * Returns the RobotKind by matching the serial number to a known RobotKind. If the serial number
   * can't be matched, returns empty.
   */
  public static Optional<RobotKind> fromSerialNumber() {
    return switch (RobotController.getSerialNumber()) {
      case "placeholder" -> Optional.of(NEW_MEMBER_BOT);
      case "placeholder2" -> Optional.of(OFFSEASON_BOT);
      case "placeholder3" -> Optional.of(COMP_BOT);
      default -> Optional.empty();
    };
  }

  public final String serialNumber;

  RobotKind(String serialNumber) {
    this.serialNumber = serialNumber;
  }
}
