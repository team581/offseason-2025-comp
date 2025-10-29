package frc.robot.robot_manager;

import com.team581.GlobalConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.arm.ArmSubsystem;
import frc.robot.config.RobotConfig;
import frc.robot.elevator.ElevatorSubsystem;

public final class MechanismVisualizer {
  private static final Translation2d MECHANISM_AREA =
      new Translation2d(
          ((ArmSubsystem.ARM_LENGTH_METERS + Units.inchesToMeters(2)) * 2.0),
          Units.inchesToMeters(RobotConfig.get().elevator().maxHeight() + 2)
              + ArmSubsystem.ARM_LENGTH_METERS);
  private static final Mechanism2d MECHANISM =
      new Mechanism2d(
          MECHANISM_AREA.getX(), MECHANISM_AREA.getY(), new Color8Bit(new Color("#121212")));
  private static final MechanismRoot2d ROOT =
      MECHANISM.getRoot("superstructure", MECHANISM_AREA.getX() / 2.0, Units.inchesToMeters(2));
  private static final MechanismLigament2d ELEVATOR =
      ROOT.append(
          new MechanismLigament2d(
              "elevator",
              Units.inchesToMeters(RobotConfig.get().elevator().minHeight())
                  + ElevatorSubsystem.CARRIAGE_HEIGHT_FROM_FLOOR_METERS,
              90,
              20,
              new Color8Bit(Color.kFirstBlue)));
  private static final MechanismLigament2d ARM =
      ELEVATOR.append(
          new MechanismLigament2d(
              "arm", ArmSubsystem.ARM_LENGTH_METERS, 90, 10, new Color8Bit(Color.kFirstRed)));

  public static void log(double currentElevatorHeight, double currentArmAngle, double deployAngle) {
    if (!GlobalConfig.IS_DEVELOPMENT) {
      return;
    }

    SmartDashboard.putData("SuperstructureVisualization", MECHANISM);

    var armAngle = currentArmAngle - 90;
    var elevatorHeight =
        Units.inchesToMeters(currentElevatorHeight)
            + ElevatorSubsystem.CARRIAGE_HEIGHT_FROM_FLOOR_METERS;

    ELEVATOR.setLength(elevatorHeight);
    ARM.setAngle(armAngle);
  }

  private MechanismVisualizer() {}
}
