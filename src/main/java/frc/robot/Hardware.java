package frc.robot;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotConfig;

public class Hardware {
  RobotConfig CONFIG = RobotConfig.get();
  public final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
  public final TalonFX elevatorTop =
      new TalonFX(CONFIG.elevator().topMotorID(), CONFIG.elevator().canBusName());
  public final TalonFX elevatorBottom =
      new TalonFX(CONFIG.elevator().bottomMotorID(), CONFIG.elevator().canBusName());

  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  public final TalonFX intakeMotor =
      new TalonFX(CONFIG.intake().motorID(), CONFIG.intake().canBusName());
  public final CANifier intakeLeftSensor = new CANifier(CONFIG.intake().leftSensorID());
  public final CANifier intakeRightSensor = new CANifier(CONFIG.intake().rightSensorID());
  public final TalonFX wristMotor =
      new TalonFX(CONFIG.wrist().motorID(), CONFIG.wrist().canBusName());

  public final TalonFX pivotMotor =
      new TalonFX(CONFIG.pivot().motorID(), CONFIG.pivot().canBusName());
}
