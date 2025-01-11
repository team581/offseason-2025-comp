package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotConfig;

public class Hardware {
  public final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  public final TalonFX intakeMotor = new TalonFX(RobotConfig.get().intake().motorID());
  public final DigitalInput intakeLeftSensor =
      new DigitalInput(RobotConfig.get().intake().leftSensorID());
  public final DigitalInput intakeRightSensor =
      new DigitalInput(RobotConfig.get().intake().rightSensorID());
  public final TalonFX wristMotor = new TalonFX(RobotConfig.get().wrist().motorID(),RobotConfig.get().wrist().canBusName());
}
