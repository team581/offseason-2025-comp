package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotConfig;

public class Hardware {
  public final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  public final TalonFX shooterTop =
      new TalonFX(
          RobotConfig.get().shooter().topMotorID(), RobotConfig.get().shooter().canBusName());
  public final TalonFX shooterBottom =
      new TalonFX(
          RobotConfig.get().shooter().bottomMotorID(), RobotConfig.get().shooter().canBusName());

  public final TalonFX queuer =
      new TalonFX(RobotConfig.get().queuer().motorID(), RobotConfig.get().queuer().canBusName());
  public final DigitalInput queuerSensor = new DigitalInput(RobotConfig.get().queuer().sensorID());

  public final TalonFX intakeMain =
      new TalonFX(
          RobotConfig.get().intake().mainMotorID(),
          RobotConfig.get().intake().mainMotorCanBusName());
  public final CANSparkMax intakeCenteringMotor =
      new CANSparkMax(RobotConfig.get().intake().centeringMotorID(), MotorType.kBrushed);

  public final TalonFX armLeft =
      new TalonFX(RobotConfig.get().arm().leftMotorID(), RobotConfig.get().arm().canBusName());
  public final TalonFX armRight =
      new TalonFX(RobotConfig.get().arm().rightMotorID(), RobotConfig.get().arm().canBusName());

  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  public final CANdle candle =
      new CANdle(RobotConfig.get().lights().deviceID(), RobotConfig.get().lights().canBusName());
}
