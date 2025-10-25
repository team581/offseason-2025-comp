package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotConfig;

public class Hardware {
  // public final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  public final TalonFX intakeMotor =
      new TalonFX(RobotConfig.get().intake().motorId(), RobotConfig.get().intake().canBusName());

  public final TalonFX deployMotor =
      new TalonFX(RobotConfig.get().deploy().motorId(), RobotConfig.get().deploy().canBusName());

  public final TalonFX leftSingulatorMotor =
      new TalonFX(
          RobotConfig.get().singulator().leftMotorId(),
          RobotConfig.get().singulator().canBusName());
  public final TalonFX rightSingulatorMotor =
      new TalonFX(
          RobotConfig.get().singulator().rightMotorId(),
          RobotConfig.get().singulator().canBusName());

  public final TalonFX clawMotor =
      new TalonFX(RobotConfig.get().claw().motorId(), RobotConfig.get().claw().canBusName());

  public final TalonFX armMotor =
      new TalonFX(RobotConfig.get().arm().motorId(), RobotConfig.get().arm().canBusName());

  public final TalonFX elevatorMotor =
      new TalonFX(
          RobotConfig.get().elevator().motorId(), RobotConfig.get().elevator().canBusName());
  public final TalonFX climberMotor =
      new TalonFX(
          RobotConfig.get().climber().climbMotorId(), RobotConfig.get().climber().canBusName());

  public final CANcoder climberCANcoder =
      new CANcoder(
          RobotConfig.get().climber().cancoderId(), RobotConfig.get().climber().canBusName());

  public final CANdle candle =
      new CANdle(RobotConfig.get().lights().candleId(), RobotConfig.get().lights().canBusName());
  public final CANdi clawCaNdi = new CANdi(RobotConfig.get().claw().candiId());

  public final DigitalInput topCradleSensor =
      new DigitalInput(RobotConfig.get().singulator().topSensorPortId());
  public final DigitalInput bottomCradleSensor =
      new DigitalInput(RobotConfig.get().singulator().bottonSensorPortId());
}
