package frc.robot;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotConfig;

public class Hardware {
  public final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
  public final TalonFX elevatorLeftMotor =
      new TalonFX(
          RobotConfig.get().elevator().leftMotorID(), RobotConfig.get().elevator().canBusName());
  public final TalonFX elevatorRightMotor =
      new TalonFX(
          RobotConfig.get().elevator().rightMotorID(), RobotConfig.get().elevator().canBusName());

  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  public final TalonFX intakeLeftMotor =
      new TalonFX(
          RobotConfig.get().intake().leftMotorID(), RobotConfig.get().intake().canBusName());
  public final TalonFX intakeRightMotor =
      new TalonFX(
          RobotConfig.get().intake().rightMotorID(), RobotConfig.get().intake().canBusName());
  public final CANifier intakeCaNifier = new CANifier(RobotConfig.get().intake().canifierID());
  public final TalonFX wristMotor =
      new TalonFX(RobotConfig.get().wrist().motorID(), RobotConfig.get().wrist().canBusName());

  public final TalonFX pivotMotor =
      new TalonFX(RobotConfig.get().pivot().motorID(), RobotConfig.get().pivot().canBusName());

  public final CANdle candle =
      new CANdle(RobotConfig.get().lights().candleID(), RobotConfig.get().lights().canBusName());

  public final TalonFX climberMotor =
      new TalonFX(RobotConfig.get().climber().motorID(), RobotConfig.get().climber().canBusName());

  public final CANcoder climberCANcoder =
      new CANcoder(RobotConfig.get().climber().motorID(), RobotConfig.get().climber().canBusName());
}
