package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotConfig;

public class Hardware {
  // public final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  public final TalonFX clawMotor =
      new TalonFX(RobotConfig.get().claw().motorId(), RobotConfig.get().claw().canBusName());
  public final CANdi clawCANdi =
      new CANdi(RobotConfig.get().claw().candiId(), RobotConfig.get().claw().canBusName());

  public final TalonFX elevatorMotor =
      new TalonFX(
          RobotConfig.get().elevator().motorId(), RobotConfig.get().elevator().canBusName());
  public final TalonFX wristMotor =
      new TalonFX(RobotConfig.get().wrist().motorId(), RobotConfig.get().wrist().canBusName());

  public final TalonFX climberClimbMotor =
      new TalonFX(
          RobotConfig.get().climber().climbMotorId(), RobotConfig.get().climber().canBusName());
  public final CANcoder climberCANcoder =
      new CANcoder(
          RobotConfig.get().climber().cancoderId(), RobotConfig.get().climber().canBusName());
  public final TalonFX climberGrabMotor =
      new TalonFX(
          RobotConfig.get().climber().grabMotorId(), RobotConfig.get().climber().canBusName());
  public final CANrange climberCANrange =
      new CANrange(
          RobotConfig.get().climber().canrangeId(), RobotConfig.get().climber().canBusName());
}
