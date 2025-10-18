package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;

public record RobotConfig(
    ClawConfig claw,
    ClimberConfig climber,
    ElevatorConfig elevator,
    ArmConfig arm,
    VisionConfig vision,
    SwerveConfig swerve) {

  public record ClawConfig(
      String canBusName,
      int motorId,
      int candiId,
      boolean sensorFlipped,
      Debouncer debouncer,
      TalonFXConfiguration motorConfig) {}

  public record ClimberConfig(
      String canBusName,
      int climbMotorId,
      int cancoderId,
      int grabMotorId,
      int canrangeId,
      double minAngle,
      double maxAngle,
      TalonFXConfiguration climbMotorConfig,
      CANcoderConfiguration cancoderConfig,
      TalonFXConfiguration grabMotorConfig,
      CANrangeConfiguration canRangeConfig) {}

  public record ElevatorConfig(
      String canBusName,
      int motorId,
      double homingVoltage,
      double homingCurrentThreshold,
      double homingEndHeight,
      double maxHeight,
      double minHeight,
      TalonFXConfiguration motorConfig) {}

  public record ArmConfig(
      String canBusName,
      int motorId,
      TalonFXConfiguration motorConfig,
      double homingPosition,
      double inchesFromCenter) {}

  public record VisionConfig(
      double xyStdDev,
      double thetaStdDev,
      Pose3d robotPoseRelativeToCalibration,
      Pose3d leftBackLimelightPosition,
      Pose3d leftFrontLimelightPosition,
      Pose3d rightLimelightPosition,
      Pose3d gamePieceDetectionLimelightPosition) {}

  public record SwerveConfig(
      PhoenixPIDController snapController,
      boolean invertRotation,
      boolean invertX,
      boolean invertY) {}

  public static RobotConfig get() {
    return CompConfig.competitionBot;
  }
}
