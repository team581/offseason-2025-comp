package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import frc.robot.vision.interpolation.InterpolatedVisionDataset;

public record RobotConfig(String robotName, SwerveConfig swerve, VisionConfig vision) {
  public record SwerveConfig(
      PhoenixPIDController snapController,
      boolean invertRotation,
      boolean invertX,
      boolean invertY,
      TalonFXConfiguration driveMotorConfig,
      TalonFXConfiguration steerMotorConfig) {}

  public record VisionConfig(
      int translationHistoryArraySize,
      double xyStdDev,
      double thetaStdDev,
      InterpolatedVisionDataset interpolatedVisionSet) {}

  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = true;
  public static final String SERIAL_NUMBER = System.getenv("serialnum");

  public static RobotConfig get() {
    return CompConfig.competitionBot;
  }
}
