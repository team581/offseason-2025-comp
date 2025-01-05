package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.vision.interpolation.InterpolatedVisionDataset;
import java.util.function.Consumer;

public record RobotConfig(
    String robotName,
    SwerveConfig swerve,
    QueuerConfig queuer,
    ShooterConfig shooter,
    IntakeConfig intake,
    ArmConfig arm,
    VisionConfig vision,
    LightsConfig lights) {
  public record SwerveConfig(
      PhoenixPIDController snapController,
      boolean invertRotation,
      boolean invertX,
      boolean invertY,
      TalonFXConfiguration driveMotorConfig,
      TalonFXConfiguration steerMotorConfig) {}

  public record QueuerConfig(
      int motorID,
      String canBusName,
      int sensorID,
      TalonFXConfiguration motorConfig,
      Debouncer debouncer) {}

  public record ShooterConfig(
      int topMotorID,
      int bottomMotorID,
      String canBusName,
      TalonFXConfiguration topMotorConfig,
      TalonFXConfiguration bottomMotorConfig,
      Consumer<InterpolatingDoubleTreeMap> feedSpotDistanceToRpm,
      Consumer<InterpolatingDoubleTreeMap> speakerDistanceToRpm) {}

  public record IntakeConfig(
      int mainMotorID,
      String mainMotorCanBusName,
      int centeringMotorID,
      TalonFXConfiguration mainMotorConfig,
      Consumer<CANSparkMax> centeringMotorConfig) {}

  public record ArmConfig(
      String canBusName,
      int leftMotorID,
      int rightMotorID,
      TalonFXConfiguration leftMotorConfig,
      TalonFXConfiguration rightMotorConfig,
      Consumer<InterpolatingDoubleTreeMap> feedSpotDistanceToAngle,
      Consumer<InterpolatingDoubleTreeMap> speakerDistanceToAngle,
      double minAngle,
      double maxAngle) {}

  public record VisionConfig(
      int translationHistoryArraySize,
      double xyStdDev,
      double thetaStdDev,
      InterpolatedVisionDataset interpolatedVisionSet) {}

  public record LightsConfig(String canBusName, int deviceID) {}

  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = true;
  public static final String SERIAL_NUMBER = System.getenv("serialnum");

  public static RobotConfig get() {
    return CompConfig.competitionBot;
  }
}
