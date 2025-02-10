package frc.robot.vision.limelight;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.fms.FmsSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.CameraHealth;
import frc.robot.vision.interpolation.CameraDataset;
import frc.robot.vision.results.GamePieceResult;
import frc.robot.vision.results.PurpleResult;
import frc.robot.vision.results.TagResult;
import java.util.Optional;

public class Limelight extends StateMachine<LimelightState> {
  private static final int[] RED_REEF_TAGS = {6, 7, 8, 9, 10, 11};
  private static final int[] BLUE_REEF_TAGS = {17, 18, 19, 20, 21, 22};

  private static final double IS_OFFLINE_TIMEOUT = 3;

  private final String limelightTableName;
  private final String name;
  private final CameraDataset cameraDataset;

  private final Timer limelightTimer = new Timer();
  private final Timer seedIMUTimer = new Timer();
  private CameraHealth cameraHealth = CameraHealth.NO_TARGETS;
  private double limelightHeartbeat = -1;

  private Optional<TagResult> interpolatedResult = Optional.empty();
  private Optional<TagResult> tagResult = Optional.empty();

  private Optional<GamePieceResult> coralResult = Optional.empty();
  private Optional<PurpleResult> purpleResult = Optional.empty();
  private double robotHeading = 0.0;

  public Limelight(String name, LimelightState initialState, CameraDataset cameraDataset) {
    // TODO(jonahsnider): Make Limelight state logging work with multiple instances, not just
    // singleton
    super(SubsystemPriority.VISION, initialState);
    limelightTableName = "limelight-" + name;
    this.name = name;
    limelightTimer.start();
    this.cameraDataset = cameraDataset;
  }

  public void sendImuData(
      double robotHeading,
      double angularVelocity,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {
    LimelightHelpers.SetRobotOrientation(
        limelightTableName, robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    this.robotHeading = robotHeading;
  }

  public void setState(LimelightState state) {
    setStateFromRequest(state);
  }

  public Optional<GamePieceResult> getCoralResult() {
    return getState() == LimelightState.CORAL ? coralResult : Optional.empty();
  }

  public Optional<PurpleResult> getPurpleResult() {
    return getState() == LimelightState.PURPLE ? purpleResult : Optional.empty();
  }

  public Optional<TagResult> getTagResult() {
    if (getState() != LimelightState.TAGS && getState() != LimelightState.REEF_TAGS) {
      return Optional.empty();
    }

    var estimatePose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightTableName);

    if (estimatePose == null) {
      return Optional.empty();
    }
    DogLog.log("Debug/" + name + "/RobotHeading", robotHeading);
    DogLog.log("Debug/" + name + "/PoseHeading", estimatePose.pose.getRotation().getDegrees());
    if (MathUtil.isNear(
        robotHeading, estimatePose.pose.getRotation().getDegrees(), 10, -180, 180)) {
      DogLog.log("Debug/" + name + "/Garbage", true);

      return Optional.empty();
    }
    DogLog.log("Debug/" + name + "/Garbage", false);

    DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", estimatePose.pose);

    if (estimatePose.tagCount == 0) {
      return Optional.empty();
    }
    // This prevents pose estimator from having crazy poses if the Limelight loses power
    if (estimatePose.pose.getX() == 0.0 && estimatePose.pose.getY() == 0.0) {
      return Optional.empty();
    }

    return Optional.of(new TagResult(estimatePose.pose, estimatePose.timestampSeconds));
  }

  private Optional<GamePieceResult> getRawCoralResult() {
    if (getState() != LimelightState.CORAL) {
      return Optional.empty();
    }
    var t2d = LimelightHelpers.getT2DArray(limelightTableName);
    if (t2d.length == 0) {
      return Optional.empty();
    }
    var coralTX = t2d[4];
    var coralTY = t2d[5];
    var latency = t2d[2] + t2d[3];
    var latencySeconds = latency / 1000.0;
    var timestamp = Timer.getFPGATimestamp() - latencySeconds;
    if (coralTX == 0.0 || coralTY == 0.0) {
      return Optional.empty();
    }

    DogLog.log("Vision/" + name + "/Coral/tx", coralTX);
    DogLog.log("Vision/" + name + "/Coral/ty", coralTY);

    return Optional.of(new GamePieceResult(coralTX, coralTY, timestamp));
  }

  private Optional<PurpleResult> getRawPurpleResult() {
    if (getState() != LimelightState.PURPLE) {
      return Optional.empty();
    }
    var t2d = LimelightHelpers.getT2DArray(limelightTableName);
    if (t2d.length == 0) {
      return Optional.empty();
    }
    var purpleTX = t2d[4];
    var purpleTY = t2d[5];
    var latency = t2d[2] + t2d[3];
    var latencySeconds = latency / 1000.0;
    var timestamp = Timer.getFPGATimestamp() - latencySeconds;
    if (purpleTX == 0.0 || purpleTY == 0.0) {
      return Optional.empty();
    }

    DogLog.log("Vision/" + name + "/Purple/tx", purpleTX);
    DogLog.log("Vision/" + name + "/Purple/ty", purpleTY);

    return Optional.of(new PurpleResult(purpleTX, purpleTY, timestamp));
  }

  private int[] getAllianceBasedReefTagIDs() {
    return FmsSubsystem.isRedAlliance() ? RED_REEF_TAGS : BLUE_REEF_TAGS;
  }

  @Override
  protected void collectInputs() {
    tagResult = getTagResult();
    coralResult = getRawCoralResult();
    purpleResult = getRawPurpleResult();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Vision/" + name + "/State", getState());
    LimelightHelpers.setPipelineIndex(limelightTableName, getState().pipelineIndex);
    switch (getState()) {
      case TAGS -> {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightTableName, new int[] {});
        updateHealth(interpolatedResult);
      }
      case CORAL -> updateHealth(coralResult);
      case PURPLE -> updateHealth(purpleResult);
      case REEF_TAGS -> {
        LimelightHelpers.SetFiducialIDFiltersOverride(
            limelightTableName, getAllianceBasedReefTagIDs());
        updateHealth(interpolatedResult);
      }
      default -> {}
    }

    LimelightHelpers.SetIMUMode(limelightTableName, seedIMUTimer.hasElapsed(2.0) ? 2 : 1);
  }

  @Override
  public void autonomousInit() {
    seedIMUTimer.reset();
    seedIMUTimer.start();
  }

  private void updateHealth(Optional<?> result) {
    var newHeartbeat = LimelightHelpers.getLimelightNTDouble(limelightTableName, "hb");
    DogLog.log("Vision/" + name + "/Heartbeat", newHeartbeat);
    if (limelightHeartbeat != newHeartbeat) {
      limelightTimer.restart();
    }
    limelightHeartbeat = newHeartbeat;

    if (limelightTimer.hasElapsed(IS_OFFLINE_TIMEOUT)) {
      cameraHealth = CameraHealth.OFFLINE;
      DogLog.logFault(limelightTableName + " is offline", AlertType.kError);
      return;
    } else {
      DogLog.clearFault(limelightTableName + " is offline");
    }

    if (!result.isEmpty()) {
      cameraHealth = CameraHealth.GOOD;
      return;
    }
    cameraHealth = CameraHealth.NO_TARGETS;
  }

  public void setBlinkEnabled(boolean enabled) {
    if (enabled) {
      LimelightHelpers.setLEDMode_ForceBlink(limelightTableName);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(limelightTableName);
    }
  }

  public CameraHealth getCameraHealth() {
    DogLog.log("Vision/" + name + "/Health", cameraHealth);
    return cameraHealth;
  }
}
