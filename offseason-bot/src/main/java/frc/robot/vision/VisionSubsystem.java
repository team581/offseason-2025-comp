package frc.robot.vision;

import com.team581.mechanisms.vision.CameraHealth;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import frc.robot.vision.results.OptionalTagResult;

public class VisionSubsystem extends StateMachineSubsystem<VisionState> {
  private final Debouncer seeingTagDebouncer = new Debouncer(1.0, DebounceType.kFalling);
  private final Debouncer seeingTagForPoseResetDebouncer =
      new Debouncer(5.0, DebounceType.kFalling);

  private final ImuSubsystem imu;
  private final Limelight leftLimelight;
  private final Limelight rightLimelight;

  private OptionalTagResult leftTagResult = new OptionalTagResult();
  private OptionalTagResult rightTagResult = new OptionalTagResult();

  private double robotHeading;

  private double angularVelocity;

  private boolean hasSeenTag = false;
  private boolean seeingTag = false;
  private boolean seeingTagDebounced = false;
  private boolean seenTagRecentlyForReset = true;

  public VisionSubsystem(ImuSubsystem imu, Limelight leftLimelight, Limelight rightLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.leftLimelight = leftLimelight;
    this.rightLimelight = rightLimelight;
  }

  @Override
  protected void collectInputs() {
    angularVelocity = imu.getRobotAngularVelocity();

    leftTagResult = leftLimelight.getTagResult();
    rightTagResult = rightLimelight.getTagResult();

    if (leftTagResult.isPresent() || rightTagResult.isPresent()) {
      hasSeenTag = true;
      seeingTag = true;
    } else {
      seeingTag = false;
    }
    seeingTagDebounced = seeingTagDebouncer.calculate(seeingTag);
    if (DriverStation.isDisabled()) {
      seenTagRecentlyForReset = true;
    } else {
      seenTagRecentlyForReset = seeingTagForPoseResetDebouncer.calculate(seeingTag);
    }
  }

  public void setEstimatedPoseAngle(double robotHeading) {
    this.robotHeading = robotHeading;
  }

  public OptionalTagResult getLefTagResult() {
    return leftTagResult;
  }

  public OptionalTagResult getRightTagResult() {
    return rightTagResult;
  }

  public boolean seeingTagDebounced() {
    return seeingTagDebounced;
  }

  public boolean seenTagRecentlyForReset() {
    return seenTagRecentlyForReset;
  }

  public boolean seeingTag() {
    return seeingTag || RobotBase.isSimulation();
  }

  public boolean hasSeenTag() {
    return hasSeenTag;
  }

  public void setState(VisionState state) {
    setStateFromRequest(state);
  }

  @Override
  protected void afterTransition(VisionState newState) {
    switch (newState) {
      case TAGS -> {
        leftLimelight.setState(LimelightState.TAGS);
        rightLimelight.setState(LimelightState.TAGS);
      }
    }
  }

  @Override
  public void whileInState(VisionState currentState) {
    leftLimelight.sendImuData(robotHeading, angularVelocity, 0.0, 0.0, 0.0, 0.0);
    rightLimelight.sendImuData(robotHeading, angularVelocity, 0.0, 0.0, 0.0, 0.0);

    DogLog.log("Vision/SeeingTag", seeingTag);
    DogLog.log("Vision/SeeingTagLast5Seconds", seenTagRecentlyForReset);
  }

  public boolean isAnyCameraOffline() {
    return leftLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || rightLimelight.getCameraHealth() == CameraHealth.OFFLINE;
  }

  public boolean isAnyCameraOnlineForTags() {
    if (RobotBase.isSimulation()) {
      return true;
    }
    return leftLimelight.isOnlineForTags() || rightLimelight.isOnlineForTags();
  }
}
