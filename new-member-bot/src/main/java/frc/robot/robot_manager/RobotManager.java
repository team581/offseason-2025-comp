package frc.robot.robot_manager;

import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefSide;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionSubsystem;

public class RobotManager extends StateMachine<RobotState> {

  public final LocalizationSubsystem localization;
  public final AutoAlign autoalign;
  public final VisionSubsystem vision;
  public final SwerveSubsystem swerve;

  public RobotManager(
      LocalizationSubsystem localization,
      AutoAlign autoalign,
      VisionSubsystem vision,
      SwerveSubsystem swerve) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.CLAW_EMPTY);

    this.localization = localization;
    this.autoalign = autoalign;
    this.vision = vision;
    this.swerve = swerve;

    var stateCount = RobotState.values().length;

    DogLog.log("RobotManager/StateCount", stateCount);
  }

  private Pose2d robotPose = new Pose2d();
  private boolean closeEnoughToReefSide = false;
  private ReefSide nearestReefSide = ReefSide.SIDE_IJ;
  private boolean tagCameraOnline = false;
  private boolean drivingFast = false;
  private ChassisSpeeds robotSpeeds = new ChassisSpeeds();

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case CLAW_EMPTY,
          CLAW_ALGAE,
          CLAW_CORAL,
          CORAL_L1_WAIT,
          CORAL_OUTTAKE,
          ALGAE_NET_WAITING,
          ALGAE_PROCESSOR_WAITING,
          CLIMBER_STOP,
          CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          UNJAM ->
          currentState;

      case CORAL_L1_SCORE -> currentState;

      case CORAL_INTAKE_GROUND -> currentState;

      case ALGAE_NET_RELEASE, ALGAE_PROCESSOR_RELEASE -> currentState;

      case ALGAE_INTAKE_L2_APPROACH, ALGAE_INTAKE_L3_APPROACH ->
          closeEnoughToReefSide ? currentState.getNextAlgaeIntakeState() : currentState;

      case ALGAE_INTAKE_L2, ALGAE_INTAKE_L3 -> currentState;

      case ALGAE_INTAKE_L2_HOLDING, ALGAE_INTAKE_L3_HOLDING ->
          cameraOnlineAndFarEnoughFromReef() || drivingFast
              ? currentState.getNextAlgaeIntakeState()
              : currentState;

      default -> currentState;
    };
  }

  @Override
  protected void collectInputs() {
    robotPose = localization.getPose();
    nearestReefSide = autoalign.getClosestReefSide();
    closeEnoughToReefSide = AutoAlign.isCloseToReefSide(robotPose, nearestReefSide.getPose(), 1.0);
    tagCameraOnline = vision.isAnyTagLimelightOnline();
    robotSpeeds = swerve.getTeleopSpeeds();
    drivingFast =
        Math.sqrt(
                Math.pow(robotSpeeds.vxMetersPerSecond, 2)
                    + Math.pow(robotSpeeds.vyMetersPerSecond, 2))
            > 5.0;
  }

  private boolean cameraOnlineAndFarEnoughFromReef() {
    if (!tagCameraOnline) {
      return timeout(0.5);
    }

    var isFarEnoughFromReefSide =
        !AutoAlign.isCloseToReefSide(robotPose, nearestReefSide.getPose(robotPose), 1.0);

    return isFarEnoughFromReefSide;
  }

  private void setStateFailsafe(RobotState newState) {
    if (getState().climbingOrRehoming) {
      return;
    }
    setStateFromRequest(newState);
  }

  public void rehomeRequest() {
    setStateFailsafe(RobotState.REHOME_ELEVATOR);
  }

  public void unjamRequest() {
    setStateFailsafe(RobotState.UNJAM);
  }

  public void intakeGroundAlgaeRequest() {
    setStateFailsafe(RobotState.ALGAE_INTAKE_FLOOR);
  }

  public void intakeL2AlgaeRequest() {
    setStateFailsafe(RobotState.ALGAE_INTAKE_L2_APPROACH);
  }

  public void intakeL3AlgaeRequest() {
    setStateFailsafe(RobotState.ALGAE_INTAKE_L3_APPROACH);
  }

  public void netWaitRequest() {
    setStateFailsafe(RobotState.ALGAE_NET_WAITING);
  }

  public void processorWaitRequest() {
    setStateFailsafe(RobotState.ALGAE_PROCESSOR_WAITING);
  }

  public void l1WaitRequest() {
    setStateFailsafe(RobotState.CORAL_L1_WAIT);
  }

  public void confirmScoreRequest() {
    switch (getState()) {
      case ALGAE_INTAKE_FLOOR,
      ALGAE_INTAKE_L2,
      ALGAE_INTAKE_L2_APPROACH,
      ALGAE_INTAKE_L2_HOLDING,
      ALGAE_INTAKE_L3,
      ALGAE_INTAKE_L3_APPROACH,
      ALGAE_INTAKE_L3_HOLDING,
      ALGAE_NET_RELEASE,
      ALGAE_OUTTAKE,
      ALGAE_PROCESSOR_RELEASE,
      CLIMBER_STOP,
      CLIMBING_1_LINEUP,
      CLIMBING_2_HANGING,
      CORAL_INTAKE_GROUND,
      CORAL_L1_SCORE,
      CORAL_OUTTAKE,
      REHOME_ELEVATOR,
      UNJAM -> {}

      case CLAW_ALGAE -> netWaitRequest();
      case CLAW_CORAL -> l1WaitRequest();

      case ALGAE_NET_WAITING -> setStateFailsafe(RobotState.ALGAE_NET_RELEASE);
      case ALGAE_PROCESSOR_WAITING -> setStateFailsafe(RobotState.ALGAE_PROCESSOR_RELEASE);
      case CORAL_L1_WAIT -> setStateFailsafe(RobotState.CORAL_L1_SCORE);
    }
  }

  public void climberSequenceForward() {
    switch (getState()) {
      case CLIMBING_1_LINEUP -> setStateFromRequest(RobotState.CLIMBING_2_HANGING);
      case CLIMBING_2_HANGING -> setStateFromRequest(RobotState.CLIMBER_STOP);
      default -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
    }
  }

  public void climberSequenceBack() {
    switch (getState()) {
      default -> {}
      case CLIMBER_STOP -> setStateFromRequest(RobotState.CLIMBING_2_HANGING);
      case CLIMBING_2_HANGING -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
      case CLIMBING_1_LINEUP -> setStateFromRequest(RobotState.CLAW_EMPTY);
    }
  }
}
