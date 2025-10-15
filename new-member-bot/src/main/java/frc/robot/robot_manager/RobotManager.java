package frc.robot.robot_manager;

import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefSide;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SnapUtil;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionSubsystem;

public class RobotManager extends StateMachine<RobotState> {
    public final LocalizationSubsystem localization;
    public final AutoAlign autoAlign;
    public final VisionSubsystem vision;
    public final SwerveSubsystem swerve;

    public RobotManager(
        LocalizationSubsystem localization,
        AutoAlign autoalign,
        VisionSubsystem vision,
        SwerveSubsystem swerve) {
      super(SubsystemPriority.ROBOT_MANAGER, RobotState.CLAW_EMPTY);

      this.localization = localization;
      this.autoAlign = autoalign;
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
    private double reefSnapAngle = 0.0;
    private ChassisSpeeds robotSpeeds = new ChassisSpeeds();

    @Override
    protected RobotState getNextState(RobotState currentState) {
      return switch (currentState) {
              case CLAW_EMPTY,
                  CLAW_ALGAE,
                  CLAW_CORAL,
                  CORAL_L1_LINEUP,
                  CORAL_OUTTAKE,
                  ALGAE_NET_WAITING,
                  ALGAE_PROCESSOR_WAITING,
                  CLIMBER_STOP,
                  CLIMBING_1_LINEUP,
                  CLIMBING_2_HANGING,
                  UNJAM,
                  ALGAE_NET_RELEASE,
                  ALGAE_PROCESSOR_RELEASE,
                  REHOME_ELEVATOR ->
                  currentState;

            case CORAL_L1_APPROACH -> closeEnoughToReefSide ? RobotState.CORAL_L1_LINEUP : currentState;
            case CORAL_L1_PLACE -> currentState;
            case CORAL_L1_RELEASE -> cameraOnlineAndFarEnoughFromReef() || drivingFast ? RobotState.CLAW_EMPTY : currentState;

            case CORAL_INTAKE_GROUND -> currentState;

            case ALGAE_INTAKE_FLOOR -> currentState;

            case ALGAE_OUTTAKE -> currentState;

            case ALGAE_INTAKE_L2_APPROACH, ALGAE_INTAKE_L3_APPROACH ->
                closeEnoughToReefSide ? currentState.getNextAlgaeIntakeState() : currentState;

            case ALGAE_INTAKE_L2, ALGAE_INTAKE_L3 -> currentState;

            case ALGAE_INTAKE_L2_HOLDING, ALGAE_INTAKE_L3_HOLDING ->
                cameraOnlineAndFarEnoughFromReef() || drivingFast
                    ? currentState.getNextAlgaeIntakeState()
                    : currentState;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
      switch (newState) {
        case CLAW_EMPTY -> {

          swerve.normalDriveRequest();
        }
        case CLAW_ALGAE -> {

          swerve.normalDriveRequest();
        }
        case CLAW_CORAL -> {

          swerve.normalDriveRequest();
        }
        case CORAL_L1_APPROACH -> {

          swerve.setSnapToAngle(reefSnapAngle);
        }
        case CORAL_L1_LINEUP -> {

          swerve.setSnapToAngle(reefSnapAngle);
        }
        case CORAL_L1_PLACE -> {

          swerve.setSnapToAngle(reefSnapAngle);
        }
        case CORAL_L1_RELEASE -> {

          swerve.setSnapToAngle(reefSnapAngle);
        }
        case ALGAE_INTAKE_L2_APPROACH -> {

          swerve.setSnapToAngle(reefSnapAngle);
        }
        case ALGAE_INTAKE_L2 -> {

          swerve.setSnapToAngle(reefSnapAngle);
        }
        case ALGAE_INTAKE_L2_HOLDING -> {

          swerve.setSnapToAngle(reefSnapAngle);
        }
        case ALGAE_INTAKE_L3_APPROACH -> {

          swerve.setSnapToAngle(reefSnapAngle);
        }
        case ALGAE_INTAKE_L3 -> {

          swerve.setSnapToAngle(reefSnapAngle);
        }
        case ALGAE_INTAKE_L3_HOLDING -> {

          swerve.setSnapToAngle(reefSnapAngle);
        }
        case ALGAE_PROCESSOR_WAITING -> {

          swerve.setSnapToAngle(SnapUtil.getProcessorAngle());
        }
        case ALGAE_PROCESSOR_RELEASE -> {

          swerve.setSnapToAngle(SnapUtil.getProcessorAngle());
        }
        case ALGAE_NET_WAITING -> {

          swerve.setSnapToAngle(SnapUtil.getNetScoringAngle(robotPose));
        }
        case ALGAE_NET_RELEASE -> {

          swerve.setSnapToAngle(SnapUtil.getNetScoringAngle(robotPose));
        }
        case CLIMBER_STOP -> {

          swerve.setSnapToAngle(SnapUtil.getCageAngle(FmsUtil.isRedAlliance()));
        }
        case CLIMBING_1_LINEUP -> {

          swerve.setSnapToAngle(SnapUtil.getCageAngle(FmsUtil.isRedAlliance()));
        }
        case CLIMBING_2_HANGING -> {

          swerve.setSnapToAngle(SnapUtil.getCageAngle(FmsUtil.isRedAlliance()));
        }
      }
  }

  @Override
  protected void collectInputs() {
    robotPose = localization.getPose();
    nearestReefSide = autoAlign.getClosestReefSide();
    closeEnoughToReefSide = AutoAlign.isCloseToReefSide(robotPose, nearestReefSide.getPose(), 1.0);
    tagCameraOnline = vision.isAnyTagLimelightOnline();
    robotSpeeds = swerve.getTeleopSpeeds();
    drivingFast =
        Math.sqrt(
                Math.pow(robotSpeeds.vxMetersPerSecond, 2)
                    + Math.pow(robotSpeeds.vyMetersPerSecond, 2))
            > 5.0;
    reefSnapAngle = autoAlign.getUsedScoringPose().getRotation().getDegrees();

    var scoringLevel = switch (getState()) {
      case CORAL_L1_APPROACH, CORAL_L1_LINEUP -> ReefPipeLevel.RAISING;
      case CORAL_L1_RELEASE -> ReefPipeLevel.BACK_AWAY;
      default -> ReefPipeLevel.RAISING;
    };

    autoAlign.setScoringLevel(scoringLevel, ReefPipeLevel.RAISING);
  }

  private boolean cameraOnlineAndFarEnoughFromReef() {
    if (!tagCameraOnline) {
      return timeout(0.5);
    }
    return !closeEnoughToReefSide;
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

  public void l1ApproachRequest() {
    setStateFailsafe(RobotState.CORAL_L1_APPROACH);
  }

  public void algaeReefIntakeRequest() {
    if (nearestReefSide.algaeHeight == ReefPipeLevel.L3) {
      intakeL3AlgaeRequest();
    } else {
      intakeL2AlgaeRequest();
    }
  }

  public void stowRequest() {
    switch (getState()) {
      case CLAW_ALGAE, CLAW_CORAL, CLAW_EMPTY -> getState();
      case ALGAE_INTAKE_L2_HOLDING, ALGAE_INTAKE_L3_HOLDING -> setStateFailsafe(RobotState.CLAW_ALGAE);
      case CORAL_L1_APPROACH, CORAL_L1_LINEUP, CORAL_L1_PLACE -> setStateFailsafe(RobotState.CLAW_CORAL);
      default -> setStateFailsafe(RobotState.CLAW_EMPTY);
    }
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
          CORAL_L1_APPROACH,
          CORAL_L1_PLACE,
          CORAL_L1_RELEASE,
          CORAL_OUTTAKE,
          REHOME_ELEVATOR,
          UNJAM -> {}

      case CLAW_EMPTY -> setStateFailsafe(RobotState.ALGAE_OUTTAKE);
      case CLAW_ALGAE -> netWaitRequest();
      case CLAW_CORAL -> l1ApproachRequest();

      case ALGAE_NET_WAITING -> setStateFailsafe(RobotState.ALGAE_NET_RELEASE);
      case ALGAE_PROCESSOR_WAITING -> setStateFailsafe(RobotState.ALGAE_PROCESSOR_RELEASE);
      case CORAL_L1_LINEUP -> setStateFailsafe(RobotState.CORAL_L1_PLACE);
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
      case CLIMBER_STOP -> setStateFromRequest(RobotState.CLIMBING_2_HANGING);
      case CLIMBING_2_HANGING -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
      case CLIMBING_1_LINEUP -> setStateFromRequest(RobotState.CLAW_EMPTY);
      default -> {}
    }
  }
}
