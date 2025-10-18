package frc.robot.robot_manager;

import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefSide;
import frc.robot.claw.ClawState;
import frc.robot.claw.ClawSubsystem;
import frc.robot.climber.ClimberState;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.elevator.ElevatorState;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SnapUtil;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionState;
import frc.robot.vision.VisionSubsystem;
import frc.robot.wrist.WristState;
import frc.robot.wrist.WristSubsystem;

public class RobotManager extends StateMachine<RobotState> {
  public final ClawSubsystem claw;
  public final ElevatorSubsystem elevator;
  public final WristSubsystem wrist;
  public final ClimberSubsystem climber;
  public final LocalizationSubsystem localization;
  public final AutoAlign autoAlign;
  public final VisionSubsystem vision;
  public final SwerveSubsystem swerve;
  public final ImuSubsystem imu;

  public RobotManager(
      ClawSubsystem claw,
      ElevatorSubsystem elevator,
      WristSubsystem wrist,
      ClimberSubsystem climber,
      LocalizationSubsystem localization,
      AutoAlign autoalign,
      VisionSubsystem vision,
      SwerveSubsystem swerve,
      ImuSubsystem imu) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.STARTING_POSITION);

    this.claw = claw;
    this.elevator = elevator;
    this.wrist = wrist;
    this. climber = climber;
    this.localization = localization;
    this.autoAlign = autoalign;
    this.vision = vision;
    this.swerve = swerve;
    this.imu = imu;

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
          STARTING_POSITION,
          ALGAE_NET_WAITING,
          ALGAE_PROCESSOR_WAITING,
          CLIMBER_STOP,
          CORAL_L1_LINEUP,
          CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          UNJAM,
          ALGAE_NET_RELEASE,
          ALGAE_PROCESSOR_RELEASE,
          REHOME_ELEVATOR ->
          currentState;

      case CORAL_L1_APPROACH -> wrist.atGoal() && elevator.atGoal() ? RobotState.CORAL_L1_LINEUP : currentState;
      case CORAL_L1_RELEASE ->
          cameraOnlineAndFarEnoughFromReef() || drivingFast ? RobotState.CLAW_EMPTY : currentState;

      case CORAL_INTAKE_GROUND -> claw.getHasGP() ? RobotState.CLAW_CORAL : currentState;
      case ALGAE_INTAKE_FLOOR -> claw.getHasGP() ? RobotState.CLAW_ALGAE : currentState;

      case ALGAE_OUTTAKE, CORAL_OUTTAKE -> claw.getHasGP() ? currentState : RobotState.CLAW_EMPTY;

      case ALGAE_INTAKE_L2_APPROACH, ALGAE_INTAKE_L3_APPROACH ->
      wrist.atGoal() && elevator.atGoal() ? currentState.getNextAlgaeIntakeState() : currentState;

      case ALGAE_INTAKE_L2, ALGAE_INTAKE_L3 -> claw.getHasGP() ? currentState.getNextAlgaeIntakeState() : currentState;

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
        claw.setState(ClawState.IDLE_NO_GP);
        elevator.setState(ElevatorState.STOWED);
        wrist.setState(WristState.STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CLAW_ALGAE -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        elevator.setState(ElevatorState.STOWED);
        wrist.setState(WristState.STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CLAW_CORAL -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        elevator.setState(ElevatorState.STOWED);
        wrist.setState(WristState.STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L1_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        elevator.setState(ElevatorState.CORAL_SCORE_LINEUP_L1);
        wrist.setState(WristState.CORAL_SCORE_LINEUP_L1);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L1_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        elevator.setState(ElevatorState.CORAL_SCORE_LINEUP_L1);
        wrist.setState(WristState.CORAL_SCORE_LINEUP_L1);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L1_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        elevator.setState(ElevatorState.CORAL_SCORE_RELEASE_L1);
        wrist.setState(WristState.CORAL_SCORE_RELEASE_L1);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L2_APPROACH -> {
        claw.setState(ClawState.IDLE_NO_GP);
        elevator.setState(ElevatorState.ALGAE_INTAKE_L2);
        wrist.setState(WristState.ALGAE_INTAKE_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L2 -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        elevator.setState(ElevatorState.ALGAE_INTAKE_L2);
        wrist.setState(WristState.ALGAE_INTAKE_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L2_HOLDING -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        elevator.setState(ElevatorState.ALGAE_INTAKE_L2);
        wrist.setState(WristState.ALGAE_INTAKE_L2);
        swerve.normalDriveRequest();
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L3_APPROACH -> {
        claw.setState(ClawState.IDLE_NO_GP);
        elevator.setState(ElevatorState.ALGAE_INTAKE_L3);
        wrist.setState(WristState.ALGAE_INTAKE_L3);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L3 -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        elevator.setState(ElevatorState.ALGAE_INTAKE_L3);
        wrist.setState(WristState.ALGAE_INTAKE_L3);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L3_HOLDING -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        elevator.setState(ElevatorState.ALGAE_INTAKE_L3);
        wrist.setState(WristState.ALGAE_INTAKE_L3);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_PROCESSOR_WAITING -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        elevator.setState(ElevatorState.PROCESSOR);
        wrist.setState(WristState.ALGAE_PROCESSOR);
        swerve.setSnapToAngle(SnapUtil.getProcessorAngle());
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_PROCESSOR_RELEASE -> {
        claw.setState(ClawState.SCORE_ALGAE_PROCESSOR);
        elevator.setState(ElevatorState.PROCESSOR);
        wrist.setState(WristState.ALGAE_PROCESSOR);
        swerve.setSnapToAngle(SnapUtil.getProcessorAngle());
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_NET_WAITING -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        elevator.setState(ElevatorState.ALGAE_NET);
        wrist.setState(WristState.ALGAE_NET);
        swerve.setSnapToAngle(SnapUtil.getNetScoringAngle(robotPose));
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_NET_RELEASE -> {
        claw.setState(ClawState.SCORE_ALGAE_NET);
        elevator.setState(ElevatorState.ALGAE_NET);
        wrist.setState(WristState.ALGAE_NET);
        swerve.setSnapToAngle(SnapUtil.getNetScoringAngle(robotPose));
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CLIMBER_STOP -> {
        claw.setState(ClawState.IDLE_NO_GP);
        elevator.setState(ElevatorState.CLIMBING);
        wrist.setState(WristState.CLIMBING);
        swerve.setSnapToAngle(SnapUtil.getCageAngle(FmsUtil.isRedAlliance()));
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CLIMBING_1_LINEUP -> {
        claw.setState(ClawState.IDLE_NO_GP);
        elevator.setState(ElevatorState.CLIMBING);
        wrist.setState(WristState.CLIMBING);
        swerve.setSnapToAngle(SnapUtil.getCageAngle(FmsUtil.isRedAlliance()));
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.LINEUP_FORWARD);
      }
      case CLIMBING_2_HANGING -> {
        claw.setState(ClawState.IDLE_NO_GP);
        elevator.setState(ElevatorState.CLIMBING);
        wrist.setState(WristState.CLIMBING);
        swerve.setSnapToAngle(SnapUtil.getCageAngle(FmsUtil.isRedAlliance()));
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.HANGING);
      }
      case ALGAE_INTAKE_FLOOR -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        elevator.setState(ElevatorState.ALGAE_INTAKE_GROUND);
        wrist.setState(WristState.ALGAE_INTAKE_GROUND);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_OUTTAKE -> {
        claw.setState(ClawState.OUTTAKING);
        elevator.setState(ElevatorState.ALGAE_OUTTAKE);
        wrist.setState(WristState.ALGAE_OUTTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_INTAKE_GROUND -> {
        claw.setState(ClawState.INTAKING_CORAL);
        elevator.setState(ElevatorState.CORAL_INTAKE);
        wrist.setState(WristState.CORAL_INTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_OUTTAKE -> {
        claw.setState(ClawState.OUTTAKING);
        elevator.setState(ElevatorState.ALGAE_OUTTAKE);
        wrist.setState(WristState.ALGAE_OUTTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case REHOME_ELEVATOR -> {
        claw.setState(ClawState.IDLE_NO_GP);
        elevator.setState(ElevatorState.MID_MATCH_HOMING);
        wrist.setState(WristState.STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case STARTING_POSITION -> {
        claw.setState(ClawState.IDLE_NO_GP);
        elevator.setState(ElevatorState.STOWED);
        wrist.setState(WristState.STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case UNJAM -> {
        claw.setState(ClawState.OUTTAKING);
        elevator.setState(ElevatorState.UNJAM);
        wrist.setState(WristState.UNJAM);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        climber.setState(ClimberState.STOPPED);
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

    var scoringLevel =
        switch (getState()) {
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

  public void intakeCoralRequest() {
    setStateFailsafe(RobotState.CORAL_INTAKE_GROUND);
  }

  public void l1ApproachRequest() {
    setStateFailsafe(RobotState.CORAL_L1_APPROACH);
  }

  public void lowLineupRequest() {
    switch (getState().heldGamePiece) {
      case ALGAE, NONE -> processorWaitRequest();
      case CORAL -> l1ApproachRequest();
    }
  }

  public void algaeReefIntakeRequest() {
    if (nearestReefSide.algaeHeight == ReefPipeLevel.L3) {
      intakeL3AlgaeRequest();
    } else {
      intakeL2AlgaeRequest();
    }
  }

  public void stowRequest() {
    if (!claw.getHasGP()) {
      setStateFailsafe(RobotState.CLAW_EMPTY);
      return;
    }
    switch (getState()) {
      case CLAW_ALGAE, CLAW_CORAL, CLAW_EMPTY -> getState();
      case ALGAE_INTAKE_L2_HOLDING, ALGAE_INTAKE_L3_HOLDING ->
          setStateFailsafe(RobotState.CLAW_ALGAE);
      case CORAL_L1_APPROACH, CORAL_L1_LINEUP -> setStateFailsafe(RobotState.CLAW_CORAL);
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
          CORAL_L1_RELEASE,
          CORAL_OUTTAKE,
          REHOME_ELEVATOR,
          STARTING_POSITION,
          UNJAM -> {}

      case CLAW_EMPTY -> setStateFailsafe(RobotState.ALGAE_OUTTAKE);
      case CLAW_ALGAE -> netWaitRequest();
      case CLAW_CORAL -> l1ApproachRequest();

      case ALGAE_NET_WAITING -> setStateFailsafe(RobotState.ALGAE_NET_RELEASE);
      case ALGAE_PROCESSOR_WAITING -> setStateFailsafe(RobotState.ALGAE_PROCESSOR_RELEASE);
      case CORAL_L1_LINEUP -> setStateFailsafe(RobotState.CORAL_L1_RELEASE);
    }
  }

  public void climberSequenceForward() {
    switch (getState()) {
      case CLIMBER_STOP -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
      case STARTING_POSITION, CLAW_EMPTY, CLAW_CORAL, CLAW_ALGAE -> {
        if (wrist.atGoal() && elevator.atGoal()) {
        setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
        }
      }
      case CLIMBING_1_LINEUP -> setStateFromRequest(RobotState.CLIMBING_2_HANGING);
      case CLIMBING_2_HANGING -> {}
      default -> {
        // Do nothing
      }
    }
  }

  public void climberSequenceStop() {
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> setStateFromRequest(RobotState.CLIMBER_STOP);
      default -> {}
    }
  }
}
