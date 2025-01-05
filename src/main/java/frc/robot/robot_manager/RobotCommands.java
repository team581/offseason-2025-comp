package frc.robot.robot_manager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;

public class RobotCommands {
  private final RobotManager robot;
  private final Subsystem[] requirements;

  public RobotCommands(RobotManager robot) {
    this.robot = robot;
    var requirementsList = List.of(robot.arm, robot.intake, robot.queuer, robot.shooter);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command intakeCommand() {
    return Commands.runOnce(robot::intakeRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_WITH_GP));
  }

  public Command intakeAssistCommand() {
    return Commands.runOnce(robot::intakeAssistRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_WITH_GP));
  }

  public Command outtakeCommand() {
    return Commands.runOnce(robot::outtakeRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command speakerCommand() {
    return Commands.runOnce(robot::prepareSpeakerRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command passCommand() {
    return Commands.runOnce(robot::preparePassRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command stowCommand() {
    return Commands.runOnce(robot::stowRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command idleWithGpCommand() {
    return Commands.runOnce(robot::idleWithGpRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_WITH_GP));
  }

  public Command stopIntakingCommand() {
    return Commands.runOnce(robot::stowRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command waitSubwooferCommand() {
    return Commands.runOnce(robot::waitSubwooferRequest, requirements);
  }

  public Command waitPodiumCommand() {
    return Commands.runOnce(robot::waitPodiumRequest, requirements);
  }

  public Command waitAmpCommand() {
    return Commands.runOnce(robot::waitAmpRequest, requirements);
  }

  public Command waitSpeakerCommand() {
    return Commands.runOnce(robot::waitSpeakerRequest, requirements);
  }

  public Command confirmShotCommand() {

    return Commands.runOnce(robot::confirmShotRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command ampCommand() {
    return Commands.runOnce(robot::prepareAmpRequest, requirements);
  }

  public Command stopShootingCommand() {
    return Commands.runOnce(robot::stopShootingRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_WITH_GP));
  }

  public Command feedingCommand() {
    return Commands.runOnce(robot::prepareFeedRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command waitFeedingCommand() {
    return Commands.runOnce(robot::waitFeedRequest, requirements);
  }

  public Command subwooferCommand() {
    return Commands.runOnce(robot::prepareSubwooferRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command podiumCommand() {
    return Commands.runOnce(robot::preparePodiumRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command climbUpCommand() {
    return Commands.runOnce(robot::nextClimbStateRequest, requirements);
  }

  public Command climbDownCommand() {
    return Commands.runOnce(robot::previousClimbStateRequest, requirements);
  }

  public Command unjamCommand() {
    return Commands.runOnce(robot::unjamRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }
}
