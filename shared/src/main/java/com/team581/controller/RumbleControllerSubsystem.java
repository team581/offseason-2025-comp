package com.team581.controller;

import com.team581.util.scheduling.SubsystemPriorityBase;
import com.team581.util.state_machines.StateMachineSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RumbleControllerSubsystem extends StateMachineSubsystem<RumbleControllerState> {
  private final Timer matchTimer = new Timer();
  private final GenericHID controller;
  public static final double MATCH_DURATION_TELEOP = 135;

  @Override
  public void teleopInit() {
    matchTimer.reset();
    matchTimer.start();
  }

  @Override
  public void disabledInit() {
    matchTimer.stop();
  }

  public RumbleControllerSubsystem(
      CommandGenericHID controller, boolean matchTimeRumble, SubsystemPriorityBase priority) {
    this(controller.getHID(), matchTimeRumble, priority);
  }

  public RumbleControllerSubsystem(
      GenericHID controller,
      boolean matchTimeRumble,
      SubsystemPriorityBase rumbleControllerPriority) {
    super(rumbleControllerPriority, RumbleControllerState.OFF);
    this.controller = controller;

    if (matchTimeRumble) {
      var rumbleCommand = runOnce(this::rumbleRequest).withName("RumbleCommand");
      new Trigger(() -> matchTimer.hasElapsed(MATCH_DURATION_TELEOP - 90)).onTrue(rumbleCommand);
      new Trigger(() -> matchTimer.hasElapsed(MATCH_DURATION_TELEOP - 60)).onTrue(rumbleCommand);
      new Trigger(() -> matchTimer.hasElapsed(MATCH_DURATION_TELEOP - 30)).onTrue(rumbleCommand);
    }
  }

  public void rumbleRequest() {
    if (!DriverStation.isAutonomous()) {
      setStateFromRequest(RumbleControllerState.ON);
    }
  }

  @Override
  protected RumbleControllerState getNextState(RumbleControllerState currentState) {
    return switch (currentState) {
      case ON -> timeout(0.5) ? RumbleControllerState.OFF : currentState;
      case OFF -> currentState;
    };
  }

  @Override
  protected void afterTransition(RumbleControllerState newState) {
    switch (newState) {
      case ON -> controller.setRumble(RumbleType.kBothRumble, 1);
      case OFF -> controller.setRumble(RumbleType.kBothRumble, 0);
    }
  }
}
