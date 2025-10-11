package com.team581.autos;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class AbstractCommandAuto extends BaseAuto {
  protected final AutoTiming timing = new AutoTiming(name());
  private final Command autoCommand = createFullAutoCommand();

  protected abstract Command createAutoCommand();

  protected abstract Command createFullAutoCommand();

  public Command getAutoCommand() {
    return autoCommand;
  }
}
