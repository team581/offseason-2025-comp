package com.team581.autos;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;

public abstract class AbstractCommandAuto implements BaseAuto {
  protected final AutoTiming timing = new AutoTiming(name());
  private Optional<Command> autoCommand = Optional.empty();

  protected abstract Command createAutoCommand();

  protected abstract Command createFullAutoCommand();

  public Command getAutoCommand() {
    if (autoCommand.isEmpty()) {
      autoCommand = Optional.of(createFullAutoCommand());
    }

    return autoCommand.orElseThrow();
  }
}
