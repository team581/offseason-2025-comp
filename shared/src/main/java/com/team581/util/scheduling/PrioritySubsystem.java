package com.team581.util.scheduling;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface PrioritySubsystem extends Subsystem {
  SubsystemPriorityBase getPriority();
}
