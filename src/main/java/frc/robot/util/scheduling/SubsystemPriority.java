package frc.robot.util.scheduling;

public enum SubsystemPriority {
  // Ensures that state machine inputs are gathered at the right time
  // Subsystem inputs are collected in reverse order of priority (so lowest priority first)
  STATE_MACHINE_INPUT_MANAGER(999),

  // 20-30 is for manager subsystems
  AUTOS(30),
  ROBOT_MANAGER(29),
  GROUND_MANAGER(28),

  // 10-19 is for sensor subsystems
  SWERVE(11),
  LOCALIZATION(11),
  AUTO_ALIGN(11),
  FMS(11),
  IMU(11),
  // Vision runs before localization so that it has fresh vision data for pose estimator
  VISION(10),

  // 0-9 is for actuator subsystems
  DEPLOY(0),
  INTAKE(0),
  CLAW(0),
  ELEVATOR(0),
  ARM(0),
  CLIMBER(0),
  LIGHTS(0),
  RUMBLE_CONTROLLER(0);

  public final int value;

  private SubsystemPriority(int priority) {
    this.value = priority;
  }
}
