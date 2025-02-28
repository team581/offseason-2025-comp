package frc.robot.auto_align.field_calibration;

record Summary(MechanismState elevator, MechanismState wrist, boolean align) {
  public String format() {
    var alignLabel = align ? "ok" : "bad";
    return "Elevator " + elevator.label + ", wrist " + wrist.label + ", align " + alignLabel;
  }

  public boolean isOk() {
    return elevator == MechanismState.OK && wrist == MechanismState.OK && align;
  }
}
