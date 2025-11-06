package frc.robot.autos.auto_state_machines;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autos.Points;

public enum AutoState {
  INTAKING(Points.GROUND_INTAKE_LEFT_STATION),
  LOLLIPOP_2(Points.LOLLIPOP_2),
  SCORE(Points.GROUND_INTAKE_LEFT_STATION);

  public final Points point;

  public Pose2d getPose() {
    return this.point.getPose();
  }

  private AutoState(Points point) {
    this.point = point;
  }
}
