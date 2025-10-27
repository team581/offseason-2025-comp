package frc.robot.autos.auto_state_machines;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autos.Points;

public enum StationAndLollipop5pcAutoState {
  INTAKING(Points.GROUND_INTAKE_LEFT_STATION.getPose()),
  LOLLIPOP_2(Points.LOLLIPOP_2.getPose()),
  PRE_LOLLIPOP_2(Points.PRE_LOLLIPOP_2.getPose()),
  SCORE(new Pose2d());

  public final Pose2d pose;

  public Pose2d getPose() {
    return this.pose;
  }

  private StationAndLollipop5pcAutoState(Pose2d pose) {
    this.pose = pose;
  }
}
