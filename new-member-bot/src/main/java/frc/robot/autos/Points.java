package frc.robot.autos;

import com.team581.autos.Point;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum Points {
  START_R1_AND_B1(Point.ofRed(new Pose2d(10.289, 0.47, Rotation2d.fromDegrees(90)))),
  START_R1_AND_B1_FORWARD(Point.ofRed(new Pose2d(10.289, 0.758, Rotation2d.kZero))),

  START_R2_AND_B2(Point.ofRed(new Pose2d(10.289, 1.903, Rotation2d.kZero))),
  START_R3_AND_B3(Point.ofRed(new Pose2d(10.289, 3.054, Rotation2d.kZero))),
  START_R3_AND_B3_LEFT_FORWARD(Point.ofRed(new Pose2d(10.289, 3.054, Rotation2d.kCW_90deg))),

  START_R4_AND_B4(Point.ofRed(new Pose2d(10.289, 5.069, Rotation2d.kZero))),
  START_R5_AND_B5(Point.ofRed(new Pose2d(10.289, 6.127, Rotation2d.kZero))),
  START_R6_AND_B6_FORWARD(Point.ofRed(new Pose2d(10.289, 7.292, Rotation2d.kZero))),
  START_R6_AND_B6(Point.ofRed(new Pose2d(10.289, 7.58, Rotation2d.fromDegrees(90)))),

  START_MIDDLE_BARGE(Point.ofRed(new Pose2d(10.289, 3.973, Rotation2d.kZero))),

  LEFT_CORAL_STATION(Point.ofRed(new Pose2d(16.194, 0.861, Rotation2d.fromDegrees(125.309)))),
  RIGHT_CORAL_STATION(Point.ofRed(new Pose2d(16.1, 7.35, Rotation2d.fromDegrees(-131.807)))),

  GROUND_INTAKE_LEFT_STATION(Point.ofRed(new Pose2d(16.124, 1.148, Rotation2d.fromDegrees(0))));

  public final Point point;

  Points(Point point) {
    this.point = point;
  }

  public Pose2d getPose() {
    return point.getPose();
  }
}
