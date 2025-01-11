package frc.robot.autos.trailblazer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.trailblazer.BaseAuto;
import frc.robot.autos.trailblazer.AutoPoint;
import frc.robot.autos.trailblazer.AutoSegment;
import frc.robot.autos.trailblazer.Trailblazer;
import frc.robot.autos.trailblazer.constraints.AutoConstraintOptions;
import frc.robot.robot_manager.RobotManager;
import frc.robot.autos.trailblazer.Trailblazer;
import frc.robot.robot_manager.RobotManager;

public class Auto3Piece2IJK extends BaseAuto{
    public Auto3Piece2IJK(RobotManager robotManager, Trailblazer trailblazer) {
        super(robotManager, trailblazer);
    }


@Override
protected Command getBlueAutoCommand() {
  return Commands.none();
}

@Override
protected Command getRedAutoCommand() {
return Commands.sequence(
    Commands.print(""),
    Commands.runOnce(
        () -> 
        robotManager.localization.resetPose(new Pose2d(0,0, Rotation2d.fromDegrees(.0)))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(9.434,1.903, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(new Pose2d(12.493,2.774, Rotation2d.fromDegrees(0.0))))),
    Commands.print("score"),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(14.202,1.568, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(new Pose2d(15.686,0.524, Rotation2d.fromDegrees(0.0))))),
    Commands.print("intake"),
    trailblazer.followSegment(
        new AutoSegment(
            new AutoPoint(new Pose2d(14.66,1.568, Rotation2d.fromDegrees(0.0))),
            new AutoPoint(new Pose2d(13.506,2.774, Rotation2d.fromDegrees(0.0))))),
    Commands.print("score"),
    trailblazer.followSegment(
        new AutoSegment(
            new AutoPoint(new Pose2d(14.202,1.568, Rotation2d.fromDegrees(0.0))),
            new AutoPoint(new Pose2d(15.686, 0.524, Rotation2d.fromDegrees(0.0))))),
    Commands.print("intake"),
    trailblazer.followSegment(
        new AutoSegment(
            new AutoPoint(new Pose2d(13.983, 2.925, Rotation2d.fromDegrees(0.0))))),
    Commands.print("")
);
}
}