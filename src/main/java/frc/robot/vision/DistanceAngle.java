package frc.robot.vision;

// Target angle should be the angle that the robot needs to be to be looking directly at the target
public record DistanceAngle(double distance, double targetAngle, boolean seesSpeakerTag) {}
