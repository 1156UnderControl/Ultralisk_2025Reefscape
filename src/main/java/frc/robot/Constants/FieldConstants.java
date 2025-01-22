// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class FieldConstants {
  public static Translation2d processor = new Translation2d(133.652, 162.095);

  public static Pose2d reefF = new Pose2d(-147.721, -23.385, new Rotation2d());
  public static Pose2d reefE = new Pose2d(-146.931, -23.260, new Rotation2d());
  public static Pose2d reefD = new Pose2d(-177.670, -29.722, new Rotation2d());
  public static Pose2d reefC = new Pose2d(-188.847, -23.248, new Rotation2d());
  public static Pose2d reefA = new Pose2d(-198.586, -6.444, new Rotation2d());
  public static Pose2d reefB = new Pose2d(-198.631, -6.442, new Rotation2d());
  public static Pose2d reefG = new Pose2d(-137.200, 6.430, new Rotation2d());
  public static Pose2d reefH = new Pose2d(-137.155, 6.432, new Rotation2d());
  public static Pose2d reefI = new Pose2d(-146.953, 23.223, new Rotation2d());
  public static Pose2d reefJ = new Pose2d(-158.139, 29.708, new Rotation2d());
  public static Pose2d reefK = new Pose2d(-177.648, 29.685, new Rotation2d());
  public static Pose2d reefL = new Pose2d(-188.870, 23.289, new Rotation2d());

  public static boolean poseOutOfField(Pose2d pose2D) {
    double x = pose2D.getX();
    double y = pose2D.getY();
    return (x <= 0 || x >= fieldLength) || (y <= 0 || y >= fieldWidth);
  }
}
