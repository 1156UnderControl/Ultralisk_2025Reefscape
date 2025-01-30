package frc.Java_Is_UnderControl.Vision.Cameras.Types.Interfaces;

import java.util.Optional;

import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimation;

public interface ICameraOdometry {

  public void setAprilTagData();

  public void updateLogsAprilTag();

  public Optional<PoseEstimation> getRobotPose();
}
