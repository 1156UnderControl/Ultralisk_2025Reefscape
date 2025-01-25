package frc.Java_Is_UnderControl.Vision.Cameras.Types.Interfaces;


import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import frc.Java_Is_UnderControl.Vision.Cameras.Data.AprilTagData;

public interface ICameraOdometry {

    public AprilTagData getAprilTagData();

    public void updateLogsAprilTag();

    public Optional<Pose3d> getRobotPose();
}