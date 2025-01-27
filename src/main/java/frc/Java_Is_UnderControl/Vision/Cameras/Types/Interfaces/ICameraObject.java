package frc.Java_Is_UnderControl.Vision.Cameras.Types.Interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import frc.Java_Is_UnderControl.Vision.Cameras.Data.ObjectData;

public interface ICameraObject {

    public ObjectData getObjectData();

    public void updateLogsObject();

    public Pose2d getObjectPose();
}
