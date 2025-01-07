package frc.Java_Is_UnderControl.Vision.Cameras;

public interface ICamera {
    void camToRobot(double cameraLensHeightMeters, double goalHeightMeters, double cameraMountAngleDEG);

    boolean hasTarget();
    int getNumberOfTargetsDetected();
    double getDistanceTarget();

    void setPipeline(int index);
}
