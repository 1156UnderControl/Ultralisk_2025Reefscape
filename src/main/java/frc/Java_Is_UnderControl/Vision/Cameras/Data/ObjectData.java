package frc.Java_Is_UnderControl.Vision.Cameras.Data;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose3dLogger;
import frc.Java_Is_UnderControl.Vision.Object_Detection.ObjectDetection;

public class ObjectData {

    private String cameraName;
    private ObjectDetection objectDetection;
    private double distanceToObject;
    private Transform3d camToRobot;
    private Transform3d bestCameraToObject;
    private Pose3d objectPose;

    ObjectData objectData;
    CustomDoubleLogger pitchLog;
    CustomDoubleLogger yawLog;
    CustomDoubleLogger skewLog;
    CustomDoubleLogger areaLog;
    CustomDoubleLogger distanceToAprilLog;
    CustomPose3dLogger aprilPoseLog;

    public ObjectData(String cameraName,
        ObjectDetection objectDetection,
        double distanceToObject,
        Transform3d bestCameraToobject,
        Transform3d camToRobot,
        Pose3d objectPose){
        
        this.objectDetection = objectDetection;
        this.distanceToObject = distanceToObject;
        this.bestCameraToObject = bestCameraToobject;
        this.camToRobot = camToRobot;
        this.objectPose = objectPose;
    }

    public String getCameraName() {
        return this.cameraName;
    }

    public double getPitch() {
        return this.objectDetection.ty;
    }

    public double getYaw() {
        return this.objectDetection.tx;
    }

    public double getDistanceTarget() {
        return this.distanceToObject;
    }

    public Transform3d getCameraPosition() {
        return this.camToRobot;
    }

    public boolean hasObjects() {
        if (this.getYaw() == 0 && this.getPitch() == 0) {
            return false;
        } else {
            return true;
        }
    }

    public Pose3d getTargetPose() {
        return this.objectPose;
    }

    public Transform3d getBestCameraToTarget() {
        return this.bestCameraToObject;
    }

    private void setLogs(){
        this.pitchLog = new CustomDoubleLogger("/Vision/" + this.objectData.getCameraName() + "/" + (this.hasTargets() ? "/pitch" : "No Objects Seen"));
        this.yawLog = new CustomDoubleLogger("/Vision/" + this.objectData.getCameraName() + "/" + (this.hasTargets() ? "/yaw" : "No Objects Seen"));
        this.distanceToAprilLog = new CustomDoubleLogger("/Vision/" + objectData.getCameraName() + "/" + (this.hasTargets() ? "/distanceToApril" : "No Objects Seen"));
        this.aprilPoseLog = new CustomPose3dLogger("/Vision/" + objectData.getCameraName() + "/" + (this.hasTargets() ? "/aprilPose" : "No Objects Seen"));
    }

    public void updateLogs(){
        if(this.hasTargets()){
            this.setLogs();
            this.pitchLog.append(this.objectData.getPitch());
            this.yawLog.append(this.objectData.getYaw());
            this.distanceToAprilLog.append(this.objectData.getDistanceTarget());
            this.aprilPoseLog.appendDegrees(this.objectData.getTargetPose());
        }
    }

    public boolean hasTargets(){
        return this.objectData.hasTargets();
    }
}
