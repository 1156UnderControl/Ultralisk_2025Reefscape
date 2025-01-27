package frc.Java_Is_UnderControl.Vision.Cameras.Data;

import edu.wpi.first.math.geometry.Transform3d;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomTransform3dLogger;
import frc.Java_Is_UnderControl.Vision.Object_Detection.ObjectDetection;

public class ObjectData {

    private String cameraName;
    private String objectName;
    private ObjectDetection objectDetection;
    private double distanceToObject;
    private Transform3d camToRobot;

    private CustomDoubleLogger yawLog;
    private CustomDoubleLogger pitchLog;
    private CustomDoubleLogger distanceToObjectLog;
    private CustomTransform3dLogger camToRobotLog;

    public ObjectData(String cameraName,
    String objectName,
    ObjectDetection objectDetection,
    double distanceToObject,
    Transform3d camToRobot){
        
        this.objectDetection = objectDetection;
        
        this.camToRobot = camToRobot;
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

    private void setLogs(){
        this.pitchLog = new CustomDoubleLogger("/Vision/" + this.cameraName + "/" + (this.hasTargets() ? "pitch" + this.objectName  : "No " + this.objectName + " Objects Seen"));
        this.yawLog = new CustomDoubleLogger("/Vision/" + this.cameraName + "/" + (this.hasTargets() ? "yaw" + this.objectName  : "No " + this.objectName + " Objects Seen"));
        this.distanceToObjectLog = new CustomDoubleLogger("/Vision/" + this.cameraName + "/" + (this.hasTargets() ? "distanceTo" + this.objectName  : "No " + this.objectName + " Objects Seen"));
        this.camToRobotLog = new CustomTransform3dLogger("/Vision/" + this.cameraName + "/" + "cameraPosition");
    }

    public void updateLogs(){
        if(this.hasTargets()){
            this.setLogs();
            this.pitchLog.append(this.getPitch());
            this.yawLog.append(this.getYaw());
            this.distanceToObjectLog.append(this.distanceToObject);
            this.camToRobotLog.appendDegrees(this.camToRobot);
        }
    }

    public boolean hasTargets(){
        if(this.getPitch() == Double.NaN && this.getYaw() == Double.NaN){
            return false;
        } else {
            return true;
        }
    }
}
