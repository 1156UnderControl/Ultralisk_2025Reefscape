package frc.Java_Is_UnderControl.Vision.Cameras.Data;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class TargetData {
    private String cameraName;
    private int targetID;
    private double targetsYaw;
    private double targetsPitch;
    private double targetsSkew;
    private double targetsArea;
    private double distanceToTargets;
    private Transform3d camToRobot;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private Transform3d bestCameraToTarget;
    private boolean isTag;

    public TargetData(
        String cameraName,
        int targetID,
        double targetsYaw,
        double targetsPitch,
        double targetsSkew,
        double targetsArea,
        double distanceToTargets,
        Transform3d bestCameraToTarget,
        Transform3d camToRobot){
        
        this.isTag = true;
        this.targetID = targetID;
        this.targetsYaw = targetsYaw;
        this.targetsPitch = targetsPitch;
        this.targetsSkew = targetsSkew;
        this.targetsArea = targetsArea;
        this.distanceToTargets = distanceToTargets;
        this.bestCameraToTarget = bestCameraToTarget;
        this.camToRobot = camToRobot;
        this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    }

    public TargetData(
        String cameraName,
        double targetsYaw,
        double targetsPitch,
        double targetsSkew,
        double targetsArea,
        double distanceToTargets,
        Transform3d camToRobot){
        
        this(cameraName, 
        0, 
        targetsYaw,
        targetsPitch,
        targetsSkew,
        targetsArea,
        distanceToTargets,
        Transform3d.kZero,
        camToRobot);
        this.isTag = false;
    }

    public String getCameraName(){
        return this.cameraName;
    }

    public double getPitch() {
        return this.targetsPitch;
    }

    public double getYaw() {
        return this.targetsYaw;
    }

    public double getSkew() {
        return this.targetsSkew;
    }

    public double getArea(){
        return this.targetsArea;
    }

    public double getDistanceTarget() {
        return this.distanceToTargets;
    }

    public Transform3d getCameraPosition(){
        return this.camToRobot;
    }

    public boolean hasTargets(){
        if(this.targetsYaw == 0 &&
            this.targetsPitch == 0 &&
            this.targetsSkew == 0 &&
            this.targetsArea == 0){
                return false;
        } else {
            return true;
        }
    }

    public Pose3d getTargetPose(){
        if(isTag){
            return aprilTagFieldLayout.getTagPose(targetID).get();
        } else {
            return null;
        }
    }

    public Transform3d getBestCameraToTarget(){
        return this.bestCameraToTarget;
    }
}
