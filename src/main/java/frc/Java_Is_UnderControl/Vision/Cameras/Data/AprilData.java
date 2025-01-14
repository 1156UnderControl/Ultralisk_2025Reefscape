package frc.Java_Is_UnderControl.Vision.Cameras.Data;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class AprilData{

    private Map<Integer, TargetData> mapTargetData;
    private TargetData aprilTagData;
    private int aprilTagID;

    public AprilData(Map<Integer, TargetData> mapTargetData){
        this.mapTargetData = mapTargetData;
    }

    private void setAprilsData() {
        for (Map.Entry<Integer, TargetData> entry : mapTargetData.entrySet()) {
            int tagID = entry.getKey();
            TargetData data = entry.getValue();
            this.aprilTagID = tagID;
            this.aprilTagData = data;
        }
    }

    public int getAprilID(){
        this.setAprilsData();
        return this.aprilTagID;
    }

    public TargetData getAprilData(){
        this.setAprilsData();
        return this.aprilTagData;
    }

    public double getPitch() {
        return this.aprilTagData.getPitch();
    }

    public double getYaw() {
        return this.aprilTagData.getYaw();
    }

    public double getSkew() {
        return this.aprilTagData.getSkew();
    }

    public double getArea(){
        return this.aprilTagData.getArea();
    }

    public double getDistanceTarget() {
        return this.aprilTagData.getDistanceTarget();
    }

    public Transform3d getCameraPosition(){
        return this.aprilTagData.getCameraPosition();
    }

    public boolean hasTargets(){
        if(this.getYaw() == 0 &&
            this.getPitch() == 0 &&
            this.getSkew() == 0 &&
            this.getArea() == 0){
                return false;
        } else {
            return true;
        }
    }

    public Pose3d getTargetPose(){
            return this.aprilTagData.getTargetPose();
    }

    public Transform3d getBestCameraToTarget(){
        return this.aprilTagData.getBestCameraToTarget();
    }
}
