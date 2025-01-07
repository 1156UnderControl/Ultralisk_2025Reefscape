package frc.Java_Is_UnderControl.Vision;

import edu.wpi.first.math.geometry.Pose2d;

public class TargetData {
    public int aprilID;
    public Pose2d targetPose;
    public double targetYaw;
    public double targetPitch;
    public double targetSkew;
    public double targetArea;
    public double distanceToTarget;

    public TargetData(int aprilID,
        double targetYaw,
        double targetPitch,
        double targetSkew,
        double targetArea,
        double distanceToTarget){

        this.aprilID = aprilID;
        this.targetYaw = targetYaw;
        this.targetPitch = targetPitch;
        this.targetSkew = targetSkew;
        this.targetArea = targetArea;
        this.distanceToTarget = distanceToTarget;
    }

    public int getAprilID(){
        return this.aprilID;
    }

    public double getYaw(){
        return this.targetYaw;
    }

    public double getPitch(){
        return this.targetPitch;
    }

    public double getSkew(){
        return this.targetSkew;
    }

    public double getArea(){
        return this.targetArea;
    }
    
    public double getDistanceTarget(){
        return this.distanceToTarget;
    }
}
