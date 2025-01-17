package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;

public class ArmSubsystem extends SubsystemBase implements IArm{

    private static ArmSubsystem armInstance = null;
    private SparkMAXMotor motorArm1;

    public static ArmSubsystem getInstance() {
        if (armInstance == null) {
          armInstance = new ArmSubsystem();
        }
        return armInstance;
    }

    private ArmSubsystem(){
        motorArm1 = new SparkMAXMotor(0, "Motor Arm");
        motorArm1.setPositionFactor(360);
    }

    @Override
    public void go180Degrees(){}

    @Override
    public boolean isAtSetPoint(){
        if(motorArm1.getPosition() == 180){
            return true;
        } else {
            return false;
        }
    }
}
