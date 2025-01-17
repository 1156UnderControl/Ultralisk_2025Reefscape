package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
        motorArm1.setPositionFactor(0.01388888888 * 360);
    }

    @Override
    public void go180Degrees(){
    }

    public void sysIdDynamicForward(){
        motorArm1.setSysID(this);
        motorArm1.sysIdDynamic(Direction.kForward).until(()-> motorArm1.getPosition() < 80);
    }

    public void sysIdDynamicReverse(){
        motorArm1.setSysID(this);
        motorArm1.sysIdDynamic(Direction.kReverse).until(()-> motorArm1.getPosition() < 80);
    }

    public void sysIdQuasistaticForward(){
        motorArm1.setSysID(this);
        motorArm1.sysIdQuasistatic(Direction.kReverse).until(()-> motorArm1.getPosition() < 80);
    }

    public void sysIdQuasistaticReverse(){
        motorArm1.setSysID(this);
        motorArm1.sysIdQuasistatic(Direction.kReverse).until(()-> motorArm1.getPosition() < 80);
    }

    @Override
    public boolean isAtSetPoint(){
        if(motorArm1.getPosition() == 80 || motorArm1.getPosition() == -80){
            return true;
        } else {
            return false;
        }
    }

    public void updateLogs(){
        SmartDashboard.putNumber("Arm Position", this.motorArm1.getPosition());
    }
}
