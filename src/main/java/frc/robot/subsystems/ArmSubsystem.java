package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
        motorArm1 = new SparkMAXMotor(2, "Motor Arm");
        motorArm1.setPositionFactor(0.0119047619 * 360);
    }

    @Override
    public void go180Degrees(){
        motorArm1.setPosition(180);
    }

    public Command sysIdDynamicForward(){
        motorArm1.setSysID(this);
        return motorArm1.sysIdDynamic(Direction.kForward).until(()-> motorArm1.getPosition() >= 0).finallyDo(()-> motorArm1.setPosition(0));
    }

    public Command sysIdDynamicReverse(){
        motorArm1.setSysID(this);
        return motorArm1.sysIdDynamic(Direction.kReverse).until(()-> motorArm1.getPosition() <= -180).finallyDo(()-> motorArm1.setPosition(-180));
    }

    public Command sysIdQuasistaticForward(){
        motorArm1.setSysID(this);
        return motorArm1.sysIdQuasistatic(Direction.kReverse).until(()-> motorArm1.getPosition() <= -180).finallyDo(()-> motorArm1.setPosition(-180));
    }

    public Command sysIdQuasistaticReverse(){
        motorArm1.setSysID(this);
        return motorArm1.sysIdQuasistatic(Direction.kReverse).until(()-> motorArm1.getPosition() >= 0).finallyDo(()-> motorArm1.setPosition(0));
    }

    public void updateLogs(){
        SmartDashboard.putNumber("Arm Position", this.motorArm1.getPosition());
    }
}
