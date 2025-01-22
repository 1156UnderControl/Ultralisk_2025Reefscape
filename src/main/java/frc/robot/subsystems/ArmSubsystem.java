package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;

public class ArmSubsystem extends SubsystemBase implements IArm{

    private static ArmSubsystem armInstance = null;
    private SparkMAXMotor motorArm1;
    private ArmFeedforward ffArm = new ArmFeedforward(0.17541, 0.68608, 0.0018468, 0.00011273);

    public static ArmSubsystem getInstance() {
        if (armInstance == null) {
          armInstance = new ArmSubsystem();
        }
        return armInstance;
    }

    private ArmSubsystem(){
        motorArm1 = new SparkMAXMotor(2, "Motor Arm");
        motorArm1.setPositionFactor(0.0126984126984127 * 360);
        motorArm1.setVelocityFactor(0.00211640211);
        motorArm1.configureMaxMagic(0.0023079, 0.0, 0.000001415, ffArm.calculate(0, 0), 20, 20, 0.01);
        motorArm1.setMotorBrake(true);
        motorArm1.burnFlash();
    }

    @Override
    public void go180Degrees(){
    }

    public BooleanSupplier atLimitReverse(){
        return () -> motorArm1.getPosition() <= -100;
    }

    public BooleanSupplier atLimitForward(){
        return () -> motorArm1.getPosition() >= 0;
    }

    public Command sysIdDynamicForward(){
        motorArm1.setSysID(this);
        return motorArm1.sysIdDynamic(Direction.kForward);
    }

    public Command sysIdDynamicReverse(){
        motorArm1.setSysID(this);
        return motorArm1.sysIdDynamic(Direction.kReverse);
    }

    public Command sysIdQuasistaticForward(){
        motorArm1.setSysID(this);
        return motorArm1.sysIdQuasistatic(Direction.kForward);
    }

    public Command sysIdQuasistaticReverse(){
        motorArm1.setSysID(this);
        return motorArm1.sysIdQuasistatic(Direction.kReverse);
    }

    private void updateLogs(){
        SmartDashboard.putNumber("Arm Position", this.motorArm1.getPosition());
    }

    public void stopMotor(){
        motorArm1.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("limit reverse?", atLimitReverse().getAsBoolean());
        SmartDashboard.putBoolean("limit forward?", atLimitForward().getAsBoolean());

        this.updateLogs();
    }
}
