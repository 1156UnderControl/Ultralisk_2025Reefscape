package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.Java_Is_UnderControl.Motors.SparkFlexMotor;

public class ArmSubsystem extends SubsystemBase implements IArm{

    private static ArmSubsystem armInstance = null;
    private SparkFlexMotor motorArm1;
    private ArmFeedforward ffArm = new ArmFeedforward(0.17187, 0.10741, 0.8275, 0.050913);
    private double positionTarget = 100;
    private double velocityTarget = 20;

    public static ArmSubsystem getInstance() {
        if (armInstance == null) {
          armInstance = new ArmSubsystem();
        }
        return armInstance;
    }

    private ArmSubsystem(){
        motorArm1 = new SparkFlexMotor(2, "Motor Arm");
        motorArm1.setPositionFactor(0.0126984126984127 * 360);
        motorArm1.setVelocityFactor(0.00211640211);
        motorArm1.configureMaxMagic(2.2667, 0.0, 1.4979, ffArm.calculate(Units.degreesToRadians(this.positionTarget), this.velocityTarget), 20, 20, 0.01);
        motorArm1.setMotorBrake(true);
        motorArm1.burnFlash();
    }

    @Override
    public void go100Degrees(){
        motorArm1.setPositionMaxMagic(this.positionTarget);
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
