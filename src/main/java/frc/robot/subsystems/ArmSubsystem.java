package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;

public class ArmSubsystem extends SubsystemBase implements IArm{

    private static ArmSubsystem armInstance = null;
    private SparkMAXMotor motorArm1;
    private SparkMAXMotor motorArm2;
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    public static ArmSubsystem getInstance() {
        if (armInstance == null) {
          armInstance = new ArmSubsystem();
        }
        return armInstance;
    }

    private ArmSubsystem(){
        motorArm1 = new SparkMAXMotor(0);
        motorArm2 = new SparkMAXMotor(1);
        motorArm1.setPositionFactor(360);
        motorArm2.setPositionFactor(360);
    }

    @Override
    public void go180Degrees(){}

    @Override
    public boolean isAtSetPoint(){
        if(motorArm1.getPosition() == 180 && motorArm2.getPosition() == 180){
            return true;
        } else {
            return false;
        }
    }

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            voltage -> {
                motorArm1.set(voltage);;
                motorArm2.set(voltage);
            },
            log -> {
                log.motor("arm_motor1")
                    .voltage(m_appliedVoltage.mut_replace(motorArm1.getSetPointVelocity() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(motorArm1.getPosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(motorArm1.getVelocity(), MetersPerSecond));
                log.motor("arm_motor2")
                    .voltage(m_appliedVoltage.mut_replace(motorArm2.getSetPointVelocity() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(motorArm2.getPosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(motorArm2.getVelocity(), MetersPerSecond));
            },
            this
        )    
    );  
}
