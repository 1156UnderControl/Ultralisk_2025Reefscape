package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.Java_Is_UnderControl.Motors.TalonFXMotor;
import frc.robot.constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase implements IClimber {
  private static ClimberSubsystem instance;
  private IMotor cageIntakeMotor = new SparkMAXMotor(ClimberConstants.ID_cageIntakeMotor, "CAGE_INTAKE");
  private IMotor climberArmMotor = new TalonFXMotor(ClimberConstants.ID_climberArmMotor, GravityTypeValue.Arm_Cosine,
      "CLIMBER_ARM");

  private int Previousvelocity = 0;
  private boolean isCageCollected;

  public static ClimberSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimberSubsystem();
    }
    return instance;
  }

  private ClimberSubsystem() {
    climberArmMotor.setMotorBrake(true);
    climberArmMotor.configureMotionProfiling(
        ClimberConstants.tunning_values_climber.PID.P,
        ClimberConstants.tunning_values_climber.PID.I,
        ClimberConstants.tunning_values_climber.PID.D,
        ClimberConstants.tunning_values_climber.KS,
        ClimberConstants.tunning_values_climber.KV,
        ClimberConstants.tunning_values_climber.KA,
        ClimberConstants.tunning_values_climber.MAX_VELOCITY,
        ClimberConstants.tunning_values_climber.MAX_ACCELERATION,
        ClimberConstants.tunning_values_climber.JERK);
    cageIntakeMotor.burnFlash();
  }

  public void climb() {

  }

  private void activateRollers(double speed) {
    cageIntakeMotor.set(speed);
  }

  private void moveArmsToPosition(double position) {
    climberArmMotor.set(position);
  }

  public void periodic() {
    SmartDashboard.putData("Subsystem Climber", ClimberSubsystem.getInstance());
  }

  @Override
  public void isAtSetPoint() {
  }

  @Override
  public void climbDeep() {
  }

  @Override
  public void release() {
  }

  @Override
  public void stop() {
    this.cageIntakeMotor.set(0);
    this.climberArmMotor.set(0);

  }

  public void detectcagecollect() {
    if (cageIntakeMotor.getVelocity() < Previousvelocity - 50) {
      isCageCollected = true;
    } else {
      isCageCollected = false;
    }
  }
}
