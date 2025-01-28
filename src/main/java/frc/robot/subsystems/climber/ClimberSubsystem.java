package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.Java_Is_UnderControl.Motors.TalonFXMotor;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase implements IClimber {
  private IMotor cageIntakeMotor = new SparkMAXMotor(ClimberConstants.ID_cageIntakeMotor, "CAGE_INTAKE");
  private IMotor climberArmMotor = new TalonFXMotor(ClimberConstants.ID_climberArmMotor, GravityTypeValue.Arm_Cosine,
      "CLIMBER_ARM");

  public ClimberSubsystem() {
  }

  public void climb () {

  private void activateRollers(double speed) {
    rollersMotor.set(speed);
  }

  private void moveArmsToPosition(double position) {
    climberArmMotor.set(position);
  }

  private void stopAll() {
    rollersMotor.stopMotor();
    ArmMotor.stopMotor();
  }

  climberArmMotor.setMotorBrake(true);climberArmMotor.configureMotionProfiling(ClimberConstants.tunning_values_climber.PID.P,ClimberConstants.tunning_values_climber.PID.I,ClimberConstants.tunning_values_climber.PID.D,ClimberConstants.tunning_values_climber.KS,ClimberConstants.tunning_values_climber.KV,ClimberConstants.tunning_values_climber.KA,ClimberConstants.tunning_values_climber.MAX_VELOCITY,ClimberConstants.tunning_values_climber.MAX_ACCELERATION,ClimberConstants.tunning_values_climber.JERK);cageIntakeMotor.burnFlash();

  }

  @Override
  public void isAtSetPoint() {
  }

  @Override
  public void climbDeep() {
  }

  @Override
  public void stop() {
  }

  @Override
  public void release() {
  }
