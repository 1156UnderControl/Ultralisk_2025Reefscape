package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;

public class ClimberSubsystem extends SubsystemBase implements IClimber {
  private IMotor alignerMotor;
  private IMotor climberMotor;

  public ClimberSubsystem() {
    this.alignerMotor = new SparkMAXMotor(0);
    this.climberMotor = new SparkMAXMotor(1);
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
}
