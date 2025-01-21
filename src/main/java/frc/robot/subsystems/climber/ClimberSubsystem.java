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
    throw new UnsupportedOperationException("Unimplemented method 'isAtSetPoint'");
  }

  @Override
  public void climbDeep() {
    throw new UnsupportedOperationException("Unimplemented method 'climbDeep'");
  }

  @Override
  public void stop() {
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }

  @Override
  public void release() {
    throw new UnsupportedOperationException("Unimplemented method 'release'");
  }
}
