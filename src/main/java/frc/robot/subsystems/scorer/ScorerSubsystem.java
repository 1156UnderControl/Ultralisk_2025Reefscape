package frc.robot.subsystems.scorer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.Java_Is_UnderControl.Motors.TalonFXMotor;

public class ScorerSubsystem extends SubsystemBase implements IScorer {

  private final IMotor pivotMotor;
  private final IMotor scoreMotor;

  public ScorerSubsystem() {
    this.pivotMotor = new TalonFXMotor(0);
    this.scoreMotor = new SparkMAXMotor(0);
  }

  @Override
  public boolean hasCoral() {
    return false;
  }

  @Override
  public void intake(){}



  @Override
  public void intakeFromHP(){}


  @Override
  public void prepareToPlaceL1() {}


  @Override
  public void prepareToPlaceL2() {}

  @Override
  public void prepareToPlaceL3() {}

  @Override
  public void prepareToPlaceL4() {}

  @Override
  public void place() {}

  @Override
  public void expell() {}

  @Override
  public boolean isSecuredToPlace() {
    return false;
  }

  @Override
  public boolean hasPlaced() {
    return false;
  }

  @Override
  public void stopIntaking() {}

  @Override
  public boolean isAtSetPoint() {
    return false;
  }
}

