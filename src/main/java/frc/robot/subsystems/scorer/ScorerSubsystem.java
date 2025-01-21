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
    throw new UnsupportedOperationException("Unimplemented method 'hasCoral'");
  }

  @Override
  public void intake() {
    throw new UnsupportedOperationException("Unimplemented method 'intake'");
  }

  @Override
  public void intakeFromHP() {
    throw new UnsupportedOperationException("Unimplemented method 'intakeFromHP'");
  }

  @Override
  public void prepareToPlaceL1() {
    throw new UnsupportedOperationException("Unimplemented method 'prepareToPlaceL1'");
  }

  @Override
  public void prepareToPlaceL2() {
    throw new UnsupportedOperationException("Unimplemented method 'prepareToPlaceL2'");
  }

  @Override
  public void prepareToPlaceL3() {
    throw new UnsupportedOperationException("Unimplemented method 'prepareToPlaceL3'");
  }

  @Override
  public void prepareToPlaceL4() {
    throw new UnsupportedOperationException("Unimplemented method 'prepareToPlaceL4'");
  }

  @Override
  public void place() {
    throw new UnsupportedOperationException("Unimplemented method 'place'");
  }

  @Override
  public void expell() {
    throw new UnsupportedOperationException("Unimplemented method 'expell'");
  }

  @Override
  public boolean isSecuredToPlace() {
    throw new UnsupportedOperationException("Unimplemented method 'isSecuredToPlace'");
  }

  @Override
  public boolean hasPlaced() {
    throw new UnsupportedOperationException("Unimplemented method 'hasPlaced'");
  }

  @Override
  public void stopIntaking() {
    throw new UnsupportedOperationException("Unimplemented method 'stopIntaking'");
  }

  @Override
  public boolean isAtSetPoint() {
    throw new UnsupportedOperationException("Unimplemented method 'isAtSetPoint'");
  }
}
