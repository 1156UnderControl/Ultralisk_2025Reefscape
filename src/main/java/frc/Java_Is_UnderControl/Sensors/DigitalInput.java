package frc.Java_Is_UnderControl.Sensors;

import java.util.function.BooleanSupplier;

public interface DigitalInput extends BooleanSupplier {
  boolean getBoolean();

  int getBinary();
}
