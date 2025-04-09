package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.robot.SuperStructure;
import frc.robot.subsystems.leds.LedSubsystem;

public class LedStateUpdater extends RunCommand {
  public LedStateUpdater(SuperStructure superStructure) {
    super(() -> {
      if (superStructure.scorer.hasAlgae()) {
        superStructure.led.setSolidColor(LedColor.BLUE);
      } else if (superStructure.scorer.hasCoral()) {
        superStructure.led.setSolidColor(LedColor.GREEN);
      } else {
        superStructure.led.turnOff();
      }
    }, LedSubsystem.getInstance());
  }
}
