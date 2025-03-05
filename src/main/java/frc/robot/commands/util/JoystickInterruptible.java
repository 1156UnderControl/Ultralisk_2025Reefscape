package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.IDriverController;

public class JoystickInterruptible extends SequentialCommandGroup {

  IDriverController driverController = DriverController.getInstance();

  public JoystickInterruptible(Command mainCommand, double deadband) {
    // Create an InstantCommand that ends when the joystick is moved
    Command monitorJoystick = new RunCommand(() -> {
      // This can be expanded for finer control
      if (Math.abs(driverController.getXtranslation()) > deadband
          || Math.abs(driverController.getYtranslation()) > deadband
          || Math.abs(driverController.getCOS_Joystick()) > deadband
          || Math.abs(driverController.getSIN_Joystick()) > deadband) {
        // The joystick has moved; the command will end
        this.cancel();
      }
    }).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    // Combine the commands in a ParallelRaceGroup
    Command combined = new ParallelRaceGroup(
        mainCommand, // The main command
        monitorJoystick // Ends the group when joystick moves
    );
    // Add the combined command to this SequentialCommandGroup
    addCommands(combined);
  }
}
