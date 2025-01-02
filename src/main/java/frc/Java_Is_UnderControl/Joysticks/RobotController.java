package frc.Java_Is_UnderControl.Joysticks;

import frc.Java_Is_UnderControl.Joysticks.Types.IController;

public class RobotController {

    public IController typeController;
  
    public RobotController(IController typeController, int controllerID) {
      this.typeController = typeController;
    }
  }
