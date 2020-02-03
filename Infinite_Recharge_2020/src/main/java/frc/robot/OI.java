/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Vector;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.Lockon;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI 
{
  Joystick F310 = new Joystick(RobotMap.DRIVER_CONTROLLER);
   private Button buttonA = new JoystickButton(F310,2);
  
  public Vector<Double> GetControllerRawAxis()
  {
    Vector<Double> axis = new Vector<Double>();
    axis.add(F310.getX());
    axis.add(F310.getY());
    axis.add(F310.getZ());
    return axis;
  }
  public void OI()
  {
    buttonA.whileHeld(new Lockon());
  }
}
