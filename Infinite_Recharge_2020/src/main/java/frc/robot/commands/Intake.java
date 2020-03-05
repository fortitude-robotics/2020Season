/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Intake extends Command 
{
  int buttonID = 0;
  boolean btnstat;
  public Intake(int btnID, Boolean BtnStatus) 
  {
    buttonID = btnID;
    btnstat = BtnStatus;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.shooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    
    if(Robot.shooter.getconv() == 0)
    {
      if(buttonID == RobotMap.BUTTON_RBUMP)
      {
      Robot.shooter.SetLowerPower(0.3);
      Robot.shooter.SetIntakePower(-0.3);
      Robot.shooter.SetMainPower(0.0);
      Robot.shooter.SetUpperPower(0.0);
      }
      if(buttonID == RobotMap.BUTTON_LBUMP)
      {
      Robot.shooter.SetLowerPower(-0.3);
      Robot.shooter.SetIntakePower(0.3);
      Robot.shooter.SetMainPower(-0.5);
      Robot.shooter.SetUpperPower(-0.3);
      }
      if(buttonID == RobotMap.BUTTON_B)
      {
      Robot.shooter.SetMainPower(0.5);
      Robot.shooter.SetLowerPower(0.3);
      Robot.shooter.SetUpperPower(0.3);
      Robot.shooter.SetIntakePower(0.0);
      }
      
    }
    else //if(Robot.shooter.GetTOFdistance() > 54 || Robot.shooter.getconv() != 0)
    {
      Robot.shooter.SetLowerPower(0.0);
      Robot.shooter.SetIntakePower(0.0);
      Robot.shooter.SetUpperPower(0.0);
      Robot.shooter.SetMainPower(0.0);
    }
    
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() 
  {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
