/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Fire extends Command 
{
  

  public Fire() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.shooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    if(Robot.shooter.getSpeed() == 0)
    {
      Robot.shooter.SetShooterPower(-1.0);
    }
    else
    {
      Robot.shooter.SetShooterPower(0.0);
    }

    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
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
