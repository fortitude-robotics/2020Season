/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
//import javax.lang.model.util.ElementScanner6;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Lockon extends Command 
{
  double x,y,leftpwr,rightpwr;
  double Scale = 20;
  public Lockon() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.limelight2);
    requires(Robot.drivetrain);
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
    x = Robot.limelight2.getX();
    y = Robot.limelight2.getY();
    if( x > 0.8 || x < -0.8 )
    {
      if(x > -0.8)
      {
        leftpwr = 0.08;
        rightpwr =  -0.08;
        Robot.drivetrain.directpwrfeed(leftpwr, rightpwr);
      }
      if (x < 0.8)
      {
        leftpwr = -0.08;
        rightpwr =  0.08;
        Robot.drivetrain.directpwrfeed(leftpwr, rightpwr);
      }
    }
    else if( y > 0.8 || y < -0.8)
    {
      if(y > -0.8)
      {
        leftpwr = -0.1;
        rightpwr =  -0.1;
        Robot.drivetrain.directpwrfeed(leftpwr, rightpwr);
      }
      if (y < 0.8)
      {
        leftpwr = 0.1;
        rightpwr =  0.1;
        Robot.drivetrain.directpwrfeed(leftpwr, rightpwr);
      }
    }
    else
    {
      Robot.drivetrain.directpwrfeed(0,0);
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
