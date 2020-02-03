/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.
//Cian was here*/
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

import java.util.Vector;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Add your docs here.
 */
public class TankDrive extends Subsystem 
{
  private CANSparkMax leftFrontMotor = new CANSparkMax(RobotMap.FRONT_LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(RobotMap.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
  private CANSparkMax leftRearMotor = new CANSparkMax(RobotMap.REAR_LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rightRearMotor = new CANSparkMax(RobotMap.REAR_RIGHT_MOTOR, MotorType.kBrushless);
  private DifferentialDrive Drivetrain = new DifferentialDrive(leftFrontMotor,rightFrontMotor);
  double RFactor;
  double LFactor;

  @Override
  public void initDefaultCommand() 
  {
    // Set the default command for a subsystem here.
    setDefaultCommand(new Drive());
  }
  /*
  public void setdrivevector(Vector<Double> axis)
  {
    double axisX = (double) axis.elementAt(0);
    double axisY = (double) axis.elementAt(1);

    double leftPWR = 0;
    double rightPWR = 0;

    if(axisY <= 0.0)
    {
      if(axisX > 0.0)
      {
        leftPWR = Math.copySign(Math.sqrt(Math.pow(axisY, 2) + Math.pow(axisX, 2)),axisX);
        rightPWR = Math.copySign(Math.sqrt(Math.abs(Math.pow(axisY, 2) - Math.pow(axisX, 2))),(Math.abs(axisY)-axisX));
      }
      else
      {
        leftPWR = Math.copySign(Math.sqrt(Math.pow(axisY, 2) + Math.pow(axisX, 2)),Math.abs(axisY)-Math.abs(axisX));
        rightPWR = Math.sqrt(Math.pow(axisY, 2) + Math.pow(Math.abs(axisX),2));
      }
    }
    else
    {
      if(axisX >= 0.0)
      {
        leftPWR = Math.copySign(Math.sqrt(Math.pow(Math.abs(axisY),2) + Math.pow(axisX, 2)),axisX-axisY);
        rightPWR = Math.copySign(Math.sqrt(Math.pow(Math.abs(axisY),2) + Math.pow(axisX, 2)),-1);
      }
      else
      {
        leftPWR = Math.copySign(Math.sqrt(Math.pow(Math.abs(axisY),2) + Math.pow(axisX, 2)),axisX);
        rightPWR = Math.copySign(Math.sqrt(Math.pow(Math.abs(axisY),2) - Math.pow(axisX, 2)),(Math.abs(axisY)-axisX));
      }
    }
  
    LFactor = leftPWR * RobotMap.SCALEFACTOR; 
    RFactor = rightPWR  * RobotMap.SCALEFACTOR;

    Drivetrain.tankDrive(-LFactor, -RFactor, false);
  }
  */
  public void directpwrfeed(double L, double R)
  {
    Drivetrain.tankDrive(-L, -R, false);
  }

  public String print()
  {
    int cpr2 = 0;
    int cpr4 = 0;
    
    CANEncoder LFM = leftFrontMotor.getEncoder(EncoderType.kHallSensor, cpr2);
    CANEncoder RFM = rightFrontMotor.getEncoder(EncoderType.kHallSensor, cpr4);
    String encVal = "(" + 
    String.format("%.2f",LFM.getVelocity()) +"  "+ String.format("%.2f",leftFrontMotor.getAppliedOutput()) +"\t"+
    String.format("%.2f",RFM.getVelocity()) +"  "+ String.format("%.2f",rightFrontMotor.getAppliedOutput()) +"\t";
    SmartDashboard.putNumber("left front", LFM.getVelocity());
    SmartDashboard.putNumber("right front", RFM.getVelocity());
    if(RFactor > 0.005 || LFactor > 0.005 || RFactor < 0 || LFactor < 0) {
      System.out.print("RFactor: " + String.format("%.3f",RFactor) + "  ");
      System.out.println("LFactor: " + String.format("%.3f",LFactor));
    }
    return encVal;
  }

  public void setfollow()
  {
    leftRearMotor.follow(leftFrontMotor);
    rightRearMotor.follow(rightFrontMotor);
  }
}
