/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * FN-1. normal mode : sensor P0 turn on all till sensor P1 , then turn off upper conveyer. then at
 * sensor P2 all conveyer off and reverse infeed for 3 seconds.
 * FN-2. Firing mode : spins up shooter to rpm for distance , then will run all conveyers feeding in balls once solenoid is out of the way
 * FN-3. Halt mode: instantly stops all conveyers and runs infeed in revers for 3 seconds
 */
public class Shooter extends Subsystem 
{ 
  //launcher
  private final TalonFX shooterUpperA = new TalonFX(5);
  private final TalonFX shooterUpperB = new TalonFX(6);
  private final TalonFX shooterLowerA = new TalonFX(7);
  private final TalonFX shooterLowerB = new TalonFX(8);
  //TalonFX config
  TalonFXConfiguration upper_cfg = new TalonFXConfiguration();
  TalonFXConfiguration lower_cfg = new TalonFXConfiguration();
  //conveyer
  private final TalonSRX lowerConv = new TalonSRX(9);
  private final TalonSRX uppperConv = new TalonSRX(10);
  private final TalonSRX mainConv = new TalonSRX(11);
  private final TalonSRX infeed = new TalonSRX(12); 


  public void setfollow()
  {
    //follow controll
    shooterUpperA.follow(shooterUpperB);
    shooterLowerA.follow(shooterLowerB);
    //ramp control
    shooterUpperA.configOpenloopRamp(10);
    shooterUpperB.configOpenloopRamp(10);
    shooterLowerA.configOpenloopRamp(10);
    shooterLowerB.configOpenloopRamp(10);
    //pid pre-settings
    shooterUpperA.configClosedLoopPeriod(0, 1, RobotMap.timeout);
    shooterLowerA.configClosedLoopPeriod(0, 1, RobotMap.timeout);
    upper_cfg.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    lower_cfg.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    //upper
    upper_cfg.slot0.kF = RobotMap.kF;
    upper_cfg.slot0.kP = RobotMap.kP;
    upper_cfg.slot0.kI = RobotMap.kI;
    upper_cfg.slot0.kD = RobotMap.kD;
    upper_cfg.slot0.integralZone = RobotMap.kIzone;
    upper_cfg.slot0.closedLoopPeakOutput = RobotMap.kPeakOutput;
    upper_cfg.neutralDeadband = RobotMap.Deadband;
    upper_cfg.motionAcceleration = 2000;
    upper_cfg.motionCruiseVelocity = 2000;
    //lower
    lower_cfg.slot0.kF = RobotMap.kF;
    lower_cfg.slot0.kP = RobotMap.kP;
    lower_cfg.slot0.kI = RobotMap.kI;
    lower_cfg.slot0.kD = RobotMap.kD;
    lower_cfg.slot0.integralZone = RobotMap.kIzone;
    lower_cfg.slot0.closedLoopPeakOutput = RobotMap.kPeakOutput;
    lower_cfg.neutralDeadband = RobotMap.Deadband;
    lower_cfg.motionAcceleration = 2000;
    lower_cfg.motionCruiseVelocity = 2000;
    //apply
    shooterUpperA.configAllSettings(upper_cfg);
    shooterLowerA.configAllSettings(lower_cfg);
  }
  public double getSpeed()
  {
    return shooterUpperA.getMotorOutputPercent();
  }
  public double 
  //set functions
  public void SetShooterPower(double pwr)
  {
    shooterUpperA.set(TalonFXControlMode.PercentOutput, pwr);
    shooterLowerA.set(TalonFXControlMode.PercentOutput, pwr);
    
  }
  public void SetIntakePower(double pwr)
  {

  }
  public void SetLoaderPower(double pwr)
  {

  }
  public void SetUpperPower(double pwr)
  {

  }
  public void SetLowerPower(double pwr)
  {

  }
  @Override
  public void initDefaultCommand() 
  {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  //for debugging ======================================================================
  public void print()
  {
    System.out.println("upper vel = " + shooterUpperA.getSelectedSensorVelocity());
    System.out.println("lower vel = " + shooterLowerA.getSelectedSensorVelocity());
  }

}
