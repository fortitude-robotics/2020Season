/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap 
{
  public final static int DRIVER_CONTROLLER = 0;

  public static double SCALEFACTOR = 0.4;

  public final static int FRONT_LEFT_MOTOR = 1;
  public final static int REAR_LEFT_MOTOR = 2;
  public final static int FRONT_RIGHT_MOTOR = 3;
  public final static int REAR_RIGHT_MOTOR = 4;
  //rpm table
  public final static double[][] SHOOTER_RPM = 
  {{240,228,216,204,192,180,168,156,144,132,120},
  {6000,5700,5400,5100,4800,4500,4200,3900,3600,3300,3000}};
  //motion magic constants
  public final static int timeout = 30;
  public final static Double kP = 0.0;
  public final static Double kI = 0.0;
  public final static Double kD = 0.0;
  public final static Double kF = 0.0;
  public final static int kIzone = 0;
  public final static Double kPeakOutput = 1.0;
  public final static Double Deadband = 0.001;


}
