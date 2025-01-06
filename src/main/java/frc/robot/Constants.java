// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //can 1-13 are swerve
  public static final int elevatorMotorID    = 14;
  public static final int elevatorFollowerID = 15;
  public static final int elevatorCANcoderID = 16;
  public static final int pivotMotorID       = 17;
  public static final int clawMotorID        = 18;

  //DIO
  public static final int pivotEncoderID      = 0;
  public static final int frontlimitSwitchID  = 1;
  public static final int backlimitSwitchID   = 2;
  public static final int bottomlimitSwitchID = 3;
  public static final int toplimitSwitchID    = 4;
  
  public static final double clawSpeed = .75;
  public static final int maxManualArmSpeed = 1;

  //TODO limits
  public static final double upperElevatorPosLimit = 86;
  public static final double lowerElevatorPosLimit = -6;
  public static final double forwardPivotPosLimit = 0;
  public static final double backPivotPosLimit = 0;
  public static final int wristMotorID = 0;
public static final int wristEncoderID = 0;
public static final double backWristPosLimit = 0;
public static final double forwardWristPosLimit = 0;
}
