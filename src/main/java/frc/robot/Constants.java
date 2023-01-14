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
  public static class OperatorConstants {
    //Joystick ports
    public static final int kLeftJoystickPort = 0;
    public static final int kRightJoystickPort = 1;

    //when joystickOutput is too low (-kDeadband <= joystickOutput <= kDeadband) 
    //to move the robot, set joystickOutput to 0 to avoid burning the motor
    public static final double kDeadband = 0.4;


  }

  public static class DriveConstants{
    //DriveMotor Device ID
    public static final int kfrontLeftMotorDeviceID = 2;
    public static final int krearLeftMotorDeviceID = 3;
    public static final int kfrontRightMotorDeviceID = 1;
    public static final int krearRightMotorDeviceID = 4;

    //conversion factor from tick to feet
    public static final double kDriveTick2Feet = 1/42 * (6 * Math.PI / 12) * 4.16;
    //reduce drive speed for stability
    public static final double kReduceDriveSpeedFactor = 0.75;


  }
}
