// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class MecanumDriveCmd extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private final DoubleSupplier m_joyLeftYOutput, m_joyLeftXOutput, m_joyRightZOutput;

  /** Creates a new MecanumDriveCmd. */
  public MecanumDriveCmd(DriveSubsystem subsystem, DoubleSupplier joyLeftYOutput, DoubleSupplier joyLeftXOutput, DoubleSupplier joyRightZOutput) {
    m_subsystem = subsystem;
    m_joyLeftYOutput = joyLeftYOutput;
    m_joyLeftXOutput = joyLeftXOutput;
    m_joyRightZOutput = joyRightZOutput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double frontLeftMotorPower, rearLeftMotorPower, frontRightMotorPower, rearRightMotorPower;
    double max = 0.0;

    /* Ref: https://github.com/gamemanual0/gm0/blob/main/source/docs/software/images/mecanum-drive/mecanum-drive-directions.png 
      FL=FrontLeft, RL=Rear Left, FR=Front Right, RR=Rear Right
              ↑                                →                                     ↷
        FL ┌─────┐FR                     FL ┌─────┐FR                          FL ┌─────┐FR
      ↑ ┌─┐│     │┌─┐↑                 ↑ ┌─┐│     │┌─┐↓                      ↑ ┌─┐│     │┌─┐↓
        └─┘│     │└─┘                    └─┘│     │└─┘                         └─┘│     │└─┘
      ↑ ┌─┐│     │┌─┐↑                 ↓ ┌─┐│     │┌─┐↑                      ↑ ┌─┐│     │┌─┐↓
        └─┘│     │└─┘                    └─┘│     │└─┘                         └─┘│     │└─┘
        RL └─────┘RR                     RL └─────┘RR                          RL └─────┘RR

        y conponent controls Fwd/Bwd   x component controls Straf R/L    z controls Rotate clockwise/counterclockwise
        Fwd:all wheels forward (+y)    Straf R: LF,RB: +x, RF,LB: -x     clocwise: LF,LB: +z, RF,RB: -z
        FL.set(y+ + )                  FL.set(y +x +)                    FL.set(y +x +z)
        RL.set(y+ + )                  RL.set(y -x + )                   RL.set(y -x +z )
        FR.set(y+ + )                  FR.set(y -x + )                   FR.set(y -x -z )
        RR.set(y+ + )                  RR.set(y +x + )                   RR.set(y +x -z )

*/
    frontLeftMotorPower = m_joyLeftYOutput.getAsDouble() + m_joyLeftXOutput.getAsDouble() + m_joyRightZOutput.getAsDouble();
    rearLeftMotorPower  = m_joyLeftYOutput.getAsDouble() - m_joyLeftXOutput.getAsDouble() + m_joyRightZOutput.getAsDouble();
    frontRightMotorPower= m_joyLeftYOutput.getAsDouble() - m_joyLeftXOutput.getAsDouble() - m_joyRightZOutput.getAsDouble();
    rearRightMotorPower = m_joyLeftYOutput.getAsDouble() + m_joyLeftXOutput.getAsDouble() - m_joyRightZOutput.getAsDouble();

    //the power applies to each wheel could be > 1 or <-1
    //If the power is >1, it will be set to 1 by the set() function, if it is < -1, it will be set to -1
    //For ex:              if FL power = 0.4, RL=0.1, FR=1.1, and RR=1.4
    //it will be round off to FL power = 0.4, RL=0.1, FR=1.0, and RR=1.0
    //This round off will cause unstability in the robot
    //Instead, we will devide the power of each wheel with the largest of x,y,z if the abs(power) of any wheel > 1 

    if (Math.abs(frontLeftMotorPower) > 1 || Math.abs(rearLeftMotorPower) > 1 ||
        Math.abs(frontRightMotorPower) > 1 || Math.abs(rearRightMotorPower) > 1){

      //find the largest power
      max = Math.max(Math.abs(frontLeftMotorPower), Math.abs(rearLeftMotorPower));
      max = Math.max(Math.abs(frontRightMotorPower), max);
      max = Math.max(Math.abs(rearRightMotorPower), max);

      //Divide everything by max
      frontLeftMotorPower /= max;
      rearLeftMotorPower /= max;
      frontRightMotorPower /= max;
      rearRightMotorPower /= max;
    }

    //set power to each wheel
    m_subsystem.setMotors(frontLeftMotorPower, rearLeftMotorPower, frontRightMotorPower, rearRightMotorPower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotors(0.0, 0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
