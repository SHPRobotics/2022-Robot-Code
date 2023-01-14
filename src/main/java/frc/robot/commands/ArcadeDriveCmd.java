// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCmd extends CommandBase {
  DriveSubsystem m_subsystem;
  DoubleSupplier m_speed;
  DoubleSupplier m_turn;

  /** Creates a new ArcadeDriveCmd. */
    public ArcadeDriveCmd(DriveSubsystem subsystem, DoubleSupplier joyLeftYOutput, DoubleSupplier joyLeftXOutput) {
      m_subsystem = subsystem;
      m_speed = joyLeftYOutput;
      m_turn = joyLeftXOutput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double realtimeSpeed = m_speed.getAsDouble() * Constants.DriveConstants.kReduceDriveSpeedFactor;
    double realtimeTurn = m_turn.getAsDouble() * Constants.DriveConstants.kReduceDriveSpeedFactor;

    //apply power to driveSubsystem
    m_subsystem.setMotors(realtimeSpeed + realtimeTurn, realtimeSpeed - realtimeTurn);
     
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
