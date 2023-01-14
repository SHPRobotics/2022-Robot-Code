// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TankDriveCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private final DoubleSupplier m_joyLeftOutput, m_joyRightOutput;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankDriveCmd(DriveSubsystem subsystem, DoubleSupplier joyLeftOutput, DoubleSupplier joyRightOutput) {
    m_subsystem = subsystem;
    m_joyLeftOutput = joyLeftOutput;
    m_joyRightOutput = joyRightOutput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double realtimeJoyLeftOutput = m_joyLeftOutput.getAsDouble() * Constants.DriveConstants.kReduceDriveSpeedFactor;
    double realtimeJoyRightOutput = m_joyRightOutput.getAsDouble() * Constants.DriveConstants.kReduceDriveSpeedFactor;

    //apply power to driveSubsystem
    m_subsystem.setMotors(realtimeJoyLeftOutput, realtimeJoyRightOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotors(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
