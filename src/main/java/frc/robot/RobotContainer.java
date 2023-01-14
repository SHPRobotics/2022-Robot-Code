// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MecanumDriveCmd;
//import frc.robot.commands.TankDriveCmd;
//import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
/* 
  private final CommandJoystick m_leftJoystick =
      new CommandJoystick(OperatorConstants.kLeftJoystickPort);
  private final CommandJoystick m_rightJoystick =
      new CommandJoystick(OperatorConstants.kRightJoystickPort);
*/
  private final Joystick m_leftJoystick = new Joystick(Constants.OperatorConstants.kLeftJoystickPort);
  private final Joystick m_rightJoystick = new Joystick(Constants.OperatorConstants.kRightJoystickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Default command of DriveSubsystem to TankDrive
    // m_DriveSubsystem.setDefaultCommand(new TankDriveCmd(m_DriveSubsystem, ()-> -m_leftJoystick.getY(), ()-> -m_rightJoystick.getY()));

    //Default command of DriveSubsystem to ArcadeDrive
    // m_DriveSubsystem.setDefaultCommand(new ArcadeDriveCmd(m_DriveSubsystem, ()-> -m_leftJoystick.getY(), ()-> m_leftJoystick.getX()));

    //Default command of DriveSubsystem to MecanumDrive
    configureButtonBindings();
    m_DriveSubsystem.setDefaultCommand(new MecanumDriveCmd(m_DriveSubsystem, ()-> -m_leftJoystick.getY(), ()-> m_leftJoystick.getX(), ()-> m_rightJoystick.getZ()));

    // Configure the trigger bindings
    //configureBindings();
  }

  private void configureButtonBindings(){
    new JoystickButton(m_leftJoystick, 1).whileTrue(new MecanumDriveCmd(m_DriveSubsystem, ()-> -m_leftJoystick.getY(), null, null));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
/* 
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_DriveSubsystem::exampleCondition)
        .onTrue(new TankDriveCmd(m_DriveSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_leftJoystick.b().whileTrue(m_DriveSubsystem.exampleMethodCommand());
*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_DriveSubsystem);
    return null;
  }
}
