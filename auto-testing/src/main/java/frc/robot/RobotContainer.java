// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final Drivetrain s_drive;
  private final Ramsete ramsete;
  private final AutoCommandSelector autoSelector;

  // controllers
  XboxController driveController;

  private SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_drive = new Drivetrain();
    ramsete = new Ramsete(s_drive);
    autoSelector = new AutoCommandSelector(s_drive, ramsete);

    driveController = new XboxController(Constants.OI.DRIVE_CONTROLLER_ID);
    
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("test", autoSelector.test);
    autoChooser.addOption("forward", autoSelector.forward);
    autoChooser.addOption("circle", autoSelector.circle);
    autoChooser.addOption("b8, 2m, 3t, goal start", autoSelector.b8_m2_t3_startG);
    autoChooser.addOption("b8, 2m, 3t, center start", autoSelector.b8_m2_t3_startC);
    autoChooser.addOption("b8, 2m, 3t, depot start", autoSelector.b8_m2_t3_startD);
    
    autoSelector.setInitialDrivePose(autoChooser.getSelected());;

    SmartDashboard.putString("pose", s_drive.getPose().toString());

    s_drive.setDefaultCommand(new RunCommand(() -> s_drive.closedCurveDrive(
      OI.getTriggers(driveController),
      OI.getLeftJoystickAxis(driveController),
      driveController.getXButtonPressed()),
      s_drive));

    // Configure the button bindings
    configureButtonBindings();
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return autoSelector.test;
    return autoChooser.getSelected();
  }
}
