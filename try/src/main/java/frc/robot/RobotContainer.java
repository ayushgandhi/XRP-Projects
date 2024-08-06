// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPOnBoardIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.PrintCommand;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.lang.Math;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final XRPOnBoardIO m_onboardIO = new XRPOnBoardIO();
  private final Arm m_arm = new Arm();

  // Assumes a gamepad plugged into channel 0
  private final Joystick m_controller = new Joystick(1);
  private final PS4Controller ex1=new PS4Controller(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Example of how to use the onboard IO
    Trigger userButton = new Trigger(m_onboardIO::getUserButtonPressed);
    userButton
        //.onTrue(new PrintCommand("USER Button Pressed"))
        .onTrue(new InstantCommand(()->m_onboardIO.setLed(true)))
        .onFalse(new InstantCommand(()->m_onboardIO.setLed(false)));
        //.onFalse(new PrintCommand("USER Button Released"));
    //double a1=ex1.getLeftX();
    //double a2=ex1.getLeftY();
    
        //boolean b1=ex1.getSquareButton();
    //boolean b2=ex1.getCrossButton();
    //JoystickButton exBut1 = new JoystickButton(ex1, 1);
    //exBut1
    //.onTrue(new InstantCommand(()->m_arm.setAngleOne(45.0),m_arm))
    //.onFalse(new InstantCommand(()->m_arm.setAngleOne(0),m_arm));

   // JoystickButton joystickAButton = new JoystickButton(m_controller, 1);
    //joystickAButton
      //  .onTrue(new InstantCommand(() -> m_arm.setAngle(45.0), m_arm))
       // .onFalse(new InstantCommand(() -> m_arm.setAngle(0.0), m_arm));

   // JoystickButton joystickBButton = new JoystickButton(m_controller, 2);
    //joystickBButton
      //  .onTrue(new InstantCommand(() -> m_arm.setAngle(90.0), m_arm))
       // .onFalse(new InstantCommand(() -> m_arm.setAngle(0.0), m_arm));

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));
    SmartDashboard.putData(m_chooser);
  }
  public void servoAngleAG(){
    double val1=m_drivetrain.getGyroAngleX();
    double val2=m_drivetrain.getGyroAngleY();
    m_arm.setAngleOne(val1);
    m_arm.setAngleTwo(val2);
  }

  public void PS4commandAG(){
    boolean b1= ex1.getCrossButtonPressed();
    boolean b2=ex1.getSquareButtonPressed();
    double a1=ex1.getLeftX();
    double a2=ex1.getLeftY();
    if(b1){
      m_arm.setAngleOne(45);
    }
    else{
      m_arm.setAngleOne(0);
    }
    if(b2){
      m_arm.setAngleTwo(45);
    }
    else{
      m_arm.setAngleTwo(0);
    }
    m_arm.setAngleOne(Math.abs(a1)*90);
    m_arm.setAngleTwo(Math.abs(a2)*90);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -m_controller.getRawAxis(1), () -> -m_controller.getRawAxis(2));
  }
}
