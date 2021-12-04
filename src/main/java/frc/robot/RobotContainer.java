// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.PIDDrive;
import frc.robot.commands.PIDTurn;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private String gameData = DriverStation.getInstance().getGameSpecificMessage();
  private final DriveTrain _driveTrain;
  private final Joystick _leftJoystick;
  private final Joystick _rightJoystick;
  private final TankDrive _tankDrive;
  private final PIDDrive _pidDrive;
  private final PIDTurn _pidTurn;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    _driveTrain = new DriveTrain();
    _leftJoystick = new Joystick(Constants.USBOrder.Zero);
    _rightJoystick = new Joystick(Constants.USBOrder.One);
    _tankDrive = new TankDrive(_driveTrain, _leftJoystick, _rightJoystick);
    _pidDrive = new PIDDrive(_driveTrain, 5);
    _pidTurn = new PIDTurn(_driveTrain, 90.);

    _driveTrain.setDefaultCommand(_tankDrive);
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
    if(gameData.length() > 0){
      switch(gameData.substring(0,1)){
        case "R":
        return new SequentialCommandGroup(_pidDrive, _pidTurn, _pidDrive, _pidTurn, _pidDrive, _pidTurn, _pidDrive, _pidTurn);
        case "Y":
        return new SequentialCommandGroup(_pidDrive, _pidTurn, _pidDrive);
        case "B":
        return new SequentialCommandGroup( new PIDDrive(_driveTrain, .5), new PIDTurn(_driveTrain, -90.), new PIDDrive(_driveTrain, -.5));
      }
    }
    
    return _pidDrive;
  }
}
