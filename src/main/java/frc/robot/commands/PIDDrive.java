// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDDrive extends PIDCommand {
  /** Creates a new PIDDrive. */
DriveTrain _driveTrain;

  public PIDDrive(DriveTrain dt, double dis) {

    super(
        // The controller that the command will use
        new PIDController(.5, .3, .3),
        // This should return the measurement
        dt::getPos,
        // This should return the setpoint (can also be a constant)
        dis,
        // This uses the output
        output -> {
          dt.tankDrive(0.6*output, 0.6*output);
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    _driveTrain = dt;
    addRequirements(dt);
    getController().setTolerance(0.05);
  }

  @Override
  public void initialize() {
    _driveTrain.resetEncoders();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
