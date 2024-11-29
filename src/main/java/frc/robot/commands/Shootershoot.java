// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class Shootershoot extends Command {
  /** Creates a new Shootershoot. */
  XboxController xbox;
  Shooter shooterSub;
  DoubleSupplier speed;
  public Shootershoot(Shooter s, DoubleSupplier d, XboxController x) {
    this.speed = d;
    this.shooterSub = s; 
    this.xbox = x;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
    addRequirements(s);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speedDouble = speed.getAsDouble();
    shooterSub.setMotorSpeed(speedDouble);
    if(xbox.getAButtonPressed()){
      shooter.turn30d;
    }

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
