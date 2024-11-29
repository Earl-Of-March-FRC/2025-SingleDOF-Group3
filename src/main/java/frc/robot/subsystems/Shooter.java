// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {

  private final TalonSRX shooterMotor = new TalonSRX(ShooterConstants.motorPort)

 // private final WPI_VictorSPX shooterMotor = new WPI_VictorSPX(ShooterConstants.motorPort);
  /** Creates a new Shooter. */
  public Shooter() {
     shooterMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    
  }

  /**
   * @param speed
   */
  public void setMotorSpeed(double speed){
    //Maybe move to robot container
    MathUtil.clamp(speed, -ShooterConstants.maxMotorSpeed, ShooterConstants.maxMotorSpeed);
    shooterMotor.set(ControlMode.Velocity,speed);

  }
}
