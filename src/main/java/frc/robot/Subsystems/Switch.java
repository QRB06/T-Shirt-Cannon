// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenixpro.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;



public class Switch extends SubsystemBase {

  public final CANSparkMax motor = new CANSparkMax(8, MotorType.kBrushless);
  public final CANcoder encoder = new CANcoder(20);
  public final PIDController pid = new PIDController(0.3, 0.01, 0);
  private double setPoint = 0;
  
  // Constants
  private static final int BOTTOM_ENCODER_VALUE = 0;
  private static final int TOP_ENCODER_VALUE = 1;
  private static final double MAX_POWER = .2;

  public Switch(){
    this.pid.setTolerance(0.1, 0.07/20);
    this.setSetpoint(this.getEncoderValue());
  }
  public void Switch_On(Double power) {
    motor.set(power);
  }
  private double getEncoderValue() {
    return (this.encoder.getPosition().getValue() - BOTTOM_ENCODER_VALUE) / (TOP_ENCODER_VALUE - BOTTOM_ENCODER_VALUE);
  }
  private double getPidPower() {
    double power = this.pid.calculate(this.getEncoderValue());
    return Math.min(MAX_POWER, Math.max(-MAX_POWER, power));
  }

  /**
   * Set the current setpoint for where the elevator should move to
   * @param setpoint Value between 0 and 1
   */
  public void setSetpoint(double setpoint) {
    this.setPoint = setpoint;
    this.pid.setSetpoint(setpoint);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double Vdegrees = Vcoder.getPosition();
    SmartDashboard.putNumber("Switch Encoders", encoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("Current Setpoint", this.setPoint);
    SmartDashboard.putNumber("Transform Encoder Value", getEncoderValue());
  }
  public void zero() {
  }
  public void brakedis() {
  }
  // Switch drives to PID
  public void driveTowardsPid() {
    double power = this.getPidPower();
    this.Switch_On(power);
  }
}
