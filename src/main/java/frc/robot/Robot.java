// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Robot extends TimedRobot {

  WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(3);
  WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(13);
  WPI_TalonSRX m_rightSlave = new WPI_TalonSRX(2);
  WPI_TalonSRX m_leftSlave = new WPI_TalonSRX(1);

  /** 
   * Black Tank
  */
  Solenoid leverSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 9);
  Solenoid cannonSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 8);
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

  double scale = 250, offset = -25;
  AnalogPotentiometer pressureTransducer = new AnalogPotentiometer(/* the AnalogIn port*/ 2, scale, offset);

  double tank_Pressure;
  boolean launch;





  //assign drivetrain motor IDs

  //assign valve motor ID

  private final XboxController m_driverController = new XboxController(0);
  //assign controller port

  DifferentialDrive drive = new DifferentialDrive(m_leftMaster, m_rightMaster);
  //create differential drivetrain with leading/master motors


  //double startTime;
  //int x;
  //create variables for use in timer and iteration logic

  @Override
  public void robotInit() {

    m_rightMaster.setInverted(true);
    m_rightSlave.setInverted(true);
    //invert right side motors

    m_leftSlave.set(ControlMode.Follower, 3);
    m_rightSlave.set(ControlMode.Follower, 13);
    //assign follower status to follower/slave motors with regards to leading/master motor IDs
    SmartDashboard.putNumber("Launch Pressure", 50);

  }

  @Override
  public void robotPeriodic() {

    boolean pressureSwitch = phCompressor.getPressureSwitchValue();
    double current = phCompressor.getCurrent();
    
    double pressure = pressureTransducer.get();
    SmartDashboard.putNumber("Current pressure", pressure);
    tank_Pressure = SmartDashboard.getNumber("Launch Pressure", 50);
    SmartDashboard.putBoolean("Switch", pressureSwitch);
    SmartDashboard.putNumber("Current", current);
    SmartDashboard.putBoolean("launch", launch);

  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    phCompressor.enableDigital();

  }

  @Override
  public void teleopPeriodic() { 

    
    if(pressureTransducer.get() < tank_Pressure){
      if(launch){
        cannonSolenoid.set(false);
      }
      else{
        cannonSolenoid.set(true);
      }

    }
    if(pressureTransducer.get() > tank_Pressure){
      cannonSolenoid.set(false);
    }
    
    if(m_driverController.getBButtonReleased()){
      cannonSolenoid.set(false);
    }

    if(m_driverController.getAButtonPressed()){
      leverSolenoid.set(true);
      launch = true;
    }
    if(m_driverController.getAButtonReleased()){
      leverSolenoid.set(false);
    }
    if(m_driverController.getXButtonPressed()){
      launch = false;
    }
    
    drive.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getRightX());
    //
      }

  @Override
  public void disabledInit() {  

  }

  @Override
  public void disabledPeriodic() {
    phCompressor.disable();

  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
