// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
//CHASIS--------------------------------------------------------------//
  WPI_TalonSRX RMtrFllw = new WPI_TalonSRX(1);           //
  WPI_TalonSRX RMtrEnc = new WPI_TalonSRX(2);            //
                                                                      //
  WPI_TalonSRX LMtrFllw = new WPI_TalonSRX(3);           //
  WPI_TalonSRX LMtrEnc = new WPI_TalonSRX(4);            //
                                                                      //
  DifferentialDrive Chasis = new DifferentialDrive(RMtrEnc, LMtrEnc); //
//--------------------------------------------------------------------//
//BRAZO-----------------------------------------------------------------------------------------------------------------------------------------------------------//
  CANSparkMax RmotbrazoLeader = new CANSparkMax (5, MotorType.kBrushless);                                                                                     //
  CANSparkMax LmotbrazoFollower = new CANSparkMax (6, MotorType.kBrushless);        
  

                                                                                                                                                                  //
  ProfiledPIDController PIDRmotbrazoLeader = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(80, 90));         //
  ProfiledPIDController PIDLmotbrazoFollower = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(80, 90));         //
//----------------------------------------------------------------------------------------------------------------------------------------------------------------//
//INTAKE-----------------------------------------------------// 
   WPI_VictorSPX Motink = new WPI_VictorSPX(7); //
//-----------------------------------------------------------//
//JOYSTICK-----------------------------//
  Joystick Joy = new Joystick(0); //
//-------------------------------------//

  // limelight, posiciones arm
  @Override
  public void robotInit() {
    RMtrEnc.setInverted(false);
    LMtrEnc.setInverted(true);  

    RMtrFllw.follow(RMtrEnc);
    LMtrFllw.follow(LMtrEnc);

    LmotbrazoFollower.follow(RmotbrazoLeader, true);

    
    //Rmotbrazoleader.getEncoder();
   //   / 42 ) * 360;
  }

  @Override
  public void robotPeriodic() {

        SmartDashboard.putNumber("Numero", LmotbrazoFollower.get());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    Chasis.arcadeDrive(Joy.getRawAxis(2) - Joy.getRawAxis(3), Joy.getRawAxis(0));

  arm(Joy.getRawAxis(5));
    
    
  if (Joy.getRawButtonPressed (1)) {
    Motink.set(1);

  }else if (Joy.getRawButtonPressed(2)) {
    Motink.set(0.5);

  }else if (Joy.getRawButtonPressed(3)) {
    Motink.set(-1);

  }else if (Joy.getRawButtonPressed(4)) {
    Motink.set(-0.5);

  }else {
    Motink.set(0);
  }

  if (Joy.getRawButtonPressed (5)) {
    GradosF(0);

     }else if (Joy.getRawButtonPressed(6)) {
    GradosF(45);

     }else if (Joy.getRawButtonPressed(7)) {
    GradosF(90);
  
  }
}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public void arm (double motvel) {
  RmotbrazoLeader.set(motvel);

  
  //LmotbrazoFollower.setInverted(true);
  //LmotbrazoFollower.set(motvel);
    
    RmotbrazoLeader.setInverted(false);
    LmotbrazoFollower.setInverted(true); //Esto sirve para no tener que poner la velocidad negativa 👌 

    LmotbrazoFollower.follow(RmotbrazoLeader); //Esto sirve para que el left motor haga lo mismo que el right motor
 }

  // public void arm(double LyRgrados) {
 // Rmotbrazo.set(LyRgrados);
  //Lmotbrazo.set(-LyRgrados);
 //} 

  public void GradosF (double Vueltas2) {
    double LftNeoEnc = LmotbrazoFollower.getEncoder().getPosition();
    LmotbrazoFollower.set( PIDLmotbrazoFollower.calculate(LftNeoEnc, Vueltas2));
  }
  //Esto sirve para 
}
