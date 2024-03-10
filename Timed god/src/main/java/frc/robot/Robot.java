// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {

  WPI_TalonSRX RMtrFllw = new WPI_TalonSRX(1);
  WPI_TalonSRX RMtrEnc = new WPI_TalonSRX(2);

  WPI_TalonSRX LMtrFllw = new WPI_TalonSRX(3);
  WPI_TalonSRX LMtrEnc = new WPI_TalonSRX(4);

  DifferentialDrive Chasis = new DifferentialDrive(RMtrEnc, LMtrEnc);

  Joystick Joy = new Joystick(0);

  @Override
  public void robotInit() {
    RMtrEnc.setInverted(false);
    LMtrEnc.setInverted(true);

    RMtrFllw.follow(RMtrEnc);
    LMtrFllw.follow(LMtrEnc);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    Chasis.arcadeDrive(Joy.getRawAxis(2) - Joy.getRawAxis(3), Joy.getRawAxis(0));
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
}
