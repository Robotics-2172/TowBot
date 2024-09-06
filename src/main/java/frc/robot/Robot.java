// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

// #ColinFlip
  double limitValue = 100;
  double deadband = 0.05;

  SlewRateLimiter limit = new SlewRateLimiter(limitValue);

  CANSparkMax l1 = new CANSparkMax(53, MotorType.kBrushless);
  CANSparkMax l2 = new CANSparkMax(56, MotorType.kBrushless);
  CANSparkMax r1 = new CANSparkMax(54, MotorType.kBrushless);
  CANSparkMax r2 = new CANSparkMax(58, MotorType.kBrushless);

  XboxController driver = new XboxController(0);


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    l1.setInverted(true);
    l2.setInverted(true);
    r1.setInverted(false );
    r2.setInverted(false );
    l1.burnFlash();
    l2.burnFlash();
    r1.burnFlash();
    r2.burnFlash();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    double left  = 0;
    double right = 0;
    double netSpeed = driver.getRightTriggerAxis() - driver.getLeftTriggerAxis();

    left  -= 0.5 * driver.getLeftX();
    right += 0.5 * driver.getLeftX();

    if (netSpeed < deadband){
      left  *= -1;
      right *= -1;
    }

    left  += netSpeed;
    right += netSpeed;

    left  = MathUtil.clamp(left , -1, 1);
    right = MathUtil.clamp(right, -1, 1);

    left  = MathUtil.applyDeadband(limit.calculate(left ), deadband);
    right = MathUtil.applyDeadband(limit.calculate(right), deadband);

    l1.set(left);
    l2.set(left);
    r1.set(right);
    r2.set(right);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
