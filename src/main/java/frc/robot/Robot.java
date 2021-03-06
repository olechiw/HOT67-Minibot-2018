/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

    public static final double ticksPerSecond = 27000;

    public static final int JOYSTICK_DRIVER = 0;
    public static final int JOYSTICK_LY = 1;
    public static final int JOYSTICK_RX = 4;

    Joystick driver = new Joystick(JOYSTICK_DRIVER);

    DriveTrain driveTrain;

    @Override
    public void robotInit() {
        driveTrain = new DriveTrain();
    }

    @Override
    public void autonomousInit() {
        driveTrain.zeroSensors();
        driveTrain.zeroTalons();
        profileFinished = false;
    }

    boolean profileFinished = false;
    @Override
    public void autonomousPeriodic() {
        if (!profileFinished)
            profileFinished = driveTrain.FollowPath();
        else
            driveTrain.zeroTalons();
    }

    @Override
    public void robotPeriodic() {
        driveTrain.readSensors();

        /* Add autonomous code here */

        driveTrain.writeDashBoard();
    }

    @Override
    public void teleopInit()
    {
        driveTrain.zeroSensors();
    }

    @Override
    public void teleopPeriodic() {
        // May have to invert driveturn/drivespeed
        driveTrain.arcadeDrive(driver.getRawAxis(JOYSTICK_RX), driver.getRawAxis(JOYSTICK_LY));
    }

    @Override
    public void testPeriodic() {
    }
}
