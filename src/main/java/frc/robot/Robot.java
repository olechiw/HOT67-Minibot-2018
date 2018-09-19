/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends IterativeRobot {

    public static final int P = 0;
    public static final int I = 0;
    public static final int D = 0;

    public static final int TALON_LEFT = 0;
    public static final int TALON_RIGHT = 1;

    public static final int JOYSTICK_DRIVER = 0;
    public static final int JOYSTICK_LY = 0;
    public static final int JOYSTICK_RX = 4;

    WPI_TalonSRX leftTalon = new WPI_TalonSRX(TALON_LEFT);
    WPI_TalonSRX rightTalon = new WPI_TalonSRX(TALON_RIGHT);

    Joystick driver = new Joystick(JOYSTICK_DRIVER);

    DifferentialDrive drive;

    @Override
    public void robotInit() {
        drive = new DifferentialDrive(leftTalon, rightTalon);

        leftTalon.setInverted(true);
        rightTalon.setInverted(false);
    }


    @Override
    public void robotPeriodic() {
        double driveSpeed = driver.getRawAxis(0);
        double driveTurn = driver.getRawAxis(4);
        drive.arcadeDrive(driveSpeed, driveTurn);
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testPeriodic() {
    }
}
