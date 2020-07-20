/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team224.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	// Declarations for autonomous choosers

	private SendableChooser<String> locChooser = new SendableChooser<>();
	private SendableChooser<String> actionChooser = new SendableChooser<>();
	private SendableChooser<String> crossChooser = new SendableChooser<>();
	private SendableChooser<String> delayChooser = new SendableChooser<>();
	// private SendableChooser<String> orientationChooser = new SendableChooser<>();

	private int delay;
	private String location;
	private String action;
	private String cross;
	private String gameData;
	boolean isAuto;
	private String switchLR;
	private String scaleLR;
	// Declarations for swerve
	SwerveDrive swerve;
	AnalogInput pressureSwitch;
	// Declarations for BoxSubsystem
	WPI_TalonSRX elevator;
	DoubleSolenoid pusher, grabber, raiser;
	Timer armTimer, raiseTimer;
	private int armPosition;

	// Declarations for ramp

	// Other Declarations/Multisystem declarations
	Compressor compress;
	Joystick drive, aux;

	// Declarations for victor

	VictorSP climb1;
	VictorSP climb2;

	/**
	 * s This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 * 
	 */
	@Override
	public void robotInit() {

		CameraServer.getInstance().startAutomaticCapture(); // roborio-224-frc.local:1181 in firefox
		boxSubsystemInit();
		joyInit();
		climbInit();
		swerve = new SwerveDrive(drive);

		pressureSwitch = new AnalogInput(0);
		delayChooser.addDefault("0", "0");
		delayChooser.addObject("1", "1");
		delayChooser.addObject("2", "2");
		delayChooser.addObject("3", "3");
		delayChooser.addObject("4", "4");
		delayChooser.addObject("5", "5");
		delayChooser.addObject("6", "6");
		delayChooser.addObject("7", "7");
		SmartDashboard.putData("Delay", delayChooser);

		locChooser.addObject("Lefta", "Left");
		locChooser.addObject("Righta", "Right");
		locChooser.addObject("Centera", "Center");
		SmartDashboard.putData("Location", locChooser);

		crossChooser.addObject("Yes", "Yes");
		crossChooser.addDefault("No", "No");
		SmartDashboard.putData("Cross", crossChooser);

		actionChooser.addObject("Scale", "Scale");
		actionChooser.addObject("Switch", "Switch");
		actionChooser.addObject("ScaleSwitch", "ScaleSwitch");
		actionChooser.addDefault("DriveForward", "Drive Forward");
		SmartDashboard.putData("Action", actionChooser);

	}

	@Override
	public void autonomousInit() {
		swerve.init();
		raiser.set(Constants.raiserUp);
		grabber.set(Constants.closeArms);
		pusher.set(Constants.pullInPusher);

		isAuto = true;
		gameData = "";
		while (gameData.length() < 3) {
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}
		// DriverStation.getInstance().getAlliance();
		location = locChooser.getSelected();
		action = actionChooser.getSelected();
		cross = crossChooser.getSelected();
		delay = Integer.parseInt(delayChooser.getSelected());
		switchLR = gameData.substring(0, 1);
		scaleLR = gameData.substring(1, 2);
		elevator.setSelectedSensorPosition(0, 0, 0);
		System.out.println(gameData);

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {

		swerve.print();
		if (isAuto) {
			// swerve.setAngleAdjustment(90);
			this.driveDistanceMotionMagic();
			// autoSwitchInit();
			// swerve.driveDistanceMotionMagic(0, 47, 90);
			// Timer.delay(1.5);
			// swerve.turnToDegrees(0);
			// Timer.delay(2);
			// swerve.driveDistanceMotionMagic(0, 105.5, 0);
			// Timer.delay(3.5);
			// grabber.set(Constants.openArms);

		}
		isAuto = false;
	}

	public void disabledPeriodic() {
		// swerve.print();
	}

	public void teleopInit() {
		// compress.start();
		swerve.disableAll();
		raiseTimer.start();
		armTimer.start();
		// swerve.print();
		// elevator.setSelectedSensorPosition(0, 0, 0);
		// raiser.set(Constants.raiserUp);// up
		// grabber.set(Constants.closeArms);// closed
		// pusher.set(Constants.pullInPusher);// pull in
		swerve.init();

	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {

		swerve.drive();
		elevator();
		grabber();
		raiser();
		climber();
		smartDashboard();
		if (drive.getRawButton(3)) {
			swerve.navx.reset();
		}
	}

	public void climber() {
		if (aux.getPOV() == 0) {
			climb1.set(1);
			climb2.set(1);
		} else if (aux.getPOV() == 180 && aux.getRawButton(12)) {
			climb1.set(-1);
			climb2.set(-1);
		} else {
			climb1.disable();
			climb2.disable();
		}
	}

	public void elevator() {
		if (aux.getRawButton(11)) {
			elevator.set(ControlMode.Position, Constants.elevatorDownEncoderValue);
		} else if (aux.getRawButton(9)) {
			elevator.set(ControlMode.Position, Constants.elevatorMiddleEncoderValue);
		} else if (aux.getRawButton(7)) {
			elevator.set(ControlMode.Position, Constants.elevatorTopEncoderValue);
		} else if (aux.getY() > .2) {
			elevator.set(ControlMode.PercentOutput, aux.getY());
		} else if (aux.getY() < -.2) {
			elevator.set(ControlMode.PercentOutput, aux.getY());
		} else {
			elevator.set(ControlMode.PercentOutput, 0);
		}
	}

	public void grabber() {
		if (aux.getTrigger() && armTimer.get() > .3) {
			switch (armPosition) {
			case 0:
				grabber.set(Constants.openArms);// open
				pusher.set(Constants.pushOutPusher);// push out
				armPosition++;
				break;
			case 1:
				grabber.set(Constants.openArms);// open
				pusher.set(Constants.pullInPusher);// pull in
				armPosition++;
				break;
			case 2:
				grabber.set(Constants.closeArms);// closed
				pusher.set(Constants.pullInPusher); // pull in
				armPosition = 0;
				break;
			}
			armTimer.reset();
		}
	}

	public void raiser() {
		if (aux.getRawButton(2) && raiseTimer.get() > .3) {
			if (raiser.get() == Value.kForward) {
				raiser.set(Value.kReverse);
			} else
				raiser.set(Value.kForward);
			raiseTimer.reset();
		}
	}

	public void joyInit() {
		drive = new Joystick(0);
		aux = new Joystick(1);
	}

	public void elevatorInit() {
		elevator = new WPI_TalonSRX(20);
		elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		// elevator.configPeakCurrentLimit(47, 0);
		// elevator.enableCurrentLimit(true);
		// elevator.configPeakCurrentDuration(100, 0);
		elevator.setSelectedSensorPosition(0, 0, 0);
		elevator.config_kP(0, Constants.elevatorP, 0);
		elevator.config_kI(0, Constants.elevatorI, 0);
		elevator.config_kD(0, Constants.elevatorD, 0);
	}

	public void armsInit() {
		grabber = new DoubleSolenoid(4, 7);
		pusher = new DoubleSolenoid(0, 1);
		raiser = new DoubleSolenoid(6, 5);
		compress = new Compressor(0);
		armTimer = new Timer();
		raiseTimer = new Timer();
		armPosition = 0;
	}

	public void boxSubsystemInit() {
		elevatorInit();
		armsInit();
	}

	private void climbInit() {
		climb1 = new VictorSP(0);
		climb2 = new VictorSP(1);
	}

	public void elevatorUp() {
		elevator.set(ControlMode.Position, Constants.elevatorTopEncoderValue);
	}

	public void elevatorMiddle() {
		elevator.set(ControlMode.Position, Constants.elevatorMiddleEncoderValue);
	}

	public void elevatorDown() {
		elevator.set(ControlMode.Position, Constants.elevatorDownEncoderValue);
	}

	public void autoSwitchInit() {
		// raiser.set(Constants.raiserDown);
		elevatorMiddle();
	}

	public void smartDashboard() {
		swerve.print();
		SmartDashboard.putNumber("PSI SENSOR", 250 * (pressureSwitch.getVoltage() / 5) - 25);
		SmartDashboard.putBoolean("Compressor PSI", compress.getPressureSwitchValue());
		SmartDashboard.putNumber("Elevator Position", elevator.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Current", elevator.getOutputCurrent());
	}

	public void autonomousDriveDistanceNavX() {

		if (delay != 0) {
			Timer.delay(delay);
		}
		// swerve.turnToDegrees(90);
		switch (location) {
		case "Left":
			switch (action) {
			case "Switch":
				switch (cross) {
				case "Yes":
					if (!switchLR.equals("L")) {

					}
				case "No":
					if (switchLR.equals("L")) {
						this.autoSwitchInit();
						swerve.driveDistanceNavX(0, 150.75, 0, 0);
						swerve.turnToDegrees(90);
						Timer.delay(1);
						swerve.turnToDegrees(90);
						swerve.driveDistanceNavX(0, 18., 0, 0, .35);
						raiser.set(Constants.raiserDown);
						Timer.delay(.5);
						grabber.set(Constants.openArms);
						pusher.set(Constants.pushOutPusher);
					} else {
						swerve.driveDistanceNavX(0, 120, 0, 0);
					}
					break;
				}
				break;
			case "Scale":
				swerve.setAngleAdjustment(90);
				switch (cross) {
				case "Yes":
					if (!scaleLR.equals("L")) {

					}
				case "No":
					if (scaleLR.equals("L")) {
						swerve.driveDistanceNavX(0, 300, -90, 0, .8);
						swerve.turnToDegrees(90);
						Timer.delay(1.75);
						swerve.driveDistanceNavX(0, 7.5, 0, 0, .3);
						this.elevatorUp();
						Timer.delay(5);
						raiser.set(Constants.raiserDown);
						Timer.delay(.5);
						grabber.set(Constants.openArms);
						pusher.set(Constants.pushOutPusher);
					} else {
						swerve.driveDistanceNavX(12, 0, -90, 0);
					}
					break;
				}
				break;
			case "ScaleSwitch":
				switch (cross) {
				case "Yes":
					break;
				case "No":
					break;
				}
				break;
			case "DriveForward":
				swerve.driveDistanceNavX(10, 0, 0, 0);
				break;
			}
			break;
		case "Right":
			switch (action) {
			case "Switch":
				switch (cross) {
				case "Yes":
					if (!switchLR.equals("R")) {

					}
				case "No":
					if (switchLR.equals("R")) {
						this.autoSwitchInit();
						swerve.driveDistanceNavX(0, 150.75, 0, 0, .65);
						swerve.turnToDegrees(-90);
						Timer.delay(1.75);
						swerve.driveDistanceNavX(0, 18, 0, 0, .35);
						raiser.set(Constants.raiserDown);
						Timer.delay(.5);
						grabber.set(Constants.openArms);
						pusher.set(Constants.pushOutPusher);
					} else {
						swerve.driveDistanceNavX(0, 120, 0, 0, .65);
					}
					break;
				}
				break;
			case "Scale":
				swerve.setAngleAdjustment(-90);
				switch (cross) {
				case "Yes":
					if (!scaleLR.equals("R")) {

					}
				case "No":
					if (scaleLR.equals("R")) {
						swerve.driveDistanceNavX(0, 300, 90, 0, .8);
						swerve.turnToDegrees(-90);
						Timer.delay(1.75);
						swerve.driveDistanceNavX(0, 7.5, 0, 0, .3);
						this.elevatorUp();
						Timer.delay(5);
						raiser.set(Constants.raiserDown);
						Timer.delay(.5);
						grabber.set(Constants.openArms);
						pusher.set(Constants.pushOutPusher);
					}
					break;
				}
				break;
			case "ScaleSwitch":
				switch (cross) {
				case "Yes":
					break;
				case "No":
					break;
				}
				break;
			case "DriveForward":
				swerve.driveDistanceMotionMagic(10, 0, 0);
				break;
			}
			break;
		case "Center":
			switch (action) {
			case "Switch":
				if (switchLR.equals("L")) {
					this.autoSwitchInit();
					swerve.driveDistanceNavX(0, 30, 0, 0, .5);
					Timer.delay(1);
					swerve.turnToDegrees(0);
					Timer.delay(1.75);
					swerve.driveDistanceNavX(0, 62.5, -90, 0, .5);
					swerve.turnToDegrees(0);
					Timer.delay(1.75);
					swerve.driveDistanceNavX(0, 75.5, 0, 0, .5);

				} else {
					this.autoSwitchInit();
					swerve.driveDistanceNavX(0, 47, 90, 0, .5);
					swerve.turnToDegrees(0);
					Timer.delay(1.75);
					swerve.driveDistanceNavX(0, 105.5, 0, 0, .5);
					raiser.set(Constants.raiserDown);
					Timer.delay(.5);
					grabber.set(Constants.openArms);
					pusher.set(Constants.pushOutPusher);

				}
				break;
			}
			break;
		case "Scale":
			switch (cross) {
			case "Yes":
				break;
			case "No":
				break;
			}
			break;
		case "ScaleSwitch":
			switch (cross) {
			case "Yes":
				break;
			case "No":
				break;
			}
			break;
		case "DriveForward":
			break;
		}

	}

	public void driveDistanceMotionMagic() {
		if (delay != 0) {
			Timer.delay(delay);
		}
		switch (location) {
		case "Left":
			switch (action) {
			case "Switch":
				if (switchLR.equals("L")) {
					this.leftSwitch();
				} else {
					this.driveForwardLeft();
				}
				break;
			case "Scale":
				swerve.setAngleAdjustment(90);
				switch (cross) {
				case "Yes":
					if (scaleLR.equals("L")) {
						// swerve.driveDistanceMotionMagic(0, 300, -90);
						// Timer.delay(5);
						// swerve.turnToDegrees(90);
						// Timer.delay(2);
						// swerve.driveDistanceMotionMagic(0, 7.5, 0);
						// elevatorUp();
						// Timer.delay(5);
						// raiser.set(Constants.raiserDown);
						// Timer.delay(.5);
						// grabber.set(Constants.openArms);
						// pusher.set(Constants.pushOutPusher);
					} else {
						// swerve.driveDistanceMotionMagic(0, 220, -90);
						// Timer.delay(5);
						// swerve.turnToDegrees(90);
						// swerve.driveDistanceMotionMagic(0, 200, 0);
						// Timer.delay(5);
						// swerve.turnToDegrees(0);
						// Timer.delay(2);
						// swerve.driveDistanceMotionMagic(0, 43, 0);
						// elevatorUp();
						// Timer.delay(3);
						// raiser.set(Constants.raiserDown);
						// grabber.set(Constants.openArms);

					}
					break;
				case "No":
					if (scaleLR.equals("L")) {
						// swerve.setAngleAdjustment(90);
						// swerve.driveDistanceMotionMagic(0, 300, -90);
						// Timer.delay(5);
						// swerve.turnToDegrees(90);
						// Timer.delay(2);
						// swerve.driveDistanceMotionMagic(0, 7.5, 0);
						// elevatorUp();
						// Timer.delay(2);
						// raiser.set(Constants.raiserDown);
						// Timer.delay(1);
						// grabber.set(Constants.openArms);
						// pusher.set(Constants.pushOutPusher);
					} else {
						// swerve.driveDistanceMotionMagic(10, 0, -90);
					}
					break;
				}
				break;
			case "ScaleSwitch":
				switch (cross) {
				case "Yes":
					break;
				case "No":
					break;
				}
				break;
			case "DriveForward":
				this.driveForwardLeft();
				break;
			}
			break;
		case "Right":
			switch (action) {
			case "Switch":
				if (switchLR.equals("R")) {
					this.rightSwitch();

					// this.autoSwitchInit();
					// swerve.driveDistanceMotionMagic(0, 57.5, -90);
					// Timer.delay(5);
					// swerve.turnToDegrees(0);
					// Timer.delay(2);
					// swerve.driveDistanceMotionMagic(0, 105.5, 0);
					// Timer.delay(5);
					// grabber.set(Constants.openArms);
				} else {
					this.driveForwardRight();
				}
				break;
			case "Scale":
				// swerve.setAngleAdjustment(-90);
				switch (cross) {
				case "Yes":
					if (scaleLR.equals("R")) {
						// swerve.driveDistanceMotionMagic(0, 150, 90);
						// Timer.delay(3);
						// swerve.turnToDegrees(0 + (0 - swerve.getAngle()));
						// Timer.delay(1.5);
						// swerve.driveDistanceMotionMagic(0, 150, 90);
						// Timer.delay(3);
						// swerve.turnToDegrees(-90);
						// Timer.delay(1.5);
						// // swerve.driveDistanceMotionMagic(0, 7.5, 0);
						// elevatorUp();
						// Timer.delay(2);
						// raiser.set(Constants.raiserDown);
						// Timer.delay(1);
						// grabber.set(Constants.openArms);
						// pusher.set(Constants.pushOutPusher);

					} else {
						// swerve.driveDistanceMotionMagic(0, 220, 90);
						// Timer.delay(5);
						// swerve.turnToDegrees(90);
						// swerve.driveDistanceMotionMagic(0, 200, 0);
						// Timer.delay(5);
						// swerve.turnToDegrees(0);
						// Timer.delay(2);
						// swerve.driveDistanceMotionMagic(0, 43, 0);
						// elevatorUp();
						// Timer.delay(3);
						// raiser.set(Constants.raiserDown);
						// grabber.set(Constants.openArms);

					}
					break;
				case "No":
					// swerve.driveDistanceMotionMagic(0, 300, 90);
					// Timer.delay(5);
					// // swerve.turnToDegrees(0);
					// Timer.delay(2);
					// // swerve.driveDistanceMotionMagic(0, 7.5, 0);
					// elevatorUp();
					// Timer.delay(2);
					// raiser.set(Constants.raiserDown);
					// Timer.delay(1);
					// grabber.set(Constants.openArms);
					// pusher.set(Constants.pushOutPusher);
					// break;
				}
				break;
			case "ScaleSwitch":
				switch (cross) {
				case "Yes":
					break;
				case "No":
					break;
				}
				break;
			case "DriveForward":
				this.driveForwardRight();
				break;
			}
			break;
		case "Center":
			switch (action) {
			case "Switch":
				if (switchLR.equals("L")) {
					this.centerSwitchLeft();

				} else {
					this.centerSwitchRight();
				}
				break;
			}
			break;
		case "Scale":
			switch (cross) {
			case "Yes":
				break;
			case "No":
				break;
			}
			break;
		case "ScaleSwitch":
			switch (cross) {
			case "Yes":
				break;
			case "No":
				break;
			}
			break;
		case "DriveForward":
			break;
		}

	}

	public void leftSwitch() {
		swerve.setAngleAdjustment(90);
		swerve.turnToDegrees(90);
		this.autoSwitchInit();
		swerve.driveDistanceMotionMagic(0, 150.75, -90);
		Timer.delay(3);
		swerve.turnToDegrees(-90);
		Timer.delay(1.75);
		swerve.turnToDegrees(-90);
		Timer.delay(1.75);
		swerve.driveDistanceMotionMagic(0, 50, 0);
		Timer.delay(1);
		raiser.set(Constants.raiserDown);
		Timer.delay(.5);
		grabber.set(Constants.openArms);
		pusher.set(Constants.pushOutPusher);
	}

	public void rightSwitch() {
		swerve.setAngleAdjustment(-90);
		swerve.turnToDegrees(-90);
		this.autoSwitchInit();
		swerve.driveDistanceMotionMagic(0, 150.75, 90);
		Timer.delay(3);
		swerve.turnToDegrees(-90);
		Timer.delay(1.75);
		swerve.turnToDegrees(-90);
		Timer.delay(1.75);
		swerve.driveDistanceMotionMagic(0, 50, 0);
		Timer.delay(1);
		raiser.set(Constants.raiserDown);
		Timer.delay(.5);
		grabber.set(Constants.openArms);
		pusher.set(Constants.pushOutPusher);
	}
	/*
	 * final auton left and right switch // this.autonomousDriveDistanceNavX(); //
	 * swerve.turnToDegrees(90); this.autoSwitchInit();
	 * swerve.driveDistanceMotionMagic(0, 150.75, 90); Timer.delay(3); //
	 * swerve.turnToDegrees(-90); // Timer.delay(1.75); //
	 * swerve.turnToDegrees(-90); // Timer.delay(1.75);
	 * swerve.driveDistanceMotionMagic(0, 12, 0); Timer.delay(1); //
	 * raiser.set(Constants.raiserDown); // Timer.delay(.5);
	 * grabber.set(Constants.openArms); pusher.set(Constants.pushOutPusher);
	 */

	public void centerSwitchLeft() {

		this.autoSwitchInit();
		swerve.driveDistanceMotionMagic(0, 30, 0);
		Timer.delay(1.75); 
		// swerve.turnToDegrees(0);
		Timer.delay(.6);
		swerve.driveDistanceMotionMagic(0, 73, -90);
		Timer.delay(2.5);
		// swerve.turnToDegrees(0);
		Timer.delay(.6);
		swerve.driveDistanceMotionMagic(0, 70, 0);
		Timer.delay(2); 
		raiser.set(Constants.raiserDown);
		Timer.delay(1);
		grabber.set(Constants.openArms);
	}

	public void centerSwitchRight() {

		autoSwitchInit();
		swerve.driveDistanceMotionMagic(0, 56, 90);
		Timer.delay(1.5);
		// swerve.turnToDegrees(0);
		Timer.delay(1);
		swerve.driveDistanceMotionMagic(0, 110, 0);
		Timer.delay(3.5);
		raiser.set(Constants.raiserDown);
		Timer.delay(.75);
		grabber.set(Constants.openArms);
	}

	public void driveForwardLeft() {
		swerve.setAngleAdjustment(90);
		swerve.driveDistanceMotionMagic(10, 0, -90);
	}

	public void driveForwardRight() {
		swerve.setAngleAdjustment(-90);
		swerve.driveDistanceMotionMagic(10, 0, 90);

	}
}