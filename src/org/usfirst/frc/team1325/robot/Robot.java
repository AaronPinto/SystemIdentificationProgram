package org.usfirst.frc.team1325.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

@SuppressWarnings("unused")
public class Robot extends IterativeRobot {
	public static final CANTalon _driveLeftFront = new CANTalon(2);//Has mag encoder plugged in Positive = For
	public static final CANTalon _driveLeftRear = new CANTalon(3);
	public static final CANTalon _driveRightFront = new CANTalon(13);//Has mag encoder plugged in Negative = For
	public static final CANTalon _driveRightRear = new CANTalon(12);

	@Override
	public void robotInit() {
		_driveLeftFront.changeControlMode(TalonControlMode.Voltage);
		_driveLeftFront.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		_driveLeftFront.reverseSensor(true);
		_driveLeftFront.setNominalClosedLoopVoltage(12.0);
		_driveLeftFront.configPeakOutputVoltage(+12f, -12f);
		_driveLeftFront.setAllowableClosedLoopErr(0);
		_driveLeftFront.enableBrakeMode(true);
		_driveLeftFront.setCurrentLimit(40);
		_driveLeftFront.EnableCurrentLimit(true);
		_driveRightFront.changeControlMode(TalonControlMode.Voltage);
		_driveRightFront.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		_driveRightFront.reverseSensor(true);
		_driveRightFront.setNominalClosedLoopVoltage(12.0);
		_driveRightFront.configPeakOutputVoltage(+12f, -12f);
		_driveRightFront.setAllowableClosedLoopErr(0);
		_driveRightFront.enableBrakeMode(true);
		_driveRightFront.setCurrentLimit(40);
		_driveRightFront.EnableCurrentLimit(true);
		_driveLeftRear.changeControlMode(TalonControlMode.Voltage);
		_driveLeftRear.setNominalClosedLoopVoltage(12.0);
		_driveLeftRear.configPeakOutputVoltage(+12f, -12f);
		_driveLeftRear.setAllowableClosedLoopErr(0);
		_driveLeftRear.enableBrakeMode(true);
		_driveLeftRear.setCurrentLimit(40);
		_driveLeftRear.EnableCurrentLimit(true);
		_driveRightRear.changeControlMode(TalonControlMode.Voltage);
		_driveRightRear.setNominalClosedLoopVoltage(12.0);
		_driveRightRear.configPeakOutputVoltage(+12f, -12f);
		_driveRightRear.setAllowableClosedLoopErr(0);
		_driveRightRear.enableBrakeMode(true);
		_driveRightRear.setCurrentLimit(40);
		_driveRightRear.EnableCurrentLimit(true);
	}

	public void robotPeriodic() {
		SmartDashboard.putNumber("Thread dT", delta);
	}

	double delta = 0;
	public ArrayList<Data> logs = new ArrayList<Data>(1000000);

	class Data {
		Double leftFrontVolt;
		Double rightFrontVolt;
		Double leftRearVolt;
		Double rightRearVolt;
		Double leftSpeed;
		Double rightSpeed;

		Data(double lFV, double rFV, double lRV, double rRV, double lS, double rS) {
			this.leftFrontVolt = lFV;
			this.rightFrontVolt = rFV;
			this.leftRearVolt = lRV;
			this.rightRearVolt = rRV;
			this.leftSpeed = lS;
			this.rightSpeed = rS;
		}
	}

	Thread logThread = new Thread();
	public void teleopInit() {
		logThread = new Thread() {
			@Override
			public void run() {
				boolean secondSide = true;
				double start, start2, startTime = start = start2 = System.nanoTime();
				while(!Thread.interrupted()) {
					delta = (System.nanoTime() - startTime) / 1000000;
					if(((System.nanoTime() - start) / 1000000000) < 120) {
						if(delta >= 2.0) {
							double now = System.nanoTime();
							Robot._driveRightFront.set(-12 * Math.sin(0.4 * Math.pow((now - start2) / 1000000000, 2)));
							Robot._driveRightRear.set(-12 * Math.sin(0.4 * Math.pow((now - start2) / 1000000000, 2)));
							logs.add(new Data(Robot._driveLeftFront.getOutputVoltage(), Robot._driveRightFront.getOutputVoltage(), 
									Robot._driveLeftRear.getOutputVoltage(), Robot._driveRightRear.getOutputVoltage(),
									Robot._driveLeftFront.getSpeed(), Robot._driveRightFront.getSpeed()));
							startTime = System.nanoTime();
						}
					} else if((((System.nanoTime() - start) / 1000000000) >= 120) && (((System.nanoTime() - start) / 1000000000) <= 240)) {
						if(secondSide) {
							secondSide = false;
							Robot._driveRightFront.set(0.0);
							Robot._driveRightRear.set(0.0);
							start2 = System.nanoTime();
						}
						if(delta >= 2.0) {
							double now = System.nanoTime();
							Robot._driveLeftFront.set(12 * Math.sin(0.4 * Math.pow((now - start2) / 1000000000, 2)));
							Robot._driveLeftRear.set(12 * Math.sin(0.4 * Math.pow((now - start2) / 1000000000, 2)));
							logs.add(new Data(Robot._driveLeftFront.getOutputVoltage(), Robot._driveRightFront.getOutputVoltage(), 
									Robot._driveLeftRear.getOutputVoltage(), Robot._driveRightRear.getOutputVoltage(),
									Robot._driveLeftFront.getSpeed(), Robot._driveRightFront.getSpeed()));
							startTime = System.nanoTime();
						}
					} else {
						Robot._driveLeftFront.set(0.0);
						Robot._driveLeftRear.set(0.0);
						Robot._driveRightFront.set(0.0);
						Robot._driveRightRear.set(0.0);
					}
				}
			}
		};
		logThread.start();
	}

	BufferedWriter bw;
	Calendar c = Calendar.getInstance();
	public void disabledInit() {
		logThread.interrupt();

		if(logThread.isInterrupted()) {
			File f = new File("/home/lvuser/logs");
			if(!f.exists()) f.mkdir();
			try {
				bw = new BufferedWriter(new FileWriter(String.format("/home/lvuser/logs/%tB%te%tY-%tl%tM%tS%tp.csv",
						c, c, c, c, c, c, c)));
				bw.write("leftFrontVolt, rightFrontVolt, leftRearVolt, rightRearVolt, leftSpeed, rightSpeed");
				bw.newLine();
				for(Data i : logs) {
					bw.write(i.leftFrontVolt + "," + i.rightFrontVolt + "," + i.leftRearVolt + "," + i.rightRearVolt + "," + 
							i.leftSpeed + "," + i.rightSpeed);
					bw.newLine();
				}
				bw.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}
}