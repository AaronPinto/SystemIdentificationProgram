package org.usfirst.frc.team1325.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.*;
import java.util.*;

@SuppressWarnings("unused")
public class Robot extends IterativeRobot {
	private static final TalonSRX _driveLeftFront = new TalonSRX(2);//Has mag encoder plugged in Positive = For
	private static final TalonSRX _driveLeftRear = new TalonSRX(3);
	private static final TalonSRX _driveRightFront = new TalonSRX(13);//Has mag encoder plugged in Negative = For
	private static final TalonSRX _driveRightRear = new TalonSRX(12);

	@Override
	public void robotInit() {
		_driveLeftFront.set(ControlMode.PercentOutput, 0.0);
		_driveLeftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		_driveLeftFront.setSensorPhase(true);
		_driveLeftFront.setNeutralMode(NeutralMode.Brake);
		_driveLeftFront.configContinuousCurrentLimit(40, 10);
		_driveLeftFront.configPeakCurrentLimit(60, 10);
		_driveLeftFront.configPeakCurrentDuration(50, 10);
		_driveLeftFront.enableCurrentLimit(true);
		_driveLeftFront.configVoltageCompSaturation(12.0, 10);
		_driveLeftFront.enableVoltageCompensation(true);
		_driveRightFront.set(ControlMode.PercentOutput, 0.0);
		_driveRightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		_driveRightFront.setSensorPhase(true);
		_driveRightFront.setNeutralMode(NeutralMode.Brake);
		_driveRightFront.configContinuousCurrentLimit(40, 10);
		_driveRightFront.configPeakCurrentLimit(60, 10);
		_driveRightFront.configPeakCurrentDuration(50, 10);
		_driveRightFront.enableCurrentLimit(true);
		_driveRightFront.configVoltageCompSaturation(12.0, 10);
		_driveRightFront.enableVoltageCompensation(true);
		_driveLeftRear.set(ControlMode.PercentOutput, 0.0);
		_driveLeftRear.setNeutralMode(NeutralMode.Brake);
		_driveLeftRear.configContinuousCurrentLimit(40, 10);
		_driveLeftRear.configPeakCurrentLimit(60, 10);
		_driveLeftRear.configPeakCurrentDuration(50, 10);
		_driveLeftRear.enableCurrentLimit(true);
		_driveLeftRear.configVoltageCompSaturation(12.0, 10);
		_driveLeftRear.enableVoltageCompensation(true);
		_driveRightRear.set(ControlMode.PercentOutput, 0.0);
		_driveRightRear.setNeutralMode(NeutralMode.Brake);
		_driveRightRear.configContinuousCurrentLimit(40, 10);
		_driveRightRear.configPeakCurrentLimit(60, 10);
		_driveRightRear.configPeakCurrentDuration(50, 10);
		_driveRightRear.enableCurrentLimit(true);
		_driveRightRear.configVoltageCompSaturation(12.0, 10);
		_driveRightRear.enableVoltageCompensation(true);
	}

	private double delta = 0;
	private Data[] logs = new Data[6000];
	{
		for(int i = 0; i < logs.length; i++)
			logs[i] = new Data();
	}

	class Data {
		Double leftFront;
		Double leftRear;
		Double rightFront;
		Double rightRear;
		int leftVelocityNativeUnits;
		int rightVelocityNativeUnits;
		Double time;
		
		Data() {
			this.leftFront = 0.0;
			this.leftRear = 0.0;
			this.rightFront = 0.0;
			this.rightRear = 0.0;
			this.leftVelocityNativeUnits = 0;
			this.rightVelocityNativeUnits = 0;
			this.time = 0.0;
		}
	}

	private Thread logThread = new Thread();
	private double avgdT, time = avgdT = 0.0;
	private int counter = 0;
	public void teleopInit() {
		logThread = new Thread(() -> {
			boolean secondSide = true;
			double start, start2, startTime = start = start2 = time = System.nanoTime();
			while(!Thread.interrupted()) {
				delta = (System.nanoTime() - startTime) / 1000000000;//While loop period
				if(((System.nanoTime() - start) / 1000000000) < 30) {//Which side
					if(delta >= 0.015) {
						startTime = System.nanoTime();
						avgdT += delta;
						counter++;
						//y = -sin(0.4x^2)
						Robot._driveRightFront.set(ControlMode.PercentOutput, -1 * Math.sin(0.4 * Math.pow((startTime - start2) /
								1000000000, 2)));
						Robot._driveRightRear.set(ControlMode.PercentOutput, -1 * Math.sin(0.4 * Math.pow((startTime - start2) /
								1000000000, 2)));
						logs[counter].leftFront = _driveLeftFront.getMotorOutputPercent();
						logs[counter].leftRear = _driveLeftRear.getMotorOutputPercent();
						logs[counter].rightFront = _driveRightFront.getMotorOutputPercent();
						logs[counter].rightRear = _driveRightRear.getMotorOutputPercent();
						logs[counter].leftVelocityNativeUnits = _driveLeftFront.getSelectedSensorVelocity(0);
						logs[counter].rightVelocityNativeUnits = _driveRightFront.getSelectedSensorVelocity(0); 
						logs[counter].time = (System.nanoTime() - time) / 1000000000;
					}
				} else if((((System.nanoTime() - start) / 1000000000) >= 30) && (((System.nanoTime() - start) / 1000000000) <= 60)) {
					if(secondSide) {
						secondSide = false;
						Robot._driveRightFront.set(ControlMode.PercentOutput, 0.0);
						Robot._driveRightRear.set(ControlMode.PercentOutput, 0.0);
						start2 = System.nanoTime();
					}
					if(delta >= 0.015) {
						avgdT += delta;
						counter++;
						startTime = System.nanoTime();
						Robot._driveLeftFront.set(ControlMode.PercentOutput, Math.sin(0.4 * Math.pow((startTime - start2) / 1000000000, 2)));
						Robot._driveLeftRear.set(ControlMode.PercentOutput, Math.sin(0.4 * Math.pow((startTime - start2) / 1000000000, 2)));
						logs[counter].leftFront = _driveLeftFront.getMotorOutputPercent();
						logs[counter].leftRear = _driveLeftRear.getMotorOutputPercent();
						logs[counter].rightFront = _driveRightFront.getMotorOutputPercent();
						logs[counter].rightRear = _driveRightRear.getMotorOutputPercent();
						logs[counter].leftVelocityNativeUnits = _driveLeftFront.getSelectedSensorVelocity(0);
						logs[counter].rightVelocityNativeUnits = _driveRightFront.getSelectedSensorVelocity(0); 
						logs[counter].time = (System.nanoTime() - time) / 1000000000;
					}
				} else {
					time = System.nanoTime();
					Robot._driveLeftFront.set(ControlMode.PercentOutput, 0.0);
					Robot._driveLeftRear.set(ControlMode.PercentOutput, 0.0);
					Robot._driveRightFront.set(ControlMode.PercentOutput, 0.0);
					Robot._driveRightRear.set(ControlMode.PercentOutput, 0.0);
				}
			}
		});
		logThread.start();
	}

	private Calendar c = Calendar.getInstance();
	public void disabledInit() {
		Robot._driveLeftFront.set(ControlMode.PercentOutput, 0.0);
		Robot._driveLeftRear.set(ControlMode.PercentOutput, 0.0);
		Robot._driveRightFront.set(ControlMode.PercentOutput, 0.0);
		Robot._driveRightRear.set(ControlMode.PercentOutput, 0.0);
		logThread.interrupt();

		if(logThread.isInterrupted()) {
			File f = new File("/home/lvuser/logs");
			if(!f.exists()) f.mkdir();
			try {
				BufferedWriter bw = new BufferedWriter(new FileWriter(String.format("/home/lvuser/logs/%tB%te%tY-%tl%tM%tS%tp.csv",
						c, c, c, c, c, c, c)));
				bw.write("leftFrontVolt, leftRearVolt, rightFrontVolt, rightRearVolt, leftSpeed, rightSpeed, time");
				bw.newLine();
				for(int i = 0; i < logs.length; i++) {
					bw.write(logs[i].leftFront + "," + logs[i].leftRear + ", " + logs[i].rightFront + 
							"," + logs[i].rightRear + "," + logs[i].leftVelocityNativeUnits + "," + logs[i].rightVelocityNativeUnits + ", "
							+ logs[i].time);
					bw.newLine();
				}
				bw.close();
				System.out.println("DONE " + logs.length + " " + avgdT / counter + " " + (System.nanoTime() - time) / 1000000000);
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}
}