package com.wefirst.ultimateascent;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class UltimateAscent extends SimpleRobot {
    
    int driveControl = Constants.TANK;

    Victor driveMotors[];
    Victor armWinch1;
    Victor armWinch2;
    Jaguar shooter;
    Victor angle;
    Victor climber;
    Servo feeder;
    RobotDrive driveTrain;
    Attack3Joystick joystickLeft;
    Attack3Joystick joystickRight;
    Attack3Joystick joystickWinch;
    AnalogChannel shooterEncoder;
    AnalogChannel winchPot;
    DigitalInput shooterLimitSwitch;
    boolean shoot = false;
    boolean recentShoot = false;
    boolean deWinch = false;
    double shootTime;
    double driveLim = 0.65d;

    public UltimateAscent() {
        super();
    }

    /**
     * Robot-wide initialization code should go here which will be called when
     * the robot is first powered on. Called exactly 1 time when the competition
     * starts.
     */
    protected void robotInit() {
        try {
            driveMotors = new Victor[4];
            driveMotors[0] = new Victor(cRIOPorts.LEFT_MOTOR_1);
            driveMotors[1] = new Victor(cRIOPorts.LEFT_MOTOR_2);
            driveMotors[2] = new Victor(cRIOPorts.RIGHT_MOTOR_1);
            driveMotors[3] = new Victor(cRIOPorts.RIGHT_MOTOR_2);
            driveTrain = new RobotDrive(driveMotors[0], driveMotors[1], driveMotors[2], driveMotors[3]);

            armWinch1 = new Victor(cRIOPorts.WINCH1);
            armWinch2 = new Victor(cRIOPorts.WINCH2);
            shooter = new Jaguar(cRIOPorts.SHOOTER);
            angle = new Victor(cRIOPorts.ANGLE);
            climber = new Victor(cRIOPorts.CLIMBER);
            feeder = new Servo(cRIOPorts.FEEDER);

            joystickLeft = new Attack3Joystick(cRIOPorts.LEFT_JOYSTICK);
            joystickRight = new Attack3Joystick(cRIOPorts.RIGHT_JOYSTICK);
            joystickWinch = new Attack3Joystick(cRIOPorts.WINCH_JOYSTICK);
            shooterEncoder = new AnalogChannel(cRIOPorts.SHOOTER_ENCODER);
            winchPot = new AnalogChannel(cRIOPorts.POTENTIOMETER);
            shooterLimitSwitch = new DigitalInput(cRIOPorts.SHOOTER_LIMIT_SWITCH);
        } catch (Exception any) {
            any.printStackTrace();
        }
    }

    /**
     * Operator control (tele-operated) code should go here. Users should add
     * Operator Control code to this method that should run while the field is
     * in the Operator Control (tele-operated) period.
     *
     * Called repeatedly while the robot is in the operator-controlled state.
     */
    public void operatorControl() {
        try {
            drive();
            //arm();
            shoot();
        } catch (Exception any) {
            any.printStackTrace();
        }
    }

    public void shoot() {
        double magnitude = -joystickWinch.getPower();
        shooter.set(magnitude);

        if (joystickWinch.getRawButton(3) && (shooterEncoder.getValue() > Constants.SHOOTER_UPPER_LIMIT || joystickWinch.getRawButton(9))) {//shooter up
            angle.set(1);
        } else if (joystickWinch.getRawButton(2) && (shooterEncoder.getValue() < Constants.SHOOTER_LOWER_LIMIT || joystickWinch.getRawButton(9))) {//shooter down
            angle.set(-0.8);
        } else {
            angle.set(0);
        }

        if (joystickWinch.getRawButton(1) && shoot == false) {
            shoot = true;
            recentShoot = true;
            shootTime = Timer.getFPGATimestamp();
        }

        if (!joystickWinch.getRawButton(9)) {
            if (shoot) {
                feeder.set(0);
                if (!shooterLimitSwitch.get()) {
                    recentShoot = false;
                }
                if ((!recentShoot && shooterLimitSwitch.get()) || shootTime + 3.0 <= Timer.getFPGATimestamp()) {
                    shoot = false;
                    feeder.set(0.5);
                }
            } else {
                feeder.set(0.5);
            }
        } else {
            if (joystickWinch.getRawButton(1)) {
                feeder.set(0);
            } else {
                feeder.set(0.5);
            }
        }
    }

    public void arm() {
        double armSpeed = joystickWinch.getY();
        if (armSpeed < 0.5 && armSpeed > -0.5) {
            armSpeed = 0;
        } else {
            deWinch = false;
        }

        if (armSpeed < 0 && (winchPot.getValue() >= Constants.POT_WINCH_UP) && !joystickWinch.getRawButton(9)) {
            armSpeed = 0;
        }

        if (joystickWinch.getRawButton(8)) {
            deWinch = true;
        }

        if (deWinch) {
            if (winchPot.getValue() <= Constants.POT_WINCH_UP) {
                armSpeed = -1;
            } else {
                armSpeed = 0;
                deWinch = false;
            }
        }

        armWinch1.set(armSpeed);
        armWinch2.set(armSpeed);

        if (joystickRight.getRawButton(2)) {//arm down
            climber.set(-1);
        } else if (joystickRight.getRawButton(3)) {//arm up
            climber.set(1);
        } else {
            climber.set(0);
        }
    }
    
    public double skim(double p) {
        double alpha = 0.6;
        if (p > 1.0) {
            return -((p - 1.0))*alpha;
        } else if (p < -1.0) {
            return -((p + 1.0))*alpha;
        }
        return 0.0;
    }

    public void drive() {
        /*if (!(joystickLeft.getRawButton(1) && joystickRight.getRawButton(1))) {
         leftSpeed *= driveLim;
         rightSpeed *= driveLim;
         }*/
        if (driveControl == Constants.TANK) {
            double leftSpeed = -joystickLeft.getY();
            double rightSpeed = -joystickRight.getY();

            if (leftSpeed < 0.2 && leftSpeed > -0.2) {
                leftSpeed = 0;
            }
            if (rightSpeed < 0.2 && rightSpeed > -0.2) {
                rightSpeed = 0;
            }
            driveTrain.tankDrive(leftSpeed, rightSpeed);
        }
        else if (driveControl == Constants.HALO)
        {
            double accelSpeed = joystickLeft.getY();
            double turnMod = joystickRight.getX();
            
            if (accelSpeed < 0.2 && accelSpeed > -0.2)
            {
                accelSpeed = 0;
            }
            
            double leftSpeed = -accelSpeed;
            double rightSpeed = -accelSpeed;
            
            if (turnMod < 0.2 && turnMod > -0.2)
            {
                turnMod = 0;
            }
            
            if (accelSpeed == 0) // turn on the spot
            {
                if (joystickRight.getRawButton(1))
                {
                    leftSpeed = turnMod;
                    rightSpeed = -turnMod;
                }
            }
            else
            {
                if (turnMod < 0)
                {
                    leftSpeed *= (1-Math.abs(turnMod));
                }
                else if (turnMod > 0)
                {
                    rightSpeed *= (1-Math.abs(turnMod));
                }
            }
            
            System.out.println("L: " + leftSpeed + " R: " + rightSpeed + " T: " + turnMod);
            
            driveTrain.tankDrive(leftSpeed, rightSpeed);
        }
        else if (driveControl == Constants.WCD)
        {
            double linSpeed = -joystickLeft.getY();
            double turnSpeed = joystickRight.getX();

            if (linSpeed < 0.2 && linSpeed > -0.2) {
                linSpeed = 0;
            }
            if (turnSpeed < 0.2 && turnSpeed > -0.2) {
                turnSpeed = 0;
            }
            double t_leftPWM = linSpeed + turnSpeed;
            double t_rightPWM = linSpeed - turnSpeed;
            double leftPWM = t_leftPWM + skim(t_rightPWM);
            double rightPWM = t_rightPWM + skim(t_leftPWM);
            driveTrain.tankDrive(leftPWM, rightPWM);
        } else if (driveControl == Constants.ARCADE) {
            double linSpeed = -joystickLeft.getY();
            double turnSpeed = -joystickLeft.getX();

            if (linSpeed < 0.2 && linSpeed > -0.2) {
                linSpeed = 0;
            }
            if (turnSpeed < 0.2 && turnSpeed > -0.2) {
                turnSpeed = 0;
            }
            driveTrain.arcadeDrive(linSpeed, turnSpeed);
        }
    }

    /**
     * Start a competition. This code tracks the order of the field starting to
     * ensure that everything happens in the right order. Repeatedly run the
     * correct method, either Autonomous or OperatorControl when the robot is
     * enabled. After running the correct method, wait for some state to change,
     * either the other mode starts or the robot is disabled. Then go back and
     * wait for the robot to be enabled again.
     */
    public void startCompetition() {
        robotInit();
        while (true) {
            if (isOperatorControl()) {
                operatorControl();
            } else {
                feeder.set(0.5);
                shooter.set(0);
                angle.set(0);
                driveTrain.tankDrive(0, 0);
                climber.set(0);
                armWinch1.set(0);
                armWinch2.set(0);
            }
        }
    }
}