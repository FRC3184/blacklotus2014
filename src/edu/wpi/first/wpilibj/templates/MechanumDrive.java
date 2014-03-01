/*----------------------------------------------------------------------------
 * Copyright (c) FIRST 2008. All Rights Reserved.                             
 * Open Source Software - may be modified and shared by FRC teams. The code   
 * must be accompanied by the FIRST BSD license file in the root directory of 
 * the project.                                                               
 *----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.camera.AxisCamera;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class MechanumDrive extends IterativeRobot {
    private Gyro gyro = new Gyro(1);
    private double gyroDriftRate=0;
    private double previousGyro=0;
    private double previousGyroTime=0;
    private double gyroStartTime=0;
    private double Speed=0;
    private Timer gyroTimer = new Timer();
    private boolean gyroCalibrate = false;
      
    private double nonSuperModeSpeedMult;
    private double nonSuperModeTurnMult;     
    
    private AxisCamera cam;
    private DriverStationEnhancedIO m_dsIO;
    private DriverStation m_driverStation;
    private DriverStationLCD m_dsLCD;
      
    private Joystick joystick_turn;
    private Joystick joystick_drive;
    private Joystick joystick_guido;
    
    private Solenoid s1;
    private Solenoid s2;
    private Solenoid s3;
    private Solenoid s4;
    
    private Solenoid sGearHigh; 
    private Solenoid sGearLow; 
    
    private Relay spike1;
    
    private SpeedController frontLeft;
    private SpeedController frontRight;
    private SpeedController backRight;
    private SpeedController backLeft;
    private SpeedController spinGuido;
    private SpeedController raiseGuido;
    
    private boolean highGear = false;
    private double backLow = 0.082;
    private double frontLow = 0.0941;
    private double backHigh = 0.21;
    private double frontHigh = 0.3;
    
    private boolean automatic = false;
    
    private double averageSpeed = 0;
    private double upShiftSpeed = 45.0;
    private double downShiftSpeed = 44.0;
    
    private Compressor compressor = new Compressor(13,1);
        
    private boolean bShootLast = false;
    private boolean bShootCurr = false;
    
    private int intShoot = 0;
    private int jsButtonShoot = 5;
    private int jsButtonTurbo = 3;
    private int jsGuidoButton = 3;
    private int jsButtonGuidoInverse = 1;
    private int jsButtonPTO = 2;
    private int jsButtonWinchUp = 1;
    private int jsButtonWinchDown = 1;
    
    public MechanumDrive()
    {
 
        joystick_drive = new Joystick(1);
        joystick_turn = new Joystick(2);
        joystick_guido = new Joystick(3);
             
        
        frontLeft = new Talon(2);
        frontRight = new Talon(3);
        backRight = new Talon(4);
        backLeft = new Talon(1);
        spinGuido = new Talon(5);
        raiseGuido = new Talon(6);
        
        
        s1 = new Solenoid(1);
        s2 = new Solenoid(2);
        s3 = new Solenoid(3);
        s4 = new Solenoid(4);
        
        sGearHigh = new Solenoid(5); 
        sGearLow = new Solenoid(6);
        
        
        m_driverStation = DriverStation.getInstance();
        m_dsIO = m_driverStation.getEnhancedIO();
        m_dsLCD = DriverStationLCD.getInstance();
        
    }
    
    public double gyroGet(){
        double gyroFix = gyroDriftRate*(gyroTimer.get()-previousGyroTime);
        previousGyroTime=gyroTimer.get();
        return (gyro.getAngle()-gyroFix);
    }
    
    
    public void calibrateGyro()
    {
        if(gyroCalibrate==false){
        
             gyroCalibrate=true;
             gyroTimer.stop();
             gyroTimer.reset();
             gyroTimer.start();
             previousGyro=gyro.getAngle();
        }
        else
        {
            if(gyroTimer.get()>5.0)
            {
                gyroDriftRate=(gyro.getAngle()-previousGyro)/gyroTimer.get();
                gyroCalibrate=false;
                gyroTimer.stop();
                gyroTimer.reset();
                gyroTimer.start();
                previousGyroTime=gyroTimer.get();
                gyro.reset();
            }
        }
        
        
        
        
    }
    
    public void robotInit() {
        //calibrateGyro();
        
        compressor.start();
        
        s2.set(true);
        s1.set(false);
        
        s4.set(true);
        s3.set(false);
        
        highGear = false;
        automatic = false;
        sGearLow.set(false);
        sGearHigh.set(true);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    
    public void teleopInit()
    {
        //calibrateGyro();
    }
    
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        double xx;
        double yy;
        double zz;
        double guidoRaise;
        double raisePwr;
        double spinPwr;
        nonSuperModeSpeedMult = (m_driverStation.getAnalogIn(1)/6.6) + .5;
        nonSuperModeTurnMult = 1;//(m_ds.getAnalogIn(2)/6.6) + .5;
        
        
        xx = cube(joystick_drive.getX());
        yy = cube(joystick_drive.getY());
        zz = cube(joystick_turn.getX());
        guidoRaise = cube(joystick_guido.getY());
        
        raisePwr = -0.75;//(joystick_guido.getRawAxis(3)+1) / 2;
        spinPwr = 0.4;//(joystick_drive.getRawAxis(3)+1) / 2;
        
        //joystick "jitter" removal
        if(Math.abs(xx)<.05){
            xx=0;}
        if(Math.abs(yy)<.05){
            yy=0;}
        if(Math.abs(zz)<.05){
            zz=0;}
        
        xx = xx * nonSuperModeSpeedMult;
        yy = yy * nonSuperModeSpeedMult;
        zz = zz * nonSuperModeTurnMult;
        
        m_dsLCD.println(DriverStationLCD.Line.kUser2, 1, spinPwr + ":" + raisePwr + ":" + guidoRaise);
        m_dsLCD.println(DriverStationLCD.Line.kUser3, 1, xx + " " + yy + " " + zz);
        m_dsLCD.updateLCD();
        //m_robotDrive.mecanumDrive_Cartesian(xx, yy, zz, 0); //gyro.getAngle());
        //m_robotDrive.mecanumDrive_Polar(yy, xx, zz);
        
        
        // code for quido arms
        if (joystick_guido.getRawButton(jsGuidoButton)){
            spinGuido.set(spinPwr * (joystick_guido.getRawButton(jsButtonGuidoInverse) ? 1 : -1));
        }
        else{
            spinGuido.set(0);
        }
        
        raiseGuido.set(-guidoRaise * raisePwr);
        
        //solenoid code for shooter       
        bShootCurr = joystick_guido.getRawButton(jsButtonShoot);
        if (bShootCurr != bShootLast)
        {
            if (bShootCurr)
            {
                intShoot = 1 - intShoot;
            }
        }
        if (bShootCurr)
        {
            intShoot=1;
        }
        else
        {
            intShoot=0;
        }
        
        bShootLast=bShootCurr;
        if (intShoot != 0)
        {
            s1.set(false);
            s2.set(true);
        } 
        else 
        {
            s1.set(true);
            s2.set(false);
        }
        
        boolean enablePTO = joystick_turn.getRawButton(jsButtonPTO);
        
        //code for power takeoff
        if (enablePTO)
        {
            s3.set(false);
            s4.set(true);
        } 
        else 
        {
            s3.set(true);
            s4.set(false);
        }
        
        //tramission shifting
        boolean turbo = joystick_drive.getRawButton(jsButtonTurbo);        
        if(highGear) {
            if((automatic && averageSpeed < downShiftSpeed) ||
                    (!automatic && turbo)){
                sGearHigh.set(true);
                sGearLow.set(false);
                highGear = false;
            }
        }
        else {
            if((automatic && averageSpeed > upShiftSpeed) ||
                    (!automatic && !turbo)){
                sGearHigh.set(false);
                sGearLow.set(true);
                highGear = true;
            }
        }
        if (highGear){
            sGearHigh.set(true);
            sGearLow.set(false);
        }
        else{
            sGearHigh.set(false);
            sGearLow.set(true);
        }
        
        if (!enablePTO) {
            jesterDrive(yy, -xx, zz, highGear);
        }
        else {
            //Tank drive
            frontLeft.set(cube(joystick_turn.getY()*-1));
            frontRight.set(cube(joystick_drive.getY()));
            
            double speed = (joystick_drive.getRawButton(jsButtonWinchDown) ? -0.5 : 0) + (joystick_turn.getRawButton(jsButtonWinchUp) ? 0.5 : 0);
            backLeft.set(speed*-1);
            backRight.set(speed);
        }
    }
    
    public void jesterDrive(double joyX, double joyY, double joyZ, boolean highGear) {
        
        
        double leftFrontX;
        double leftFrontY;
        double leftFrontZ;
        
        double rightRearX;
        double rightFrontX;
        double leftRearX;
        
        double reductionFactor = 0;
        if (!compareDouble(joyX, 0.0, 0.01)) {
            reductionFactor++;            
        }
        if (!compareDouble(joyY, 0.0, 0.01)) {
            reductionFactor++;            
        }
        if (!compareDouble(joyZ, 0.0, 0.01)) {
            reductionFactor++;            
        }
                
        if (reductionFactor == 0) {
            frontLeft.set(0.0);
            frontRight.set(0.0);
            backLeft.set(0.0);
            backRight.set(0.0);
            return;
        }
        else {
            leftFrontX = joyX / reductionFactor;
            leftFrontY = joyY / reductionFactor;
            leftFrontZ = joyZ / reductionFactor;
        }
        
        rightRearX = -leftFrontX;
        rightFrontX = leftFrontX;
        leftRearX = -leftFrontX;
        
        double rightRearY;
        double rightFrontY;
        double leftRearY = rightFrontY = rightRearY = leftFrontY;
        
        double rightRearZ = leftFrontZ;
        double rightFrontZ = -leftFrontZ;
        double leftRearZ = -leftFrontZ;
        
        //lfX *= -1;
        //rfX *= -1;
        
        double dFrontLeft = (leftFrontX + leftFrontY + leftFrontZ);//*-1;
        double dBackLeft = (leftRearX + leftRearY + leftRearZ);//*-1;
        double dFrontRight = (rightFrontX + rightFrontY + rightFrontZ);//*-1;
        double dBackRight = (rightRearX + rightRearY + rightRearZ);//*-1;
        
        
        String ln1 = "";
        String ln2 = "";
        String ln3 = "";
        String ln4 = "";
        String ln5 = "";
        
        //Modify values here
        if (highGear) {
            
            dBackRight *= 1;
            dBackLeft *= 1;
            
            dFrontLeft *= (backHigh / frontHigh);
            dFrontRight *= (backHigh / frontHigh);
        }
        else {
                       
            dBackRight *= 1;
            dBackLeft *= 1;
            
            dFrontRight *= (backLow / frontLow);
            dFrontLeft *= (backLow / frontLow);
        }
        frontLeft.set(dFrontLeft);
        backLeft.set(dBackLeft);
        frontRight.set(dFrontRight);
        backRight.set(dBackRight);

        
        
//        ln1 = String.format("%01.3f %2$01.3f %3$01.3f", joyX, joyY, joyZ);
//        ln2 = String.format("%01.3d %01.3d %01.3d %01.3d", lfX, lfY, lfZ, dFrontLeft);
//        ln3 = String.format("%01.3d %01.3d %01.3d %01.3d", lrX, lrY, lrZ, dBackLeft);
//        ln4 = String.format("%01.3d %01.3d %01.3d %01.3d", rfX, rfY, rfZ, dFrontRight);
//        ln5 = String.format("%01.3d %01.3d %01.3d %01.3d", rrX, rrY, rrZ, dBackRight);
//        
//        m_dsLCD.println(DriverStationLCD.Line.kUser1, 1, ln1);
//        m_dsLCD.println(DriverStationLCD.Line.kUser2, 1, ln2);
//        m_dsLCD.println(DriverStationLCD.Line.kUser3, 1, ln3);
//        m_dsLCD.println(DriverStationLCD.Line.kUser4, 1, ln4);
//        m_dsLCD.println(DriverStationLCD.Line.kUser5, 1, ln5);
                
//        m_dsLCD.println(DriverStationLCD.Line.kUser1, 1, ""+dBackLeft);
//        m_dsLCD.println(DriverStationLCD.Line.kUser2, 1, ""+dBackRight);
//        m_dsLCD.println(DriverStationLCD.Line.kUser3, 1, ""+lrX + " " + lrY + " " + lrZ);
//        m_dsLCD.println(DriverStationLCD.Line.kUser4, 1, ""+joyX+ " " + joyY + " " + joyZ);
        m_dsLCD.println(DriverStationLCD.Line.kUser5, 1, "" + dFrontLeft + ":" + dBackLeft + ":"+ dFrontRight + ":"+ dBackRight);
        m_dsLCD.updateLCD();
    }
    
    private boolean compareDouble(double d1, double d2, double diff) {
        return Math.abs(d1-d2) < diff;
    }

    private double cube(double d) {
        return d*d*d;
    }
    
}