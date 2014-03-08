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
    private double speed=0;
    private Timer gyroTimer = new Timer();
    private boolean gyroCalibrate = false;
      
    private double nonSuperModeSpeedMult;
    private double nonSuperModeTurnMult;     
    
    private AxisCamera cam;
    private DriverStationEnhancedIO m_dsIO;
    private DriverStation m_driverStation;
    private DriverStationLCD m_dsLCD;
      
    private Joystick joystickTurn;
    private Joystick joystickDrive;
    private Joystick joystickGuido;
    
    private Solenoid sDepressurizeShooter;
    private Solenoid sPressurizeShooter;
    private Solenoid sDisablePTO;
    private Solenoid sEnablePTO;
    
    private Solenoid sGearHigh; 
    private Solenoid sGearLow; 
    
    private Solenoid sShooterShoot;
    private Solenoid sShooterNoShoot;
    
    private Relay spike1;
    
    private SpeedController frontLeft;
    private SpeedController frontRight;
    private SpeedController backRight;
    private SpeedController backLeft;
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
    
    private int intShoot = 0;
    
    private static final int jsButtonPressurize = 2;
    private static final int jsButtonTurbo = 3;
    private static final int jsButtonPTO = 2;
    private static final int jsButtonWinchUp = 1;
    private static final int jsButtonWinchDown = 1;
    private static final int jsButtonFullGuidoPower = 6;
    private static final int jsButtonShoot = 1;
    private static final int jsButtonLeftWall = 4;
    private static final int jsButtonRightWall = 5;
    
    private static final double GUIDO_RAISE_POWER = 1;
    private long time;
    private long startTime;
    
    
    
    
    public MechanumDrive()
    {
 
        joystickDrive = new Joystick(1);
        joystickTurn = new Joystick(2);
        joystickGuido = new Joystick(3);
             
        
        frontLeft = new Talon(2);
        frontRight = new Talon(4);
        backRight = new Talon(3);
        backLeft = new Talon(1);
        
        raiseGuido = new Talon(5);
        
        
        sDepressurizeShooter = new Solenoid(7);
        sPressurizeShooter = new Solenoid(8);
        sDisablePTO = new Solenoid(2);
        sEnablePTO = new Solenoid(1);
        
        sGearHigh = new Solenoid(6); 
        sGearLow = new Solenoid(5);
        
        sShooterShoot = new Solenoid(4);
        sShooterNoShoot = new Solenoid(3);
                
        
        m_driverStation = DriverStation.getInstance();
        m_dsIO = m_driverStation.getEnhancedIO();
        m_dsLCD = DriverStationLCD.getInstance();
                 
    }
    
    
    public void robotInit() {
        //calibrateGyro();
        
        compressor.start();
        
        sPressurizeShooter.set(true);
        sDepressurizeShooter.set(false);
        
        sEnablePTO.set(true);
        sDisablePTO.set(false);
        
        highGear = false;
        automatic = false;
        sGearLow.set(false);
        sGearHigh.set(true);
        
        //cam = AxisCamera.getInstance();
    }
    
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {        
        double xx;
        double yy;
        double zz;
        double guidoRaise;
        nonSuperModeSpeedMult = (m_driverStation.getAnalogIn(1)/6.6) + .5;
        nonSuperModeTurnMult = 1;//(m_ds.getAnalogIn(2)/6.6) + .5;
        
        
        xx = MathUtil.cube(joystickDrive.getX());
        yy = MathUtil.cube(joystickDrive.getY());
        zz = MathUtil.cube(joystickTurn.getX());
        guidoRaise = MathUtil.cube(joystickGuido.getY());
        
        //joystick "jitter" removal
        xx = MathUtil.removeJitter(xx);
        yy = MathUtil.removeJitter(yy);
        zz = MathUtil.removeJitter(zz);
        guidoRaise = MathUtil.removeJitter(guidoRaise);
        
        xx *= nonSuperModeSpeedMult;
        yy *= nonSuperModeSpeedMult;
        zz *= nonSuperModeTurnMult;
        
        m_dsLCD.println(DriverStationLCD.Line.kUser2, 1, ""+guidoRaise);
        m_dsLCD.println(DriverStationLCD.Line.kUser3, 1, xx + " " + yy + " " + zz);
        m_dsLCD.updateLCD();
        
        
        raiseGuido(guidoRaise);
        
        //solenoid code for shooter
        pressurizeShooter(joystickGuido.getRawButton(jsButtonPressurize));
        
        sShooterShoot.set(joystickGuido.getRawButton(jsButtonShoot));
        sShooterNoShoot.set(!joystickGuido.getRawButton(jsButtonShoot));
       
        boolean enablePTO = joystickTurn.getRawButton(jsButtonPTO);
        
        //code for power takeoff
        ptoSwitch(enablePTO);
        
        //tramission shifting
        highGear = transmission(highGear, joystickDrive.getRawButton(jsButtonTurbo));
        
        if (!enablePTO) {
            jesterDrive(zz, xx, -yy, highGear);
        }
        else {
            //Tank drive
            tankDrive(MathUtil.removeJitter(MathUtil.cube(joystickTurn.getY())), MathUtil.removeJitter(MathUtil.cube(joystickDrive.getY())));           
            
            doPTO((joystickDrive.getRawButton(jsButtonWinchDown) ? -0.5 : 0) + (joystickTurn.getRawButton(jsButtonWinchUp) ? 0.5 : 0));            
        }
    }
    public void autonomousInit() {
        
        startTime = time = System.currentTimeMillis();
        ptoSwitch(false);
    }
    public void autonomousPeriodic() {
        m_dsLCD.println(DriverStationLCD.Line.kUser1, 1, ""+(time - startTime));
        time = System.currentTimeMillis();
       
        if (time - startTime < 3000L) {
            jesterDrive(0, 0, 0.5, false);
        }
        else {
            jesterDrive(0,0,0,false);
        }
        m_dsLCD.updateLCD();
    }
    
       
    private void raiseGuido(double amt) {
        if (joystickGuido.getRawButton(jsButtonFullGuidoPower))
        {
            //raiseGuido.set((amt / Math.abs(amt)) * GUIDO_RAISE_POWER);
            raiseGuido.set(amt * GUIDO_RAISE_POWER);
        }
        else
        {
            raiseGuido.set(amt * GUIDO_RAISE_POWER * 0.5);
        }
    }
    
    private boolean transmission(boolean highGear, boolean turbo) {
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
        return highGear;
    }
    
    private void pressurizeShooter(boolean current) {
        if (current != bShootLast)
        {
            if (current)
            {
                intShoot = 1 - intShoot;
            }
        }
        if (current)
        {
            intShoot=1;
        }
        else
        {
            intShoot=0;
        }
        
        bShootLast=current;
        if (intShoot != 0)
        {
            sDepressurizeShooter.set(false);
            sPressurizeShooter.set(true);
        } 
        else 
        {
            sDepressurizeShooter.set(true);
            sPressurizeShooter.set(false);
        }
    }
    private void ptoSwitch(boolean enable) {
        sDisablePTO.set(!enable);
        sEnablePTO.set(enable);
    }
    
    private void doPTO(double speed) {
        backLeft.set(speed*-1 * (!joystickDrive.getRawButton(jsButtonLeftWall) ? 1 : 0));
        backRight.set(speed * (!joystickDrive.getRawButton(jsButtonRightWall) ? 1 : 0));
    }
    private void tankDrive(double left, double right) {
        frontLeft.set(left*-1);
        frontRight.set(right);
    }
    
    public void jesterDrive(double joyX, double joyY, double joyZ, boolean highGear) {
        
        
        double leftFrontX;
        double leftFrontY;
        double leftFrontZ;
        
        double rightRearX;
        double rightFrontX;
        double leftRearX;
        
        double reductionFactor = 0;
        if (!MathUtil.compareDouble(joyX, 0.0, 0.01)) {
            reductionFactor++;            
        }
        if (!MathUtil.compareDouble(joyY, 0.0, 0.01)) {
            reductionFactor++;            
        }
        if (!MathUtil.compareDouble(joyZ, 0.0, 0.01)) {
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
    
}