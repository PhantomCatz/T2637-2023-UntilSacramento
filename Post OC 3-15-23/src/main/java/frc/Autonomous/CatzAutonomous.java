package frc.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.*;

public class CatzAutonomous 
{
    private final double SDS_L1_GEAR_RATIO = 8.14;       //SDS mk4i L1 ratio
    private final double SDS_L2_GEAR_RATIO = 6.75;       //SDS mk4i L2 ratio

    private final double DRV_S_GEAR_RATIO = SDS_L1_GEAR_RATIO; //set to which modules are being used

    private final double DRV_S_THREAD_PERIOD = 0.02;

    private final double TALONFX_INTEGRATED_ENC_CNTS_PER_REV = 2048.0;

    private final double DRVTRAIN_WHEEL_DIAMETER             = 4.0;
    private final double DRVTRAIN_WHEEL_CIRCUMFERENCE        = (Math.PI * DRVTRAIN_WHEEL_DIAMETER);
    private final double DRVTRAIN_ENC_COUNTS_TO_INCH         = DRVTRAIN_WHEEL_CIRCUMFERENCE / TALONFX_INTEGRATED_ENC_CNTS_PER_REV / DRV_S_GEAR_RATIO;

    //drive straight variables
    //drive power
    private final double DRV_S_KP = 0.02; //TBD

    private final double DRV_S_THRESHOLD_INCH = 0.5;
    private final double DRV_S_MIN_POWER      = 0.1;
    private final double DRV_S_MAX_POWER      = 1.0;

    //turn power
    private final double DRV_S_ERROR_GAIN    = 0.05;
    private final double DRV_S_RATE_GAIN     = 0.001;

    //turn in place variables
    private final static double PID_TURN_THRESHOLD   = 1.25;

	private final static double PID_TURN_IN_PLACE_KP = 0.008;
    
    private final static double TURN_DRIVE_MAX_POS_POWER  =  0.4;
	private final static double TURN_DRIVE_MAX_NEG_POWER  = -0.4;

    private final static double TURN_DRIVE_MIN_POWER = 0.1;

    private final static double TURN_IN_PLACE_PERIOD = 0.010;


	private static double turnCurrentError; 

	private static double turnCurrentAngle;
	private static double turnTargetAngle;

    private Timer autonTimer;
    public CatzLog data;



    public CatzAutonomous()
    {
        autonTimer = new Timer();
    }



    /*-----------------------------------------------------------------------------------------
    *
    *  DriveStraight
    *
    *----------------------------------------------------------------------------------------*/
	
    /**
     * Method to drive straight autonomously
     * 
     * 
     * 
     * @param distanceInch The distance to travel in inches (positive distance will drive forward field relative and negative distance will drive backward field relative)
     * 
     * @param directionDeg The direction which the robot will translate 
     * 
     * @param maxTime      The timeout time
     */


    /*      -----------------
     *     |
     *      -----------
     *   →  |  ↓ (=)    _____
     *     _|          |     |
     *   →  |  ↑ (^)   |     |  
     *     _|          |_____|
     *   →  | --> (+)
     *      | <-- (-)
     *      -------------------------------     
     * 
     *   Key:
     *      (^) == directed +90 degrees  
     *      (=) == directed -90 degrees
     *      (+) == directed positive distance
     *      (-) == directed negative distance
     * 
     *       →  == driver's POV
     */

    public void DriveStraight(double distanceInch, double directionDeg, double maxTime)
    {
        double distanceRemainInch    = 0.0;
        double distanceRemainAbsInch = 0.0;

        double drvPower      = 0.0;
        double drvPowerKp    = 0.0;
        double drvPowerClamp = 0.0;

        double turnPower    = 0.0;
        double angleKpPower = 0.0;
        double angleKdPower = 0.0;

        double angleError     = 0.0;
        double prevAngleError = 0.0;
        double angleErrorRate = 0.0;

        double time     = 0.0;
        double prevTime = -1.0;

        double deltaPositionCnts  = 0.0;

        double currentAngle     = 0.0;
        double startingAngle    = 0.0;

        boolean done = false;

        Robot.drivetrain.LT_FRNT_MODULE.resetDrvDistance(); // Reset the encoder position so the calculation is easier.
        deltaPositionCnts = 0.0;

        startingAngle         = Robot.navX.getAngle();
        distanceRemainInch    = distanceInch;
        distanceRemainAbsInch = Math.abs(distanceRemainInch);

        autonTimer.reset();
        autonTimer.start();

        while(!done)
        {
            time = autonTimer.get();

            if(time > maxTime)
            {
		//We've exceeded the timeout, so stop driving    
                done = true;
                Robot.drivetrain.translateTurn(directionDeg, 0.0, 0.0, Robot.drivetrain.getGyroAngle());
            }
            else
            {
		//Did not exceed the timeout yet, so continue driving
                currentAngle = Robot.navX.getAngle();
                
                angleError = startingAngle - currentAngle;

		    
		/*
                 * On the first iteration, delta time will be a really small value, So when you are calculating the angle error rate, 
                 * you will be dividing the angle error with a really really small number,
                 * resulting in a really big angle error rate, which we do not want. 
                 * 
                 * 
                 * On the first iteration, angle error shouldn't really occur, because it is only a fraction of a millisecond. 
                 * By initializing previous time as -1, we can use that to determine if it's the first iteration. If it is the first iteration,
                 * we will set the angle error rate to zero.
                 */ 
                if(prevTime == -1.0)
                {
                    angleErrorRate = 0.0;
                }
                else
                {
                    angleErrorRate = (angleError - prevAngleError) / (time - prevTime);
                }
    
                deltaPositionCnts   = Robot.drivetrain.LT_FRNT_MODULE.getDrvDistance();
                distanceRemainInch    = distanceInch - (deltaPositionCnts * DRVTRAIN_ENC_COUNTS_TO_INCH);
                distanceRemainAbsInch = Math.abs(distanceRemainInch);

                if(distanceRemainAbsInch <= DRV_S_THRESHOLD_INCH)
                {
		    //The distance remaining is less than the stop threshold, then it means you have arrived at your destination. So you break out of the loop
			
                    done = true;
                    Robot.drivetrain.translateTurn(directionDeg, 0.0, 0.0, Robot.drivetrain.getGyroAngle());
                }
                else
                {
		    //You are still not at the destination, so calculate new motor power
			
                    drvPowerKp    = (DRV_S_KP * distanceRemainAbsInch);                 //Multiply the distance remaining with KP to get the power.
                    drvPowerClamp = Clamp(DRV_S_MIN_POWER, drvPowerKp, DRV_S_MAX_POWER);//Clamp the power between the min and max power.
                    drvPower      = drvPowerClamp * Math.signum(distanceInch);          //Get the sign of inputted distance so you know whether to go forward or backward.


		    //Calculates how much to turn the wheels in order to go in a straight line, depending on its angle error.
                    angleKpPower = DRV_S_ERROR_GAIN * angleError;
                    angleKdPower = DRV_S_RATE_GAIN  * angleErrorRate;

                    turnPower = Clamp(-1.0, angleKpPower + angleKdPower, 1.0);

                    Robot.drivetrain.translateTurn(directionDeg, drvPower, turnPower, Robot.drivetrain.getGyroAngle());

                    prevTime       = time;
                    prevAngleError = angleError;
                }
            }

            if(DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_DRV_STRAIGHT)
            {
                data = new CatzLog(time, deltaPositionCnts, distanceRemainInch,
                                                            drvPowerKp,
                                                            drvPowerClamp, 
                                                            drvPower, 
                                                            currentAngle,
                                                            angleError, 
                                                            angleErrorRate, 
                                                            angleKpPower,
                                                            angleKdPower,
                                                            turnPower,
                                                            Robot.drivetrain.LT_FRNT_MODULE.getAngle(),
                                                            0.0, 0.0, 0);  
                Robot.dataCollection.logData.add(data);
            }

            Timer.delay(DRV_S_THREAD_PERIOD);
        }
	
	//Broke out of the loop, meaning the robot as reached its destination, therefore setting the wheel power to zero.
        Robot.drivetrain.translateTurn(directionDeg, 0.0, 0.0, Robot.drivetrain.getGyroAngle());
    }


    /*-----------------------------------------------------------------------------------------
    *
    *  TurnInPlace
    *
    *----------------------------------------------------------------------------------------*/
    public void TurnInPlace(double degreesToTurn, double timeoutSeconds)
    {
        boolean turnInPlaceDone = false;

        double  currentTime       = 0.0;
        double  angleRemainingAbs = 999.0;
        double  turnPower = 0.0;

        autonTimer.reset();
        autonTimer.start(); 

        turnCurrentAngle  = Robot.navX.getAngle();
        turnTargetAngle   = degreesToTurn + turnCurrentAngle;

        while (turnInPlaceDone == false)
        {
            currentTime  = autonTimer.get();
            turnCurrentAngle = Robot.navX.getAngle();
    
            // calculate angle error
            turnCurrentError      = turnTargetAngle - turnCurrentAngle;
            angleRemainingAbs = Math.abs(turnCurrentError);

            if (angleRemainingAbs < PID_TURN_THRESHOLD) 
            { 
		//angle remaining is below the threshold, which means turn in place is done, so stop.
                turnInPlaceDone = true;
                Robot.drivetrain.rotateInPlace(0.0);
            }
            else
            {
                if (currentTime > timeoutSeconds) 
                {
                    turnInPlaceDone = true;
                    Robot.drivetrain.rotateInPlace(0.0);
                } 
                else
                {
                    turnPower = turnCurrentError * PID_TURN_IN_PLACE_KP;

                    //Clamp
                    //MAX POWER
                    if(turnPower >= TURN_DRIVE_MAX_POS_POWER)
                    {
                        turnPower = TURN_DRIVE_MAX_POS_POWER;
                    } 
                    else if(turnPower <= TURN_DRIVE_MAX_NEG_POWER)
                    {
                        turnPower = TURN_DRIVE_MAX_NEG_POWER;
                    }

                    //MIN POWER
                    if (Math.abs(turnPower) < TURN_DRIVE_MIN_POWER)
                    {
                        turnPower = Math.signum(turnPower) * TURN_DRIVE_MIN_POWER;
                    }
                    
                    Robot.drivetrain.rotateInPlace(turnPower);
                }
            }

            if(DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_TURN_IN_PLACE)
            {
            data = new CatzLog(currentTime, turnCurrentAngle, turnCurrentError, turnPower, 
                                -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999);

            Robot.dataCollection.logData.add(data);
            }

            Timer.delay(TURN_IN_PLACE_PERIOD);
        }
    }

    public double Clamp(double min, double in, double max)
    {
        if(in > max)
        {
            return max;
        }
        else if(in < min)
        {
            return min;
        }
        else
        {
            return in;
        }
    }
}
