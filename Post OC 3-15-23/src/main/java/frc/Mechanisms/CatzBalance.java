package frc.Mechanisms;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.Robot;

public class CatzBalance
{
    public static Timer timer = new Timer();
    public static double prevTime = 0.0;
    public static double time = 0.0;

    public  Boolean startBalance = false;

    public static double prevPitchAngle = 0.0;
    public static double pitchAngle = 0.0;
    public static double angleRate = 0.0;
    public static double power = 0.0;
    public static double pitchTerm = 0.0;
    public static double rateTerm = 0.0;

    public static CatzLog data;

    public final double ANG_SLOWBAND = 10.0; 
    public final double ANG_GAIN = 0.015;
    public final double RATE_GAIN = 0.0075;
    public final double MAX_POWER = 0.175;
    public final double BALANCE_THREAD_PERIOD = 0.02;

    public CatzBalance()
    {
        AutoBalance();
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

    public void AutoBalance()
    {
        final Thread balanceThread = new Thread()
        {
            public void run()
            {
                timer.reset();
                timer.start();

                if(Robot.dataCollection.getLogDataID() == Robot.dataCollection.LOG_ID_BALANCE)
                {
                    data = new CatzLog(ANG_SLOWBAND, ANG_GAIN, RATE_GAIN, MAX_POWER, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);  
                    Robot.dataCollection.logData.add(data);
                }

                while(true)
                {
                    
                    if(startBalance)
                    {
                        System.out.println("B");
                        time = timer.get();

                        pitchAngle = Robot.navX.getPitch(); //check sign when changing to getRoll
                        angleRate = (pitchAngle - prevPitchAngle)/(time - prevTime);

                        // PID without the I
                        pitchTerm = pitchAngle * ANG_GAIN;
                        rateTerm = angleRate * RATE_GAIN;

                        power = Clamp(-MAX_POWER, pitchTerm + rateTerm, MAX_POWER);
                        if(Math.abs(pitchAngle) < ANG_SLOWBAND)
                        {
                            power = power/2;
                        }

                        Robot.drivetrain.drive(0.0, power, 0.0);

                        prevPitchAngle = pitchAngle;
                        prevTime = time;

                        //if(Robot.dataCollection.getLogDataID() == DataCollection.LOG_ID_BALANCE){
                            data = new CatzLog(time, pitchAngle, angleRate, power, pitchTerm, rateTerm, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);  
                            Robot.dataCollection.logData.add(data);
                        //}
                    }
                    Timer.delay(BALANCE_THREAD_PERIOD);
                }
            }
        };

        balanceThread.start();
    }

    public void StartBalancing()
    {
        System.out.println("start");
        timer.reset();
        timer.start();
        
        prevTime = 0.0;
        prevPitchAngle = 0.0;

        startBalance = true;
    }

    public void StopBalancing()
    {
        startBalance = false;
        System.out.println("stopped");
    }

    public void SmartDashboardBalanceDebug(){
        SmartDashboard.putNumber("Pitch Rate", angleRate);
        SmartDashboard.putNumber("Power", power);
    }

    public void SmartDashboardBalance(){
        //SmartDashboard.putNumber("Pitch", pitchAngle);
    }
}