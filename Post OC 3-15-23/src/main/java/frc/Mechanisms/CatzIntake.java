package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.Robot;

public class CatzIntake {

  private Thread intakeThread;
  private final double INTAKE_THREAD_PERIOD  = 0.020;
  
  private CatzLog data;

  //------------------------------------------------------------------------------------------------
  //
  //  Roller
  //
  //------------------------------------------------------------------------------------------------
  private WPI_TalonFX intakeRollerMotor;

  private final int    INTAKE_ROLLER_MC_ID           = 11; 

  private final double INTAKE_MOTOR_POWER_ROLLER_IN  = 1.0; //-1.0
  private final double INTAKE_MOTOR_POWER_ROLLER_OUT =  -1.0; //1.0  may be fixing wiring issue on motor controller (original values used to work)
  private final double INTAKE_MOTOR_POWER_OFF =  0.0;

  private final double INTAKE_INPUT_THRESHOLD = 0.2;

  private final int INTAKE_ROLLER_OFF = 0;  //TBD to see if still needed
  private final int INTAKE_ROLLER_IN  = 1;
  private final int INTAKE_ROLLER_OUT = 2;
  private final int INTAKE_ROLLER_UNINITIALIZED = -999;

  private int   intakeRollerState = INTAKE_ROLLER_OFF;


  //------------------------------------------------------------------------------------------------
  //
  //  Deploy/Stow and Pivot
  //
  //------------------------------------------------------------------------------------------------
  private WPI_TalonFX intakePivotMotor;
  
  public final PneumaticsModuleType PH_TYPE = PneumaticsModuleType.REVPH;
    
  private static DoubleSolenoid intakeSolenoid;

  private final int INTAKE_LATCH_ENGAGE_PCM_PORT  = 0;  /* TBD warning... temporary values, port #'s not confirmed yet by electronics' */
  private final int INTAKE_LATCH_RELEASE_PCM_PORT = 1;  


  private final int     INTAKE_PIVOT_MC_ID           = 10; 
  private final double  INTAKE_PIVOT_REL_ENCODER_CPR = 2048.0;

  private final double  CURRENT_LIMIT_AMPS            = 60.0;
  private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 60.0;
  private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
  private final boolean ENABLE_CURRENT_LIMIT          = true;

  private SupplyCurrentLimitConfiguration EncCurrentLimit;


  //Pivot status
  private  final int INTAKE_PIVOT_MODE_NULL                = 0;
  private  final int INTAKE_PIVOT_MODE_DEPLOY              = 1;
  private  final int INTAKE_PIVOT_MODE_DEPLOY_CALC         = 10;
  private  final int INTAKE_PIVOT_MODE_FULLY_DEPLOYED      = 2;
  private  final int INTAKE_PIVOT_MODE_INITIALIZATION      = 3;
  private  final int INTAKE_PIVOT_MODE_STOW_CALC           = 14;
  private  final int INTAKE_MODE_STOW_HOLD                 = 5;
  private  final int INTAKE_PIVOT_MODE_STOW                = 6;

  //Pivot IntakeMode initialization
  private int intakePivotMode = INTAKE_PIVOT_MODE_NULL;
  //-------------------------------------------------------------------------------------------
  //Gravity Constants
  //----------------------------------------------------------------------------------------
  private double GRAVITY_CONSTANT = 9.8; //meters pder second square
  private double MASS_INTAKE = 7.436; //kg
  private double PIVOTPOINT_TO_CENTER_DISTAMCE = 0.2865;// meters


  //------------------------------------------------------------------------------------------------
  //  Gear ratio
  //------------------------------------------------------------------------------------------------
  private final double INTAKE_PIVOT_PINION_GEAR = 11.0;
  private final double INTAKE_PIVOT_SPUR_GEAR   = 56.0;  
  private final double INTAKE_PIVOT_GEAR_RATIO  =INTAKE_PIVOT_SPUR_GEAR/INTAKE_PIVOT_PINION_GEAR;

  private final double INTAKE_PIVOT_SPROCKET_1  = 16.0;
  private final double INTAKE_PIVOT_SPROCKET_2  = 56.0;
  private final double INTAKE_PIVOT_SPROCKET_RATIO  = INTAKE_PIVOT_SPROCKET_2/INTAKE_PIVOT_SPROCKET_1;
  
  private final double INTAKE_PIVOT_FINAL_RATIO = INTAKE_PIVOT_GEAR_RATIO*INTAKE_PIVOT_SPROCKET_RATIO;
  
  
  //------------------------------------------------------------------------------------------------
  //  Angle Definitions
  //------------------------------------------------------------------------------------------------
  private final static double INTAKE_STOWED_ANGLE   =  0.0;
  private final static double INTAKE_DEPLOYED_ANGLE = 97.32;   //TBD angle need correct

  private final static double INTAKE_STOW_INITIAL_ANGLE   = INTAKE_DEPLOYED_ANGLE;
  private final static double INTAKE_DEPLOY_INITIAL_ANGLE = INTAKE_STOWED_ANGLE;

  private final double INTAKE_STOW_CUT_ANGLE   = 35;
  private final double INTAKE_STOW_CUT_ANGLE_2 = 5;


  //------------------------------------------------------------------------------------------------
  //  Motion Magic Approach
  //------------------------------------------------------------------------------------------------
  private final int    INTAKE_STOW_SLOT = 0;
  private final double INTAKE_STOW_KP   = 0.08; 
  private final double INTAKE_STOW_KI   = 0.00;
  private final double INTAKE_STOW_KD   = 0.00;

  private final int    INTAKE_DEPLOY_SLOT = 1;
  private final double INTAKE_DEPLOY_KP   = 0.035;
  private final double INTAKE_DEPLOY_KI   = 0.00;
  private final double INTAKE_DEPLOY_KD   = 0.00;

  private final static double INTAKE_DEPLOY_SET_POSITION_START_ANGLE = 80.0;
  private final static double INTAKE_STOW_REDUCE_POWER_ANGLE         = 55.0;

  private final double INTAKE_DEPLOYED_ANGLE_COUNTS   = -(INTAKE_DEPLOYED_ANGLE * (INTAKE_PIVOT_REL_ENCODER_CPR / 360.0) * INTAKE_PIVOT_FINAL_RATIO);
  private final double INTAKE_STOWED_ANGLE_COUNTS     =  (INTAKE_STOWED_ANGLE   * (INTAKE_PIVOT_REL_ENCODER_CPR / 360.0) * INTAKE_PIVOT_FINAL_RATIO);
  
  private final double INTAKE_SOFTLIMIT_OFFSET_ANGLE        = 0.0;  //Setting to zero assuming deploy code will not run motor into the hard-stops
  private final double INTAKE_SOFTLIMIT_OFFSET_ANGLE_COUNTS = (INTAKE_SOFTLIMIT_OFFSET_ANGLE * (INTAKE_PIVOT_REL_ENCODER_CPR / 360.0) * INTAKE_PIVOT_FINAL_RATIO);

  private final double INTAKE_SOFTLIMIT_MAX_ANGLE_COUNTS    = INTAKE_DEPLOYED_ANGLE_COUNTS - INTAKE_SOFTLIMIT_OFFSET_ANGLE_COUNTS;
  private final double INTAKE_SOFTLIMIT_MIN_ANGLE_COUNTS    = INTAKE_STOWED_ANGLE_COUNTS   + INTAKE_SOFTLIMIT_OFFSET_ANGLE_COUNTS;


  //------------------------------------------------------------------------------------------------
  //  5th order polynomial approach
  //------------------------------------------------------------------------------------------------
  //coeffients
  private  final double DEG2RAD = Math.PI / 180.0;

  private final double INTAKE_STOW_TIME         = 0.26;
  private final double INTAKE_STOW_CALC_Kp      = 0.000;
  private final double INTAKE_STOW_CALC_Kd      = 0.000;

  private final double INTAKE_DEPLOY_TIME       = 0.3;
  private final double INTAKE_DEPLOY_CALC_Kp    = 0.000;
  private final double INTAKE_DEPLOY_CALC_Kd    = 0.000;
 

  private final double COEFF1  =  10.0;
  private final double COEFF2  = -15.0;
  private final double COEFF3  =   6.0;


  private final double INTAKE_INERTIA    = 0.61;  //kg * m^2 
  private final double INTAKE_MAX_TORQUE = 5.84;  //Newton Meters

  private final double B_DEPLOY  = (INTAKE_DEPLOYED_ANGLE - INTAKE_DEPLOY_INITIAL_ANGLE) / INTAKE_DEPLOY_TIME;
  private final double A3_DEPLOY = COEFF1 * B_DEPLOY / INTAKE_DEPLOY_TIME / INTAKE_DEPLOY_TIME;
  private final double A4_DEPLOY = COEFF2 * B_DEPLOY / INTAKE_DEPLOY_TIME / INTAKE_DEPLOY_TIME / INTAKE_DEPLOY_TIME;
  private final double A5_DEPLOY = COEFF3 * B_DEPLOY / INTAKE_DEPLOY_TIME / INTAKE_DEPLOY_TIME / INTAKE_DEPLOY_TIME / INTAKE_DEPLOY_TIME;

  private final double B_STOW    = (INTAKE_STOWED_ANGLE - INTAKE_STOW_INITIAL_ANGLE) / INTAKE_STOW_TIME;
  private final double A3_STOW   = COEFF1 * B_STOW   / INTAKE_STOW_TIME / INTAKE_STOW_TIME;
  private final double A4_STOW   = COEFF2 * B_STOW   / INTAKE_STOW_TIME / INTAKE_STOW_TIME / INTAKE_STOW_TIME;  
  private final double A5_STOW   = COEFF3 * B_STOW   / INTAKE_STOW_TIME / INTAKE_STOW_TIME / INTAKE_STOW_TIME / INTAKE_STOW_TIME;
  
  private final double ALPHA3_DEPLOY = (A3_DEPLOY * DEG2RAD * INTAKE_INERTIA) / INTAKE_PIVOT_FINAL_RATIO/ INTAKE_MAX_TORQUE;
  private final double ALPHA4_DEPLOY = (A4_DEPLOY * DEG2RAD * INTAKE_INERTIA) / INTAKE_PIVOT_FINAL_RATIO / INTAKE_MAX_TORQUE;
  private final double ALPHA5_DEPLOY = (A5_DEPLOY * DEG2RAD * INTAKE_INERTIA) / INTAKE_PIVOT_FINAL_RATIO / INTAKE_MAX_TORQUE;


  private final double ALPHA3_STOW   = (A3_STOW   * DEG2RAD * INTAKE_INERTIA) / INTAKE_PIVOT_FINAL_RATIO / INTAKE_MAX_TORQUE;
  private final double ALPHA4_STOW   = (A4_STOW   * DEG2RAD * INTAKE_INERTIA) / INTAKE_PIVOT_FINAL_RATIO / INTAKE_MAX_TORQUE;
  private final double ALPHA5_STOW   = (A5_STOW   * DEG2RAD * INTAKE_INERTIA) / INTAKE_PIVOT_FINAL_RATIO / INTAKE_MAX_TORQUE;


  private final double INTAKE_DEPLOY_LATCH_ENGAGE_DELAY = 0.2;
  private final double INTAKE_STOW_LATCH_ENGAGE_DELAY   = 0.250;

  private final double INTAKE_PIVOT_DEPLOY_POWER = -0.2;    //TBD May not need depending on final implementation
  private final double INTAKE_PIVOT_STOW_POWER   = 0.7;

  private final int INTAKE_PIVOT_STOWED = 0;       //TBD  MAY DELETE
  private final int INTAKE_PIVOT_DEPLOYED = 1;
  private final int INTAKE_PIVOT_IN_TRANSIT = 2;
  private final int INTAKE_PIVOT_UNINITIALIZED = -999;
  private int intakePivotState = INTAKE_PIVOT_STOWED;


  private double powerForMotor;
  private double finalMotorPower   = 0.0;

  private double angleDot          = 0.0;
  private double angleOld          = 0.0;
  private double currentAngle      = 0.0;
  private double targetAngle       = 0.0;
  private double targetAngularRate = 0.0;
  private double deltaAngle        = 0.0;
  private double targetAngularAcceleration = 0.0;

  private Timer  pivotTimer;
  private double time      = 0.0;    //TBD - why this method? why are we saving this time? normally used with pivotTimer
  private double timeOld   = 0.0;
  private double deltaTime = 0.0;

  private double deploymentMotorRawPosition;

  public boolean intakeActive;
  boolean takedata;
  int intakeState = 0;
  double gravityTorque;

  private int intakeStowCount = -1;

  //Cooling
  private static DoubleSolenoid coolingSolenoid;



//---------------------------------------------definitions part end--------------------------------------------------------------
  
    /*-----------------------------------------------------------------------------------------
    *  
    *  CatzIntake()
    *
    *----------------------------------------------------------------------------------------*/
    public CatzIntake() {
    //need add softlimits

    intakeRollerMotor = new WPI_TalonFX(INTAKE_ROLLER_MC_ID);
    intakePivotMotor  = new WPI_TalonFX(INTAKE_PIVOT_MC_ID);

    EncCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, 
                                                                                CURRENT_LIMIT_TRIGGER_AMPS, 
                                                                                CURRENT_LIMIT_TIMEOUT_SECONDS);
    intakeRollerMotor.configFactoryDefault();
    intakeRollerMotor.setNeutralMode(NeutralMode.Coast);
    intakeRollerMotor.configSupplyCurrentLimit(EncCurrentLimit);


    intakePivotMotor.configFactoryDefault();
    intakePivotMotor.setNeutralMode(NeutralMode.Brake);
    intakePivotMotor.configSupplyCurrentLimit(EncCurrentLimit);
  
    intakePivotMotor.config_kP(INTAKE_STOW_SLOT, INTAKE_STOW_KP);
    intakePivotMotor.config_kI(INTAKE_STOW_SLOT, INTAKE_STOW_KI);
    intakePivotMotor.config_kD(INTAKE_STOW_SLOT, INTAKE_STOW_KD);

    intakePivotMotor.config_kP(INTAKE_DEPLOY_SLOT, INTAKE_DEPLOY_KP);
    intakePivotMotor.config_kI(INTAKE_DEPLOY_SLOT, INTAKE_DEPLOY_KI);
    intakePivotMotor.config_kD(INTAKE_DEPLOY_SLOT, INTAKE_DEPLOY_KD);

    intakePivotMotor.configForwardSoftLimitThreshold(INTAKE_SOFTLIMIT_MIN_ANGLE_COUNTS);
    intakePivotMotor.configReverseSoftLimitThreshold(INTAKE_SOFTLIMIT_MAX_ANGLE_COUNTS);

    intakePivotMotor.configForwardSoftLimitEnable(true);
    intakePivotMotor.configReverseSoftLimitEnable(true);

    intakeSolenoid = new DoubleSolenoid(PH_TYPE, INTAKE_LATCH_ENGAGE_PCM_PORT,
                                                 INTAKE_LATCH_RELEASE_PCM_PORT);

    intakeSolenoid.set(DoubleSolenoid.Value.kReverse);  // Release the Latch

    coolingSolenoid = new DoubleSolenoid(PH_TYPE, 6, 7);

    coolingSolenoid.set(DoubleSolenoid.Value.kOff);

    pivotTimer = new Timer();
   
    intakeControl();

  }
    /*-----------------------------------------------------------------------------------------
    *  
    *  Cooling
    *
    *----------------------------------------------------------------------------------------*/
    public void coolPDH()
    {
      coolingSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void coolBreaker()
    {
      coolingSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void coolingOff()
    {
      coolingSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  Roller Methods
    *
    *----------------------------------------------------------------------------------------*/
    public void intakeRollerIn()
    {
      Robot.indexer.indexerFrntIngest();
      intakeRollerMotor.set(ControlMode.PercentOutput,INTAKE_MOTOR_POWER_ROLLER_IN);
      intakeRollerState = INTAKE_ROLLER_IN;
      intakeActive = true;
    }

    public void intakeRollerOut()
    {
    
      intakeRollerMotor.set(ControlMode.PercentOutput,INTAKE_MOTOR_POWER_ROLLER_OUT);
      intakeRollerState = INTAKE_ROLLER_OUT;
    }


    public void intakeRollerOff()
    {
        intakeRollerMotor.set(ControlMode.PercentOutput,INTAKE_MOTOR_POWER_OFF);
        intakeRollerState = INTAKE_ROLLER_OFF;
        intakeActive = false;
        Robot.indexer.indexerFrntStop();
    }


    public void procCmdRoller(double xboxValueIn, double xboxValueOut)
    {
      if (xboxValueIn > INTAKE_INPUT_THRESHOLD)
      {
        intakeRollerIn();  
      }
      else if (xboxValueOut > INTAKE_INPUT_THRESHOLD)
      { 
        intakeRollerOut();     
      }
      else
      {
        intakeRollerOff();
      }
    }

    

    /*-----------------------------------------------------------------------------------------
    *  
    *  Deploy/Stow Methods
    *
    *----------------------------------------------------------------------------------------*/
    public void intakeControl()
    {
      intakeThread = new Thread(() ->
      { 
        while(true)
        {
          
          switch(intakePivotMode)
          {
              case INTAKE_PIVOT_MODE_NULL:
            
              break;
               
              case INTAKE_PIVOT_MODE_DEPLOY:
                currentAngle = getIntakeAngle();
                if(currentAngle > 20.0)  //INTAKE_DEPLOY_SET_POSITION_START_ANGLE )
                {
                  intakePivotMotor.set(ControlMode.PercentOutput, 0.0);
                  //intakePivotMotor.set(ControlMode.Position, INTAKE_DEPLOYED_ANGLE_COUNTS); //TBD review
                  intakePivotMode = INTAKE_PIVOT_MODE_NULL;
                 // intakePivotMotor.set(ControlMode.MotionMagic, INTAKE_PIVOT_DEPLOY_ROTATION);
                }
              break;
              
              
              case INTAKE_PIVOT_MODE_DEPLOY_CALC:
                currentAngle = getIntakeAngle();   
                time         = pivotTimer.get();

                if((currentAngle > INTAKE_DEPLOY_SET_POSITION_START_ANGLE) || (time > INTAKE_DEPLOY_TIME)) //Angle TBD      
                {
                  intakePivotMotor.set(ControlMode.PercentOutput,INTAKE_MOTOR_POWER_OFF);
                  intakePivotMode = INTAKE_PIVOT_MODE_NULL;  
                  intakeState = 1;
                } 
                else
                {
                  intakeState = 2;
                  deltaAngle  = currentAngle - angleOld;
                  angleOld    = currentAngle;

                  deltaTime = time - timeOld;              
                  timeOld   = time;
                
                  if(Math.abs(deltaTime) < 0.00000000001 ) 
                  {
                    angleDot = 0.0;
                  }
                  else 
                  {
                    //System.out.println("else statement of delta angle");
                    angleDot = deltaAngle / deltaTime;
                  }

                  double timePow2 = Math.pow(time, 2);
                  double timePow3 = Math.pow(time, 3);
                  double timePow4 = Math.pow(time, 4);
                  double timePow5 = Math.pow(time, 5);
                  
                 
                  targetAngle       = (    A3_DEPLOY * timePow3) + (    A4_DEPLOY * timePow4) + (    A5_DEPLOY * timePow5);
                  targetAngularRate = (3 * A3_DEPLOY * timePow2) + (4 * A4_DEPLOY * timePow3) + (5 * A5_DEPLOY * timePow4);
                  targetAngularAcceleration = 6*A3_DEPLOY*time+12*A4_DEPLOY*timePow2+20*A5_DEPLOY*timePow3;
                  //powerForMotor     = ((((ALPHA3_DEPLOY * timePow3) + (ALPHA4_DEPLOY * timePow4) + (ALPHA5_DEPLOY * timePow5) 
                  //- (GRAVITY_CONSTANT * PIVOTPOINT_TO_CENTER_DISTAMCE * MASS_INTAKE * Math.sin(targetAngle)))/INTAKE_PIVOT_FINAL_RATIO)/ INTAKE_MAX_TORQUE);  
                  gravityTorque =     (GRAVITY_CONSTANT * PIVOTPOINT_TO_CENTER_DISTAMCE * MASS_INTAKE * Math.sin(targetAngle*DEG2RAD));
                  powerForMotor =     (((INTAKE_INERTIA/INTAKE_PIVOT_FINAL_RATIO)/INTAKE_MAX_TORQUE)*(targetAngularAcceleration*DEG2RAD))
                                        - (gravityTorque/(INTAKE_PIVOT_FINAL_RATIO*INTAKE_MAX_TORQUE));  
                  finalMotorPower = powerForMotor + INTAKE_DEPLOY_CALC_Kp * (targetAngle       - currentAngle) + 
                                                    INTAKE_DEPLOY_CALC_Kd * (targetAngularRate - angleDot); 

                  //Flip sign of final motor power to account for physical orientation of pivot motor
                  finalMotorPower = -finalMotorPower;

                  intakePivotMotor.set(ControlMode.PercentOutput,finalMotorPower);
                  //intakePivotMotor.set(ControlMode.PercentOutput, 0);
                }
              
              break;

              case INTAKE_PIVOT_MODE_STOW:
                currentAngle = getIntakeAngle();

                intakeSolenoid.set(DoubleSolenoid.Value.kForward);

                if(currentAngle <= 1.0)
                {
                  intakeStowCount++;
                  System.out.println("increment counter");
                  if(intakeStowCount >= 15)
                  { 
                    System.out.println("Engage Latch, Stow Done");
                    intakeSolenoid.set(DoubleSolenoid.Value.kReverse);      //kForward); inversed
                    intakePivotMode = INTAKE_PIVOT_MODE_NULL;
                    intakePivotMotor.set(ControlMode.PercentOutput, INTAKE_MOTOR_POWER_OFF);
                  }
                }
                else if(currentAngle <= 10.0)
                {
                  System.out.println("Power 0.25");
                  intakeStowCount = 0;
                  intakePivotMotor.set(ControlMode.PercentOutput, 0.25);
                }
                else if(currentAngle <= INTAKE_STOW_CUT_ANGLE)
                {
                  if(intakeStowCount < 0)
                  {

                  }
                  else
                  {
                    System.out.println("Power 0.0");
                    intakeStowCount = 0;
                    intakePivotMotor.set(ControlMode.PercentOutput, INTAKE_MOTOR_POWER_OFF);
                  }
                }
              break;
              
              
              case INTAKE_PIVOT_MODE_STOW_CALC:

                currentAngle = getIntakeAngle();     
                time         = pivotTimer.get();

                if((currentAngle < INTAKE_STOWED_ANGLE) ||time > INTAKE_STOW_TIME)       
                {
                  intakeState = 1;

                  intakePivotMotor.set(ControlMode.PercentOutput, INTAKE_MOTOR_POWER_OFF);

                  Timer.delay(INTAKE_STOW_LATCH_ENGAGE_DELAY);          //Wait for intake to reach 
                  intakeSolenoid.set(DoubleSolenoid.Value.kForward);    //Engage the Intake Latch

                  intakePivotMode = INTAKE_PIVOT_MODE_NULL;  
                } 
                else
                {
                  //System.out.println("stow active");
                  intakeState = 2;
                  deltaAngle  = currentAngle - angleOld;
                  angleOld    = currentAngle;
                  
                  deltaTime = time - timeOld;              
                  timeOld   = time;
                
                  if(deltaTime < 0.00000000001 ) 
                  {
                    angleDot = 0.0;
                  }
                  else 
                  {
                    //System.out.println("else statement of delta angle");
                    angleDot = deltaAngle / deltaTime;
                  }

                  double timePow2 = Math.pow(time, 2);
                  double timePow3 = Math.pow(time, 3);
                  double timePow4 = Math.pow(time, 4);
                  double timePow5 = Math.pow(time, 5);

                  targetAngle       = INTAKE_DEPLOYED_ANGLE + (    A3_STOW * timePow3         ) + (    A4_STOW * timePow4) + (    A5_STOW * timePow5);
                  targetAngularRate = (3 * A3_STOW * Math.pow(time, 2)) + (4 * A4_STOW * timePow3) + (5 * A5_STOW * timePow4);

                  targetAngularAcceleration = 6*A3_STOW*time+12*A4_STOW*timePow2+20*A5_STOW*timePow3;

                  gravityTorque =     (GRAVITY_CONSTANT * PIVOTPOINT_TO_CENTER_DISTAMCE * MASS_INTAKE * Math.sin(targetAngle*DEG2RAD));
                  powerForMotor =     (((INTAKE_INERTIA/INTAKE_PIVOT_FINAL_RATIO)/INTAKE_MAX_TORQUE)*(targetAngularAcceleration*DEG2RAD))
                                          - (gravityTorque/(INTAKE_PIVOT_FINAL_RATIO*INTAKE_MAX_TORQUE));
                  
                  finalMotorPower = powerForMotor + INTAKE_STOW_CALC_Kp * (targetAngle       - currentAngle) + 
                                                    INTAKE_STOW_CALC_Kd * (targetAngularRate - angleDot); 

                  //Flip sign of final motor power to account for physical orientation of pivot motor
                  finalMotorPower = -finalMotorPower;

                  intakePivotMotor.set(ControlMode.PercentOutput, finalMotorPower);
                  //intakePivotMotor.set(ControlMode.PercentOutput, 0.0);
                }
              break;

              default:
                intakePivotMotor.set(ControlMode.PercentOutput, INTAKE_MOTOR_POWER_OFF);
              break;

          }   //end of switch

          if((DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_INTAKE))
          {
            data = new CatzLog(pivotTimer.get(), targetAngle, currentAngle, intakeStowCount, intakePivotMotor.getMotorOutputPercent(), targetAngularRate, angleDot, 
                                     -finalMotorPower, powerForMotor, deltaAngle, deltaTime, intakeState, 
                                     targetAngularAcceleration, 
                                     gravityTorque, 
                                    -999.0,
                                     DataCollection.boolData);  
            Robot.dataCollection.logData.add(data);
          }

          Timer.delay(INTAKE_THREAD_PERIOD);
        
        }  //end of while true
              
      });
      intakeThread.start();
    
    }   //end of intakeControl();


    /*-----------------------------------------------------------------------------------------
    *  
    * intakeDeploy()
    *
    *----------------------------------------------------------------------------------------*/
    public void intakeDeploy()
    {
      pivotTimer.reset();
      pivotTimer.start();

      //intakeSolenoid.set(DoubleSolenoid.Value.kReverse);  // Release the Latch //JH 03/11/2023 Things might inversed
      intakeSolenoid.set(DoubleSolenoid.Value.kForward);
      Timer.delay(INTAKE_DEPLOY_LATCH_ENGAGE_DELAY);        //Wait for latch to clear 

      timeOld   = 0.0;
      angleOld  = 0.0;
      angleDot  = 9999.00;
      
      intakePivotMotor.set(ControlMode.PercentOutput,INTAKE_PIVOT_DEPLOY_POWER);
      intakePivotMode = INTAKE_PIVOT_MODE_DEPLOY;
      //intakePivotMode = INTAKE_PIVOT_MODE_DEPLOY_CALC;
    }

    public void intakeStow()
    {
      pivotTimer.reset();
      pivotTimer.start();

      timeOld   = 0.0;
      angleOld  = 0.0;
      angleDot  = 9999.00;

      intakeStowCount = -1;

      //intakePivotMode = INTAKE_PIVOT_MODE_STOW_CALC;
      intakePivotMotor.set(ControlMode.PercentOutput, 0.35);
      intakePivotMode = INTAKE_PIVOT_MODE_STOW;
    }


    public void procCmdIntake(boolean xboxValueDeploy, boolean xboxValueStow)
    {

      if (xboxValueDeploy == true)
      {
          if(Robot.elevatorState == Robot.DEPLOYED)
          {
              //flash colors indicating that you can't deploy
          }
          else
          {
            intakeDeploy();
            Robot.intakeState = Robot.DEPLOYED;
          }
      }
      else if (xboxValueStow == true)
      {
        intakeStow();
        Robot.intakeState = Robot.STOWED;
      }
    }


    /*-----------------------------------------------------------------------------------------
    *
    *  getIntakePositionDegrees
    *
    *----------------------------------------------------------------------------------------*/
    public double getIntakeAngle()
    {
        deploymentMotorRawPosition = intakePivotMotor.getSelectedSensorPosition();

        double motorShaftRevolution = deploymentMotorRawPosition / INTAKE_PIVOT_REL_ENCODER_CPR;
        double pivotShaftRevolution = motorShaftRevolution       / INTAKE_PIVOT_FINAL_RATIO;
        double pivotAngle           = pivotShaftRevolution * -360.0; //motor  spin forward is positive 
        
        return pivotAngle;   
    }


    /*-----------------------------------------------------------------------------------------
    *  
    *  Smart Dashboard
    *
    *----------------------------------------------------------------------------------------*/
    public void smartDashboardIntake()
    {
        SmartDashboard.putNumber("PivotAngle", getIntakeAngle());
        SmartDashboard.putNumber("Final Motor Power", finalMotorPower);
    }

    public void smartDashboardIntake_Debug()
    {
      SmartDashboard.putNumber("PivotCounts", deploymentMotorRawPosition);
      SmartDashboard.putNumber("getClosedLoopError", intakePivotMotor.getClosedLoopError());
      SmartDashboard.putNumber("getClosedLoopTarget", intakePivotMotor.getClosedLoopTarget());
      SmartDashboard.putNumber("getStatorCurrent", intakePivotMotor.getStatorCurrent());
      SmartDashboard.putNumber("Intake state", intakePivotMode);
      
    }
  

    public void engageLatch()
    {
      intakeSolenoid.set(DoubleSolenoid.Value.kForward);
      System.out.println("Latch Engaged");
    }
    public void disengageLatch()
    {
      intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
      System.out.println("Latch Disengaged");
    }
} 