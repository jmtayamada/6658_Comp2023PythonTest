import wpilib
import wpilib.drive
import wpimath.controller
import ctre
import navx
import numpy as np
from wpilib.shuffleboard import Shuffleboard
import wpilib.shuffleboard


seconds = 0

c_ArmLiftTotalValue = 540       # Constant for max value ArmLift encoder should ever go to
c_ArmExtendMaxValue = 100       # Constant for max value ArmExtend encoder should ever go to
c_gyroMountAngle = 45           # Constant for what angle the gyro is mounted at

class Robot(wpilib.TimedRobot):
    
    def __init__(self, period: seconds = 0.02) -> None:
        super().__init__(period)

        # Camera
        wpilib.CameraServer.launch()

        # Create four motors for driving, put them in a list for PID control in autonomouse
        self.MotorFL = ctre.WPI_TalonSRX(5)
        self.MotorFL.setInverted(True)
        self.MotorFR = ctre.WPI_TalonSRX(3)
        self.MotorBL = ctre.WPI_TalonSRX(0)
        self.MotorBL.setInverted(True)
        self.MotorBR = ctre.WPI_TalonSRX(6)
        self.DriveMotors = [self.MotorFL, self.MotorFR, self.MotorBL, self.MotorBR]

        # Create two motors to control the arms
        self.ArmLift = ctre.WPI_TalonSRX(1)
        self.ArmLift.setInverted(True)
        self.ArmExtend = ctre.WPI_TalonSRX(4)

        # Differential Drives to control wheel motors in teleop
        self.frontDrive = wpilib.drive.DifferentialDrive(self.MotorFL, self.MotorFR)
        self.backDrive = wpilib.drive.DifferentialDrive(self.MotorBL, self.MotorBR)

        # Create Encoders and set distance per pulse for Arm extension and Arm Lift
        self.ArmExtendEncoder = wpilib.Encoder(0, 1)
        self.ArmExtendEncoder.setDistancePerPulse(180/4)    # 180 distance per 4 pulses

        self.ArmLiftEncoder = wpilib.Encoder(2, 3)
        self.ArmLiftEncoder.setDistancePerPulse(25/4)       # 25 distance per 4 pulses

        # Joysticks
        self.DriveStick = wpilib.Joystick(1)
        self.HelperStick = wpilib.XboxController(0)

        # # Shuffleboard values for PID
        # self.PIDArmLiftTest = (
        #     Shuffleboard.getTab("Configuration")
        #     .add(title="PIDArmLift", defaultValue=1)
        #     .withWidget(wpilib.shuffleboard.BuiltInWidgets.kPIDController)
        #     .withPosition(0, 0)
        #     .withSize(2, 1)
        #     .getEntry()
        # )
        # self.PIDArmExtendTest = (
        #     Shuffleboard.getTab("Configuration")
        #     .add(title="PIDArmExtend", defaultValue=1)
        #     .withWidget(wpilib.shuffleboard.BuiltInWidgets.kPIDController)
        #     .withPosition(0, 1)
        #     .withSize(2, 1)
        #     .getEntry()
        # )

        # PID controllers for arm lift and arm extension
        self.PIDArmLift = wpimath.controller.PIDController(.0625, 0, 0)
        self.PIDArmLift.setTolerance(5)
        self.PIDArmExtend = wpimath.controller.PIDController(.0625, 0, 0)
        self.PIDArmExtend.setTolerance(2)

        # Double solenoid for arm grabbing
        self.ArmGrab = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, 2, 3)

        # PID controller for balancing the robot
        self.PIDBalance = wpimath.controller.PIDController(1, 0, 1)
        self.PIDBalance.setTolerance(1)

        # navx gyro, mounted at an angle on actual robot
        self.gyro = navx.AHRS(wpilib._wpilib.SPI.Port(0))

        # timer for timing, and getting delta time
        self.timer = wpilib.Timer()

        # variable to keep track of arm
        self.ArmOpened = False #kForward to close, kReverse to open

        # constants
        self.liftTargetAngle = 0        # Target angle for Arm lift
        self.extendTargetAngle = 0      # target angle for Arm extend
        self.previousTime = 0           # variable for use in getting delta time

        self.autonomousePhaseNum = 1    # Variable to track autonomouse phase


    def DeltaTime(self):
        # function to get delta time
        return self.timer.get() - self.previousTime

    
    def getArmLiftEncoderValue(self):
        # function to get clamped values of arm lift encoder (for when the arm lift encoders return values to high or too low)
        return float(np.clip(self.ArmLiftEncoder.get(), 0, c_ArmLiftTotalValue))
    
    
    def getArmExtendEncoderValue(self):
        # same as previous, but for arm extend
        return float(np.clip(self.ArmExtend.get(), 0, c_ArmExtendMaxValue))
    

    def autonomousInit(self) -> None:
        # reset all used variables
        self.timer.reset()
        self.timer.start()
        self.ArmLiftEncoder.reset()
        self.ArmExtendEncoder.reset()
        self.autonomousePhaseNum = 1
        self.ArmOpened = False
        self.ArmGrab.set(wpilib.DoubleSolenoid.Value.kForward)


    def autonomousPeriodic(self) -> None:
        # phase 1, lift the arm
        if self.autonomousePhaseNum == 1:
            self.ArmLift.set(self.PIDArmLift.calculate(self.getArmLiftEncoderValue, 470))
            if self.PIDArmLift.atSetpoint(self.getArmLiftEncoderValue, 470) == True:
                self.autonomousePhaseNum == 2
        # phase 2, extend the arm
        if self.autonomousePhaseNum == 2:
            self.ArmExtend.set(self.PIDArmExtend.calculate(self.getArmExtendEncoderValue, 100))
            if self.PIDArmLift.atSetpoint(self.getArmExtendEncoderValue, 100):
                self.timer.reset()
                self.timer.start()
                self.autonomousePhaseNum == 3
        # phase 3, open arm + close arm
        if self.autonomousePhaseNum == 3:
            self.ArmGrab.set(wpilib.DoubleSolenoid.Value.kReverse)
            if self.timer.get() > 2:
                self.ArmGrab.set(wpilib.DoubleSolenoid.Value.kForward)
                self.autonomousePhaseNum = 4
        # phase 4, de-extend the arm
        if self.autonomousePhaseNum == 4:
            self.ArmExtend.set(self.PIDArmExtend.calculate(self.getArmExtendEncoderValue, 0))
            if self.PIDArmLift.atSetpoint(self.getArmExtendEncoderValue, 0):
                self.autonomousePhaseNum == 5
        # stage 5, lower the arm
        if self.autonomousePhaseNum == 5:
            self.ArmLift.set(self.PIDArmLift.calculate(self.getArmLiftEncoderValue, 0))
            if self.PIDArmLift.atSetpoint(self.getArmLiftEncoderValue, 0) == True:
                self.timer.reset()
                self.timer.start()
                self.autonomousePhaseNum == 6
        # stage 6, move backwards untill out of comunity zone, or hit the seesaw
        if self.autonomousePhaseNum == 6:
            for obj in self.DriveMotors:
                obj.set(-.75)
            if self.gyro.getPitch < c_gyroMountAngle - 5:
                self.autonomousePhaseNum = 7
            if self.timer.get() > 8:
                for obj in self.DriveMotors:
                    obj.set(0)
                    self.autonomousePhaseNum = 100
        # if hit the seesaw, set speed to 1/4 untill robot reaches the top of the seesaw
        if self.autonomousePhaseNum == 7:
            for obj in self.DriveMotors:
                obj.set(-.25)
            if self.gyro.getPitch > c_gyroMountAngle - 10:
                self.autonomousePhaseNum == 8
        # at top of seesaw, control with PID to auto balance
        if self.autonomousePhaseNum == 8:
            for obj in self.DriveMotors:
                obj.set(self.PIDBalance.calculate(self.gyro.getPitch, c_gyroMountAngle))


    def teleopInit(self) -> None:
        # reset used variables
        self.previousTime = 0
        self.timer.reset()
        self.timer.start()
        self.ArmLiftEncoder.reset()
        self.ArmExtendEncoder.reset()
        self.ArmOpened = False
        self.ArmGrab.set(wpilib.DoubleSolenoid.Value.kForward)


    def teleopPeriodic(self) -> None:
        # get x and z angles, and add together
        t_zangle = self.DriveStick.getRawAxis(2)
        t_xangle = self.DriveStick.getRawAxis(0)
        t_finalangle = t_zangle/2 + t_xangle/2
        # get y and w values, multiply together for forward/reverse/speed control, and pass the previous values into arcade drive
        self.frontDrive.arcadeDrive(self.DriveStick.getRawAxis(1) * self.DriveStick.getRawAxis(3), t_finalangle, True)
        self.backDrive.arcadeDrive(self.DriveStick.getRawAxis(1) * self.DriveStick.getRawAxis(3), t_finalangle, True)

        # change arm lift target angle based on joystick input + clamp the value
        self.liftTargetAngle = np.clip(
            -self.HelperStick.getLeftY() * c_ArmLiftTotalValue * self.DeltaTime() + self.liftTargetAngle, 
            0, 
            c_ArmLiftTotalValue
        )
        # PID to calculate output
        t_ArmLiftMotorPower = np.clip(
            self.PIDArmLift.calculate(self.getArmLiftEncoderValue(), self.liftTargetAngle),
            -1,
            1
        )
        # if moving the arm up, go normal speed, if moving down, set half speed
        if t_ArmLiftMotorPower > 0:
            self.ArmLift.set(t_ArmLiftMotorPower/4)
        else:
            self.ArmLift.set(t_ArmLiftMotorPower/8)

        # # change arm extend distance based on joystick input + clamp the value
        # self.extendTargetAngle = np.clip(
        #     self.HelperStick.getRightY() * c_ArmExtendMaxValue * self.DeltaTime()/2 + self.extendTargetAngle,
        #     0,
        #     c_ArmExtendMaxValue
        # )
        # # pid to calclate the motor power
        # self.ArmExtend.set(self.PIDArmLift.calculate(self.getArmExtendEncoderValue(), self.extendTargetAngle))

        # if x button pressed, reverse direction of solenoid
        if self.HelperStick.getXButtonPressed():
            if self.ArmOpened:
                self.ArmGrab.set(wpilib.DoubleSolenoid.Value.kForward)
                self.ArmOpened = False
            else:
                self.ArmGrab.set(wpilib.DoubleSolenoid.Value.kReverse)
                self.ArmOpened = True

        # # for PID tuning
        # if self.HelperStick.getYButtonPressed():
        #     self.PIDArmLift.setP(self.PIDArmLift.getP()*2)
        #     print(self.PIDArmLift.getP())
        # if self.HelperStick.getAButtonPressed():
        #     self.PIDArmLift.set(self.PIDArmLift.getP() / 2)
        #     print(self.PIDArmLift.getP())

        # set previousTime variable to the current time, so that delta time can be gotten on the next loop
        self.previousTime = self.timer.get()


if __name__ == "__main__":
    wpilib.run(Robot)