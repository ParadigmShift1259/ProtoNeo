#include <frc/Timer.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>

#include <ctre/phoenix6/TalonFX.hpp>

using namespace ctre::phoenix;
using namespace ctre::phoenix6;
using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::configs;

constexpr int kFlywheelLeadID = 11;
constexpr int kFlywheelFollowID = 12;

class Shooter : public frc2::SubsystemBase {
    // private final SparkFlex m_flywheelMotorLead = new SparkFlex(ConstantsCANIDS.kFlywheelLeadID, MotorType.kBrushless);
    // private final SparkFlex m_flywheelMotorFollow = new SparkFlex(ConstantsCANIDS.kFlywheelFollowID, MotorType.kBrushless);
    // private SparkClosedLoopController m_flywheelCtlr = m_flywheelMotorLead.getClosedLoopController();
    TalonFX m_flywheelMotorLead{kFlywheelLeadID};
    TalonFX m_flywheelMotorFollow{kFlywheelFollowID};
    VelocityVoltage m_vvReq{units::angular_velocity::turns_per_second_t{0}};

public:
    Shooter(){
        TalonFXConfiguration cfg;
        FeedbackConfigs& fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1; // TODO figure out gear ratio
        
        MotionMagicConfigs& mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = 5_tps; // 5 (mechanism) rotations per second cruise
        mm.MotionMagicAcceleration = 10_tr_per_s_sq; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel 
        mm.MotionMagicJerk = 100_tr_per_s_cu;

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = 60;
        slot0.kI = 0;
        slot0.kD = 0.5;

        cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;

        StatusCode status = StatusCode::StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_flywheelMotorLead.GetConfigurator().Apply(cfg);
            if (status.IsOK()) break;
        }
        if (!status.IsOK()) {
            printf("Could not configure device. Error: %d\n", (int)status);
        }

        cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

        for (int i = 0; i < 5; ++i) {
            status = m_flywheelMotorFollow.GetConfigurator().Apply(cfg);
            if (status.IsOK()) break;
        }
        if (!status.IsOK()) {
            printf("Could not configure device. Error: %d\n", (int)status);
        }
        m_flywheelMotorFollow.SetControl(new Follower(m_flywheelMotorLead.getDeviceID(), MotorAlignmentValue.Aligned));

        // SparkFlexConfig configFlex = new SparkFlexConfig();
        // configFlex.idleMode(SparkMaxConfig.IdleMode.kCoast)
        //     .inverted(false)
        //     .closedLoopRampRate(0.0)
        //     .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0)
        //                 .p(0.5);
        // m_flywheelMotorLead.configure(configFlex, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // configFlex
        //     .follow(11)
        //     .inverted(true);
        // m_flywheelMotorFollow.configure(configFlex, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configMax = new SparkMaxConfig();
        configMax.idleMode(SparkMaxConfig.IdleMode.kBrake)
            .inverted(false)
            .closedLoopRampRate(0.0)
            .closedLoop.outputRange(-1.0,1.0, ClosedLoopSlot.kSlot0)
                       .p(0.5);
        m_turretMot.configure(configMax, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        configMax.closedLoop.p(0.5);
        m_hoodMot.configure(configMax, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public double getAngularDisplacement(Pose2d currentPose, Pose2d targetPose, Rotation2d turretAngle){
        currentPose.transformBy(new Transform2d(0.0, 0.0, Rotation2d.kZero)); // offset of robot center to turret center
        double xDisplacement = targetPose.getX() - currentPose.getX();
        double yDisplacement = targetPose.getY() - currentPose.getY();
        return Math.atan2(yDisplacement, xDisplacement) - currentPose.getRotation().getRadians() - turretAngle.getRadians();
    }

    public double getAimingRotations(double angle){
        double rotations = angle;
        return rotations; //TODO: Figure out angle to rotations
    }

    public void setRPM(double rpm){
        // m_flywheelCtlr.setSetpoint(rpm, ControlType.kVelocity);
        m_flywheelMotorLead.setControl(m_vvReq.withVelocity(rpm/60.0));

    }

    public void aimTurret(double angle){
        m_turretCtlr.setSetpoint(getAimingRotations(angle), ControlType.kPosition);
    }

    public void moveHood(double angle){
        m_hoodCtlr.setSetpoint(angle, ControlType.kPosition);
    }
}
