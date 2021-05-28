#include <cmath>
#include <lely/ev/loop.hpp>
#if _WIN32
#include <lely/io2/win32/ixxat.hpp>
#include <lely/io2/win32/poll.hpp>
#elif defined(__linux__)
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#else
#error This file requires Windows or Linux.
#endif
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/io2/can_rt.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/ev/co_task.hpp>

#include <iostream>
#if _WIN32
#include <thread>
#endif

/*------- INIT ATI-NETCANOEM-SENSOR ------- */

#include <linux/can.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <unistd.h>

#include <sstream>
#include <fstream>
#include <iomanip>

#include <json.hpp>
#include <thread>
#include <atomic>
#include "PID.h"
#include "InteractionController.h"

#define USE_FORCETORQUE_SENSOR
//#undef USE_FORCETORQUE_SENSOR

using json = nlohmann::json;

typedef struct can_frame CANframe;
typedef struct ifreq InterfReq;
typedef struct sockaddr_can SockAddrCan;
typedef struct timeval TimeVal;


constexpr int logSize = 250;

std::array<float, logSize> logPosition;
std::array<float, logSize> logForceZ;
std::array<float, logSize> logDt;
int logCounter = 0;

void printCANFrame(const CANframe&& frame)
{
    std::cout << frame.can_id << " : ";
    
    for(int i = 0; i < frame.can_dlc; i++)
    {
        std::cout << std::uppercase << std::hex << frame.data[i] << " ";
    }
    
    std::cout << std::endl << std::resetiosflags;
}


/// Responses are big-endian!
/// https://www.ati-ia.com/app_content/documents/9610-05-1030.pdf
class NetCANOEMCANMsg
{
public:
    
    enum OpCode
    {
        ///Read SG Data requisition
        SGData_req = 0x0,
        ///Read SG Data answer 1: status code (2 bytes), sg0 (2 bytes), sg2 (2 bytes) and sg4 (2 bytes)
        SGData_ans1 = 0x0,
        ///Read SG Data answer 2: sg1 (2 bytes), sg3 (2 bytes) and sg5 (2 bytes)
        SGData_ans2 = 0x1,
        ///Read Matrix requisition: 1 byte for the axis (0: Fx, 1: Fy, 2: Fz, 3: Tx, 4: Ty, 5: Tz)
        Matrix_req = 0x2,
        ///Read Matrix answer: SG0 (4 bytes) and SG1 (4 bytes)
        Matrix_ans1 = 0x2,
        ///Read Matrix answer: SG2 (4 bytes) and SG3 (4 bytes)
        Matrix_ans2 = 0x3,
        ///Read Matrix answer: SG4 (4 bytes) and SG5 (4 bytes)
        Matrix_ans3 = 0x4,
        ///Read Force/Torque Serial Number requisition
        SerialNumber_req = 0x5,
        ///Read Force/Torque Serial Number answer: 8 bytes with ASCI string serial number
        SerialNumber_ans = 0x5,
        ///Set active calibration requisition
        SetActiveCalibration_req = 0x6,
        ///Set active calibration answer: 1 byte echoing the selected calibration index
        SetActiveCalibration_ans = 0x6,
        ///Read Counts Per Unit requisition
        CountsPerUnits_req = 0x7,
        ///Read Counts Per Unit answer: 4 bytes for counts per force and 4 bytes for counts per torque
        CountsPerUnits_ans = 0x7,
        ///Read Unit Codes requisition
        UnitCodes_req = 0x8,
        ///Read Unit Codes answer:
        /// 1 byte for Force unit code
        ///     lbf     1
        ///     N       2
        ///     Klbf    3
        ///     kN      4
        ///     kgf     5
        ///     gf      6
        /// and 1 byte for torque unit code
        ///     lbf-in  1
        ///     lbf-ft  2
        ///     N-m     3
        ///     N-mm    4
        ///     kgfcm   5
        ///     kN-m    6
        UnitCodes_ans = 0x8,
        ///Read Diagnostic ADC Voltages requisition: 1 byte for diagnostic index (0: MID_VSG, 1: Unused, 2: Thermistor, 3: Power, 4: DAC, 5: Ground)
        DiagADCVoltages_req = 0x9,
        ///Read Diagnostic ADC Voltages answer: 2 bytes with requested ADC diagnostic
        DiagADCVoltages_ans = 0x9,
        ///Reset
        Reset_req = 0xC,
        ///Read Firmware Version requisition
        FirmwareVersion_req = 0xF,
        ///Read Firmware Version answer: 1 byte major version, 1 byte minor version, 2 bytes build number
        FirmwareVersion_ans = 0xF,
    };
    
    ///Status (2 bytes sent in SG Data Requitision?)
    ///Bit  Critical?        Name                                        .Can occur after firmware-upgrade; replace NETCANOEM if this happens during normal operation
    /// 0                    Watchdog Rest                               .
    /// 1                    DAC/ADC check result too high               .
    /// 2      Yes           DAC/ADC check result too high               .
    /// 3      Yes           Artificial analog ground out of range       .
    /// 4      Yes           Power supply too high                       .
    /// 5      Yes           Power supply too low                        .
    /// 6      Yes           Bad active calibration                      .
    /// 7      Yes           EEPROM failure                              .
    /// 8                    Configuration invalid                       .
    /// 9                    Reserved                                    .
    /// 10                   Reserved                                    .
    /// 11     Yes           Sensor temperature too high                 .
    /// 12     Yes           Sensor temperature too low                  .
    /// 13                   Reserved                                    .
    /// 14                   CAN bus error                               .
    /// 15                   Any error causes this bit to turn on        .
    
    enum Axis
    {
        Fx = 0x0,
        Fy = 0x1,
        Fz = 0x2,
        Tx = 0x3,
        Ty = 0x4,
        Tz = 0x5,
    };
    
    explicit NetCANOEMCANMsg(uint16_t baseID = 0x20): _baseIDRaw(baseID), _baseIDForUse(baseID << 4u)
    {
    }
    
    inline void setBaseID(uint16_t newBaseID)
    {
        _baseIDRaw = newBaseID;
        _baseIDForUse = newBaseID << 4u;
    }
    
    inline uint16_t getBaseID()
    {
        return _baseIDForUse;
    }
    
    CANframe getCANFrameRequest(const OpCode opCode) const
    {
        CANframe canFrame;
        canFrame.can_dlc = 0;
        canFrame.can_id = _baseIDForUse | opCode;
        
        return canFrame;
    }
    
    ///To get Matrix
    CANframe getCANFrameRequest(const OpCode opCode, const Axis axis) const
    {
        CANframe canFrame = getCANFrameRequest(opCode);
        canFrame.can_dlc = 1;
        canFrame.data[0] = axis;
        
        return canFrame;
    }
    
    typedef uint16_t CalibrationIndex;
    
    ///To set calibration
    CANframe getCANFrameRequest(const OpCode opCode, const CalibrationIndex calibration) const
    {
        if (calibration < 0 || calibration > 15)
        {
            throw std::runtime_error("Calibration must be in the range [0, 15]. Calibration requested:" + std::to_string(calibration));
        }
        
        CANframe canFrame = getCANFrameRequest(opCode);
        canFrame.can_dlc = 1;
        canFrame.data[0] = calibration;
    
        return canFrame;
    }
    
    enum DiagnosticADCVoltage
    {
        MID_VSG = 0x0,
        //Unused = 0x1, //?!
        Thermistor = 0x2,
        Power = 0x3,
        DAC = 0x4,
        Ground = 0x5,
    };
    
    ///To get ADC voltage diagnostic
    CANframe getCANFrameRequest( const OpCode opCode, const DiagnosticADCVoltage diag) const
    {
        CANframe canFrame = getCANFrameRequest(opCode);
        canFrame.can_dlc = 1;
        canFrame.data[0] = diag;
    
        return canFrame;
    };

//    CANframe getCANFrameRequest(const OpCode opCode, const uint16_t newBaseID) const
//    {
//
//    };

//    enum BaudRate
//    {
//        BaudRate125000,
//        BaudRate250000,
//        BaudRate500000,
//        BaudRate1000000,
//        BaudRate2000000,
//    };
//    CANframe getCANFrameRequest(const OpCode opCode, const uint16_t baudRateDivisor) const
//    {
//
//    };
    
    std::string toSerial(const CANframe& canFrame) const
    {
        if( canFrame.can_id != (_baseIDForUse | OpCode::SerialNumber_ans))
        {
            throw std::runtime_error("This is not a serial number answer frame!");
        }
        
        std::stringstream ssSerial;
        
        for (int i = 0; i < canFrame.can_dlc; i++)
        {
            ssSerial << canFrame.data[i];
        }
        
        return ssSerial.str();
    }
    
    typedef struct
    {
        int major;
        int minor;
        int buildNumber;
    } FirmwareVersion;
    
    FirmwareVersion toFirmwareVersion(const CANframe& canFrame) const
    {
        if( canFrame.can_id != (_baseIDForUse | OpCode::FirmwareVersion_ans))
        {
            throw std::runtime_error("This is not a firmware version answer frame!");
        }
        
        return FirmwareVersion{canFrame.data[0], canFrame.data[1], ( canFrame.data[2] << 8u ) | canFrame.data[3]};
    }
    
    int toCalibrationIndexSelected(const CANframe& canFrame) const
    {
        if( canFrame.can_id != (_baseIDForUse | OpCode::SetActiveCalibration_ans))
        {
            throw std::runtime_error("This is not a selected calibration index answer frame!");
        }
        return canFrame.data[0];
    }
    
    typedef struct
    {
        float Force;
        float Torque;
    } CountsPerUnit;
    
    CountsPerUnit toCountsPerUnit(const CANframe& canFrame) const
    {
        if( canFrame.can_id != (_baseIDForUse | OpCode::CountsPerUnits_ans))
        {
            throw std::runtime_error("This is not a counts per unit answer frame!");
        }
        return CountsPerUnit{   (float)
                                ((canFrame.data[0] << 24u) |
                                (canFrame.data[1] << 16u) |
                                (canFrame.data[2] << 8u)  |
                                (canFrame.data[3])),
                                (float)
                                ((canFrame.data[4] << 24u) |
                                (canFrame.data[5] << 16u) |
                                (canFrame.data[6] << 8u)  |
                                (canFrame.data[7]))};
    }
    
    enum ForceUnit
    {
        lbf = 0x1,
        N = 0x2,
        Klbf = 0x3, //or klbf?
        kN = 0x4,
        kgf = 0x5,
        gf = 0x6,
    };
    
    enum TorqueUnit
    {
        lbf_in = 0x1,
        lbf_ft = 0x2,
        N_m = 0x3,
        N_mm = 0x4,
        kgf_cm = 0x5,
        kN_m = 0x6,
    };
    
    typedef struct
    {
        ForceUnit Force;
        TorqueUnit Torque;
    } ForceTorqueUnit;
    
    ForceTorqueUnit toForceTorqueUnit(const CANframe& canFrame) const
    {
        if( canFrame.can_id != (_baseIDForUse | OpCode::UnitCodes_ans))
        {
            throw std::runtime_error("This is not a force-torque unit answer frame!");
        }
        return ForceTorqueUnit {(ForceUnit)canFrame.data[0], (TorqueUnit)canFrame.data[1]};
    }
    
    int toDiagnosticADCVoltage(const CANframe& canFrame) const
    {
        if( canFrame.can_id != (_baseIDForUse | OpCode::DiagADCVoltages_ans))
        {
            throw std::runtime_error("This is not a firmware version answer frame!");
        }
        return (( canFrame.data[0] << 8u) | canFrame.data[1] );
    }
    
    typedef struct {
        ///Bit 0
        bool b00WatchDogReset;
        ///Bit 1 - Critical
        bool b01DAC_ADC_tooHigh;
        ///Bit 2 - Critical
        bool b02DAC_ADC_tooLow;
        ///Bit 3 - Critical
        bool b03ArtificialAnalogGroundOutOfRange;
        ///Bit 4 - Critical
        bool b04PowerSupplyHigh;
        ///Bit 5 - Critical
        bool b05PowerSupplyLow;
        ///Bit 6 - Critical
        bool b06BadActiveCalibration;
        ///Bit 7 - Critical
        bool b07EEPROMFailure;
        ///Bit 8
        bool b08ConfigurationInvalid;
        //Bit 9
        //bool Reserved;
        //Bit 10
        //bool Reserved;
        ///Bit 11 - Critical
        bool b11SensorTempHigh;
        ///Bit 12 - Critical
        bool b12SensorTempLow;
        //Bit 13
        //bool Reserved;
        ///Bit 14
        bool b14CANBusError;
        ///Bit 15
        bool b15AnyError;
    } Status;
    
    Status toStatus(const CANframe& canFrame) const
    {
        if( canFrame.can_id != (_baseIDForUse | OpCode::SGData_ans1))
        {
            throw std::runtime_error("This is not a SG data first answer frame (this one has the status in its first 2 bytes)!");
        }
        
        unsigned int status = ( canFrame.data[0] << 8u ) | canFrame.data[1];
        
        return Status{
                static_cast<bool>(status >> 0u & 0x1),
                static_cast<bool>(status >> 1u & 0x1),
                static_cast<bool>(status >> 2u & 0x1),
                static_cast<bool>(status >> 3u & 0x1),
                static_cast<bool>(status >> 4u & 0x1),
                static_cast<bool>(status >> 5u & 0x1),
                static_cast<bool>(status >> 6u & 0x1),
                static_cast<bool>(status >> 7u & 0x1),
                static_cast<bool>(status >> 8u & 0x1),
              //static_cast<bool>(status >> 9u & 0x1),
              //static_cast<bool>(status >> 10u & 0x1),
                static_cast<bool>(status >> 11u & 0x1),
                static_cast<bool>(status >> 12u & 0x1),
              //static_cast<bool>(status >> 13u & 0x1),
                static_cast<bool>(status >> 14u & 0x1),
                static_cast<bool>(status >> 15u),
    
        };
        
    }
    
    std::tuple<float, float> toMatrixElements(const CANframe &canFrame) const
    {
        if( canFrame.can_id != (_baseIDForUse | OpCode::Matrix_ans1) &&
            canFrame.can_id != (_baseIDForUse | OpCode::Matrix_ans2) &&
            canFrame.can_id != (_baseIDForUse | OpCode::Matrix_ans3) )
        {
            throw std::runtime_error("This is none of the matrix answer (1, 2 nor 3) frame!");
        }
        
        std::tuple<float, float> ret;
        int int0 = (canFrame.data[0] << 24u | canFrame.data[1] << 16u | canFrame.data[2] << 8u | canFrame.data[3]);
        int int1 = (canFrame.data[4] << 24u | canFrame.data[5] << 16u | canFrame.data[6] << 8u | canFrame.data[7]);
        
        float float0;
        float float1;
        
        memcpy(&float0, &int0, sizeof(float));
        memcpy(&float1, &int1, sizeof(float));
        
        std::get<0>(ret) = float0;
        std::get<1>(ret) = float1;
        
        return ret;
    }
    
    template<typename T>
    std::tuple<T, T, T> toSGData(const CANframe& canFrame) const
    {
        if( (canFrame.can_id != (_baseIDForUse | OpCode::SGData_ans1)) && (canFrame.can_id != (_baseIDForUse | OpCode::SGData_ans2)))
        {
            throw std::runtime_error("This is not a SG data answer frame!");
        }
        
        std::tuple<T, T, T> ret;
        
        if( canFrame.can_id == (_baseIDForUse | OpCode::SGData_ans1) )
        {
            //TODO SHOULD BE FLOAT?!
            std::get<0>(ret) = (T)((canFrame.data[2] << 8u) | canFrame.data[3]);
            std::get<1>(ret) = (T)((canFrame.data[4] << 8u) | canFrame.data[5]);
            std::get<2>(ret) = (T)((canFrame.data[6] << 8u) | canFrame.data[7]);
        }
        else if ( canFrame.can_id == (_baseIDForUse | OpCode::SGData_ans2) )
        {
            //TODO SHOULD BE FLOAT?!
            std::get<0>(ret) = (T)((canFrame.data[0] << 8u) | canFrame.data[1]);
            std::get<1>(ret) = (T)((canFrame.data[2] << 8u) | canFrame.data[3]);
            std::get<2>(ret) = (T)((canFrame.data[4] << 8u) | canFrame.data[5]);
        }
        
        return ret;
    }

private:
    uint16_t _baseIDRaw;
    uint16_t _baseIDForUse;
};

std::ostream &operator<<(std::ostream &os, NetCANOEMCANMsg::FirmwareVersion fwv)
{
    return os << "v" << fwv.major << "." << fwv.minor << "." << fwv.buildNumber;
}

std::ostream &operator<<(std::ostream &os, std::array<std::array<float,6>,6> matrix)
{
    for(std::array<float,6>& row : matrix)
    {
        os << "\t";
        for(float& el : row)
        {
            os << std::setw(14) << el << "  ";
        }
        
        os << std::endl;
    }
    
    return os;
}

std::string forceUnitToStr(NetCANOEMCANMsg::ForceUnit fu)
{
    std::string ret;
    
    switch (fu)
    {
        case NetCANOEMCANMsg::ForceUnit::N:
            ret = "N";
            break;
        case NetCANOEMCANMsg::ForceUnit::lbf:
            ret = "lbf";
            break;
        case NetCANOEMCANMsg::ForceUnit::kN:
            ret = "kN";
            break;
        case NetCANOEMCANMsg::ForceUnit::Klbf:
            ret = "Klbf";
            break;
        case NetCANOEMCANMsg::ForceUnit::kgf:
            ret = "kgf";
            break;
        case NetCANOEMCANMsg::ForceUnit::gf:
            ret = "gf";
            break;
    }
    
    return ret;
}

std::string torqueUnitToStr(NetCANOEMCANMsg::TorqueUnit tu)
{
    std::string ret;
    
    switch (tu)
    {
        case NetCANOEMCANMsg::TorqueUnit::N_m:
            ret = "N_m";
            break;
        case NetCANOEMCANMsg::TorqueUnit::N_mm:
            ret = "N_mm";
            break;
        case NetCANOEMCANMsg::TorqueUnit::lbf_in:
            ret = "lbf_in";
            break;
        case NetCANOEMCANMsg::TorqueUnit::lbf_ft:
            ret = "lbf_ft";
            break;
        case NetCANOEMCANMsg::TorqueUnit::kN_m:
            ret = "kN_m";
            break;
        case NetCANOEMCANMsg::TorqueUnit::kgf_cm:
            ret = "kgf_cm";
            break;
    }
    
    return ret;
}

std::array<float, 6> multiply(const std::array<std::array<float, 6>, 6>& matrix, const std::array<int16_t, 6>& sg)
{
    std::array<float, 6> ret{};
    
    for(int row = 0; row < 6; row++)
    {
        for(int col = 0; col < 6; col++)
        {
            ret[row] += matrix[row][col] * sg[col];
        }
    }
    
    return ret;
}

int keepRunning = 1;

void intHandler(int dummy) {
    keepRunning = 0;
}

float sharedFZ = 0;

std::atomic<bool> keepRunningAti{true};

can_frame lelyCanFrame2linuxCanFrame(can_msg canMsg) {
    
    can_frame canFrame;
    
    canFrame.can_dlc = canMsg.len;
    
    for(int i = 0; i < 8; i++)
        canFrame.data[i] = canMsg.data[i];
    
    canFrame.can_id = canMsg.id;
    
    return canFrame;
}

can_msg linuxCanFrame2lelyCanFrame(can_frame canFrame) {
    
    can_msg canMsg;
    
    canMsg.len = canFrame.can_dlc;
    
    for(int i = 0; i < canFrame.can_dlc; i++)
        canMsg.data[i] = canFrame.data[i];
    
    canMsg.id = canFrame.can_id;
    
    return std::move(canMsg);
}

int forcesensorreceivecount = 0;

#ifdef USE_FORCETORQUE_SENSOR

void updateForceTorqueSensor(lely::io::CanChannel* chanRouter, lely::io::CanRouter* canRouter, std::mutex* fzMtx, std::array<std::array<float, 6>, 6>* matrix) {
    using namespace std::chrono_literals;
    std::array<int16_t, 6> sg{};
    std::array<float, 6> ft{}; // Fx, Fy, Fz, Tx, Ty, Tz
    NetCANOEMCANMsg::Status sensorStatus;
    can_frame canFrame;
    canFrame.can_dlc = 8;
    
    can_msg canMsg;
    
    NetCANOEMCANMsg helper{0x7F};
    //keepRunningAti = 0;
    
    const uint16_t sgData_id_ans1 = helper.getBaseID() | NetCANOEMCANMsg::OpCode::SGData_ans1;
    bool sgData_ans1_arrived = false;
    const uint16_t sgData_id_ans2 = helper.getBaseID() | NetCANOEMCANMsg::OpCode::SGData_ans2;
    bool sgData_ans2_arrived = false;
    
    /*lely::io::CanRouterReadFrame canRouterReadFrame0x7FX_SG024(sgData_id_ans1, lely::io::CanFlag::NONE,
        [=,&canMsg, &canRouter, &canRouterReadFrame0x7FX_SG024] (const can_msg* msg, ::std::error_code ec) {
            if (!ec) {
                assert(msg);
                assert(msg->id == sgData_id_ans1);
    
                std::cout << "sg0 sg2 sg4 received\t";
                std::cout << std::uppercase << std::hex << canMsg.id << " # ";
                for (auto d : canMsg.data)
                {
                    std::cout << std::uppercase << std::hex << d << "  ";
                }
                std::cout << std::endl << std::resetiosflags;
            }
            
            if (ec != std::errc::operation_canceled) canRouter->submit_read_frame(canRouterReadFrame0x7FX_SG024);
        }
    );
    
    canRouter->submit_read_frame(canRouterReadFrame0x7FX_SG024);
    
    lely::io::CanRouterReadFrame canRouterReadFrame0x7FX_SG135(sgData_id_ans1, lely::io::CanFlag::NONE,
       [=,&canMsg, &canRouter, &canRouterReadFrame0x7FX_SG135] (const can_msg* msg, ::std::error_code ec) {
           if (!ec) {
               assert(msg);
               assert(msg->id == sgData_id_ans1);
    
               std::cout << "sg1 sg3 sg5 received\t";
               std::cout << std::uppercase << std::hex << canMsg.id << " # ";
               for (auto d : canMsg.data)
               {
                   std::cout << std::uppercase << std::hex << d << "  ";
               }
               std::cout << std::endl << std::resetiosflags;
           }
    
           if (ec != std::errc::operation_canceled) canRouter->submit_read_frame(canRouterReadFrame0x7FX_SG135);
       }
    );
    canRouter->submit_read_frame(canRouterReadFrame0x7FX_SG135);*/
    
    while(keepRunningAti){
        
        canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::SGData_req);
        
        chanRouter->write(linuxCanFrame2lelyCanFrame(canFrame));
        
        bool tryAgain = false;

        tryAgain = false;
        
        uint_least32_t id;
        lely::io::CanFlag canFlag;
        
        lely::ev::Future<const can_msg *, int> f1;
        lely::ev::Future<const can_msg *, int> f2;
    
        canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::SGData_req);
        
        PID pid{10, 1, 1};
        
        do {
            bool sgData_ans1_arrived = false;
            
            f1 = canRouter->async_read_frame(sgData_id_ans1, lely::io::CanFlag::NONE);
            f2 = canRouter->async_read_frame(sgData_id_ans2, lely::io::CanFlag::NONE);
    
            //canRouter->submit_read_frame(sgData_id_ans1,  lely::io::CanFlag::NONE,[&canMsg](const can_msg* msg, ::std::error_code ec) {if (!ec) memcpy(&canMsg, msg, sizeof(can_msg)); });
    
            //canRouter->submit_read_frame(sgData_id_ans2,  lely::io::CanFlag::NONE,[&canMsg](const can_msg* msg, ::std::error_code ec) {if (!ec) memcpy(&canMsg, msg, sizeof(can_msg));});
            
            chanRouter->write(linuxCanFrame2lelyCanFrame(canFrame), 0);
    
            while (!f1.is_ready()) {
                std::this_thread::yield();
            }
            while (!f2.is_ready()) {
                std::this_thread::yield();
            }
    
            forcesensorreceivecount++;
            
            //std::cout << "f1\t" << f1.get().value()->id << "\t" << std::uppercase << std::hex << (uint8_t)f1.get().value()->data[0] << std::resetiosflags;
            //std::cout << std::endl;
    
            //std::cout << "f2\t" << f2.get().value()->id << "\t" << std::uppercase << std::hex << (uint8_t)f2.get().value()->data[0] << std::resetiosflags;
            //std::cout << std::endl;
            
            std::tie(sg[0], sg[2], sg[4]) = helper.toSGData<int16_t>(lelyCanFrame2linuxCanFrame(*f1.get().value()));
    
            sensorStatus = helper.toStatus(lelyCanFrame2linuxCanFrame(*f1.get().value()));
    
            std::tie(sg[1], sg[3], sg[5]) = helper.toSGData<int16_t>(lelyCanFrame2linuxCanFrame(*f2.get().value()));
    
    
            ft = multiply(*matrix, sg);
    
            fzMtx->lock();
            sharedFZ = ft[2];
            fzMtx->unlock();

            //std::cout << "fz: " << sharedFZ << "\n";
    
            //std::cout << "FT: ";
            //std::for_each(ft.begin(), ft.end(), [](const float& v) { std::cout << std::setw(10) << std::setprecision(3) << v; });
            //std::cout << "\t\tpower supply low: " << sensorStatus.b05PowerSupplyLow;
            //std::cout << std::endl;
    
            //usleep(10000);
            
            //std::this_thread::yield();
            
            std::this_thread::sleep_for(5ms);
            
        }while(keepRunningAti);
    }
}

#endif // USE_FORCETORQUE_SENSOR

/*------- END ATI-NETCANOEM-SENSOR ------- */

constexpr float referencePosition = -3.141596f/3.f;
constexpr float gainP = 1000.f;
constexpr float gainI = 0.f;
constexpr float gainD = 100.f;

constexpr float gainKy = 20.f;
constexpr float gainBy = 8.f;

PID pidController{gainP, gainI, gainD, referencePosition};
InteractionController intCtrl{gainKy, gainBy};


// Get time stamp in microseconds. From here: https://stackoverflow.com/a/49066369/6609908
uint64_t micros()
{
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
                                                                        now().time_since_epoch()).count();
    return us;
}

uint64_t lastTime_us = 0;
uint64_t currentTime_us = 0;
uint64_t max_dt_us = 0;
uint64_t min_dt_us = 9999999999;
uint64_t mean_dt_us = 0;
uint64_t dt_count = 0;
uint64_t dt_us = 0;

using namespace std::chrono_literals;
using namespace lely;

// This driver inherits from FiberDriver, which means all CANopen event
// callbacks, such as OnBoot, run as a task inside a "fiber" (or stackful
// coroutine).
class MyDriver : public canopen::FiberDriver {
public:
    using FiberDriver::FiberDriver;
    
private:
    //This function gets called when the boot-up process of the slave completes.
    // The 'st' parameter contains the last known NMT state of the slave
    // (typically pre-operation), 'es' the error code (0 on success), and 'what'
    // a description of the error, if any.
    void
    OnBoot(canopen::NmtState /*st*/, char es,
           const std::string& what) noexcept override {
        if (!es || es == 'L')
        {
            std::cout << "slave " << static_cast<int>(id()) << " booted successfully"
                      << std::endl;
        } else {
            std::cout << "slave " << static_cast<int>(id())
            << " failed on boot: " << what << std::endl;
        }
    }
    
    // This function gets called during the boot-up process for the slave. The
    // 'res' parameter is the function that MUST be invoked when the configuration
    // is complete. Because this function runs as a task inside a coroutine, it
    // can suspend itself and wait for an asynchronous function such as an SDO
    // request, to complete.
    void
    OnConfig(std::function<void(std::error_code ec)> res) noexcept override {
        try {
            // Perform a few SDO write requests to configure the slave. The
            // AsyncWrite() function returns a future which becomes ready once the
            // request completes, and the Wait()     function suspends the coroutine for
            // this task until the future is ready
            
            // Configure the slave to monitor the heartbeat of the master (node-ID 2)
            // with a timeout of 2000 ms. about 2 << 16 : 2 is the node-ID
            //Wait(AsyncWrite<uint32_t>(0x1016, 1, (2 << 16) | 2000));
            // Configure the slave to produce a heartbeat every 1000 ms.
            //Wait(AsyncWrite<uint16_t>(0x1017, 0, 2000));
            // Configure the heartbeat consumer on the master.
            //ConfigHeartbeat(1900ms);
            
//            // set CAN bitrate (baud rate) to 250kbps
//            Wait(AsyncWrite<uint16_t>(0x2001, 0, 3));
//            // save all parameters
//            Wait(AsyncWrite<uint32_t>(0x1010, 1, 0x73617665));
    
            std::cout << "\n--- 0x1803:03 " << "on config ---\n";
            // set inhibit time to 10 * 100us for PDO1
            Wait(AsyncWrite<uint16_t>(0x1800, 3, 10));
            // set inhibit time to 10 * 100us for PDO2
            Wait(AsyncWrite<uint16_t>(0x1801, 3, 10));
            // set inhibit time to 10 * 100us for PDO3
            Wait(AsyncWrite<uint16_t>(0x1802, 3, 10));
            // set inhibit time to 10 * 100us for PDO4
            Wait(AsyncWrite<uint16_t>(0x1803, 3, 10));
    
            //SetProfileVelocityMode();
            //SetProfilePositionMode();
            //SetPositionMode(); // NOT WORKING!
            SetCurrentMode();
            // Report success (empty error code).
    
            lastTime_us = micros();
            
            res({});
        } catch (canopen::SdoError& e) {
            // If one of the SDO requests resulted in an error, abort the
            // configuration and report the error code.
            res(e.code());
        }
    }
    
    void SetProfilePositionMode(uint32_t maxFollowingError = 2000, /* qc 0x6065.00 */
                         int32_t minPositionLimit = -2147483648, /* qc 0x607D.01 */
                         int32_t maxPositionLimit = 2147483647, /* qc 0x607D.02 */
                         uint32_t maxProfileVelocity = 25000, /* rpm 0x607F.00 */
                         uint32_t profileVelocity = 1000, /* rpm 0x6081.00 */
                         uint32_t profileAcceleration = 10000, /* rpm/s 0x6083.00 */
                         uint32_t profileDeceleration = 10000, /* rpm/s 0x6084.00 */
                         uint32_t quickstopDeceleration = 10000, /* rpm/s 0x6085.00 */
                         int16_t motionProfileType = 0 /* 0 for linear or 1 for sin² 0x6086.00 */) {
        
        master.Command(lely::canopen::NmtCommand::STOP, id());
        master.Command(lely::canopen::NmtCommand::ENTER_PREOP, id());
        
        // set profile position operation mode
        Wait(AsyncWrite<uint8_t>(0x6060, 0, 0x01));
        
        // set max following error
        Wait(AsyncWrite<uint32_t>(0x6065, 0, reinterpret_cast<uint32_t &&>(maxFollowingError)));
        // set min position limit
        Wait(AsyncWrite<int32_t>(0x607D, 1,reinterpret_cast<int32_t &&>(minPositionLimit)));
        // set max position limit
        Wait(AsyncWrite<int32_t>(0x607D, 2,reinterpret_cast<int32_t &&>(maxPositionLimit)));
        
        // set max profile velocity
        Wait(AsyncWrite<uint32_t>(0x607F, 0, reinterpret_cast<uint32_t &&>(maxProfileVelocity)));
        
        // set profile velocity
        Wait(AsyncWrite<uint32_t>(0x6081, 0, reinterpret_cast<uint32_t &&>(profileVelocity)));
        
        // set profile acceleration
        Wait(AsyncWrite<uint32_t>(0x6083, 0, reinterpret_cast<uint32_t &&>(profileAcceleration)));
        
        // set profile deceleration
        Wait(AsyncWrite<uint32_t>(0x6084, 0, reinterpret_cast<uint32_t &&>(profileDeceleration)));
        
        // set quick stop deceleration
        Wait(AsyncWrite<uint32_t>(0x6085, 0, reinterpret_cast<uint32_t &&>(quickstopDeceleration)));
        
        // set motion profile type
        Wait(AsyncWrite<int16_t>(0x6086, 0, reinterpret_cast<int16_t &&>(motionProfileType)));
        
        Shutdown();
        SwitchOn();
        
        master.Command(lely::canopen::NmtCommand::START, id());
        
        // set target position
        Wait(AsyncWrite<int32_t>(0x607A, 0, 0));
        // set Controlword to start absolute positioning immediately
        StartAbsPositioningImmediate();
    }
    
    
    
    void SetCurrentMode(uint16_t continuousCurrentLimit = 5000, /* mA 0x6410.01 */
                                uint16_t maxSpeed = 9500, /* rpm 0x6410.04 */
                                uint16_t thermalTimeConstantWinding = 70 /* ? 0x6410.05 */
                                ) {
        
        master.Command(lely::canopen::NmtCommand::STOP, id());
        master.Command(lely::canopen::NmtCommand::ENTER_PREOP, id());
        
        // set profile position operation mode
        Wait(AsyncWrite<int8_t>(0x6060, 0, 0xFD));
        
        
        Wait(AsyncWrite<uint16_t>(0x6410, 1, reinterpret_cast<uint16_t &&>(continuousCurrentLimit)));
        
        Wait(AsyncWrite<uint16_t>(0x6410, 4, reinterpret_cast<uint16_t &&>(maxSpeed)));
        
        Wait(AsyncWrite<uint16_t>(0x6410, 5,reinterpret_cast<uint16_t &&>(thermalTimeConstantWinding)));
        
        Shutdown();
        SwitchOn();
        
        master.Command(lely::canopen::NmtCommand::START, id());
        
        // set target position
        //Wait(AsyncWrite<int16_t>(0x2030, 0, 0));
    }
    
    void SetPositionMode(uint32_t maxFollowingError = 2000, /* qc 0x6065.00 */
                                int32_t minPositionLimit = -2147483648, /* qc 0x607D.01 */
                                int32_t maxPositionLimit = 2147483647 /* qc 0x607D.02 */) {
        
        master.Command(lely::canopen::NmtCommand::STOP, id());
        master.Command(lely::canopen::NmtCommand::ENTER_PREOP, id());
        
        // set profile position operation mode
        Wait(AsyncWrite<uint8_t>(0x6060, 0, 0xFF));
        
        // set max following error
        Wait(AsyncWrite<uint32_t>(0x6065, 0, reinterpret_cast<uint32_t &&>(maxFollowingError)));
        // set min position limit
        Wait(AsyncWrite<int32_t>(0x607D, 1,reinterpret_cast<int32_t &&>(minPositionLimit)));
        // set max position limit
        Wait(AsyncWrite<int32_t>(0x607D, 2,reinterpret_cast<int32_t &&>(maxPositionLimit)));
        
        Shutdown();
        SwitchOn();
        
        master.Command(lely::canopen::NmtCommand::START, id());
        
        // set target position
        //Wait(AsyncWrite<int32_t>(0x2062, 0, 0));
        // set Controlword to start absolute positioning immediately
        
        StartAbsPositioningImmediate();
    }
    
    void SetProfileVelocityMode(uint32_t maxProfileVelocity = 25000, /* rpm */
                                uint32_t profileAcceleration = 10000, /* rpm/s */
                                uint32_t profileDeceleration = 10000, /* rpm/s */
                                uint32_t quickStopDeceleration = 10000, /* rpm/s */
                                uint8_t motionProfileType = 0 /* 0: linear, 1: sin^2 */) {
        
        master.Command(lely::canopen::NmtCommand::STOP, id());
        master.Command(lely::canopen::NmtCommand::ENTER_PREOP, id());
        
        // set profile velocity operation mode
        Wait(AsyncWrite<uint8_t>(0x6060, 0, 0x03));
        
        // set max profile  veloocity
        Wait(AsyncWrite<uint32_t>(0x607F, 0, reinterpret_cast<unsigned int &&>(maxProfileVelocity)));
        // set profile acceleration
        Wait(AsyncWrite<uint32_t>(0x6083, 0, reinterpret_cast<unsigned int &&>(profileAcceleration)));
        // set profile deceleration
        Wait(AsyncWrite<uint32_t>(0x6084, 0, reinterpret_cast<unsigned int &&>(profileDeceleration)));
        // set quick stop deceleration
        Wait(AsyncWrite<uint32_t>(0x6085, 0, reinterpret_cast<unsigned int &&>(quickStopDeceleration)));
        // set motion profile
        Wait(AsyncWrite<uint16_t>(0x6086, 0, reinterpret_cast<unsigned int &&>(motionProfileType)));
        
        Shutdown();
        SwitchOn();
    
        master.Command(lely::canopen::NmtCommand::START, id());
    
        // set target velocity
        Wait(AsyncWrite<int32_t>(0x60FF, 0, 0));
        // set Controlword
        SwitchOn();
    }
    
    void Shutdown() {
        Wait(AsyncWrite<uint16_t>(0x6040, 0, 0x0006));
    }
    
    void SwitchOn() {
        Wait(AsyncWrite<uint16_t>(0x6040, 0, 0x000F));
    }
    
    void StartAbsPositioningImmediate() {
        Wait(AsyncWrite<uint16_t>(0x6040, 0, 0x003F));
    }
    
    int maxVel = 2000;
    int minVel = -2000;
    
    void OnSync(uint8_t cnt, const time_point& t) noexcept override {
        //std::cout << std::uppercase << std::hex << idx << ":" << subidx << std::resetiosflags << " ";
        /*
        if(velocity > maxVel)
        {
            std::cout << "MAIOR QUE " << maxVel << "\n";
            direction *= -1;
            velocity = maxVel;
        }
        else if(velocity < minVel)
        {
            std::cout << "MENOR QUE " << minVel << "\n";
            direction *= -1;
            velocity = minVel;
        }
        */
        //tpdo_mapped[0x60FF][0] = static_cast<int32_t>(velocity);
        
        //for profile position
        //tpdo_mapped[0x607A][0] = static_cast<int32_t>((sharedFZ + 21)*10);
        //tpdo_mapped[0x6040][0] = static_cast<uint16_t>(0x003F);
        //master.TpdoEvent(2);
    
        //for non-profile position
        //std::cout << "sharedFZ transformed: " << (sharedFZ + 21.f)*10.f << std::endl;
        //Wait(AsyncWrite<int32_t>(0x2062, 0, static_cast<int32_t>((sharedFZ + 21.f)*10.f)));
        
        //for velocity
        //tpdo_mapped[0x60FF][0] = static_cast<int32_t>((sharedFZ + 21)*10*2);
        //tpdo_mapped[0x6040][0] = static_cast<uint16_t>(0x000F);
        //master.TpdoEvent(3);
    
        //for current position MOTORZAO
        //std::cout << "sharedFZ transformed: " << static_cast<int16_t>(sharedFZ) << std::endl
        //<< "actual position: " << static_cast<int32_t>(rpdo_mapped[0x6064][0]) << std::endl
        //<< "actual velocity: " << static_cast<int32_t>(rpdo_mapped[0x606C][0]) << std::endl
        //<< "actual current: " << static_cast<int16_t>(rpdo_mapped[0x6078][0]) << std::endl;
        ////Wait(AsyncWrite<int32_t>(0x2030, 0, static_cast<int32_t>(sharedFZ + 21.f)));
        //tpdo_mapped[0x2030][0] = static_cast<int16_t>(sharedFZ);
        //tpdo_mapped[0x6040][0] = static_cast<uint16_t>(0x0003);
        //master.TpdoEvent(1);
        
        currentTime_us = micros();
        dt_us = currentTime_us - lastTime_us;
        
        //for PID controller
        // angular position in radians
        currentPosition = (static_cast<int32_t>(rpdo_mapped[0x6064][0])*2.f*3.141596f/2000.f)/gear;
        
        // angular velocity in rad/s
        currentVelocity = (static_cast<int32_t>(rpdo_mapped[0x606C][0])*3.141596f/30.f)/gear;
        
        // torque in Newton*meter
        //currentTorque = ((static_cast<int16_t>(rpdo_mapped[0x6078][0])/1000.f)*motorzaoTorqueConstant/1000.f)*gear;
        
        //currentTorque from force-torque-sensor
        currentTorque = (sharedFZ+21)*.1f;
        
        //currentTorque = intCtrl.getTorqueFromObserver(controlSignal, currentVelocity, dt_us/1000000.f);
        
        angularCorrection = intCtrl.getControlSignal(currentTorque, dt_us/1000000.f);
        
        controlSignal = -pidController.getControlSignal(currentPosition - angularCorrection, currentVelocity, 0);
        
        if (std::fabs(controlSignal) > 5000) {
            //std::cout << "controlSignal is too large: " << controlSignal << ". limiting to 5000" << std::endl;
            controlSignal = 4500 * ( (float{0} < controlSignal) - (controlSignal < float{0}) );
        }
        
        tpdo_mapped[0x2030][0] = static_cast<int16_t>(controlSignal);
        tpdo_mapped[0x6040][0] = static_cast<uint16_t>(0x0003);
    
    
        if (logCounter < logSize) {
            logPosition[logCounter] = currentPosition;
            logForceZ[logCounter] = currentTorque;
            logDt[logCounter] = dt_us/1000.f;
            logCounter++;
            //std::cout << logCounter << std::endl;
        }
        else
        {
            //std::cout << logSize << " data points recorded!" << std::endl;
        }
        
        /*if(++printCounter % 10 == 0) {
            printCounter = 0;
            std::cout << "pos:\t" << std::setw(10) << std::setprecision(2) << currentPosition << ";"
                      << "\trawPos;\t" << std::setw(10) << std::setprecision(2)
                      << static_cast<int32_t>(rpdo_mapped[0x6064][0]) << ";"
                      << "\tvel;\t" << std::setw(10) << std::setprecision(2) << currentVelocity << ";"
                      << "\tcur;\t" << std::setw(10) << std::setprecision(2) << currentTorque << ";"
                      << "\tu;\t" << std::setw(10) << std::setprecision(2) << controlSignal << ";"
                      << "\tposE;\t" << std::setw(10) << std::setprecision(2) << pidController.errorPosition << ";"
                      << "\tdt;\t" << std::setw(15) << std::setprecision(2) << dt_us << ";" << std::endl;
        }*/
        
        //max_dt_us = std::max(max_dt_us, dt_us) > 7840493192397 ? 0 : std::max(max_dt_us, dt_us);
        //min_dt_us = std::min(min_dt_us, dt_us);
    
        lastTime_us = currentTime_us;
        //dt_count++;
        //mean_dt_us += dt_us;
        
        /*if (abs((int)velocity % 100) == 3)
        {
            std::cout << "velocity: " << std::setw(5) << (int32_t)tpdo_mapped[0x60FF][0] << "\tctrlword: " << std::setw(5) << (uint16_t)tpdo_mapped[0x6040][0] << "\tflag: " << std::setw(5) << flag << "\n";
        }*/
  
        //velocity = velocity + direction;
    }
    
    // This function is similar to OnConfig(), but it gets called by the
    // AsyncDeconfig() method of the master.
    void
    OnDeconfig(std::function<void(std::error_code ec)> res) noexcept override {
        try
        {
    
            master.Command(lely::canopen::NmtCommand::ENTER_PREOP);
            
            std::cout << "Fiber Deconfig\n";
            //Disable the heartbeat consumer on the master.
            ConfigHeartbeat(0ms);
            // Disable the heartbeat producer on the slave.
            Wait(AsyncWrite<uint16_t>(0x1017, 0, 0));
            // Disable the heartbeat consumer on the slave.
            Wait(AsyncWrite<uint32_t>(0x1016, 1, 0));
    
            master.Command(lely::canopen::NmtCommand::RESET_NODE);
            
            std::cout << "max_dt_ms: " << max_dt_us << "\tmin_dt_ms: " << min_dt_us << "\tmean_dt_ms: " << mean_dt_us / (1.f * dt_count) << std::endl;
            
            // shutdown
            Shutdown();
    
            res({});
        } catch (canopen::SdoError& e) {
            res(e.code());
        }
    }
    
    float velocity = maxVel;
    float flag = 0;
    float direction = 3;
    float currentPosition;
    float currentVelocity;
    float currentTorque;
    float motorzaoTorqueConstant = 38.5f;
    float gear = 3.5f;
    float controlSignal = 0;
    
    float angularCorrection = 0;
    
    int printCounter = 0;
};

class MyCoTask : public lely::ev::CoTask {
public:
    
    MyCoTask(int& forceZ, lely::canopen::AsyncMaster& master) :
    lely::ev::CoTask(), forceZ(forceZ), master(master)
    {}
    
    virtual void
    operator()() noexcept override
    {
        co_reenter (*this) {
            while(forceZ < 100) {
                //master.TpdoWrite(1, 0x206B, 0, 20+forceZ);
                //std::cout << forceZ++;
                //master.
                forceZ++;
                co_yield get_executor().post((ev_task&)(*this));
            }
        }
    }

private:
    int& forceZ;
    lely::canopen::AsyncMaster& master;
    
};


int
main() {
    // Initialize the I/O library. This is required on Windows, but a no-op on
    // Linux (for now).
    io::IoGuard io_guard;
#if _WIN32
    // Load vcinpl2.dll (or vcinpl.dll if CAN FD is disabled).
  io::IxxatGuard ixxat_guard;
#endif
    // Create an I/O context to synchronize I/O services during shutdown.
    io::Context ctx;
    // Create an platform-specific I/O polling instance to monitor the CAN bus, as
    // well as timers and signals.
    io::Poll poll(ctx);
    // Create a polling event loop and pass it the platform-independent polling
    // interface. If no tasks are pending, the event loop will poll for I/O
    // events.
    ev::Loop loop(poll.get_poll());
    // I/O devices only need access to the executor interface of the event loop.
    auto exec = loop.get_executor();
    // Create a timer using a monotonic clock, i.e., a clock that is not affected
    // by discontinuous jumps in the system time.
    io::Timer timer(poll, exec, CLOCK_MONOTONIC);
#if _WIN32
    // Create an IXXAT CAN controller and channel. The VCI requires us to
  // explicitly specify the bitrate and restart the controller.
  io::IxxatController ctrl(0, 0, io::CanBusFlag::NONE, 125000);
  ctrl.restart();
  io::IxxatChannel chanCANopenMaster(ctx, exec);
#elif defined(__linux__)
    // Create a virtual SocketCAN CAN controller and channel, and do not modify
    // the current CAN bus state or bitrate.
    io::CanController ctrl("can1");
    io::CanChannel chanCANopenMaster(poll, exec);
    io::CanChannel chanRouter(poll, exec);
#endif
    chanCANopenMaster.open(ctrl);
    chanRouter.open(ctrl);
    
//#define NO_SLAVE
//#define NO_MASTER
#ifdef NO_MASTER
    #define NO_SLAVE
#endif //NO_MASTER

#ifndef NO_MASTER
    
    // Create a CANopen master with node-ID 2. The master is asynchronous, which
    // means every user-defined callback for a CANopen event will be posted as a
    // task on the event loop, instead of being invoked during the event
    // processing by the stack
    canopen::AsyncMaster master(timer, chanCANopenMaster, "/home/debian/lely-bbb/master-dcf-motorzao-current-3000.dcf", "/home/debian/lely-bbb/master-dcf-motorzao-current-3000.bin", 2);
    //canopen::AsyncMaster master(timer, chanCANopenMaster, "/home/debian/lely-bbb/master-motorzinho.dcf", "/home/debian/lely-bbb/master-motorzinho.bin", 2);
/*    master.OnRpdo([&](int i, ::std::error_code ec, const void* numsei, ::std::size_t s){
        std::cout << i << "\t" << ec << "\t" << numsei << "\t" << s << "\n";
    });
    */
    /*master.OnSync([&](int i, ::std::error_code ec, const void* numsei,::std::size_t s){
        std::cout << i << "\t" << ec << "\t" << numsei << "\t" << s << "\n";
    });*/
    
#endif // !NO_MASTER

#if !defined(NO_MASTER) || !defined(NO_SLAVE)
    // Create a driver for the slave with node-ID 1.
    MyDriver driver(exec, master, 1);
#endif // !defined(NO_MASTER) && !defined(NO_SLAVE)

    // Create a signal handler.
    io::SignalSet sigset(poll, exec);
    //Watch for Ctrl+C or process termination.
    sigset.insert(SIGHUP);
    sigset.insert(SIGINT);
    sigset.insert(SIGTERM);
    
    //Submit a task to be executed when a signal is raised. We don't care which.
    sigset.submit_wait([&](int /*signo*/) {
        // If the signal is raised again, terminate immediately.
        sigset.clear();
    
        keepRunningAti = false;
#ifndef NO_MASTER
        // Tell the master to start de deconfiguration process for node 1, and
        // submit a task to be executed once that process completes.
        master.AsyncDeconfig(1).submit(exec, [&]() {
            // Perform a clean shutdown.
#endif // !NO_MASTER
            std::cout << std::endl << "LIVELY 1111111!" << std::endl;
            
            ctx.shutdown();
#ifndef NO_MASTER
            std::cout << std::endl << "LIVELY 2222222!" << std::endl;
        });
#endif // !NO_MASTER
    });

#ifndef NO_MASTER
    //Start the NMT service of the master by pretending to receive a 'reset
    // node' command.
    master.Reset();
    
    master.Command(lely::canopen::NmtCommand::RESET_NODE);
    master.Command(lely::canopen::NmtCommand::RESET_COMM);
    master.Command(lely::canopen::NmtCommand::STOP);
    master.Command(lely::canopen::NmtCommand::ENTER_PREOP);
    
    // making TPDO 1 synchronous
    master.Write<uint8_t>(0x1800, 2, 0x01);
    // making TPDO 2 synchronous
    master.Write<uint8_t>(0x1801, 2, 0x01);
    // making TPDO 3 synchronous
    master.Write<uint8_t>(0x1802, 2, 0x01);
    // making TPDO 4 synchronous
    //master.Write<uint8_t>(0x1803, 2, 0x01);
#endif // !NO_MASTER

#if _WIN32
    // Create two worker threads to ensure the blocking canChannelReadMessage()
    // and canChannelSendMessage() used by the IXXAT CAN channel do not hold up
    // the event loop.
    std::thread workers[] = {std::thread([&]() { loop.run(); }),
                           std::thread([&]() { loop.run(); })};
#endif

#ifdef USE_FORCETORQUE_SENSOR
/*--------------INIT ATI-NETCANOEM--------------*/
    int socketCan;
    
    if ((socketCan = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket");
        return 1;
    }
    
    SockAddrCan addr;
    
    InterfReq ifr;
    strcpy(ifr.ifr_name, "can1" );
    ioctl(socketCan, SIOCGIFINDEX, &ifr);
    
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(socketCan, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        return 1;
    }
    
    NetCANOEMCANMsg helper{0x7F};
    
    CANframe canFrame;
    
    /*std::cout << "asking for transducer calibration matrix" << std::endl;
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::SerialNumber_req);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    std::string serialNumber = "";
    
    serialNumber = helper.toSerial(canFrame);
    
    std::cout << "serial: " << serialNumber << std::endl;
    
    
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::FirmwareVersion_req);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    read(socketCan, &canFrame, sizeof(canFrame));
    
    auto firmwareVersion = helper.toFirmwareVersion(canFrame);
    
    std::cout << "firmware version: " << firmwareVersion << std::endl;
    */
    
    bool tryAgain;
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::SetActiveCalibration_req, 0);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    int calibraiontIndex;
    
    tryAgain = false;
    do
    {
        read(socketCan, &canFrame, sizeof(canFrame));
        try {
            calibraiontIndex = helper.toCalibrationIndexSelected(canFrame);
        }
        catch (std::runtime_error e)
        {
            tryAgain = true;
            continue;
        }
        tryAgain = false;
    } while(tryAgain);
    
    std::cout << "selected calibration index: " << calibraiontIndex << std::endl;
    
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::CountsPerUnits_req);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    NetCANOEMCANMsg::CountsPerUnit countsPer;
    
    tryAgain = false;
    do
    {
        read(socketCan, &canFrame, sizeof(canFrame));
        try
        {
            countsPer = helper.toCountsPerUnit(canFrame);
        }
        catch (std::runtime_error e)
        {
            tryAgain = true;
            continue;
        }
        tryAgain = false;
    } while(tryAgain);
    
    std::cout << "Counts per force: " << countsPer.Force << std::endl;
    std::cout << "Counts per torque: " << countsPer.Torque << std::endl;
    
    
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::UnitCodes_req);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    NetCANOEMCANMsg::ForceTorqueUnit unitCode;
    
    tryAgain = false;
    do
    {
        read(socketCan, &canFrame, sizeof(canFrame));
        try
        {
            unitCode = helper.toForceTorqueUnit(canFrame);
        }
        catch (std::runtime_error e)
        {
            tryAgain = true;
            continue;
        }
        tryAgain = false;
    } while(tryAgain);
    
    std::cout << "force unit: " << forceUnitToStr(unitCode.Force) << std::endl;
    std::cout << "torque unit: " << torqueUnitToStr(unitCode.Torque) << std::endl;
    
    std::cout << "Diagnostic ADC voltages:" << std::endl;
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::DiagADCVoltages_req,NetCANOEMCANMsg::DiagnosticADCVoltage::MID_VSG);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    int mid_vsg_volt;
    
    tryAgain = false;
    do
    {
        read(socketCan, &canFrame, sizeof(canFrame));
        try
        {
            mid_vsg_volt = helper.toDiagnosticADCVoltage(canFrame);
        }
        catch (std::runtime_error e)
        {
            tryAgain = true;
            continue;
        }
        tryAgain = false;
    } while(tryAgain);
    
    std::cout << "\tMID_VSG voltage\t\t" << mid_vsg_volt << "V" << std::endl;
    
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::DiagADCVoltages_req,NetCANOEMCANMsg::DiagnosticADCVoltage::Thermistor);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    int thermistor_volt;
    
    tryAgain = false;
    do
    {
        read(socketCan, &canFrame, sizeof(canFrame));
        try
        {
            thermistor_volt = helper.toDiagnosticADCVoltage(canFrame);
        }
        catch (std::runtime_error e)
        {
            tryAgain = true;
            continue;
        }
        tryAgain = false;
    } while(tryAgain);
    
    std::cout << "\tThermistor voltage\t" << thermistor_volt << "V" << std::endl;
    
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::DiagADCVoltages_req,NetCANOEMCANMsg::DiagnosticADCVoltage::Power);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    int power_volt;
    
    tryAgain = false;
    do
    {
        read(socketCan, &canFrame, sizeof(canFrame));
        try
        {
            power_volt = helper.toDiagnosticADCVoltage(canFrame);
        }
        catch (std::runtime_error e)
        {
            tryAgain = true;
            continue;
        }
        tryAgain = false;
    } while(tryAgain);
    
    std::cout << "\tPower voltage\t\t" << power_volt << "V" << std::endl;
    
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::DiagADCVoltages_req,NetCANOEMCANMsg::DiagnosticADCVoltage::DAC);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    int dac_volt;
    
    tryAgain = false;
    do
    {
        read(socketCan, &canFrame, sizeof(canFrame));
        try
        {
            dac_volt = helper.toDiagnosticADCVoltage(canFrame);
        }
        catch (std::runtime_error e)
        {
            tryAgain = true;
            continue;
        }
        tryAgain = false;
    } while(tryAgain);
    
    std::cout << "\tDAC voltage\t\t\t" << dac_volt << "V" << std::endl;
    
    
    canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::DiagADCVoltages_req,NetCANOEMCANMsg::DiagnosticADCVoltage::Ground);
    
    write(socketCan, &canFrame, sizeof(canFrame));
    
    int ground_volt;
    
    tryAgain = false;
    do
    {
        read(socketCan, &canFrame, sizeof(canFrame));
        try
        {
            ground_volt = helper.toDiagnosticADCVoltage(canFrame);
        }
        catch (std::runtime_error e)
        {
            tryAgain = true;
            continue;
        }
        tryAgain = false;
    } while(tryAgain);
    
    std::cout << "\tGround voltage\t\t" << ground_volt << "V" << std::endl;
    
    std::cout << std::endl << std::endl << "\t\tCOMO SE LÊ ESTA CARAIA?!" << std::endl << std::endl << std::endl ;
    
    
    std::cout << "Reading Matrix" << std::endl;
    
    /// [[FxSG0, FxSG1, FxSG2, FxSG3, FxSG4, FxSG5],
    /// [FySG0, FySG1, FySG2, FySG3, FySG4, FySG5],
    /// [FzSG0, FzSG1, FzSG2, FzSG3, FzSG4, FzSG5],
    /// [TxSG0, TxSG1, TxSG2, TxSG3, TxSG4, TxSG5],
    /// [TySG0, TySG1, TySG2, TySG3, TySG4, TySG5],
    /// [TzSG0, TzSG1, TzSG2, TzSG3, TzSG4, TzSG5]]
    std::array<std::array<float, 6>, 6> matrix{};
    
    for(int row = 0; row < 6; row++)
    {
        
        canFrame = helper.getCANFrameRequest(NetCANOEMCANMsg::OpCode::Matrix_req, (NetCANOEMCANMsg::Axis)row);
        
        write(socketCan, &canFrame, sizeof(canFrame));
        
        for(int col = 0; col < 6; col=col+2)
        {
            read(socketCan, &canFrame, sizeof(canFrame));
            
            std::tie(matrix[row][col], matrix[row][col+1]) = helper.toMatrixElements(canFrame);
        }
    }
    
    std::cout << "Matrix: " << std::endl;
    std::cout << matrix << std::endl;
    
    for(int row = 0; row < 3; row++)
    {
        for(int col = 0; col < 6; col++)
        {
            matrix[row][col] *= 1/countsPer.Force;
        }
    }
    
    for(int row = 3; row < 6; row++)
    {
        for(int col = 0; col < 6; col++)
        {
            matrix[row][col] *= 1/countsPer.Torque;
        }
    }
    
    std::cout << "Matrix divided by counts: " << std::endl;
    std::cout << matrix << std::endl;
    
    lely::io::CanRouter canRouter(chanRouter, exec);
    
    std::mutex fzMtx;
    std::thread atiThread(updateForceTorqueSensor, &chanRouter, &canRouter, &fzMtx, &matrix);
    
    //TODO create a bias?!
#endif //USE_FORCETORQUE_SENSOR
/*--------------END ATI-NETCANOEM--------------*/

    //MyCoTask myCoTask(fz, master);
    //loop.get_executor().post(myCoTask);
    // Run the event loop until no tasks remain (or the I/O context is shut down).
    loop.run();
    
    //std::this_thread::sleep_for(5000ms);
    
#ifdef USE_FORCETORQUE_SENSOR
    if(atiThread.joinable())
    {
        keepRunningAti = false;
        std::cout << "esperando atiThread dar join\n";
        atiThread.join();
        std::cout << "atiThread deu join!\n";
    }
#endif //USE_FORCETORQUE_SENSOR
    std::cout <<"\n\nACABOU!\n\n";
    
#if _WIN32
    // Wait for the worker threads to finish.
  for (auto& worker : workers) worker.join();
#endif
    
    std::ofstream logPositionForceZ;
    logPositionForceZ.open("/home/debian/lely-bbb/qd" + std::to_string(referencePosition) + "_K" + std::to_string(gainKy) + "_B" + std::to_string(gainBy)+ ".csv");
    
    logPositionForceZ   << std::setw(20) << std::setprecision(5) << "q [rad]" << ","
                        << std::setw(20) << std::setprecision(5) << "tau [Nm]" << ","
                        << std::setw(20) << std::setprecision(5) << "dt [ms]" << std::endl;
    for(int i = 0; i < logSize; ++i) {
        logPositionForceZ   << std::setw(20) << std::setprecision(5) << logPosition[i] << ","
                            << std::setw(20) << std::setprecision(5) << logForceZ[i] << ","
                            << std::setw(20) << std::setprecision(5) << logDt[i] << std::endl;
    }
    
    logPositionForceZ.close();
    
    return 0;
}
