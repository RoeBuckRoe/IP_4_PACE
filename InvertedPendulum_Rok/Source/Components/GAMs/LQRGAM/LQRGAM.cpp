#include "LQRGAM.h"
#include "MotorSTM32Constants.h"

#include "stdio.h"

#include "AdvancedErrorManagement.h"

#include <algorithm>
//TODO remove debugging
#include <iostream>
namespace InvertedPendulum {

namespace {

const MARTe::float32 twoPi = 6.28318530718f;
const MARTe::int32 encoderStepsInCircle = 2400;
const MARTe::int32 encoderStepsInHalfCircle = encoderStepsInCircle / 2;
const MARTe::int32 motorStepsInCircle = 3200;
const MARTe::int32 motorStepsInThirdOfCircle = motorStepsInCircle / 3;

/**
 * @param position Measured encoder position.
 * @oaram positionBottom Bottom position of the encoder.
 * @return Normalized measured position (where 0 steps represents bottom position).
 */
MARTe::int32 normalizeEncoderPos(MARTe::uint32 position,
                                 MARTe::uint32 positionBottom) {
    MARTe::int32 positionNorm;
    int bottomNorm;
    if (position > positionBottom) {
        if (position - positionBottom > encoderStepsInCircle) {
            positionNorm = static_cast<MARTe::int32>(position) - (1 << 16);
        }
        else {
            positionNorm = static_cast<MARTe::int32>(position);
        }
        bottomNorm = static_cast<MARTe::int32>(positionBottom);
    }
    else {
        if (positionBottom - position > encoderStepsInCircle) {
            bottomNorm = static_cast<MARTe::int32>(positionBottom) - (1 << 16);
        }
        else {
            bottomNorm = static_cast<MARTe::int32>(positionBottom);
        }
        positionNorm = static_cast<MARTe::int32>(position);
    }

    return positionNorm - bottomNorm;
}

/**
 * @return True if `x` and `y` have opposite signs. Else false.
 */
bool oppositeSigns(MARTe::int32 x, MARTe::int32 y)
{
    return ((x ^ y) >> 31) != 0;
}

/**
 * @param position Pendulum position in encoder steps.
 * @return Pendulum angle in radians.
 */
MARTe::float32 encoderStepsToRadians(MARTe::int32 position) {
    MARTe::float32 angle = twoPi * position / encoderStepsInCircle;
    return angle;
}

/**
 * @param position Pendulum position in encoder steps (where 0 steps is bottom position).
 * @return Pendulum angle in radians (where 0 radians is the top position).
 */
MARTe::float32 normalizedEncoderStepsToRadians(MARTe::int32 position) {
    // Normalized position has bottom at 0 steps. Add half a circle of steps to
    // move the 0 steps to the top position.
    position += encoderStepsInHalfCircle;

    // Keep the position between -halfCircle and +halfCircle.
    position += encoderStepsInHalfCircle;
    position %= encoderStepsInCircle;
    position -= encoderStepsInHalfCircle;

    MARTe::float32 angle = twoPi * position / encoderStepsInCircle;
    return angle;
}

/**
 * @param position Motor position in motor steps
 * @return Motor angle in radians.
 */
MARTe::float32 motorStepsToRadians(MARTe::int32 position) {
    MARTe::float32 angle = twoPi * position / motorStepsInCircle;
    return angle;
}

/**
 * @param angle Motor angle in radians.
 * @return Motor position in motor steps.
 */
MARTe::float32 radiansToMotorSteps(MARTe::float32 angle) {
    MARTe::float32 position = angle * motorStepsInCircle / twoPi;
    return position;
}

/**
 * @param position Current measured position.
 * @param prevPositions Previous measured position.
 * @param period Time between the two measurements in seconds.
 * @return Interpolated speed.
 */
MARTe::float32 calculateSpeed(MARTe::int32 position,
                              MARTe::int32 prevPosition,
                              MARTe::float32 period) {
    MARTe::float32 speed = (position - prevPosition) / period;
    return speed;
}

} // namespace

LQRGAM::LQRGAM() : GAM(),
                           inputMotorState(NULL_PTR(MARTe::uint8*)),
                           inputEncoderPosition(NULL_PTR(MARTe::uint32*)),
                           inputMotorPosition(NULL_PTR(MARTe::int32*)),
                           inputAbsoluteTime(NULL_PTR(MARTe::uint64*)),
                           inputPrevEncoderPosition(NULL_PTR(MARTe::uint32*)),
                           inputPrevMotorPosition(NULL_PTR(MARTe::int32*)),
                           inputPrevAbsoluteTime(NULL_PTR(MARTe::uint64*)),
                           inputEncoderPositionBottom(NULL_PTR(MARTe::uint32*)),
                           inputEnableBalance(NULL_PTR(MARTe::uint8*)),
                           outputCommand(NULL_PTR(MARTe::uint8*)),
                           outputCommandParam(NULL_PTR(MARTe::int32*)),
                           outputRtAcc(NULL_PTR(MARTe::float32*)),
                           outputRtPeriod(NULL_PTR(MARTe::float32*)),
                           outputSwitchState(NULL_PTR(MARTe::uint8*)),
                           inputVectorPointer(NULL_PTR(MARTe::float64 **)),
                           outputVectorPointer(NULL_PTR(MARTe::float64 **)),
                           gainMatrixPointer(NULL_PTR(MARTe::float64 **))
                           ControlInputPointer(NULL_PTR(MARTe::float64 **)),
                           StateVectorPointer(NULL_PTR(MARTe::float64 **)),
                           gainMatrixOfColumns(0u),
                           gainMatrixOfRows(0u),
                           ControlInputOfColumns(0u),
                           ControlInputOfRows(0u),
                           StateVectorOfColumns(0u),
                           StateVectorOfRows(0u),
                           /*k1(0.0f),
                           k2(0.0f),
                           k3(0.0f),
                           k4(0.0f),*/
                           firstMove(true),
                           positiveDirection(true),
                           balanceEnabled(true),
                           exit(false),
                           swingUpKick(0) {
}

LQRGAM::~LQRGAM() {
    if (gainMatrixPointer != NULL_PTR(float64 **)) {
        for (uint32 row = 0u; row < gainMatrixOfRows; row++) {
            if (gainMatrixPointer[row] != NULL_PTR(float64 *)) {
                delete gainMatrixPointer[row];
                gainMatrixPointer[row] = NULL_PTR(float64 *);
            }
        }
        delete[] gainMatrixPointer;
        gainMatrixPointer = NULL_PTR(float64 **);
    }
    if (ControlInputPointer != NULL_PTR(float64 **)) {
        for (uint32 row = 0u; row < ControlInputOfRows; row++) {
            if (ControlInputPointer[row] != NULL_PTR(float64 *)) {
                delete ControlInputPointer[row];
                ControlInputPointer[row] = NULL_PTR(float64 *);
            }
        }
        delete[] ControlInputPointer;
        ControlInputPointer = NULL_PTR(float64 **);
    }
    if (StateVectorPointer != NULL_PTR(float64 **)) {
        for (uint32 row = 0u; row < StateVectorOfRows; row++) {
            if (StateVectorPointer[row] != NULL_PTR(float64 *)) {
                delete StateVectorPointer[row];
                StateVectorPointer[row] = NULL_PTR(float64 *);
            }
        }
        delete[] StateVectorPointer;
        StateVectorPointer = NULL_PTR(float64 **);
    }
}

bool LQRGAM::Initialise(MARTe::StructuredDataI& data) {
    bool ok = GAM::Initialise(data);

/*    if (ok) {
        ok = data.Read("K1", k1);
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::ParametersError, "No k1 has been specified.");
        }
    }
    if (ok) {
        ok = data.Read("K2", k2);
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::ParametersError, "No k2 has been specified.");
        }
    }
    if (ok) {
        ok = data.Read("K3", k3);
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::ParametersError, "No k3 has been specified.");
        }
    }
    if (ok) {
        ok = data.Read("K4", k4);
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::ParametersError, "No k4 has been specified.");
        }
    } 
*/
    if (ok) { //Load Weight Matrix
        AnyType functionsMatrix;
        functionsMatrix = data.GetType("GainMatrix");
        ok = (functionsMatrix.GetDataPointer() != NULL);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Error getting type for GainMatrix");
        }
        if (ok) {
            //0u are columns
            gainMatrixOfColumns = functionsMatrix.GetNumberOfElements(0u);
            ok = (gainMatrixOfColumns > 0u);
            if (!ok) {
                REPORT_ERROR(ErrorManagement::InitialisationError, "The number of input matrix columns must be positive");
            }
        }
        if (ok) {
            //1u are rows...
            gainMatrixOfRows = functionsMatrix.GetNumberOfElements(1u);
            ok = (gainMatrixOfRows > 0u);
            if (!ok) {
                REPORT_ERROR(ErrorManagement::InitialisationError,"the number of input matrix rows must be positive");

            }
        }
        if (ok) { // allocate input matrix memory and read matrix coefficients
            gainMatrixPointer = new float64 *[gainMatrixOfRows];
            //lint -e{613} Possible use of null pointer--> If new fails the program crashes.
            for (uint32 i = 0u; (i < gainMatrixOfRows) && ok; i++) {
                gainMatrixPointer[i] = new float64[gainMatrixOfColumns];
            }
            if (ok) {
                Matrix<float64> matrix(gainMatrixPointer, gainMatrixOfRows, gainMatrixOfColumns);
                ok = (data.Read("GainMatrix", matrix));
                if (!ok) {
                    REPORT_ERROR(ErrorManagement::InitialisationError, "Error reading gainMatrix");
                }
            }
        }  
    }
    if (ok) { //Control Input Vector
        AnyType functionsMatrix;
        functionsMatrix = data.GetType("ControlInput");
        ok = (functionsMatrix.GetDataPointer() != NULL);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Error getting type for ControlInput");
        }
        if (ok) {
            //0u are columns
            ControlInputOfColumns = functionsMatrix.GetNumberOfElements(0u);
            ok = (ControlInputOfColumns > 0u);
            if (!ok) {
                REPORT_ERROR(ErrorManagement::InitialisationError, "The number of input matrix columns must be positive");
            }
        }
        if (ok) {
            //1u are rows...
            ControlInputOfRows = functionsMatrix.GetNumberOfElements(1u);
            ok = (ControlInputOfRows == gainMatrixOfRows);
            if (!ok) {
                REPORT_ERROR(ErrorManagement::InitialisationError,"the number of Control Input Vector rows must be equal to Gain Matrix rows");

            }
        }
        if (ok) { // allocate input matrix memory and read matrix coefficients
            ControlInputPointer = new float64 *[ControlInputOfRows];
            //lint -e{613} Possible use of null pointer--> If new fails the program crashes.
            for (uint32 i = 0u; (i < ControlInputOfRows) && ok; i++) {
                ControlInputPointer[i] = new float64[ControlInputOfColumns];
            }
            if (ok) {
                Matrix<float64> matrix(ControlInputPointer, ControlInputOfRows, ControlInputOfColumns);
                ok = (data.Read("ControlInput", matrix));
                if (!ok) {
                    REPORT_ERROR(ErrorManagement::InitialisationError, "Error reading ControlInput");
                }
            }
        }  
    }
    if (ok) { //State Vector
        AnyType functionsMatrix;
        functionsMatrix = data.GetType("StateVector");
        ok = (functionsMatrix.GetDataPointer() != NULL);
        if (!ok) {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Error getting type for StateVector");
        }
        if (ok) {
            //0u are columns
            StateVectorOfColumns = functionsMatrix.GetNumberOfElements(0u);
            ok = (StateVectorOfColumns == gainMatrixOfColumns);
            if (!ok) {
                REPORT_ERROR(ErrorManagement::InitialisationError, "The number of StateVector columns must be equal to GainMatrix columns");
            }
        }
        if (ok) {
            //1u are rows...
            StateVectorOfRows = functionsMatrix.GetNumberOfElements(1u);
            ok = (StateVectorOfRows > 0u);
            if (!ok) {
                REPORT_ERROR(ErrorManagement::InitialisationError,"the number of input matrix rows must be positive");

            }
        }
        if (ok) { // allocate input matrix memory and read matrix coefficients
            StateVectorPointer = new float64 *[StateVectorOfRows];
            //lint -e{613} Possible use of null pointer--> If new fails the program crashes.
            for (uint32 i = 0u; (i < StateVectorOfRows) && ok; i++) {
                StateVectorPointer[i] = new float64[StateVectorOfColumns];
            }
            if (ok) {
                Matrix<float64> matrix(StateVectorPointer, StateVectorOfRows, StateVectorOfColumns);
                ok = (data.Read("StateVector", matrix));
                if (!ok) {
                    REPORT_ERROR(ErrorManagement::InitialisationError, "Error reading StateVector");
                }
            }
        }  
    }
    return ok;
}

bool LQRGAM::Setup() {
    // Validate input signals
    bool ok = GetNumberOfInputSignals() == 9;
    if (!ok) {
        REPORT_ERROR(MARTe::ErrorManagement::ParametersError, "Number of input signals must be 9.");
    }
    if (ok) {
        ok = GetSignalType(MARTe::InputSignals, 0u) == MARTe::UnsignedInteger8Bit;
        if (ok) {
            inputMotorState = static_cast<MARTe::uint8*>(GetInputSignalMemory(0u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "First input signal shall be of type "
                    "uint8");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::InputSignals, 1u) == MARTe::UnsignedInteger32Bit;
        if (ok) {
            inputEncoderPosition = static_cast<MARTe::uint32*>(GetInputSignalMemory(1u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Second input signal shall be of type "
                    "uint32");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::InputSignals, 2u) == MARTe::SignedInteger32Bit;
        if (ok) {
            inputMotorPosition = static_cast<MARTe::int32*>(GetInputSignalMemory(2u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Third input signal shall be of type "
                    "int32");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::InputSignals, 3u) == MARTe::UnsignedInteger64Bit;
        if (ok) {
            inputAbsoluteTime = static_cast<MARTe::uint64*>(GetInputSignalMemory(3u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Fourth input signal shall be of type "
                    "uint64");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::InputSignals, 4u) == MARTe::UnsignedInteger32Bit;
        if (ok) {
            inputPrevEncoderPosition = static_cast<MARTe::uint32*>(GetInputSignalMemory(4u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Fifth input signal shall be of type "
                    "uint32");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::InputSignals, 5u) == MARTe::SignedInteger32Bit;
        if (ok) {
            inputPrevMotorPosition = static_cast<MARTe::int32*>(GetInputSignalMemory(5u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Sixth input signal shall be of type "
                    "int32");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::InputSignals, 6u) == MARTe::UnsignedInteger64Bit;
        if (ok) {
            inputPrevAbsoluteTime = static_cast<MARTe::uint64*>(GetInputSignalMemory(6u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Seventh input signal shall be of type "
                    "uint64");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::InputSignals, 7u) == MARTe::UnsignedInteger32Bit;
        if (ok) {
            inputEncoderPositionBottom = static_cast<MARTe::uint32*>(GetInputSignalMemory(7u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Eighth input signal shall be of type "
                    "uint32");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::InputSignals, 8u) == MARTe::UnsignedInteger8Bit;
        if (ok) {
            inputEnableBalance = static_cast<MARTe::uint8*>(GetInputSignalMemory(8u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Ninth input signal shall be of type "
                    "uint8");
        }
    }
    // Validate output signals
    if (ok) {
        bool ok = GetNumberOfOutputSignals() == 5;
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::ParametersError, "Number of output signals must be 5.");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::OutputSignals, 0u) == MARTe::UnsignedInteger8Bit;
        if (ok) {
            outputCommand = static_cast<MARTe::uint8*>(GetOutputSignalMemory(0u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "First output signal shall be of type "
                    "uint8");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::OutputSignals, 1u) == MARTe::SignedInteger32Bit;
        if (ok) {
            outputCommandParam = static_cast<MARTe::int32*>(GetOutputSignalMemory(1u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Second output signal shall be of "
                    "type int32");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::OutputSignals, 2u) == MARTe::Float32Bit;
        if (ok) {
            outputRtAcc = static_cast<MARTe::float32*>(GetOutputSignalMemory(2u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Third output signal shall be of type "
                    "float32");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::OutputSignals, 3u) == MARTe::Float32Bit;
        if (ok) {
            outputRtPeriod = static_cast<MARTe::float32*>(GetOutputSignalMemory(3u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Fourth output signal shall be of type "
                    "float32");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::OutputSignals, 4u) == MARTe::UnsignedInteger8Bit;
        if (ok) {
            outputSwitchState = static_cast<MARTe::uint8*>(GetOutputSignalMemory(4u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Fifth output signal shall be of type "
                    "uint8");
        }
    }
    if (ok){
        InputVector = Matrix<float64>(ControlInputPointer, ControlInputOfRows, ControlInputOfColumns);
        StateVector = Matrix<float64>(StateVectorPointer, StateVectorOfRows, StateVectorOfColumns);
        GainMatrix = Matrix<float64>(gainMatrixPointer, gainMatrixOfRows, gainMatrixOfColumns);
    }
    return ok;
}

bool LQRGAM::Execute() {
    const MARTe::float32 microsecondsToSeconds = 1e-6f;
    MARTe::float32 rtPeriod = (*inputAbsoluteTime - *inputPrevAbsoluteTime) *
            microsecondsToSeconds;

    *outputCommand = MotorCommands::NoOp;
    *outputCommandParam = 0u;
    *outputRtAcc = 0.0f;
    *outputRtPeriod = rtPeriod;
    *outputSwitchState = 0u;

    if (exit) {
        return true;
    }
    // Make sure motor does not go too far from the center position.
    if (std::abs(*inputMotorPosition) < motorStepsInThirdOfCircle) {
        *outputSwitchState = 3u;
        exit = true;
        std::cout << "motor out of position " << std::endl;
        return;
    }

    MARTe::int32 normPosition = normalizeEncoderPos(*inputEncoderPosition, *inputEncoderPositionBottom);
    MARTe::int32 normPrevPosition = normalizeEncoderPos(*inputPrevEncoderPosition, *inputEncoderPositionBottom);

    MARTe::float32 motorSpeed = calculateSpeed(*inputMotorPosition, *inputPrevMotorPosition, rtPeriod);
    MARTe::float32 encoderSpeed = calculateSpeed(normPosition, normPrevPosition, rtPeriod);

    MARTe::float32 motorPositionRad = motorStepsToRadians(*inputMotorPosition);
    MARTe::float32 encoderPositionRad = normalizedEncoderStepsToRadians(normPosition);
    MARTe::float32 motorSpeedRad = motorStepsToRadians(motorSpeed);
    MARTe::float32 encoderSpeedRad = encoderStepsToRadians(encoderSpeed);

    // If encoder is too far way from the highest position stop balancing.
    if (std::abs(encoderPositionRad) < 0.15f) {
        *outputSwitchState = 3u;
        exit = true;
        std::cout << "encoder unbalanced " << std::endl;
        return;
    }

    StateVector(0u,0u) = encoderPositionRad; 
    StateVector(1u,0u) = motorPositionRad;
    StateVector(2u,0u) = encoderSpeedRad;
    StateVector(3u,0u) = motorSpeedRad;

    ok = InputVector.Product(GainMatrix, StateVector);
    
    /*MARTe::float32 acceleration = k1 * motorPositionRad +
            k2 * encoderPositionRad +
            k3 * motorSpeedRad +
            k4 * encoderSpeedRad;
    */

    std::cout << "motorPositionRad " << motorPositionRad << std::endl;
    std::cout << "encoderPositionRad " << encoderPositionRad << std::endl;
    std::cout << "motorSpeedRad " << motorSpeedRad << std::endl;
    std::cout << "encoderSpeedRad " << encoderSpeedRad << std::endl;
    std::cout << "RT acc " << radiansToMotorSteps(InputVector(0u,0u)) << std::endl;

    *outputRtAcc = radiansToMotorSteps(acceleration);
    *outputCommand = MotorCommands::RT_MoveMotor;
    return true;
}

CLASS_REGISTER(LQRGAM, "1.0");

} // namespace InvertedPendulum
