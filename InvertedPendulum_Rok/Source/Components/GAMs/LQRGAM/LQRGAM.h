#ifndef SOURCE_COMPONENTS_GAMS_LQRGAM_LQRGAM_H_
#define SOURCE_COMPONENTS_GAMS_LQRGAM_LQRGAM_H_

#include "GAM.h"
#include "Matrix.h"

namespace InvertedPendulum {

//TODO documentation
class LQRGAM : public MARTe::GAM, public MARTe::StatefulI {
public:
    CLASS_REGISTER_DECLARATION()

    LQRGAM();

    virtual ~LQRGAM();

    virtual bool Initialise(MARTe::StructuredDataI& data);

    virtual bool Setup();

    virtual bool Execute();

    virtual bool PrepareNextState(const MARTe::char8* const currentStateName,
                                  const MARTe::char8* const nextStateName);

private:
   
    void Balance(MARTe::float32 rtPeriod);

    MARTe::uint8* inputMotorState;
    MARTe::uint32* inputEncoderPosition;
    MARTe::int32* inputMotorPosition;
    MARTe::uint64* inputAbsoluteTime;
    MARTe::uint32* inputPrevEncoderPosition;
    MARTe::int32* inputPrevMotorPosition;
    MARTe::uint64* inputPrevAbsoluteTime;
    MARTe::uint32* inputEncoderPositionBottom;
    MARTe::uint8* inputEnableBalance;
    MARTe::uint8* outputCommand;
    MARTe::int32* outputCommandParam;
    MARTe::float32* outputRtAcc;
    MARTe::float32* outputRtPeriod;
    MARTe::uint8* outputSwitchState;

    /* MARTe::float32 k1;
    MARTe::float32 k2;
    MARTe::float32 k3;
    MARTe::float32 k4; */

    /**
     * This is the input signal of the system (usually called U = Accelaration). It is the excitation of the system.
     */
    MARTe::float64 **ControlInputPointer;
    MARTe::Matrix<float64> InputVector;
    MARTe::uint8 ControlInputOfColumns;
    MARTe::uint8 ControlInputOfRows;

    /**
     * Output of the system (usually this vector is represented by a X = State Matrix).
     */
    MARTe::float64 **StateVectorPointer;
    MARTe::Matrix<float64> StateVector;
    MARTe::uint8 StateVectorOfColumns;
    MARTe::uint8 StateVectorOfRows;

    /**
     * LQR Gain of the system (usually this vector is represented by a K  = LQR weights).
     */
    MARTe::float64 **gainMatrixPointer;
    MARTe::Matrix<float64> GainMatrix;
    MARTe::uint8 gainMatrixOfColumns;
    MARTe::uint8 gainMatrixOfRows;


    bool firstMove;
    bool positiveDirection;
    bool balanceEnabled;
    bool exit;
    MARTe::int32 swingUpKick;
};

} // namespace InvertedPendulum

#endif // SOURCE_COMPONENTS_GAMS_LQRGAM_LQRGAM_H_
