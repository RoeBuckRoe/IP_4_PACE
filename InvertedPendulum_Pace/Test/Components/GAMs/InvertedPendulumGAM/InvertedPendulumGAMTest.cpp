/**
 * @file InvertedPendulumGAMTest.h
 * @brief Header file for class InvertedPendulumGAMTest
 * @date 21/05/2024
 * @author Jawad Muhammad
 *
 */

#define DLL_API

/*---------------------------------------------------------------------------*/
/*                         Standard header includes                          */
/*---------------------------------------------------------------------------*/
#include "stdio.h"
/*---------------------------------------------------------------------------*/
/*                         Project header includes                           */
/*---------------------------------------------------------------------------*/

#include "InvertedPendulumGAMTest.h"

/*---------------------------------------------------------------------------*/
/*                           Static definitions                              */
/*---------------------------------------------------------------------------*/

namespace {

}

/*---------------------------------------------------------------------------*/
/*                           Method definitions                              */
/*---------------------------------------------------------------------------*/

namespace MARTe {
class InvertedPendulumGAMTestHelper: public MFI::InvertedPendulumGAM {
public:
    CLASS_REGISTER_DECLARATION()InvertedPendulumGAMTestHelper(/*list of parameters*/) {
        //parameter initializations
    }

    void *GetInputSignalsMemory(const char8 *const signalName) {
        uint32 signalIdx;
        bool ok = GetSignalIndex( InputSignals, signalIdx, signalName);
        return  GetInputSignalMemory(signalIdx);
    }
    void *GetOutputSignalsMemory(const char8 *const signalName) {
        uint32 signalIdx;
        bool ok = GetSignalIndex( OutputSignals, signalIdx, signalName);
        return  GetOutputSignalMemory(signalIdx);
    }
    // void *GetInputSignalsMemory() {
    //     return GAM::GetInputSignalsMemory();
    // }
    // void *GetOutputSignalsMemory() {
    //     return GAM::GetOutputSignalsMemory();
    // }
    // void *GetInputSignalsMemory(uint32 idx) {
    //     return GAM::GetInputSignalMemory(idx);
    // }

    // void *GetOutputSignalsMemory(uint32 idx) {
    //     return GAM::GetOutputSignalMemory(idx);
    // }

    bool HelperInitialise() {
        bool ok = false;
        //insert parameters into the config class as shown below
        //ok = config.Write("Parameter name", initialised_parameter_variable);
        
        return ok;
    }
    bool HelperSetup() {
        bool ok;
        uint32 numberOfElements = 1;
        uint32 numberOfInputSignals = 2;
        uint32 byteSizePerSignal = numberOfElements * sizeof(float64);
        uint32 totalInputBytes = byteSizePerSignal*numberOfInputSignals;
        uint32 totalOutputBytes = byteSizePerSignal;

        ok = configSignals.CreateAbsolute("Signals.InputSignals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("NumberOfElements", 1);
        ok &= configSignals.Write("DataSource", "Reference");
        ok &= configSignals.Write("NumberOfDimensions", 1);
        ok &= configSignals.Write("Type", "float64");
        ok &= configSignals.Write("ByteSize", byteSizePerSignal);
        ok &= configSignals.MoveAbsolute("Signals.InputSignals");
        ok &= configSignals.CreateRelative("1");
        ok &= configSignals.Write("NumberOfElements", 1);
        ok &= configSignals.Write("DataSource", "Measurement");
        ok &= configSignals.Write("NumberOfDimensions", 1);
        ok &= configSignals.Write("Type", "float64");
        ok &= configSignals.Write("ByteSize", byteSizePerSignal);
        ok &= configSignals.MoveToAncestor(1u);
        ok &= configSignals.Write("ByteSize", totalInputBytes);

        ok &= configSignals.MoveToRoot();
        ok &= configSignals.CreateAbsolute("Memory.InputSignals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("DataSource", "Reference");
        ok &= configSignals.CreateRelative("Signals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("Samples", 1);
        ok &= configSignals.MoveToAncestor(3u);
        ok &= configSignals.CreateRelative("1");
        ok &= configSignals.Write("DataSource", "Measurement");
        ok &= configSignals.CreateRelative("Signals");
        ok &= configSignals.CreateRelative("1");
        ok &= configSignals.Write("Samples", 1);

        ok &= configSignals.MoveToRoot();
        ok &= configSignals.CreateAbsolute("Signals.OutputSignals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("NumberOfElements", 1);
        ok &= configSignals.Write("DataSource", "Reference");
        ok &= configSignals.Write("NumberOfDimensions", 1);
        ok &= configSignals.Write("Type", "float64");
        ok &= configSignals.Write("ByteSize", byteSizePerSignal);
        ok &= configSignals.MoveAbsolute("Signals.OutputSignals");
        ok &= configSignals.Write("ByteSize", totalOutputBytes);

        ok &= configSignals.MoveToRoot();
        ok &= configSignals.CreateAbsolute("Memory.OutputSignals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("DataSource", "Reference");
        ok &= configSignals.CreateRelative("Signals");
        ok &= configSignals.CreateRelative("0");
        ok &= configSignals.Write("Samples", 1);

        ok &= configSignals.MoveToRoot();
        return ok;
    }

    bool IsEqualLargerMargins(float64 f1,
                              float64 f2) {
        float64 *min = reinterpret_cast<float64*>(const_cast<uint64*>(&EPSILON_FLOAT64));
        float64 minLarger = *min * 2;
        return ((f1 - f2) < (minLarger)) && ((f1 - f2) > -(minLarger));
    }

    ConfigurationDatabase config;
    ConfigurationDatabase configSignals;
private :
    //list of the parameters to assign as memebers of the helper

};
CLASS_REGISTER(InvertedPendulumGAMTestHelper, "1.0")

InvertedPendulumGAMTest::InvertedPendulumGAMTest() {
//Auto-generated constructor stub for InvertedPendulumGAMTest

//TODO Verify if manual additions are needed here
}

InvertedPendulumGAMTest::~InvertedPendulumGAMTest() {
//Auto-generated destructor stub for InvertedPendulumGAMTest

//TODO Verify if manual additions are needed here
}

bool InvertedPendulumGAMTest::TestExecuteStateSwapUp() {
    bool ret;
    InvertedPendulumGAMTestHelper gam;
    ret = gam.HelperInitialise();
    ret &= gam.Initialise(gam.config);

    ret &= gam.HelperSetup();
    ret &= gam.SetConfiguredDatabase(gam.configSignals);
    ret &= gam.AllocateInputSignalsMemory();
    ret &= gam.AllocateOutputSignalsMemory();
    ret &= gam.Setup();

 //********************############### Input Signals #########################*************************************
    MARTe::int32*  INPUT_rotor_position_steps    = static_cast<int32*>(gam.GetInputSignalsMemory("rotor_position_steps"));
    MARTe::uint32* INPUT_L6474_Board_Pwm1Counter = static_cast<uint32*>(gam.GetInputSignalsMemory("L6474_Board_Pwm1Counter"));
    MARTe::uint32* INPUT_message_count           = static_cast<uint32*>(gam.GetInputSignalsMemory("MessageCount"));
    MARTe::uint32* INPUT_encoder_counter         = static_cast<uint32*>(gam.GetInputSignalsMemory("encoder_counter"));
   //********************########################################################*************************************

   //********************############### Output Signals #########################*************************************
   MARTe::int32* OUTPUT_rotor_control_target_steps = static_cast<int32*>(gam.GetOutputSignalsMemory("rotor_control_target_steps"));
   MARTe::uint8* OUTPUT_gpioState                  = static_cast<uint8*>(gam.GetOutputSignalsMemory("gpioState"));
   MARTe::uint32* OUTPUT_L6474_Board_Pwm1Period    = static_cast<uint32*>(gam.GetOutputSignalsMemory("L6474_Board_Pwm1Period"));
   MARTe::uint8* OUTPUT_break_Control_Loop         = static_cast<uint8*>(gam.GetOutputSignalsMemory("break_Control_Loop"));
   MARTe::uint8* OUTPUT_state                      = static_cast<uint8*>(gam.GetOutputSignalsMemory("state"));
//********************########################################################*************************************

    // float64 *gamMemoryInR = static_cast<float64 *>(gam.GetInputSignalsMemory());
    // float64 *gamMemoryInM = static_cast<float64 *>(gam.GetInputSignalsMemory(1));
    // float64 *gamMemoryOut = static_cast<float64 *>(gam.GetOutputSignalsMemory());
    uint32 maxRep = 400;
    float64 *expectedVale = new float64[maxRep];
    //generate output
    *expectedVale = 1;
    for (uint32 i = 1u; i < maxRep; i++) {
        if (expectedVale[i - 1] == 1) {
            expectedVale[i] = 0;
        }
        else if (expectedVale[i - 1] == 0) {
            expectedVale[i] = 1;
        }
        else {
            ret = false;
        }
    }
    // *gamMemoryInM = 0;
    // for (uint32 i = 0u; i < maxRep && ret; i++) {
    //     //set inputs
    //     *INPUT_rotor_position_steps = 1;
    //     gam.Execute();
    //     //Check output
    //     ret &= (*gamMemoryOut == expectedVale[i]);
    //     if (!ret) {
    //         printf("output value = %.17lf. expectedValue = %.17lf. index = %u \n", *gamMemoryOut, expectedVale[i], i);
    //     }
    //     *gamMemoryInM = *gamMemoryOut;
    // }
    return ret;
}


}

