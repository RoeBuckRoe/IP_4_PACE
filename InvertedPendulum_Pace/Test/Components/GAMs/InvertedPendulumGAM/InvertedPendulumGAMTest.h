/**
 * @file InvertedPendulumGAMTest.h
 * @brief Header file for class InvertedPendulumGAMTest
 * @date 21/05/2024
 * @author Jawad Muhammad
 *

 */

#ifndef InvertedPendulumGAMTEST_H_
#define InvertedPendulumGAMTEST_H_

/*---------------------------------------------------------------------------*/
/*                        Standard header includes                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                        Project header includes                            */
/*---------------------------------------------------------------------------*/

#include "InvertedPendulumGAM.h"
/*---------------------------------------------------------------------------*/
/*                           Class declaration                               */
/*---------------------------------------------------------------------------*/

namespace MARTe {

class InvertedPendulumGAMTest: MFI::InvertedPendulumGAM{
//TODO Add the macro DLL_API to the class declaration (i.e. class DLL_API InvertedPendulumGAMTest)
public:
    /**
     * @brief Default constructor. NOOP
     */
    InvertedPendulumGAMTest();

    /**
     * @brief Default destructor. NOOP
     */
    virtual ~InvertedPendulumGAMTest();

    /**
     * @brief Test the PIDGAM::Execute() with two inputs and only the proportional term.
     */
    bool TestExecuteStateSwapUp();

};

}

/*---------------------------------------------------------------------------*/
/*                        Inline method definitions                          */
/*---------------------------------------------------------------------------*/

#endif /* InvertedPendulumGAMTEST_H_ */

