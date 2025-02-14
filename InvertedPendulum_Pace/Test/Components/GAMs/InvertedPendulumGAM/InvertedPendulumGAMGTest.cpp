/**
 * @file InvertedPendulumGAMTest.h
 * @brief Header file for class InvertedPendulumGAMTest
 * @date 21/05/2024
 * @author Jawad Muhammad
 *.
 */

#define DLL_API

/*---------------------------------------------------------------------------*/
/*                         Standard header includes                          */
/*---------------------------------------------------------------------------*/
#include <limits.h>
#include "gtest/gtest.h"
/*---------------------------------------------------------------------------*/
/*                         Project header includes                           */
/*---------------------------------------------------------------------------*/

#include "InvertedPendulumGAM.h"
#include "InvertedPendulumGAMTest.h"
/*---------------------------------------------------------------------------*/
/*                           Static definitions                              */
/*---------------------------------------------------------------------------*/
using namespace MARTe;

TEST(InvertedPendulumGAMTest,TestExecuteStateSwapUp) {
    InvertedPendulumGAMTest test;
    ASSERT_TRUE(test.TestExecuteStateSwapUp());
}



