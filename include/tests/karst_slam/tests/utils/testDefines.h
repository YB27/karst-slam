#ifndef TESTDEFINES_H
#define TESTDEFINES_H

/**
 * Useful macro used in the unitary tests 
 */


namespace karst_slam{namespace tests{namespace utils{

// For colors on graphic display
#define RED_DISPLAY "\033[0;31m" //!< Red color display on terminal
#define GREEN_DISPLAY "\033[0;32m" //!< Green color display on terminal
#define RESET_DISPLAY "\033[0m" //!< Reset color display on terminal

/** Trick to allow comma inside TEST_NAME arguments
 * Note that macro argument should be invoked with parenthesis for UNPACK */
#define UNPACK( ... ) __VA_ARGS__

/** Macro to put at the start of the test function */
#define START_TEST int res = 0;

/** Used in the main test file which calls all the test */
#define TEST(TEST_NAME) try{ \
                        res += UNPACK TEST_NAME();\
                        } catch(const std::exception& e){ \
                            std::cout << e.what() << std::endl; \
                            res = -1; \
                        }
/** Same as TEST(TEST_NAME) Macro in the case where the test function has an argument */        
#define TEST_ARG1(TEST_NAME, ARG1)try{ \
                                  res += UNPACK TEST_NAME(ARG1);\
                                  } catch(const std::exception& e){ \
                                      std::cout << e.what() << std::endl; \
                                      res = -1; \
                                  }
/** Macro to put at the end of the test function */
#define END_TEST return res;

/** Display the success status of a test */
#define PASSED cout << GREEN_DISPLAY << "[PASSED]  " << RESET_DISPLAY << __PRETTY_FUNCTION__ << endl;

/** Display the failed status of a test */
#define FAILED cout << RED_DISPLAY << "[FAILED] "    << RESET_DISPLAY << __PRETTY_FUNCTION__ << endl;

/** Display the test results */
#define DISPLAY_RESULT(RES)  if(RES) \
{ \
    FAILED \
    return -1; \
} \
else \
{ \
     PASSED \
     return 0; \
}

/** Display results in case of test based on matrix computation */
#define DISPLAY_RESULT_EQUAL_MATRIX_EPS(MATA, MATB, EPS) if(!equalMatrix(MATA,MATB,EPS)) \
{ \
    FAILED \
    cout << #MATA << endl; \
    cout << MATA << endl; \
    cout << #MATB << endl; \
    cout << MATB << endl; \
    return -1; \
} \
else \
{ \
   PASSED \
   return 0; \
}

/** Same as DISPLAY_RESULT_EQUAL_MATRIX_EPS(MATA, MATB, EPS) with a fixed threshold epsilon */
#define DISPLAY_RESULT_EQUAL_MATRIX(MATA, MATB) DISPLAY_RESULT_EQUAL_MATRIX_EPS(MATA, MATB, 1e-2)

}}} // end namespaces
#endif // TESTDEFINES_H
