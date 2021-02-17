// HDevEngine/C++ example for executing external procedure
//
// (C) 2005-2020 MVTec Software GmbH
//
// Purpose:
// This example program shows how the classes HDevEngine and HDevOperatorImpl
// can be used in order to implement a fin detection application.
// Almost the complete functionality of the application is contained in the
// HDevelop program fin_detection.hdev, which can be found in the
// directory hdevelop.
// In the action routine DetectFin(), the HDevelop program
// 'fin_detection.hdev' is loaded and executed.
// The class MyHDevOperatorImpl is used in order to implement HDevelop
// internal operators by overloading the corresponding member functions.

#ifndef __APPLE__
#  include "HalconCpp.h"
#  include "HDevThread.h"
#else
#  ifndef HC_LARGE_IMAGES
#    include <HALCONCpp/HalconCpp.h>
#    include <HALCONCpp/HDevThread.h>
#    include <HALCON/HpThread.h>
#  else
#    include <HALCONCppxl/HalconCpp.h>
#    include <HALCONCppxl/HDevThread.h>
#    include <HALCONxl/HpThread.h>
#  endif
#  include <stdio.h>
#  include <CoreFoundation/CFRunLoop.h>
#endif

#include "my_hdevoperatorimpl.h"
#include "my_error_output.h"

#include <string>

using namespace HalconCpp;
using namespace HDevEngineCpp;


// uncomment the following #define for printing the full error description
// instead of the message only
//#define FULL_ERR_MSG

void DetectFin();

// Action routine
void DetectFin()
{
  try
  {
    HDevEngine my_engine;
    MyHDevOperatorImpl op_impl;
    my_engine.SetHDevOperatorImpl(&op_impl);

    std::string halcon_examples = (std::string)HSystem::GetSystem("example_dir")[0].S();
    std::string program_path(halcon_examples), ext_proc_path(halcon_examples);

    program_path  += "/hdevengine/hdevelop/fin_detection.hdev";
    ext_proc_path += "/hdevengine/procedures";

    // Set external procedure path
    my_engine.SetProcedurePath(ext_proc_path.c_str());

    // Load program
    HDevProgram my_program;
    my_program.LoadProgram(program_path.c_str());
    // Execute program
    HDevProgramCall prog_call = my_program.Execute();

    // get result
    HTuple result = prog_call.GetCtrlVarTuple("FinArea");
    printf("\nFin Area: %f\n\n",result[0].D());

    my_engine.SetHDevOperatorImpl(NULL);
  }
  catch (HDevEngineException& hdev_exception)
  {
    // If an exception occurs during program execution:
    // -> print error message and exit
#ifdef FULL_ERR_MSG
    DispErrorMessage(hdev_exception);
#else
    DispMessage(hdev_exception.Message());
#endif
    exit(0);
  }
  WaitSeconds(3.0);
}


#ifdef __APPLE__
// On OS X systems, we must have a CFRunLoop running on the main thread in
// order for the HALCON graphics operators to work correctly, and run the
// main function in a separate thread. A CFRunLoopTimer is used to make sure
// the action function is not called before the CFRunLoop is running.
// Note that starting with macOS 10.12, the run loop may be stopped when a
// window is closed, so we need to put the call to CFRunLoopRun() into a loop
// of its own.
HTuple      gStartMutex;
H_pthread_t gActionThread;
HBOOL       gTerminate = FALSE;

static void timer_callback(CFRunLoopTimerRef timer, void *info)
{
  UnlockMutex(gStartMutex);
}

static Herror apple_run(void **parameters)
{
  // Wait until the timer has fired to start processing.
  LockMutex(gStartMutex);
  UnlockMutex(gStartMutex);

  DetectFin();

  // Tell the main thread to terminate itself.
  LockMutex(gStartMutex);
  gTerminate = TRUE;
  UnlockMutex(gStartMutex);
  CFRunLoopStop(CFRunLoopGetMain());
  return H_MSG_OK;
}

static void apple_main(void)
{
  Herror                error;
  CFRunLoopTimerRef     Timer;
  CFRunLoopTimerContext TimerContext = { 0, 0, 0, 0, 0 };

  CreateMutex("type","sleep",&gStartMutex);
  LockMutex(gStartMutex);

  error = HpThreadHandleAlloc(&gActionThread);
  if (H_MSG_OK != error)
  {
    fprintf(stderr,"HpThreadHandleAlloc failed: %d\n", error);
    exit(1);
  }

  error = HpThreadCreate(gActionThread,0,apple_run);
  if (H_MSG_OK != error)
  {
    fprintf(stderr,"HpThreadCreate failed: %d\n", error);
    exit(1);
  }

  Timer = CFRunLoopTimerCreate(kCFAllocatorDefault,
                               CFAbsoluteTimeGetCurrent(),0,0,0,
                               timer_callback,&TimerContext);
  if (!Timer)
  {
    fprintf(stderr,"CFRunLoopTimerCreate failed\n");
    exit(1);
  }
  CFRunLoopAddTimer(CFRunLoopGetCurrent(),Timer,kCFRunLoopCommonModes);

  for (;;)
  {
    HBOOL terminate;

    CFRunLoopRun();

    LockMutex(gStartMutex);
    terminate = gTerminate;
    UnlockMutex(gStartMutex);

    if (terminate)
      break;
  }

  CFRunLoopRemoveTimer(CFRunLoopGetCurrent(),Timer,kCFRunLoopCommonModes);
  CFRelease(Timer);

  error = HpThreadHandleFree(gActionThread);
  if (H_MSG_OK != error)
  {
    fprintf(stderr,"HpThreadHandleFree failed: %d\n", error);
    exit(1);
  }

  ClearMutex(gStartMutex);
}
#endif


int main(void)
{
#if defined(_WIN32)
  SetSystem("use_window_thread", "true");
#endif

#ifndef __APPLE__
  // Call action routine
  DetectFin();
#else
  apple_main();
#endif

  return 0;
}
