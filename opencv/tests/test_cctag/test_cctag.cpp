/* 
* 
*/

#include "cctag/utils/FileDebug.hpp"
#include "cctag/utils/VisualDebug.hpp"
#include "cctag/utils/Exceptions.hpp"
#include "cctag/Detection.hpp"
#ifdef CCTAG_WITH_CUDA
#include "cctag/cuda/device_prop.hpp"
#include "cctag/cuda/debug_macros.hpp"
#endif // CCTAG_WITH_CUDA

#include <iostream>
#include <string>

using namespace cctag;

int main(int argc, char *argv[]) {

    std::cout << "Test" << std::endl;
    return 0;

}