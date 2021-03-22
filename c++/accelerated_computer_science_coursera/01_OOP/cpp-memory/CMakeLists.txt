cmake_minimum_required(VERSION 3.0)
project(cpp-memory)

set(SRC_DIR .)
set(LIBS '')

include_directories(${SRC_DIR})

set (   SRC_FILES 
        Cube.cpp 
    )

# ./main
add_executable(main2 main.cpp ${SRC_FILES})
#target_link_libraries(main2 ${LIBS})

# ./foo
add_executable(foo foo.cpp ${SRC_FILES})
#target_link_libraries(foo ${LIBS})

# ./addressOf
add_executable(addressOf addressOf.cpp ${SRC_FILES})
#target_link_libraries(addressOf ${LIBS})

# ./puzzle
add_executable(puzzle puzzle.cpp ${SRC_FILES})
#target_link_libraries(puzzle ${LIBS})
