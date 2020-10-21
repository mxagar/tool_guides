SET(SRC_DIR ${CMAKE_SOURCE_DIR})

SET(EXEC_NAME ${LIB1_NAME})
MESSAGE("Configuring " ${EXEC_NAME})

SET(files
    ${SRC_DIR}/lib_1/factorial.h
    ${SRC_DIR}/lib_1/factorial.cpp
)

SET(headers
    ${SRC_DIR}/lib_1/factorial.h
)

SOURCE_GROUP("Source Files" FILES ${files})

INCLUDE_DIRECTORIES(
	${CMAKE_SOURCE_DIR}/lib_1
    ${SRC_DIR}
)

LINK_DIRECTORIES(
	${CMAKE_SOURCE_DIR}/lib_1
    ${SRC_DIR}
)

IF(PROJECT_BUILD_STATIC)
    ADD_LIBRARY(${EXEC_NAME}
        ${files}
    )
    # Add BUILD_STATIC definition for the Windows DLLPORT macro on the headers
    # This is irrelevant for Linux/Mac, though
    TARGET_COMPILE_DEFINITIONS(${EXEC_NAME} PUBLIC BUILD_STATIC)
ELSE()
    ADD_LIBRARY(${EXEC_NAME} SHARED
        ${files}
    )
ENDIF()

TARGET_LINK_LIBRARIES(
	${EXEC_NAME}
)

# I we want to do 'make install'
INSTALL(TARGETS ${EXEC_NAME} DESTINATION ${PROJECT_LIBDIR})
INSTALL(FILES ${headers} DESTINATION ${PROJECT_INCDIR})
