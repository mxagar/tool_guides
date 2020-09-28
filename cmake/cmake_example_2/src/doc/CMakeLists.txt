CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(MIKELTEST)

SET(DOXYGEN_DOCU ON CACHE BOOL "Build the doxygen documentation")

IF(DOXYGEN_DOCU)
    
	INCLUDE(FindDoxygen)

	FIND_PACKAGE(Doxygen)

	IF( DOXYGEN_FOUND )
		
    	SET( DOXYFILE_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}"/../src" )

	    SET( DOXYFILE_OUTPUT_DIR ${CMAKE_BINARY_DIR}"/doc" )

    	CONFIGURE_FILE(
						${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
			            ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY
    	               ) 
	
    	ADD_CUSTOM_TARGET( doc
    	                   ${DOXYGEN_EXECUTABLE}
    	                   ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    	                   WORKING_DIRECTORY  ${CMAKE_CURRENT_BINARY_DIR}
    	                   COMMENT "Generating API documentation with Doxygen" VERBATIM
    	                 )

	ENDIF(DOXYGEN_FOUND)
    
ENDIF(DOXYGEN_DOCU)
