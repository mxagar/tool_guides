# CmakeLists.txt in Test dir
	
# Make sure the compiler can find include files from our Hello library.
# HELLO_SOURCE_NAME: HELLO was defined in top-level folder and the variable HELLO_SOURCE_NAME was automatically generated and inherited by all subfolders
INCLUDE_DIRECTORIES(${HELLO_SOURCE_DIR}/HelloLib)
	
# Add binary called "helloWorld" that is built from the source file "test.cc".
# Again, as in ../Hello/CMakeLists.txt, I understand the executable is generated out of the source code
# ADD_EXECUTABLE(PRODUCT=executable_name INPUT=source_filename)
# The extension is automatically found.
ADD_EXECUTABLE(helloWorld test)
		
# Link the executable to the Hello library.
# TARGET_LINK_LIBRARIES(<target binary> <library to link to target>)
TARGET_LINK_LIBRARIES(helloWorld Hello)
