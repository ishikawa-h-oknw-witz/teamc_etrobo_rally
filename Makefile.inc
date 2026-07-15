mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_CXXOBJS += \
    control/PIDController.o \
    control/DistanceCalculator.o \
    control/DirectionController.o \
    control/TrapezoidController.o \
    runner/LineTracer.o \
    runner/ScenarioRunner.o \
    Logger.o \

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-spike/Makefile

endif

APPL_DIRS += \
    $(mkfile_path)app \
    $(mkfile_path)unit \
    $(mkfile_path)control \
    $(mkfile_path)device \
    $(mkfile_path)runner \
    $(mkfile_path)state

INCLUDES += \
	-I$(mkfile_path)../../../drivers/include \
	-I$(mkfile_path)../../../drivers/include/spike \
	-I$(mkfile_path)../../../drivers/include/libcpp/spike \
	-I$(mkfile_path)control \
    -I$(mkfile_path)device \
	-I$(mkfile_path)runner \
    -I$(mkfile_path)state \
