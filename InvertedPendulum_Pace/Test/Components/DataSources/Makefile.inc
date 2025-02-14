SPB    = STM32.x

PACKAGE=Components
ROOT_DIR=../../..
MAKEDEFAULTDIR=$(MARTe2_DIR)/MakeDefaults

include $(MAKEDEFAULTDIR)/MakeStdLibDefs.$(TARGET)

#LIBRARIES_STATIC+=$(BUILD_DIR)/STM32/STM32$(LIBEXT)
LIBRARIES_STATIC+=$(BUILD_DIR)/STM32/STM32Test$(LIBEXT)

all: $(OBJS) $(SUBPROJ) \
    $(BUILD_DIR)/DataSourcesTests$(LIBEXT)
	echo  $(OBJS)

#include depends.$(TARGET)

include $(MAKEDEFAULTDIR)/MakeStdLibRules.$(TARGET)
