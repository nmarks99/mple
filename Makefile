#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS := $(DIRS) $(filter-out $(DIRS), configure)
DIRS := $(DIRS) $(filter-out $(DIRS), deps/toppra)
DIRS := $(DIRS) $(filter-out $(DIRS), deps/ompl)
DIRS := $(DIRS) $(filter-out $(DIRS), deps/pinocchio)
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard *App))

DIRS += tests
tests_DEPEND_DIRS += deps/toppra deps/pinocchio

ifeq ($(BUILD_IOCS), YES)
DIRS += $(wildcard iocs)
endif

define DIR_template
 $(1)_DEPEND_DIRS += configure
endef
$(foreach dir, $(filter-out configure,$(DIRS)),$(eval $(call DIR_template,$(dir))))

mpleApp_DEPEND_DIRS += deps/toppra deps/ompl deps/pinocchio
iocs_DEPEND_DIRS += mpleApp

include $(TOP)/configure/RULES_TOP


