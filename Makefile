#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS := $(DIRS) $(filter-out $(DIRS), configure)
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard *App))

DIRS += tests

ifeq ($(BUILD_IOCS), YES)
DIRS += $(wildcard iocs)
endif

define DIR_template
 $(1)_DEPEND_DIRS += configure
endef
$(foreach dir, $(filter-out configure,$(DIRS)),$(eval $(call DIR_template,$(dir))))

iocs_DEPEND_DIRS += mpleApp

include $(TOP)/configure/RULES_TOP


