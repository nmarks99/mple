TOP = ../..
include $(TOP)/configure/CONFIG

CMAKE ?= cmake

TOPPRA_SOURCE = $(CURDIR)/toppra/cpp
EIGEN_SOURCE = $(abspath $(CURDIR)/../eigen)

CMAKE_BUILD_TYPE = $(if $(filter YES,$(HOST_OPT)),Release,RelWithDebInfo)

.PHONY: all host inc build install buildInstall clean realclean archclean
.PHONY: runtests tapfiles clean-tests test-results junitfiles

all: install
host: install$(DIVIDER)$(EPICS_HOST_ARCH)
inc build install buildInstall: install$(DIVIDER)$(EPICS_HOST_ARCH)

inc$(DIVIDER)% build$(DIVIDER)% buildInstall$(DIVIDER)%: install$(DIVIDER)%

install$(DIVIDER)%:
	@if [ "$*" != "$(EPICS_HOST_ARCH)" ]; then \
	    echo "TOPPRA cross-build for '$*' is not configured" >&2; \
	    exit 1; \
	fi
	$(CMAKE) -S $(EIGEN_SOURCE) -B $(CURDIR)/O.$*/eigen \
	    -DBUILD_TESTING=OFF \
	    -DEIGEN_BUILD_TESTING=OFF \
	    -DEIGEN_BUILD_BLAS=OFF \
	    -DEIGEN_BUILD_LAPACK=OFF \
	    -DEIGEN_BUILD_DOC=OFF \
	    -DEIGEN_BUILD_DEMOS=OFF
	$(CMAKE) -S $(TOPPRA_SOURCE) -B $(CURDIR)/O.$*/toppra \
	    -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) \
	    -DCMAKE_CXX_STANDARD=17 \
	    -DCMAKE_INSTALL_PREFIX=$(CURDIR)/install/$* \
	    -DEigen3_DIR=$(CURDIR)/O.$*/eigen \
	    -DBUILD_TESTING=OFF \
	    -DPYTHON_BINDINGS=OFF \
	    -DINSTALL_DOCUMENTATION=OFF \
	    -DBUILD_SHARED_LIBS=ON
	$(CMAKE) --build $(CURDIR)/O.$*/toppra --target install

clean: clean$(DIVIDER)$(EPICS_HOST_ARCH)
clean$(DIVIDER)%:
	$(RMDIR) $(CURDIR)/O.$*

archclean: clean
realclean:
	$(RMDIR) $(CURDIR)/O.* $(CURDIR)/install

runtests tapfiles clean-tests test-results junitfiles: ;
runtests$(DIVIDER)% tapfiles$(DIVIDER)% clean-tests$(DIVIDER)% test-results$(DIVIDER)% junitfiles$(DIVIDER)%: ;
