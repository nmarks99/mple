TOP = ../..
include $(TOP)/configure/CONFIG

CMAKE ?= cmake

OMPL_SOURCE = $(CURDIR)/ompl
EIGEN_SOURCE = $(abspath $(CURDIR)/../eigen)

CMAKE_BUILD_TYPE = $(if $(filter YES,$(HOST_OPT)),Release,RelWithDebInfo)

.PHONY: all host inc build install buildInstall clean realclean archclean
.PHONY: check-cmake runtests tapfiles clean-tests test-results junitfiles

check-cmake:
	@command -v "$(CMAKE)" >/dev/null 2>&1 || { \
	    echo "error: CMake is required to build OMPL; install cmake or set CMAKE=/path/to/cmake" >&2; \
	    exit 1; \
	}

all: install
host: install$(DIVIDER)$(EPICS_HOST_ARCH)
inc build install buildInstall: install$(DIVIDER)$(EPICS_HOST_ARCH)

inc$(DIVIDER)% build$(DIVIDER)% buildInstall$(DIVIDER)%: install$(DIVIDER)%

install$(DIVIDER)%: check-cmake
	@if [ "$*" != "$(EPICS_HOST_ARCH)" ]; then \
	    echo "OMPL cross-build for '$*' is not configured" >&2; \
	    exit 1; \
	fi
	$(CMAKE) -S $(EIGEN_SOURCE) -B $(CURDIR)/O.$*/eigen \
	    -DCMAKE_MESSAGE_LOG_LEVEL=NOTICE \
	    -DBUILD_TESTING=OFF \
	    -DEIGEN_BUILD_TESTING=OFF \
	    -DEIGEN_BUILD_BLAS=OFF \
	    -DEIGEN_BUILD_LAPACK=OFF \
	    -DEIGEN_BUILD_DOC=OFF \
	    -DEIGEN_BUILD_DEMOS=OFF
	$(CMAKE) -S $(OMPL_SOURCE) -B $(CURDIR)/O.$*/ompl \
	    -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) \
	    -DCMAKE_CXX_STANDARD=17 \
	    -DCMAKE_INSTALL_PREFIX=$(CURDIR)/install/$* \
	    -DCMAKE_INSTALL_LIBDIR=lib \
	    -DEigen3_DIR=$(CURDIR)/O.$*/eigen \
	    -DOMPL_BUILD_SHARED=ON \
	    -DOMPL_BUILD_TESTS=OFF \
	    -DOMPL_BUILD_DEMOS=OFF \
	    -DOMPL_BUILD_PYTHON_BINDINGS=OFF \
	    -DOMPL_BUILD_VAMP=OFF
	+$(CMAKE) --build $(CURDIR)/O.$*/ompl --target install

clean: clean$(DIVIDER)$(EPICS_HOST_ARCH)
clean$(DIVIDER)%:
	$(RMDIR) $(CURDIR)/O.$*

archclean: clean
realclean:
	$(RMDIR) $(CURDIR)/O.* $(CURDIR)/install

runtests tapfiles clean-tests test-results junitfiles: ;
runtests$(DIVIDER)% tapfiles$(DIVIDER)% clean-tests$(DIVIDER)% test-results$(DIVIDER)% junitfiles$(DIVIDER)%: ;
