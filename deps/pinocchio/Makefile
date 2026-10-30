TOP = ../..
include $(TOP)/configure/CONFIG

CMAKE ?= cmake

PREFIX = $(CURDIR)/install/$(EPICS_HOST_ARCH)
BUILD_ROOT = $(CURDIR)/O.$(EPICS_HOST_ARCH)
EIGEN_SOURCE = $(abspath $(CURDIR)/../eigen)
CMAKE_BUILD_TYPE = $(if $(filter YES,$(HOST_OPT)),Release,RelWithDebInfo)
CMAKE_COMMON = \
    -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) \
    -DCMAKE_INSTALL_PREFIX=$(PREFIX) \
    -DCMAKE_INSTALL_LIBDIR=lib \
    -DCMAKE_PREFIX_PATH=$(PREFIX) \
    -DBUILD_TESTING=OFF

.PHONY: all host inc build install buildInstall clean realclean archclean
.PHONY: check-cmake check-sources runtests tapfiles clean-tests test-results junitfiles

check-cmake:
	@command -v "$(CMAKE)" >/dev/null 2>&1 || { \
	    echo "error: CMake is required to build Pinocchio; install cmake or set CMAKE=/path/to/cmake" >&2; \
	    exit 1; \
	}

check-sources:
	@for source in tinyxml2 console_bridge urdfdom_headers urdfdom pinocchio; do \
	    if [ ! -f "$(CURDIR)/$$source/CMakeLists.txt" ]; then \
	        echo "error: missing deps/pinocchio/$$source submodule" >&2; \
	        exit 1; \
	    fi; \
	done
	@if [ ! -f "$(CURDIR)/pinocchio/cmake/CMakeLists.txt" ]; then \
	    echo "error: missing Pinocchio's cmake submodule" >&2; \
	    exit 1; \
	fi

all: install
host: install$(DIVIDER)$(EPICS_HOST_ARCH)
inc build install buildInstall: install$(DIVIDER)$(EPICS_HOST_ARCH)

inc$(DIVIDER)% build$(DIVIDER)% buildInstall$(DIVIDER)%: install$(DIVIDER)%

install$(DIVIDER)%: check-cmake check-sources
	@if [ "$*" != "$(EPICS_HOST_ARCH)" ]; then \
	    echo "Pinocchio cross-build for '$*' is not configured" >&2; \
	    exit 1; \
	fi
	$(CMAKE) -S $(EIGEN_SOURCE) -B $(BUILD_ROOT)/eigen \
	    -DCMAKE_MESSAGE_LOG_LEVEL=NOTICE \
	    -DBUILD_TESTING=OFF \
	    -DEIGEN_BUILD_TESTING=OFF \
	    -DEIGEN_BUILD_BLAS=OFF \
	    -DEIGEN_BUILD_LAPACK=OFF \
	    -DEIGEN_BUILD_DOC=OFF \
	    -DEIGEN_BUILD_DEMOS=OFF
	$(CMAKE) -S $(CURDIR)/tinyxml2 -B $(BUILD_ROOT)/tinyxml2 \
	    $(CMAKE_COMMON) \
	    -DBUILD_SHARED_LIBS=ON \
	    -Dtinyxml2_BUILD_TESTING=OFF
	+$(CMAKE) --build $(BUILD_ROOT)/tinyxml2 --target install
	$(CMAKE) -S $(CURDIR)/console_bridge -B $(BUILD_ROOT)/console_bridge \
	    $(CMAKE_COMMON) \
	    -DBUILD_SHARED_LIBS=ON
	+$(CMAKE) --build $(BUILD_ROOT)/console_bridge --target install
	$(CMAKE) -S $(CURDIR)/urdfdom_headers -B $(BUILD_ROOT)/urdfdom_headers \
	    $(CMAKE_COMMON) \
	    -DAPPEND_PROJECT_NAME_TO_INCLUDEDIR=OFF
	+$(CMAKE) --build $(BUILD_ROOT)/urdfdom_headers --target install
	$(CMAKE) -S $(CURDIR)/urdfdom -B $(BUILD_ROOT)/urdfdom \
	    $(CMAKE_COMMON) \
	    -DAPPEND_PROJECT_NAME_TO_INCLUDEDIR=OFF \
	    -DBUILD_SHARED_LIBS=ON
	+$(CMAKE) --build $(BUILD_ROOT)/urdfdom --target install
	$(CMAKE) -S $(CURDIR)/pinocchio -B $(BUILD_ROOT)/pinocchio \
	    $(CMAKE_COMMON) \
	    -DCMAKE_CXX_STANDARD=17 \
	    -DEigen3_DIR=$(BUILD_ROOT)/eigen \
	    -DBUILD_PYTHON_INTERFACE=OFF \
	    -DBUILD_EXAMPLES=OFF \
	    -DBUILD_BENCHMARK=OFF \
	    -DBUILD_WITH_URDF_SUPPORT=ON \
	    -DBUILD_WITH_COLLISION_SUPPORT=OFF
	+$(CMAKE) --build $(BUILD_ROOT)/pinocchio --target install

clean: clean$(DIVIDER)$(EPICS_HOST_ARCH)
clean$(DIVIDER)%:
	$(RMDIR) $(CURDIR)/O.$*

archclean: clean
realclean:
	$(RMDIR) $(CURDIR)/O.* $(CURDIR)/install

runtests tapfiles clean-tests test-results junitfiles: ;
runtests$(DIVIDER)% tapfiles$(DIVIDER)% clean-tests$(DIVIDER)% test-results$(DIVIDER)%: ;
