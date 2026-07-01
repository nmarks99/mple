#pragma once
#include "asynPortDriver.h"

#include <pvxs/server.h>
#include <pvxs/sharedpv.h>
#include <pvxs/nt.h>
#include <pvxs/iochooks.h>

class MpleDriver : public asynPortDriver {
  public:
    MpleDriver(const char* port, const char* script);
    virtual void poll(void);
    // virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
    // virtual asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value);
  private:
    asynUser* pasynUserDriver_;
    pvxs::server::SharedPV plan_pv_ = pvxs::server::SharedPV::buildReadonly();
    pvxs::server::SharedPV validate_pv_ = pvxs::server::SharedPV::buildReadonly();
    pvxs::server::SharedPV execute_pv_ = pvxs::server::SharedPV::buildReadonly();
};
