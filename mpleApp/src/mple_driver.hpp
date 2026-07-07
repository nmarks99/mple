#pragma once
#include <memory>
#include "asynPortDriver.h"
#include "mple_adapter_registry.hpp"

class RPCHandler;

class MpleDriver : public asynPortDriver {
  public:
    MpleDriver(const char* port, const char* script);
    virtual void poll(void);
    int idx_mt_active_;
    int idx_mt_error_;

  private:
    asynUser* pasynUserDriver_;
    std::unique_ptr<RPCHandler> rpc_handler_;
};
