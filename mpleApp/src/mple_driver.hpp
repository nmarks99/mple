#pragma once
#include <memory>
#include "asynPortDriver.h"
#include "mple_adapter_registry.hpp"
#include "collisions.hpp"

class RPCHandler;

class MpleDriver : public asynPortDriver {
  public:
    MpleDriver(const char* port, const char* script, const char* obstacles_file);
    virtual void poll(void);

    // Used by RPCHandler to check for collisions
    CollisionWorld collision_world;

    // asyn parameter indices which might be used externally to this class
    int idx_mt_active_;
    int idx_mt_error_;

  private:
    asynUser* pasynUserDriver_;
    std::unique_ptr<RPCHandler> rpc_handler_;
};
