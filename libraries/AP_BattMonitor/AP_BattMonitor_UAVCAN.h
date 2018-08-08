#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

#define AP_BATTMONITOR_UAVCAN_TIMEOUT_MICROS         5000000 // sensor becomes unhealthy if no successful readings for 5 seconds
class BattInfoCb;
class AP_BattMonitor_UAVCAN : public AP_BattMonitor_Backend
{
public:

    enum BattMonitor_UAVCAN_Type {
        UAVCAN_BATTERY_INFO = 0
    };

    /// Constructor
    AP_BattMonitor_UAVCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_UAVCAN_Type type, AP_BattMonitor_Params &params);
    static void handle_battery_info_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BattInfoCb &cb);

    ~AP_BattMonitor_UAVCAN();
    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    void init() override {}

    bool has_current() const override {
        return true;
    }

    void handle_battery_info(const BattInfoCb &cb);
    virtual AP_UAVCAN* get_uavcan_manager() override { return _ap_uavcan; }
    virtual uint8_t get_uavcan_node() override { return _node_id; }

    static void subscribe_battmon_uavcan_messages(AP_UAVCAN* ap_uavcan);
    static AP_BattMonitor_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id);

private:
    AP_BattMonitor::BattMonitor_State _interim_state;
    BattMonitor_UAVCAN_Type _type;
    AP_UAVCAN* _ap_uavcan;
    uint8_t _node_id;
    float full_charge_capacity_wh;
    AP_HAL::Semaphore *_sem_battmon;
};
