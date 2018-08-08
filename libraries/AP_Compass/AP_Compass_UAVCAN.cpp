/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_Compass_UAVCAN.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

#if HAL_OS_POSIX_IO
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#include <uavcan/equipment/ahrs/MagneticFieldStrength.hpp>
#include <uavcan/equipment/ahrs/MagneticFieldStrength2.hpp>

extern const AP_HAL::HAL& hal;

#define debug_mag_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { printf(fmt, ##args); }} while (0)


//Frontend Registry Binders
UC_REGISTRY_BINDER(MagCb, uavcan::equipment::ahrs::MagneticFieldStrength);
UC_REGISTRY_BINDER(Mag2Cb, uavcan::equipment::ahrs::MagneticFieldStrength2);

AP_Compass_UAVCAN::DetectedModules AP_Compass_UAVCAN::_detected_modules[] = {0};
AP_HAL::Semaphore* AP_Compass_UAVCAN::_sem_registry = nullptr;


void AP_Compass_UAVCAN::subscribe_compass_uavcan_messages(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength, MagCb> *mag_listener;
    mag_listener = new uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength, MagCb>(*node);
    const int mag_listener_res = mag_listener->start(MagCb(ap_uavcan, &handle_magnetic_field));
    if (mag_listener_res < 0) {
        AP_HAL::panic("UAVCAN Mag subscriber start problem\n\r");
        return;
    }

    uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength2, Mag2Cb> *mag2_listener;
    mag2_listener = new uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength2, Mag2Cb>(*node);
    const int mag2_listener_res = mag2_listener->start(Mag2Cb(ap_uavcan, &handle_magnetic_field_2));
    if (mag2_listener_res < 0) {
        AP_HAL::panic("UAVCAN Mag subscriber start problem\n\r");
        return;
    }
}

bool AP_Compass_UAVCAN::take_registry()
{
    if (_sem_registry == nullptr) {
        _sem_registry = hal.util->new_semaphore();
    }
    return _sem_registry->take(HAL_SEMAPHORE_BLOCK_FOREVER);
}

void AP_Compass_UAVCAN::give_registry()
{
    _sem_registry->give();
}

void AP_Compass_UAVCAN::handle_magnetic_field_2(AP_UAVCAN* ap_uavcan, uint8_t node_id, const Mag2Cb &cb)
{
    Vector3f mag_vector;
    uint8_t sensor_id = cb.msg->sensor_id;
    AP_Compass_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, sensor_id);
    if (driver == nullptr) {
        return;
    }
    mag_vector[0] = cb.msg->magnetic_field_ga[0];
    mag_vector[1] = cb.msg->magnetic_field_ga[1];
    mag_vector[2] = cb.msg->magnetic_field_ga[2];
    driver->handle_mag_msg(mag_vector);
}

void AP_Compass_UAVCAN::handle_magnetic_field(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MagCb &cb)
{
    Vector3f mag_vector;
    AP_Compass_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, 0);
    if (driver == nullptr) {
        return;
    }
    mag_vector[0] = cb.msg->magnetic_field_ga[0];
    mag_vector[1] = cb.msg->magnetic_field_ga[1];
    mag_vector[2] = cb.msg->magnetic_field_ga[2];
    driver->handle_mag_msg(mag_vector);
}

AP_Compass_UAVCAN* AP_Compass_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t sensor_id)
{
    AP_Compass_UAVCAN* driver = nullptr;
    if (ap_uavcan == nullptr) {
        return nullptr;
    }
    Compass& _frontend = AP::compass();
    for (uint8_t i = 0; i < _frontend._backend_count; i++) {
        if (_frontend._backends[i] == nullptr) {
            continue;
        }
        if (_frontend._backends[i]->get_uavcan_manager() == ap_uavcan && 
            _frontend._backends[i]->get_uavcan_node() == node_id &&
            _frontend._backends[i]->get_uavcan_sensor_id() == sensor_id) {
            return (AP_Compass_UAVCAN*)_frontend._backends[i];
        }
    }

    if (driver == nullptr) {
        if (take_registry()) {
            bool already_detected = false;
            //Check if there's an empty spot for possible registeration
            for (uint8_t i = 0; i < COMPASS_MAX_BACKEND; i++) {
                if (_detected_modules[i].ap_uavcan == ap_uavcan && 
                    _detected_modules[i].node_id == node_id &&
                    _detected_modules[i].sensor_id == sensor_id) {
                    //Already Detected
                    already_detected = true;
                }
            }
            if (!already_detected) {
                for (uint8_t i = 0; i < COMPASS_MAX_BACKEND; i++) {
                    if (_detected_modules[i].ap_uavcan == nullptr) {
                        _detected_modules[i].ap_uavcan = ap_uavcan;
                        _detected_modules[i].node_id = node_id;
                        _detected_modules[i].sensor_id = sensor_id;
                        break;
                    }
                }
            }
            give_registry();
        }
    }
    return driver;
}

AP_Compass_Backend* AP_Compass_UAVCAN::probe(Compass& _frontend)
{
    if (!take_registry()) {
        return nullptr;
    }
    AP_Compass_UAVCAN* driver = nullptr;
    for (uint8_t i = 0; i < COMPASS_MAX_BACKEND; i++) {
        if (_detected_modules[i].ap_uavcan != nullptr) {
            //Register new Compass mode to a backend
            driver = new AP_Compass_UAVCAN(_frontend, _detected_modules[i].ap_uavcan, _detected_modules[i].node_id, _detected_modules[i].sensor_id);
            if (driver) {
                driver->init();
                debug_mag_uavcan(2, _detected_modules[i].ap_uavcan->get_driver_num(),"Found Mag Node %d on Bus %d Sensor ID %d\n", _detected_modules[i].node_id, _detected_modules[i].ap_uavcan->get_driver_num(), _detected_modules[i].sensor_id);
            }
            _detected_modules[i].ap_uavcan = nullptr;
            break;
        }
    }
    give_registry();
    return driver;
}

/*
  constructor - registers instance at top Compass driver
 */
AP_Compass_UAVCAN::AP_Compass_UAVCAN(Compass &compass, AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t sensor_id):
    AP_Compass_Backend(compass),
    _ap_uavcan(ap_uavcan),
    _node_id(node_id),
    _sensor_id(sensor_id)
{
    _sem_mag = hal.util->new_semaphore();
}

AP_Compass_UAVCAN::~AP_Compass_UAVCAN()
{
    delete _sem_mag;
}

void AP_Compass_UAVCAN::init()
{
    _instance = register_compass();

    struct DeviceStructure {
        uint8_t bus_type : 3;
        uint8_t bus: 5;
        uint8_t address;
        uint8_t devtype;
    };
    union DeviceId {
        struct DeviceStructure devid_s;
        uint32_t devid;
    };
    union DeviceId d;

    d.devid_s.bus_type = 3;
    d.devid_s.bus = _ap_uavcan->get_driver_num();
    d.devid_s.address = _node_id;
    d.devid_s.devtype = 1;

    set_dev_id(_instance, d.devid);
    set_external(_instance, true);

    _sum.zero();
    _count = 0;

    debug_mag_uavcan(2, _ap_uavcan->get_driver_num(),  "AP_Compass_UAVCAN loaded\n\r");
}

void AP_Compass_UAVCAN::read(void)
{
    // avoid division by zero if we haven't received any mag reports
    if (_count == 0) {
        return;
    }

    if (_sem_mag->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _sum /= _count;

        publish_filtered_field(_sum, _instance);

        _sum.zero();
        _count = 0;
        _sem_mag->give();
    }
}

void AP_Compass_UAVCAN::handle_mag_msg(Vector3f &mag)
{
    Vector3f raw_field = mag * 1000.0;

    // rotate raw_field from sensor frame to body frame
    rotate_field(raw_field, _instance);

    // publish raw_field (uncorrected point sample) for calibration use
    publish_raw_field(raw_field, _instance);

    // correct raw_field for known errors
    correct_field(raw_field, _instance);

    if (_sem_mag->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        // accumulate into averaging filter
        _sum += raw_field;
        _count++;
        _sem_mag->give();
    }
}

#endif
