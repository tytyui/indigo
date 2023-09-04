// Copyright (C) 2020 Rumen G. Bogdanovski
// All rights reserved.
//
// You can use this software under the terms of 'INDIGO Astronomy
// open-source license' (see LICENSE.md).
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHORS 'AS IS' AND ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// version history
// 2.0 by Rumen G. Bogdanovski

/** grus focuser driver
 \file indigo_focuser_grus.c
 */

#define DRIVER_VERSION  0x0001
#define DRIVER_NAME     "indigo_focuser_grus"

#include <indigo/indigo_driver_xml.h>
#include <indigo/indigo_io.h>

#include "indigo_focuser_grus.h"

#define SERIAL_BAUDRATE "9600"
#define PRIVATE_DATA    ((grus_private_data *)device->private_data)

#define X_MOTOR_MODE_PROPERTY              (PRIVATE_DATA->motor_mode_property)
#define X_MOTOR_MODE_IDLE_OFF_ITEM         (X_MOTOR_MODE_PROPERTY->items+0)
#define X_MOTOR_MODE_ALWAYS_ON_ITEM        (X_MOTOR_MODE_PROPERTY->items+1)

#define X_MOTOR_MODE_PROPERTY_NAME         "X_MOTOR_MODE"
#define X_MOTOR_MODE_IDLE_OFF_ITEM_NAME    "OFF_WHEN_IDLE"
#define X_MOTOR_MODE_ALWAYS_ON_ITEM_NAME   "ALWAYS_ON"

#define X_SETTLE_TIME_PROPERTY             (PRIVATE_DATA->settle_time_property)
#define X_SETTLE_TIME_ITEM                 (X_SETTLE_TIME_PROPERTY->items+0)

#define X_SETTLE_TIME_PROPERTY_NAME        "X_SETTLE_TIME"
#define X_SETTLE_TIME_ITEM_NAME            "SETTLE_TIME"


typedef struct
{
    int handle;
    indigo_timer * focuser_timer, * temperature_timer;
    indigo_property * motor_mode_property;
    indigo_property * settle_time_property;
    pthread_mutex_t port_mutex;
} grus_private_data;

static void focuser_connection_handler(indigo_device * device)
{

}

static indigo_result focuser_attach(indigo_device * device)
{
    assert(device != NULL);
    assert(PRIVATE_DATA != NULL);
    
    if(indigo_focuser_attach(device, DRIVER_NAME, DRIVER_VERSION) == INDIGO_OK)
    {
        PRIVATE_DATA->handle = -1;
        SIMULATION_PROPERTY->hidden = true;
        DEVICE_PORT_PROPERTY->hidden = false;
        DEVICE_PORTS_PROPERTY->hidden = false;
        DEVICE_BAUDRATE_PROPERTY->hidden = false;
        indigo_copy_value(DEVICE_BAUDRATE_ITEM->text.value, SERIAL_BAUDRATE);

        INFO_PROPERTY->count = 6;
        strcpy(INFO_DEVICE_MODEL_ITEM->text.value, FOCUSER_GRUS_NAME);
        //最小位置
        FOCUSER_LIMITS_PROPERTY->hidden = false;
        FOCUSER_LIMITS_MAX_POSITION_ITEM->number.min = 1;
        FOCUSER_LIMITS_MAX_POSITION_ITEM->number.max = 0xFFFF;
        FOCUSER_LIMITS_MAX_POSITION_ITEM->number.step = 1;
        FOCUSER_LIMITS_MAX_POSITION_ITEM->number.value = 1;
        FOCUSER_LIMITS_MAX_POSITION_ITEM->number.target = 1;
        //最大位置
        FOCUSER_LIMITS_MIN_POSITION_ITEM->number.min = 1;
        FOCUSER_LIMITS_MIN_POSITION_ITEM->number.max = 0xFFFF;
        FOCUSER_LIMITS_MIN_POSITION_ITEM->number.step = 1;
        FOCUSER_LIMITS_MIN_POSITION_ITEM->number.value = 0xFFFF;
        FOCUSER_LIMITS_MIN_POSITION_ITEM->number.target = 0xFFFF;
        //电机速度
        FOCUSER_SPEED_PROPERTY->hidden = false;
        FOCUSER_SPEED_ITEM->number.min = 1;
        FOCUSER_SPEED_ITEM->number.max = 16;
        FOCUSER_SPEED_ITEM->number.step = 1;
        FOCUSER_SPEED_ITEM->number.value = 4;
        FOCUSER_SPEED_ITEM->number.target = 4;
        //位置
        FOCUSER_POSITION_ITEM->number.min = FOCUSER_LIMITS_MIN_POSITION_ITEM->number.value;
        FOCUSER_POSITION_ITEM->number.max = FOCUSER_LIMITS_MAX_POSITION_ITEM->number.value;
        FOCUSER_POSITION_ITEM->number.step = 1;
        FOCUSER_POSITION_ITEM->number.target = 5000;
        FOCUSER_POSITION_ITEM->number.value = 5000;
        //当前位置状态
        FOCUSER_ON_POSITION_SET_PROPERTY->hidden = false;
        //电机反转
        FOCUSER_REVERSE_MOTION_PROPERTY->hidden = false;
        //回差
        FOCUSER_BACKLASH_PROPERTY->hidden = false;
        FOCUSER_BACKLASH_ITEM->number.min = 0;
        FOCUSER_BACKLASH_ITEM->number.max = 250;
        FOCUSER_BACKLASH_ITEM->number.step = 1;
        FOCUSER_BACKLASH_ITEM->number.value = 0;
        FOCUSER_BACKLASH_ITEM->number.target = 0;

        ADDITIONAL_INSTANCES_PROPERTY->hidden = DEVICE_CONTEXT->base_device != NULL;

        X_MOTOR_MODE_PROPERTY = indigo_init_switch_property(
            NULL, device->name, 
            X_MOTOR_MODE_PROPERTY_NAME, 
            "Advanced", "Motor Power", 
            INDIGO_OK_STATE,
            INDIGO_RW_PERM,
            INDIGO_ONE_OF_MANY_RULE, 2);
        if(X_MOTOR_MODE_PROPERTY == NULL)
            return INDIGO_FAILED;
        X_MOTOR_MODE_PROPERTY->hidden = false;
        indigo_init_switch_item(
            X_MOTOR_MODE_IDLE_OFF_ITEM,
            X_MOTOR_MODE_IDLE_OFF_ITEM_NAME,
            "OFF When Idle", false);
        indigo_init_switch_item(
            X_MOTOR_MODE_ALWAYS_ON_ITEM,
            X_MOTOR_MODE_ALWAYS_ON_ITEM_NAME,
            "Always ON", false);
        
        X_SETTLE_TIME_PROPERTY = indigo_init_number_property(
            NULL, device->name,
            X_SETTLE_TIME_PROPERTY_NAME,
            "Advanced", "Settle Time",
            INDIGO_OK_STATE,
            INDIGO_RW_PERM, 1);
        if(X_SETTLE_TIME_PROPERTY == NULL)
            return INDIGO_FAILED;
        indigo_init_number_item(
            X_SETTLE_TIME_ITEM,
            X_SETTLE_TIME_ITEM_NAME,
            "Settle time (ms)",
            0, 999, 10, 0);

        pthread_mutex_init(&PRIVATE_DATA->port_mutex, NULL);
        INDIGO_DEVICE_ATTACH_LOG(DRIVER_NAME, device->name);
        return indigo_focuser_enumerate_properties(device, NULL, NULL);
    }
}

static indigo_result focuser_enumerate_properties(indigo_device * device, indigo_client * client, indigo_property * property)
{
    if(IS_CONNECTED)
    {
        if(indigo_property_match(X_MOTOR_MODE_PROPERTY, property))
            indigo_define_property(device, X_MOTOR_MODE_PROPERTY, NULL);
        if(indigo_property_match(X_SETTLE_TIME_PROPERTY, property))
            indigo_define_property(device, X_SETTLE_TIME_PROPERTY, NULL);
    }
    return indigo_focuser_enumerate_properties(device, NULL, NULL);
}

static indigo_result focuser_change_property(indigo_device * device, indigo_client * client, indigo_property * property)
{

}

static indigo_result focuser_detach(indigo_device * device)
{
    assert(device != NULL);
    if(IS_CONNECTED)
    {
        indigo_set_switch(CONNECTION_PROPERTY, CONNECTION_DISCONNECTED_ITEM, true);
        focuser_connection_handler(device);
    }
    indigo_release_property(X_MOTOR_MODE_PROPERTY);
    indigo_release_property(X_SETTLE_TIME_PROPERTY);
    pthread_mutex_destroy(&PRIVATE_DATA->port_mutex);
    indigo_global_unlock(device);
    INDIGO_DEVICE_DETACH_LOG(DRIVER_NAME, device->name);

    return indigo_focuser_detach(device);
}

indigo_result indigo_focuser_grus(indigo_driver_action action, indigo_driver_info *info)
{
    static grus_private_data * private_data = NULL;
    static indigo_driver_action last_action = INDIGO_DRIVER_SHUTDOWN;
    static indigo_device * focuser = NULL;

    static indigo_device focuser_template = INDIGO_DEVICE_INITIALIZER(
        "GrusFocus",
        focuser_attach,
        focuser_enumerate_properties,
        focuser_change_property,
        NULL,
        focuser_detach
    );

    SET_DRIVER_INFO(info, "GrusFocus", __FUNCTION__, DRIVER_VERSION, false, last_action);
    if(action == last_action)
        return INDIGO_OK;
    
    switch (action)
    {
        case INDIGO_DRIVER_INIT:
            last_action = action;
            private_data = indigo_safe_malloc(sizeof(grus_private_data));
            focuser = indigo_safe_malloc_copy(sizeof(indigo_device), &focuser_template);
            focuser->private_data = private_data;
            indigo_attach_device(focuser);
            break;
        case INDIGO_DRIVER_SHUTDOWN:
            VERIFY_NOT_CONNECTED(focuser);
            last_action = action;
            if(focuser != NULL)
            {
                indigo_detach_device(focuser);
                free(focuser);
                focuser = NULL;
            }
            if(private_data != NULL)
            {
                free(private_data);
                private_data = NULL;
            }
            break;
        case INDIGO_DRIVER_INFO:
            break; 
    }

    return INDIGO_OK;
}