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

#include <unistd.h>
#include <errno.h>
#include <sys/time.h>

#include <indigo/indigo_driver_xml.h>
#include <indigo/indigo_io.h>

#include "indigo_focuser_grus.h"

#define SERIAL_BAUDRATE "9600"
#define PRIVATE_DATA    ((grus_private_data *)device->private_data)
#define GRUS_CMD_LEN    9

#define PORT_MUTEX                          (PRIVATE_DATA->port_mutex)
#define PORT_MUTEX_T                        (&PORT_MUTEX)
#define MUTEX_LOCK()                        pthread_mutex_lock(PORT_MUTEX_T)
#define MUTEX_UNLOCK()                      pthread_mutex_unlock(PORT_MUTEX_T);

#define DRV_DEBUG(fmt, ...)                 INDIGO_DRIVER_DEBUG(DRIVER_NAME, fmt, ##__VA_ARGS__)
#define DRV_ERROR(fmt, ...)                 INDIGO_DRIVER_ERROR(DRIVER_NAME, fmt, ##__VA_ARGS__)
#define DRV_LOG(fmt, ...)                   INDIGO_DRIVER_LOG(DRIVER_NAME, fmt, ##__VA_ARGS__)
#define DRV_TRACE(fmt, ...)                 INDIGO_DRIVER_TRACE(DRIVER_NAME, fmt, ##__VA_ARGS__)

#define X_MOTOR_MODE_PROPERTY               (PRIVATE_DATA->motor_mode_property)
#define X_MOTOR_MODE_IDLE_OFF_ITEM          (X_MOTOR_MODE_PROPERTY->items+0)
#define X_MOTOR_MODE_ALWAYS_ON_ITEM         (X_MOTOR_MODE_PROPERTY->items+1)

#define X_MOTOR_MODE_PROPERTY_NAME          "X_MOTOR_MODE"
#define X_MOTOR_MODE_IDLE_OFF_ITEM_NAME     "OFF_WHEN_IDLE"
#define X_MOTOR_MODE_ALWAYS_ON_ITEM_NAME    "ALWAYS_ON"

#define X_SPEED_MODE_1_ITEM                 (FOCUSER_SPEED_PROPERTY->items+0)
#define X_SPEED_MODE_2_ITEM                 (FOCUSER_SPEED_PROPERTY->items+1)
#define X_SPEED_MODE_4_ITEM                 (FOCUSER_SPEED_PROPERTY->items+2)
#define X_SPEED_MODE_8_ITEM                 (FOCUSER_SPEED_PROPERTY->items+3)
#define X_SPEED_MODE_16_ITEM                (FOCUSER_SPEED_PROPERTY->items+4)

#define X_SPEED_MODE_1_ITEM_NAME            "SPEED_MODE_1"
#define X_SPEED_MODE_2_ITEM_NAME            "SPEED_MODE_2"
#define X_SPEED_MODE_4_ITEM_NAME            "SPEED_MODE_4"
#define X_SPEED_MODE_8_ITEM_NAME            "SPEED_MODE_8"
#define X_SPEED_MODE_16_ITEM_NAME           "SPEED_MODE_16"

#define X_BACKLASH_ENABLE_PROPERTY          (PRIVATE_DATA->backlash_enable_property)
#define X_BACKLASH_ENABLE_IN_ITEM           (X_BACKLASH_ENABLE_PROPERTY->items+0)
#define X_BACKLASH_ENABLE_OUT_ITEM          (X_BACKLASH_ENABLE_PROPERTY->items+1)

#define X_BACKLASH_IN_PROPERTY              (PRIVATE_DATA->backlash_in_property)
#define X_BACKLASH_OUT_PROPERTY             (PRIVATE_DATA->backlash_out_property)

#define X_BACKLASH_IN_ITEM                  (X_BACKLASH_IN_PROPERTY->items+0)
#define X_BACKLASH_OUT_ITEM                 (X_BACKLASH_OUT_PROPERTY->items+0)

#define X_BACKLASH_ENABLE_PROPERTY_NAME     "X_BACKLASH_ENABLE"
#define X_BACKLASH_ENABLE_IN_ITEM_NAME      "BACKLASH_ENABLE_IN"
#define X_BACKLASH_ENABLE_OUT_ITEM_NAME     "BACKLASH_ENABLE_OUT"
#define X_BACKLASH_IN_PROPERTY_NAME         "X_BACKLASH_IN"
#define X_BACKLASH_OUT_PROPERTY_NAME        "X_BACKLASH_OUT"
#define X_BACKLASH_IN_ITEM_NAME             "BACKLASH_IN"
#define X_BACKLASH_OUT_ITEM_NAME            "BACKLASH_OUT"

#define GRUS_COMMAND(dev,cmd)               grus_command(dev, cmd, NULL, 0, 200)

typedef struct
{
    uint32_t in_value;
    uint32_t out_value;
    bool in_enable;
    bool out_enable;
} backlash_data;

typedef struct
{
    int handle;
    uint32_t current_position;
    uint32_t target_position;
    uint32_t max_position;
    bool poitive_last_move;
    backlash_data backlash;
    indigo_timer * focuser_timer;
    indigo_timer * temperature_timer;
    indigo_property * motor_mode_property;
    indigo_property * settle_time_property;
    indigo_property * backlash_enable_property;
    indigo_property * backlash_in_property;
    indigo_property * backlash_out_property;
    pthread_mutex_t port_mutex;
} grus_private_data;

typedef enum
{
    MOTOR_MODE_IDLE_OFF = 0,
    MOTOR_MODE_ALWAYS_ON = 1
} motormode_t;

typedef enum
{
    SPEED_MODE_1 = 1,
    SPEED_MODE_2 = 2,
    SPEED_MODE_4 = 4,
    SPEED_MODE_8 = 8,
    SPEED_MODE_16 = 16
} speedmode_t;

typedef enum
{
    BL_TYPE_IN = 1,
    BL_TYPE_OUT = 2
} backlash_type_t;

#define NO_TEMP_READ (-127)

static bool grus_command(indigo_device * device, const char * command, char * response, int max, int sleep)
{
    char c;
    struct timeval tv;
    MUTEX_LOCK();
    while(true)
    {
        fd_set readout;
        FD_ZERO(&readout);
        FD_SET(PRIVATE_DATA->handle, &readout); 
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        long result = select(PRIVATE_DATA->handle+1, &readout, NULL, NULL, &tv);
        if(result == 0)
            break;
        if(result < 0)
        {
            MUTEX_UNLOCK();
            return false;
        }
        result = read(PRIVATE_DATA->handle, &c, 1);
        if(result < 1)
        {
            MUTEX_UNLOCK();
            return false;
        }
    }
    indigo_write(PRIVATE_DATA->handle, command, strlen(command));
    if(sleep > 0)
        indigo_usleep(sleep);
    if(response != NULL)
    {
        int index = 0;
        int timeout = 3;
        while ((index < max))
        {
            fd_set readout;
            FD_ZERO(&readout);
            FD_SET(PRIVATE_DATA->handle, &readout);
            tv.tv_sec = 0;
            tv.tv_usec = 100000;
            timeout = 0;
            long result = select(PRIVATE_DATA->handle+1, &readout, NULL, NULL, &tv);
            if(result <= 0)
                break;
            result = read(PRIVATE_DATA->handle, &c, 1);
            if(result < 1)
            {
                MUTEX_UNLOCK();
                DRV_ERROR("Failed to read from %s -> %s (%d)", DEVICE_PORT_ITEM->text.value, strerror(errno), errno);
                return false;
            }
            response[index++] = c;
            if(c == '#')
                break;
        }
        response[index] = 0;
    }
    MUTEX_UNLOCK();
    DRV_DEBUG("Command %s -> %s", command, response != NULL ? response : "NULL");
    return true;
}

static bool grus_command_get_int_value(indigo_device * device, const char * command, char expect, uint32_t * value)
{
    if(!value)  return false;
    char response[GRUS_CMD_LEN] = {0};
    if(grus_command(device, command, response, GRUS_CMD_LEN, 200))
    {
        char format[GRUS_CMD_LEN];
        sprintf(format, ":%c%%06d#", expect);
        int parsed = sscanf(response, format, value);
        if(parsed != 1) return false;
        DRV_DEBUG("%s -> %s = %d", command, response, *value);
        return true;
    }
    DRV_ERROR("No Response");
    return false;
}

static bool grus_command_valid(indigo_device * device, const char * command, char expect)
{
    uint32_t value;
    return grus_command_get_int_value(device, command, expect, &value);
}

static bool grus_set_reverse(indigo_device * device, bool enabled)
{
    char cmd[GRUS_CMD_LEN];
    snprintf(cmd, GRUS_CMD_LEN, ":R%06d#", enabled? 1 : 0);
    return grus_command_valid(device, cmd, 'R');
}

static bool grus_goto_position(indigo_device * device, uint32_t position)
{
    char cmd[GRUS_CMD_LEN];
    snprintf(cmd, GRUS_CMD_LEN, ":G%06d#", position);
    return GRUS_COMMAND(device, cmd);
}

static bool grus_goto_position_comp(indigo_device * device, uint32_t position)
{
    uint32_t target_position = 
        indigo_compensate_backlash(
            position,
            (int)PRIVATE_DATA->current_position,
            (int)FOCUSER_BACKLASH_ITEM->number.value,
            &PRIVATE_DATA->poitive_last_move);
    return grus_goto_position(device, target_position);
}

static bool grus_sync_position(indigo_device * device, uint32_t position)
{
    char cmd[GRUS_CMD_LEN];
    snprintf(cmd, GRUS_CMD_LEN, ":S%06d#", position);
    return grus_command_valid(device, cmd, 'D');
}

static bool grus_get_position(indigo_device * device, uint32_t * position)
{
    return grus_command_get_int_value(device, ":S000000#", 'D', position);
}

static bool grus_set_max_position(indigo_device * device, int position)
{
    char cmd[GRUS_CMD_LEN];
    snprintf(cmd, GRUS_CMD_LEN, ":L%06d#", position);
    return grus_command_valid(device, cmd, 'L');
}

static bool grus_get_max_position(indigo_device * device, int * position)
{
    return grus_command_get_int_value(device, ":L000000", 'L', position);
}

static bool grus_set_speed(indigo_device * device, int speed)
{
    char cmd[GRUS_CMD_LEN];
    snprintf(cmd, GRUS_CMD_LEN, ":Z%06d#", speed);
    return grus_command_valid(device, cmd, 'Z');
}

static bool grus_get_speed(indigo_device * device, int * speed)
{
    return grus_command_get_int_value(device, ":Z000000#", 'Z', speed);
}

static bool grus_get_motor_mode(indigo_device * device, int * mode)
{
    return grus_command_get_int_value(device, ":F000002#", 'F', mode);    
}

static bool grus_set_motor_mode(indigo_device * device, int mode)
{
    char cmd[GRUS_CMD_LEN];
    snprintf(cmd, GRUS_CMD_LEN, ":F%06d#", mode);
    return grus_command_valid(device, cmd, 'F');
}

static bool grus_stop(indigo_device * device)
{
    return GRUS_COMMAND(device, ":Q000000#");
}

static bool grus_clear_backlash(indigo_device * device, int type)
{
    char cmd[GRUS_CMD_LEN];
    snprintf(cmd, GRUS_CMD_LEN, ":B1%01d0000#", type);
    return grus_command_valid(device, cmd, 'B');
}

static bool grus_set_backlash(indigo_device * device, int type, int value)
{
    char cmd[GRUS_CMD_LEN];
    snprintf(cmd, GRUS_CMD_LEN, ":B1%01d1%03d", type, value);
    return grus_command_valid(device, cmd, 'B');
}

static bool grus_get_backlash(indigo_device * device, backlash_data * data)
{
    if(data == NULL)
        return false;
    char response[GRUS_CMD_LEN + GRUS_CMD_LEN];
    bool result = grus_command(device, ":B000000#", response, GRUS_CMD_LEN + GRUS_CMD_LEN, 200);
    if(!result)
        return false;
    int parsed = sscanf(response, ":B01%01d%03d#:B02%01d%03d#", 
        &data->in_enable, &data->in_value, 
        &data->out_enable, &data->out_value);
    if(parsed != 4)
        return false;
    return true;
}

//TODO
static void focuser_connection_handler(indigo_device * device)
{

}

//TODO
static void focuser_timer_handler(indigo_device * device)
{
    
}

static void update_speed_mode_switches(indigo_device * device)
{
    speedmode_t value;

    if(!grus_get_speed(device, &value))
    {
        DRV_ERROR("grus_get_speed(%d) failed", PRIVATE_DATA->handle);
        return;
    }

    switch (value)
    {
        case SPEED_MODE_1:
            indigo_set_switch(FOCUSER_SPEED_PROPERTY, X_SPEED_MODE_1_ITEM, true);
            break;
        case SPEED_MODE_2:
            indigo_set_switch(FOCUSER_SPEED_PROPERTY, X_SPEED_MODE_2_ITEM, true);
            break;
        case SPEED_MODE_4:
            indigo_set_switch(FOCUSER_SPEED_PROPERTY, X_SPEED_MODE_4_ITEM, true);
            break;
        case SPEED_MODE_8:
            indigo_set_switch(FOCUSER_SPEED_PROPERTY, X_SPEED_MODE_8_ITEM, true);
            break;
        case SPEED_MODE_16:
            indigo_set_switch(FOCUSER_SPEED_PROPERTY, X_SPEED_MODE_16_ITEM, true);
            break;
        default:
            DRV_ERROR("grus_get_speed(%d) wrong value %d", PRIVATE_DATA->handle, value);    
    }
}

static void update_motor_mode_switches(indigo_device * device)
{
    motormode_t value;
    if(!grus_get_motor_mode(device, &value))
    {
        DRV_ERROR("grus_get_motor_mode(%d) failed", PRIVATE_DATA->handle);
        return;
    }

    switch (value)
    {
        case MOTOR_MODE_IDLE_OFF:
            indigo_set_switch(X_MOTOR_MODE_PROPERTY, X_MOTOR_MODE_IDLE_OFF_ITEM, true);
            break;
        case MOTOR_MODE_ALWAYS_ON:
            indigo_set_switch(X_MOTOR_MODE_PROPERTY, X_MOTOR_MODE_ALWAYS_ON_ITEM, true);
            break;
        default:
            DRV_ERROR("grus_get_motor_mode(%d) wrong value %d", PRIVATE_DATA->handle, value);
    }
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
        //电机速度
        FOCUSER_SPEED_PROPERTY = indigo_init_switch_property(
            NULL, device->name,
            FOCUSER_SPEED_ITEM_NAME,
            FOCUSER_MAIN_GROUP, 
            "Focuser speed",
            INDIGO_OK_STATE,
            INDIGO_RW_PERM,
            INDIGO_ONE_OF_MANY_RULE, 5);
        if(FOCUSER_SPEED_PROPERTY == NULL)
            return INDIGO_FAILED;
        FOCUSER_SPEED_PROPERTY->hidden = false;
        indigo_init_switch_item(X_SPEED_MODE_1_ITEM, X_SPEED_MODE_1_ITEM_NAME, "Speed 1", false);
        indigo_init_switch_item(X_SPEED_MODE_2_ITEM, X_SPEED_MODE_2_ITEM_NAME, "Speed 2", false);
        indigo_init_switch_item(X_SPEED_MODE_4_ITEM, X_SPEED_MODE_4_ITEM_NAME, "Speed 4", false);
        indigo_init_switch_item(X_SPEED_MODE_8_ITEM, X_SPEED_MODE_8_ITEM_NAME, "Speed 8", false);
        indigo_init_switch_item(X_SPEED_MODE_16_ITEM, X_SPEED_MODE_16_ITEM_NAME, "Speed 16", false);

        //内部回差
        X_BACKLASH_ENABLE_PROPERTY = indigo_init_switch_property(
            NULL, device->name,
            X_BACKLASH_ENABLE_PROPERTY_NAME,
            "Advance", "Internal Backlash Enable",
            INDIGO_OK_STATE,
            INDIGO_RW_PERM,
            INDIGO_ANY_OF_MANY_RULE, 2);
        if(X_BACKLASH_ENABLE_PROPERTY == NULL)
            return INDIGO_FAILED;
        X_BACKLASH_ENABLE_PROPERTY->hidden = false;
        indigo_init_switch_item(X_BACKLASH_ENABLE_IN_ITEM, X_BACKLASH_ENABLE_IN_ITEM_NAME, "Enable In", false);
        indigo_init_switch_item(X_BACKLASH_ENABLE_OUT_ITEM, X_BACKLASH_ENABLE_OUT_ITEM_NAME, "Enable Out", false);
        
        //IN回差
        X_BACKLASH_IN_PROPERTY = indigo_init_number_property(
            NULL, device->name,
            X_BACKLASH_IN_PROPERTY_NAME,
            "Advanced", "Internel Backlash In",
            INDIGO_OK_STATE,
            INDIGO_RW_PERM, 1);
        if(X_BACKLASH_IN_PROPERTY == NULL)
            return INDIGO_FAILED;
        indigo_init_number_item(X_BACKLASH_IN_ITEM, X_BACKLASH_IN_ITEM_NAME, "In Value", 0, 250, 1, 0);
        //OUT回差
        X_BACKLASH_OUT_PROPERTY = indigo_init_number_property(
            NULL, device->name,
            X_BACKLASH_OUT_PROPERTY_NAME,
            "Advanced", "Internel Backlash In",
            INDIGO_OK_STATE,
            INDIGO_RW_PERM, 1);
        if(X_BACKLASH_OUT_PROPERTY == NULL)
            return INDIGO_FAILED;
        indigo_init_number_item(X_BACKLASH_OUT_ITEM, X_BACKLASH_OUT_ITEM_NAME, "Out Value", 0, 250, 1, 0);
        
        //电机通电状态
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
        indigo_init_switch_item(X_MOTOR_MODE_IDLE_OFF_ITEM, X_MOTOR_MODE_IDLE_OFF_ITEM_NAME, "OFF When Idle", false);
        indigo_init_switch_item(X_MOTOR_MODE_ALWAYS_ON_ITEM, X_MOTOR_MODE_ALWAYS_ON_ITEM_NAME, "Always ON", false);
        
        pthread_mutex_init(PORT_MUTEX_T, NULL);
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
        if(indigo_property_match(X_BACKLASH_ENABLE_PROPERTY, property))
            indigo_define_property(device, X_BACKLASH_ENABLE_PROPERTY, NULL);
        if(indigo_property_match(X_BACKLASH_IN_PROPERTY, property))
            indigo_define_property(device, X_BACKLASH_IN_PROPERTY, NULL);
        if(indigo_property_match(X_BACKLASH_OUT_PROPERTY, property))
            indigo_define_property(device, X_BACKLASH_OUT_PROPERTY, NULL);
    }
    return indigo_focuser_enumerate_properties(device, NULL, NULL);
}

static indigo_result focuser_change_property(indigo_device * device, indigo_client * client, indigo_property * property)
{
    assert(device != NULL);
    assert(DEVICE_CONTEXT != NULL);
    assert(property != NULL);
    if(indigo_property_match_changeable(CONNECTION_PROPERTY, property))
    {
        if(indigo_ignore_connection_change(device, property))
            return INDIGO_OK;
        indigo_property_copy_values(CONNECTION_PROPERTY, property, false);
        CONNECTION_PROPERTY->state = INDIGO_BUSY_STATE;
        indigo_update_property(device, CONNECTION_PROPERTY, NULL);
        indigo_set_timer(device, 0, focuser_connection_handler, NULL);
        return INDIGO_OK;
    }
    else if(indigo_property_match_changeable(FOCUSER_REVERSE_MOTION_PROPERTY, property))
    {
        indigo_property_copy_values(FOCUSER_REVERSE_MOTION_ENABLED_ITEM, property, false);
        FOCUSER_REVERSE_MOTION_PROPERTY->state = INDIGO_OK_STATE;
        if(!grus_set_reverse(device, FOCUSER_REVERSE_MOTION_ENABLED_ITEM->sw.value))
        {
            DRV_ERROR("grus_set_reverse(%d, %d) failed", PRIVATE_DATA->handle, FOCUSER_REVERSE_MOTION_ENABLED_ITEM->sw.value);
            FOCUSER_REVERSE_MOTION_PROPERTY->state = INDIGO_ALERT_STATE;
        }
        indigo_update_property(device, FOCUSER_REVERSE_MOTION_PROPERTY, NULL);
        return INDIGO_OK;
    }
    else if(indigo_property_match_changeable(FOCUSER_POSITION_PROPERTY, property))
    {
        indigo_property_copy_values(FOCUSER_POSITION_PROPERTY, property, false);
        if(FOCUSER_POSITION_ITEM->number.target < FOCUSER_POSITION_ITEM->number.min ||
            FOCUSER_POSITION_ITEM->number.target > FOCUSER_POSITION_ITEM->number.max)
        {
            FOCUSER_POSITION_PROPERTY->state = INDIGO_ALERT_STATE;
            FOCUSER_STEPS_PROPERTY->state = INDIGO_ALERT_STATE;
            indigo_update_property(device, FOCUSER_POSITION_PROPERTY, NULL);
            indigo_update_property(device, FOCUSER_STEPS_PROPERTY, NULL);
        }
        else if(FOCUSER_POSITION_ITEM->number.target == PRIVATE_DATA->current_position)
        {
            FOCUSER_POSITION_PROPERTY->state = INDIGO_OK_STATE;
            FOCUSER_STEPS_PROPERTY->state = INDIGO_OK_STATE;
            indigo_update_property(device, FOCUSER_POSITION_PROPERTY, NULL);
            indigo_update_property(device, FOCUSER_STEPS_PROPERTY, NULL);
        }
        else
        {
            FOCUSER_POSITION_PROPERTY->state = INDIGO_BUSY_STATE;
            FOCUSER_STEPS_PROPERTY->state = INDIGO_BUSY_STATE;
            PRIVATE_DATA->target_position = FOCUSER_POSITION_ITEM->number.target;
            FOCUSER_POSITION_ITEM->number.value = PRIVATE_DATA->current_position;
            indigo_update_property(device, FOCUSER_POSITION_PROPERTY, NULL);
            indigo_update_property(device, FOCUSER_STEPS_PROPERTY, NULL);

            if(FOCUSER_ON_POSITION_SET_GOTO_ITEM->sw.value)
            {
                FOCUSER_POSITION_PROPERTY->state = INDIGO_BUSY_STATE;
                FOCUSER_STEPS_PROPERTY->state = INDIGO_BUSY_STATE;
                if(!grus_goto_position_comp(device, (uint32_t)PRIVATE_DATA->target_position))
                {
                    DRV_ERROR("grus_goto_position_comp(%d, %d) failed", PRIVATE_DATA->handle, PRIVATE_DATA->target_position);
                }
                indigo_set_timer(device, 0.5, focuser_timer_handler, &PRIVATE_DATA->focuser_timer);
            }
            else
            {
                FOCUSER_POSITION_PROPERTY->state = INDIGO_OK_STATE;
                FOCUSER_STEPS_PROPERTY->state = INDIGO_OK_STATE;
                if(!grus_sync_position(device, PRIVATE_DATA->target_position))
                {
                    DRV_ERROR("grus_sync_position(%d, %d) failed", PRIVATE_DATA->handle, PRIVATE_DATA->target_position);
                    FOCUSER_POSITION_PROPERTY->state = INDIGO_ALERT_STATE;
                    FOCUSER_STEPS_PROPERTY->state = INDIGO_ALERT_STATE;
                }
                uint32_t position;
                if(!grus_get_position(device, &position))
                {
                    DRV_ERROR("grus_get_position(%d) failed", PRIVATE_DATA->handle);
                    FOCUSER_POSITION_PROPERTY->state = INDIGO_ALERT_STATE;
                    FOCUSER_STEPS_PROPERTY->state = INDIGO_ALERT_STATE;
                }
                else
                {
                    FOCUSER_POSITION_ITEM->number.value = PRIVATE_DATA->current_position = (double)position;
                }
                indigo_update_property(device, FOCUSER_POSITION_PROPERTY, NULL);
                indigo_update_property(device, FOCUSER_STEPS_PROPERTY, NULL);
            }
        }
        return INDIGO_OK;
    }
    else if(indigo_property_match_changeable(FOCUSER_LIMITS_PROPERTY, property))
    {
        indigo_property_copy_values(FOCUSER_LIMITS_PROPERTY, property, false);
        FOCUSER_LIMITS_PROPERTY->state = INDIGO_OK_STATE;
        PRIVATE_DATA->max_position = (int)FOCUSER_LIMITS_MAX_POSITION_ITEM->number.target;
        if(!grus_set_max_position(device, PRIVATE_DATA->max_position))
        {
            DRV_ERROR("grus_set_max_position(%d) failed", PRIVATE_DATA->handle);
            FOCUSER_LIMITS_PROPERTY->state = INDIGO_ALERT_STATE;
        }
        if(!grus_get_max_position(device, &PRIVATE_DATA->max_position))
        {
            DRV_ERROR("grus_get_max_position(%d) failed", PRIVATE_DATA->handle);
        }
        FOCUSER_LIMITS_MAX_POSITION_ITEM->number.value = (double)PRIVATE_DATA->max_position;
        indigo_update_property(device, FOCUSER_LIMITS_PROPERTY, NULL);
        return INDIGO_OK;
    }
    else if(indigo_property_match_changeable(FOCUSER_SPEED_PROPERTY, property))
    {
        indigo_property_copy_values(FOCUSER_SPEED_PROPERTY, property, false);
        FOCUSER_SPEED_PROPERTY->state = INDIGO_OK_STATE;
        speedmode_t mode = SPEED_MODE_4;
        if(X_SPEED_MODE_1_ITEM->sw.value)
            mode = SPEED_MODE_1;
        else if(X_SPEED_MODE_2_ITEM->sw.value)
            mode = SPEED_MODE_2;
        else if(X_SPEED_MODE_4_ITEM->sw.value)
            mode = SPEED_MODE_4;
        else if(X_SPEED_MODE_8_ITEM->sw.value)
            mode = SPEED_MODE_8;
        else if(X_SPEED_MODE_16_ITEM->sw.value)
            mode = SPEED_MODE_16;
        if(!grus_set_speed(device, mode))
        {
            DRV_ERROR("grus_set_speed(%d, %d) failed", PRIVATE_DATA->handle, mode);
            FOCUSER_SPEED_PROPERTY->state = INDIGO_ALERT_STATE;
        }
        update_speed_mode_switches(device);
        indigo_update_property(device, FOCUSER_SPEED_PROPERTY, NULL);
        return INDIGO_OK;
    }
    else if(indigo_property_match_changeable(FOCUSER_STEPS_PROPERTY, property))
    {
        indigo_property_copy_values(FOCUSER_STEPS_PROPERTY, property, false);
        if(FOCUSER_STEPS_ITEM->number.value < FOCUSER_STEPS_ITEM->number.min || 
            FOCUSER_STEPS_ITEM->number.value > FOCUSER_STEPS_ITEM->number.max)
        {
            FOCUSER_STEPS_PROPERTY->state = INDIGO_ALERT_STATE;
            FOCUSER_POSITION_PROPERTY->state = INDIGO_ALERT_STATE;
            indigo_update_property(device, FOCUSER_STEPS_PROPERTY, NULL);
            indigo_update_property(device, FOCUSER_POSITION_PROPERTY, NULL);
        }
        else
        {
            FOCUSER_STEPS_PROPERTY->state = INDIGO_BUSY_STATE;
            FOCUSER_POSITION_PROPERTY->state = INDIGO_BUSY_STATE;
            indigo_update_property(device, FOCUSER_STEPS_PROPERTY, NULL);
            indigo_update_property(device, FOCUSER_POSITION_PROPERTY, NULL);
            uint32_t position;
            if(!grus_get_position(device, &position))
                DRV_ERROR("grus_get_position(%d) failed", PRIVATE_DATA->handle);
            else 
                PRIVATE_DATA->current_position = (double)position;

            if(FOCUSER_DIRECTION_MOVE_INWARD_ITEM->sw.value)
                PRIVATE_DATA->target_position = PRIVATE_DATA->current_position - FOCUSER_STEPS_ITEM->number.value;
            else if(FOCUSER_DIRECTION_MOVE_OUTWARD_ITEM->sw.value)
                PRIVATE_DATA->target_position = PRIVATE_DATA->current_position + FOCUSER_STEPS_ITEM->number.value;

            if(PRIVATE_DATA->target_position < FOCUSER_POSITION_ITEM->number.max)
                PRIVATE_DATA->target_position = FOCUSER_POSITION_ITEM->number.max;
            else if(PRIVATE_DATA->target_position > FOCUSER_POSITION_ITEM->number.min)
                PRIVATE_DATA->target_position = FOCUSER_POSITION_ITEM->number.min;
            
            FOCUSER_POSITION_ITEM->number.value = PRIVATE_DATA->current_position;
            if(!grus_goto_position_comp(device, (uint32_t)PRIVATE_DATA->target_position))
            {
                DRV_ERROR("grus_goto_position_comp(%d, %d) failed", PRIVATE_DATA->handle, PRIVATE_DATA->target_position);
                FOCUSER_STEPS_PROPERTY->state = INDIGO_ALERT_STATE;
                FOCUSER_POSITION_PROPERTY->state = INDIGO_ALERT_STATE;
            }
            indigo_set_timer(device, 0.5, focuser_timer_handler, &PRIVATE_DATA->focuser_timer);
        }
        return INDIGO_OK;
    }
    else if(indigo_property_match_changeable(FOCUSER_ABORT_MOTION_PROPERTY, property))
    {
        indigo_property_copy_values(FOCUSER_ABORT_MOTION_PROPERTY, property, false);
        FOCUSER_STEPS_PROPERTY->state = INDIGO_OK_STATE;
        FOCUSER_POSITION_PROPERTY->state = INDIGO_OK_STATE;
        FOCUSER_ABORT_MOTION_PROPERTY->state = INDIGO_OK_STATE;
        indigo_cancel_timer(device, &PRIVATE_DATA->focuser_timer);

        if(!grus_stop(device))
        {
            DRV_ERROR("grus_stop(%d) failed", PRIVATE_DATA->handle);
            FOCUSER_ABORT_MOTION_PROPERTY->state = INDIGO_ALERT_STATE;
        }

        uint32_t position;
        if(!grus_get_position(device, &position))
        {
            DRV_ERROR("grus_get_position(%d) failed", PRIVATE_DATA->handle);
            FOCUSER_ABORT_MOTION_PROPERTY->state = INDIGO_ALERT_STATE;
        }
        else
        {
            PRIVATE_DATA->current_position = position;
        }

        FOCUSER_POSITION_ITEM->number.value = PRIVATE_DATA->current_position;
        FOCUSER_ABORT_MOTION_ITEM->sw.value = false;
        indigo_update_property(device, FOCUSER_POSITION_PROPERTY, NULL);
        indigo_update_property(device, FOCUSER_STEPS_PROPERTY, NULL);
        indigo_update_property(device, FOCUSER_ABORT_MOTION_PROPERTY, NULL);

        return INDIGO_OK;
    }
    else if(indigo_property_match_changeable(FOCUSER_COMPENSATION_PROPERTY, property))
    {
        indigo_property_copy_values(FOCUSER_COMPENSATION_PROPERTY, property, false);
        FOCUSER_COMPENSATION_PROPERTY->state = INDIGO_OK_STATE;
        indigo_update_property(device, FOCUSER_COMPENSATION_PROPERTY, NULL);
        return INDIGO_OK;
    }
    else if(indigo_property_match_changeable(FOCUSER_BACKLASH_PROPERTY, property))
    {
        indigo_property_copy_values(FOCUSER_BACKLASH_PROPERTY, property, false);
        FOCUSER_BACKLASH_PROPERTY->state = INDIGO_OK_STATE;
        indigo_update_property(device, FOCUSER_BACKLASH_PROPERTY, NULL);
        return INDIGO_OK;
    }
    else if(indigo_property_match_changeable(X_MOTOR_MODE_PROPERTY, property))
    {
        indigo_property_copy_values(X_MOTOR_MODE_PROPERTY, property, false);
        X_MOTOR_MODE_PROPERTY->state = INDIGO_OK_STATE;
        motormode_t mode = MOTOR_MODE_ALWAYS_ON;
        if(X_MOTOR_MODE_IDLE_OFF_ITEM->sw.value)
            mode = MOTOR_MODE_IDLE_OFF;
        else if(X_MOTOR_MODE_ALWAYS_ON_ITEM->sw.value)
            mode = MOTOR_MODE_ALWAYS_ON;
        if(!grus_set_motor_mode(device, mode))
        {
            DRV_ERROR("grus_set_motor_mode(%d, %d) failed", PRIVATE_DATA->handle, mode);
            X_MOTOR_MODE_PROPERTY->state = INDIGO_ALERT_STATE;
        }
        update_motor_mode_switches(device);
        indigo_update_property(device, X_MOTOR_MODE_PROPERTY, NULL);

        return INDIGO_OK;
    }
    else if(indigo_property_match_changeable(FOCUSER_MODE_PROPERTY, property))
    {
        indigo_property_copy_values(FOCUSER_MODE_PROPERTY, property, false);
        FOCUSER_MODE_PROPERTY->state = INDIGO_OK_STATE;
        if(FOCUSER_MODE_MANUAL_ITEM->sw.value)
        {
            indigo_define_property(device, FOCUSER_ON_POSITION_SET_PROPERTY, NULL);
            indigo_define_property(device, FOCUSER_SPEED_PROPERTY, NULL);
            indigo_define_property(device, FOCUSER_REVERSE_MOTION_PROPERTY, NULL);
            indigo_define_property(device, FOCUSER_DIRECTION_PROPERTY, NULL);
            indigo_define_property(device, FOCUSER_STEPS_PROPERTY, NULL);
            indigo_define_property(device, FOCUSER_ABORT_MOTION_PROPERTY, NULL);
            indigo_define_property(device, FOCUSER_BACKLASH_PROPERTY, NULL);
            indigo_delete_property(device, FOCUSER_POSITION_PROPERTY, NULL);
            FOCUSER_POSITION_PROPERTY->perm = INDIGO_RW_PERM;
            indigo_define_property(device, FOCUSER_POSITION_ITEM, NULL);
        }
        else
        {
            indigo_delete_property(device, FOCUSER_ON_POSITION_SET_PROPERTY, NULL);
            indigo_delete_property(device, FOCUSER_SPEED_PROPERTY, NULL);
            indigo_delete_property(device, FOCUSER_REVERSE_MOTION_PROPERTY, NULL);
            indigo_delete_property(device, FOCUSER_DIRECTION_PROPERTY, NULL);
            indigo_delete_property(device, FOCUSER_STEPS_PROPERTY, NULL);
            indigo_delete_property(device, FOCUSER_ABORT_MOTION_PROPERTY, NULL);
            indigo_delete_property(device, FOCUSER_BACKLASH_PROPERTY, NULL);
            indigo_delete_property(device, FOCUSER_POSITION_PROPERTY, NULL);
            FOCUSER_POSITION_PROPERTY->perm = INDIGO_RO_PERM;
            indigo_define_property(device, FOCUSER_POSITION_ITEM, NULL);
        }
        FOCUSER_MODE_PROPERTY->state = INDIGO_OK_STATE;
        indigo_update_property(device, FOCUSER_MODE_PROPERTY, NULL);
        return INDIGO_OK;
    }
    else if(indigo_property_match_changeable(X_BACKLASH_ENABLE_PROPERTY, property))
    {
        indigo_property_copy_values(X_BACKLASH_ENABLE_PROPERTY, property, false);
        X_BACKLASH_ENABLE_PROPERTY->state = INDIGO_OK_STATE;
        if(X_BACKLASH_ENABLE_IN_ITEM->sw.value)
        {
            if(!grus_get_basklash(device, &PRIVATE_DATA->backlash))
            {
                DRV_ERROR("grus_get_basklash(%d) failed", PRIVATE_DATA->handle);
                X_BACKLASH_ENABLE_PROPERTY->state = INDIGO_ALERT_STATE;
            }
            indigo_define_property(device, X_BACKLASH_IN_PROPERTY, NULL);
            X_BACKLASH_IN_ITEM->number.value = PRIVATE_DATA->backlash.in_value;
            indigo_update_property(device, X_BACKLASH_IN_PROPERTY, NULL);
        }
        else
        {
            if(!grus_clear_backlash(device, BL_TYPE_IN))
            {
                DRV_ERROR("grus_clear_backlash(%d, %d) failed", PRIVATE_DATA->handle, BL_TYPE_IN);
                X_BACKLASH_ENABLE_PROPERTY->state = INDIGO_ALERT_STATE;
            }
            indigo_delete_property(device, X_BACKLASH_IN_PROPERTY, NULL);
        }
        if(X_BACKLASH_ENABLE_OUT_ITEM->sw.value)
        {
            if(!grus_get_backlash(device,  &PRIVATE_DATA->backlash))
            {
                DRV_ERROR("grus_get_backlash(%d) failed", PRIVATE_DATA->handle);
                X_BACKLASH_ENABLE_PROPERTY->state = INDIGO_ALERT_STATE;
            }
            indigo_define_property(device, X_BACKLASH_OUT_PROPERTY, NULL);
            X_BACKLASH_IN_ITEM->number.value = PRIVATE_DATA->backlash.out_value;
            indigo_update_property(device, X_BACKLASH_OUT_PROPERTY, NULL);
        }
        else
        {
            if(!grus_clear_backlash(device, BL_TYPE_OUT))
            {
                DRV_ERROR("grus_clear_backlash(%d, %d) failed", PRIVATE_DATA->handle, BL_TYPE_OUT);
                X_BACKLASH_ENABLE_PROPERTY->state = INDIGO_ALERT_STATE;
            }
            indigo_delete_property(device, X_BACKLASH_OUT_PROPERTY, NULL);
        }
        indigo_update_property(device, X_BACKLASH_ENABLE_PROPERTY, NULL);
        return INDIGO_OK;
    }
    else if(indigo_property_match_changeable(X_BACKLASH_IN_PROPERTY, property))
    {
        indigo_property_copy_values(X_BACKLASH_IN_PROPERTY, property, false);
        X_BACKLASH_IN_PROPERTY->state = INDIGO_OK_STATE;
        if(!grus_get_backlash(device, &PRIVATE_DATA->backlash))
        {
            DRV_ERROR("grus_get_backlash(%d) failed", PRIVATE_DATA->handle);
            X_BACKLASH_IN_PROPERTY->state = INDIGO_ALERT_STATE;
        }
        X_BACKLASH_IN_ITEM->number.value = PRIVATE_DATA->backlash.in_value;
        if(!grus_set_backlash(device, BL_TYPE_IN, PRIVATE_DATA->backlash.in_value))
        {
            DRV_ERROR("grus_set_backlash(%d, %d, %d) failed", PRIVATE_DATA->handle, BL_TYPE_IN, PRIVATE_DATA->backlash.in_value);
            X_BACKLASH_IN_PROPERTY->state = INDIGO_ALERT_STATE;
        }
        indigo_update_property(device, X_BACKLASH_IN_PROPERTY, NULL);
        return INDIGO_OK;
    }
    else if(indigo_property_match_changeable(X_BACKLASH_OUT_PROPERTY, property))
    {
        indigo_property_copy_values(X_BACKLASH_OUT_PROPERTY, property, false);
        X_BACKLASH_OUT_PROPERTY->state = INDIGO_OK_STATE;
        if(!grus_get_backlash(device, &PRIVATE_DATA->backlash))
        {
            DRV_ERROR("grus_get_backlash(%d) failed", PRIVATE_DATA->handle);
            X_BACKLASH_OUT_PROPERTY->state = INDIGO_ALERT_STATE;
        }
        X_BACKLASH_OUT_ITEM->number.value = PRIVATE_DATA->backlash.out_value;
        if(!grus_set_backlash(device, X_BACKLASH_OUT_PROPERTY, NULL))
        {
            DRV_ERROR("grus_set_backlash(%d, %d, %d) failed", PRIVATE_DATA->handle, BL_TYPE_OUT, PRIVATE_DATA->backlash.out_value);
            X_BACKLASH_OUT_PROPERTY->state = INDIGO_ALERT_STATE;
        }
        indigo_update_property(device, X_BACKLASH_OUT_PROPERTY, NULL);
        return INDIGO_OK;
    }
    else if(indigo_property_match_changeable(CONFIG_PROPERTY, property))
    {
        if(indigo_switch_match(CONFIG_SAVE_ITEM, property))
        {
            indigo_save_property(device, NULL, X_MOTOR_MODE_PROPERTY);
            indigo_save_property(device, NULL, X_BACKLASH_ENABLE_PROPERTY);
            indigo_save_property(device, NULL, X_BACKLASH_IN_PROPERTY);
            indigo_save_property(device, NULL, X_BACKLASH_OUT_PROPERTY);
        }
        return INDIGO_OK;
    }
    return indigo_focuser_change_property(device, client, property);
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
    indigo_release_property(X_BACKLASH_ENABLE_PROPERTY);
    indigo_release_property(X_BACKLASH_IN_PROPERTY);
    indigo_release_property(X_BACKLASH_OUT_PROPERTY);
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
