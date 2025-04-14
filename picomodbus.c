/*
  picomodbus.c - connect to rp2040 (i.e., PICO) ioexpander via modbus RTU

  Part of grblHAL

  Copyright (c) 2023 Expatria Technologies Inc.
  Copyright (c) 2025 Terje Io
  Copyright (c) 2025 Mitchell Grams

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#if PICOMODBUS_ENABLE == 1

#include "grbl/hal.h"
#include "grbl/modbus.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/task.h"

#ifndef PICO_ADDRESS
#define PICO_ADDRESS          10
#endif

#define QUEUE_SIZE            8

#define PICO_RETRIES          5
#define PICO_RETRY_DELAY      250
#define PICO_POLL_INTERVAL    1000
// LONGER INTERVAL FOR KEEPALIVE, BUT SHORTER TO ENSURE RESPONSE TO MESSAGES?
// HOW TO AVOID TRYING TO SEND ANOTHER MESSAGE WHY RETRIES ARE HAPPENING?

#ifndef PICO_ADDR_KEEPALIVE
#define PICO_ADDR_KEEPALIVE   0x0120
#endif

#ifndef PICO_ADDR_DOUT
#define PICO_ADDR_DOUT        0x0110
#endif

#ifndef PICO_ADDR_AOUT
#define PICO_ADDR_AOUT        0x0121
#endif

static enumerate_pins_ptr on_enumerate_pins;
static on_report_options_ptr on_report_options;
static driver_reset_ptr driver_reset;

static void picomodbus_rx_packet (modbus_message_t *msg);
static void picomodbus_rx_exception (uint8_t code, void *context);

static const modbus_callbacks_t callbacks = {
    .retries = PICO_RETRIES,
    .retry_delay = PICO_RETRY_DELAY,    
    .on_rx_packet = picomodbus_rx_packet,
    .on_rx_exception = picomodbus_rx_exception
};

typedef struct {
    uint16_t index;
    modbus_message_t picomodbus_packet;
} QueueItem;

QueueItem message_queue[QUEUE_SIZE];
int front = 0;
int rear = -1;
int item_count = 0;
modbus_message_t current_message;
modbus_message_t * current_msg_ptr = &current_message;
uint16_t current_index;

modbus_message_t keepalive_msg = {
    .context = NULL,
    .crc_check = false,
    .adu[0] = PICO_ADDRESS,
    .adu[1] = ModBus_WriteRegister,
    .adu[2] = (uint8_t)(PICO_ADDR_KEEPALIVE >> 8),
    .adu[3] = (uint8_t)(PICO_ADDR_KEEPALIVE & 0xFF),
    .adu[4] = 0,
    .adu[5] = 0x01,
    .tx_length = 8,
    .rx_length = 8
};

typedef struct {
    uint16_t addr;
    xbar_t aux;
} picomodbus_aux_t;

static uint16_t a_out[1];
static uint16_t d_out[1];
static pin_function_t aux_dout_base = Output_Aux0, aux_aout_base = Output_Analog_Aux0;
static io_ports_data_t analog;
static io_ports_data_t digital;

picomodbus_aux_t aux_dout[] = {
    {
        .addr = PICO_ADDR_DOUT,
        .aux = {
            .pin = 0,
            .port = &d_out[0],
            .group = PinGroup_AuxOutput,
            .cap = {
                .output = On,
                .claimable = On
            },
            .mode = {
                .output = On,
                .analog = On
            }
        }
    },
    {
        .addr = PICO_ADDR_DOUT,
        .aux = {
            .pin = 1,
            .port = &d_out[0],
            .group = PinGroup_AuxOutput,
            .cap = {
                .output = On,
                .claimable = On
            },
            .mode = {
                .output = On,
                .analog = On
            }
        }
    },
    {
        .addr = PICO_ADDR_DOUT,
        .aux = {
            .pin = 2,
            .port = &d_out[0],
            .group = PinGroup_AuxOutput,
            .cap = {
                .output = On,
                .claimable = On
            },
            .mode = {
                .output = On,
                .analog = On
            }
        }
    }
};

picomodbus_aux_t aux_aout[] = {
    {
        .addr = PICO_ADDR_AOUT,
        .aux = {
            .port = &a_out[0],
            .group = PinGroup_AuxOutputAnalog,
            .cap = {
                .output = On,
                .analog = On,
                .claimable = On
            },
            .mode = {
                .output = On,
                .analog = On
            }
        }
    }
};

static bool enqueue_message(modbus_message_t data) {
    static uint16_t message_index;
    if (item_count == QUEUE_SIZE) {
        report_message("Warning: PicoModbus queue is full.", Message_Warning);
        return 0;
    }
    rear = (rear + 1) % QUEUE_SIZE;
    message_queue[rear].picomodbus_packet = data;
    message_queue[rear].index = message_index;
    message_queue[rear].picomodbus_packet.context = &message_queue[rear].index;
    message_index++;
    item_count++;
        return 1;
}

static bool dequeue_message() {
    if (item_count == 0) {
        //report_message("Error: queue is empty", Message_Info);
        return 0;
    }
    current_message = (message_queue[front].picomodbus_packet);
    front = (front + 1) % QUEUE_SIZE;
    item_count--;
    return 1;
}

static bool peek_message() {
    if (item_count == 0) {
        return 0;
    }
    current_message = (message_queue[front].picomodbus_packet);

    return 1;
}

static bool analog_out (uint8_t port, float value)
{
    if(port < analog.out.n_ports) {

        uint16_t *val = (uint16_t *)aux_aout[port].aux.port; //get current value

        *val = (uint16_t)value;

        modbus_message_t cmd = {
            .context = NULL,
            .crc_check = false,
            .adu[0] = PICO_ADDRESS,
            .adu[1] = ModBus_WriteRegister,
            .adu[2] = (uint8_t)(aux_aout[port].addr >> 8),
            .adu[3] = (uint8_t)(aux_aout[port].addr & 0xFF),
            .adu[4] = (uint8_t)(*val >> 8),
            .adu[5] = (uint8_t)*val,
            .tx_length = 8,
            .rx_length = 8
        };

        enqueue_message(cmd);
    }

    return true;
}

static void digital_out (uint8_t port, bool on)
{
    if(port < digital.out.n_ports) {

        uint16_t *val = (uint16_t *)aux_dout[port].aux.port; //get current value

        if(on)
            *val |= (1 << aux_dout[port].aux.pin);
        else
            *val &= ~(1 << aux_dout[port].aux.pin);

        modbus_message_t cmd = {
            .context = NULL,
            .crc_check = false,
            .adu[0] = PICO_ADDRESS,
            .adu[1] = ModBus_WriteRegister,
            .adu[2] = (uint8_t)(aux_dout[port].addr >> 8),
            .adu[3] = (uint8_t)(aux_dout[port].addr & 0xFF),
            .adu[4] = 0,
            .adu[5] = (uint8_t)*val,
            .tx_length = 8,
            .rx_length = 8
        };

        enqueue_message(cmd);
    }
}

static float analog_out_state (xbar_t *output)
{
    float value = -1.0f;

    if(output->id < analog.out.n_ports)
        value = (float)*(uint16_t *)output->port;

    return value;
}

static float digital_out_state (xbar_t *output)
{
    float value = -1.0f;

    if(output->id < digital.out.n_ports)
        value = (float)(!!(*(uint16_t *)output->port & (1 << output->pin)));

    return value;
}

static xbar_t *a_get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    if(dir == Port_Input && port < analog.in.n_ports) {
        info = &pin;
    }

    if(dir == Port_Output && port < analog.out.n_ports) {
        memcpy(&pin, &aux_aout[port].aux, sizeof(xbar_t));
        pin.get_value = analog_out_state;
        info = &pin;
    }

    return info;
}

static xbar_t *d_get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    if(dir == Port_Input && port < digital.in.n_ports) {
//...
        info = &pin;
    }

    if(dir == Port_Output && port < digital.out.n_ports) {
        memcpy(&pin, &aux_dout[port].aux, sizeof(xbar_t));
        pin.get_value = digital_out_state;
//        pin.set_value = digital_out_state;
        info = &pin;
    }

    return info;
}

static void a_set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
//    if(dir == Port_Input && port < analog.in.n_ports)
//        aux_ain[port].description = description;

    if(dir == Port_Output && port < analog.out.n_ports)
        aux_aout[port].aux.description = description;
}

static void d_set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
//    if(dir == Port_Input && port < digital.in.n_ports)
//        aux_din[port].description = description;

    if(dir == Port_Output && port < digital.out.n_ports)
        aux_dout[port].aux.description = description;
}

static enumerate_pins_ptr on_enumerate_pins;

static void onEnumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {};

    on_enumerate_pins(low_level, pin_info, data);

    uint_fast8_t idx;

    for(idx = 0; idx < sizeof(aux_dout) / sizeof(picomodbus_aux_t); idx ++) {

        memcpy(&pin, &aux_dout[idx].aux, sizeof(xbar_t));

        if(!low_level)
            pin.port = "PicoModbus:";

        pin_info(&pin, data);
    };

    for(idx = 0; idx < sizeof(aux_aout) / sizeof(picomodbus_aux_t); idx ++) {

        memcpy(&pin, &aux_aout[idx].aux, sizeof(xbar_t));

        if(!low_level)
            pin.port = "PicoModbus:";

        pin_info(&pin, data);
    };
}

static void get_aux_max (xbar_t *pin, void *data)
{
    if(pin->group == PinGroup_AuxOutput)
        aux_dout_base = max(aux_dout_base, pin->function + 1);
    else if(pin->group == PinGroup_AuxOutputAnalog)
        aux_aout_base = max(aux_aout_base, pin->function + 1);
}

static void picomodbus_send (){

    //can only send if there is something in the queue.
    if(peek_message()){
        modbus_send(current_msg_ptr, &callbacks, true);
    }
}

static void picomodbus_rx_packet (modbus_message_t *msg)
{
    //check the context/index and pop it off the queue if it matches.
    if(*((uint16_t*)msg->context) == *((uint16_t*)current_msg_ptr->context)){
        dequeue_message();
    }
    //else it should stay on the queue to be re-transmitted.
    
}

// Function to flush the queue
static void flush_queue() {
    front = 0;
    rear = -1;
    item_count = 0;
}

static void raise_alarm (void *data)
{
    system_raise_alarm(Alarm_AbortCycle);
    report_message("PicoModbus communication error.", Message_Warning);
    flush_queue();
}

static void picomodbus_rx_exception (uint8_t code, void *context)
{
    if(sys.cold_start){ // is this necessary? Copied from vfd
        protocol_enqueue_foreground_task(raise_alarm, NULL);
    }
    else{
        system_raise_alarm(Alarm_AbortCycle);
        report_message("PicoModbus communication error.", Message_Warning);
        flush_queue();
    }
}

static void picomodbus_poll (void *data)
{
    sys_state_t state = state_get();

    // stop sending messages if alarm?
    // if there is a message try to send it.
    if(!(state & (STATE_ESTOP|STATE_ALARM))){
        if(!item_count)
            enqueue_message(keepalive_msg); // Otherwise send keepalive to feed pico watchdog

        picomodbus_send();
    }
    task_add_delayed(picomodbus_poll, NULL, PICO_POLL_INTERVAL);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("PicoModbus", "0.01");
}

static void OnReset (void)
{
    flush_queue();
    driver_reset();
}

void picomodbus_init (void) {

    uint_fast8_t idx;

    hal.enumerate_pins(false, get_aux_max, NULL);

    digital.out.n_ports = sizeof(aux_dout) / sizeof(picomodbus_aux_t);

    for(idx = 0; idx < digital.out.n_ports; idx ++) {
        aux_dout[idx].aux.id = idx;
        aux_dout[idx].aux.function = aux_dout_base + idx;
}

    io_digital_t dports = {
        .ports = &digital,
        .digital_out = digital_out,
        .get_pin_info = d_get_pin_info,
        .set_pin_description = d_set_pin_description,
    };

    ioports_add_digital(&dports);

    analog.out.n_ports = sizeof(aux_aout) / sizeof(picomodbus_aux_t);

    for(idx = 0; idx < analog.out.n_ports; idx ++) {
        aux_aout[idx].aux.id = idx;
        aux_aout[idx].aux.function = aux_aout_base + idx;
    }

    io_analog_t aports = {
        .ports = &analog,
        .analog_out = analog_out,
        .get_pin_info = a_get_pin_info,
        .set_pin_description = a_set_pin_description,
    };

    ioports_add_analog(&aports);

    on_enumerate_pins = hal.enumerate_pins;
    hal.enumerate_pins = onEnumeratePins;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    driver_reset = hal.driver_reset;
    hal.driver_reset = OnReset;

    task_add_delayed(picomodbus_poll, NULL, PICO_POLL_INTERVAL);
}

#endif // PICOMODBUS_ENABLE