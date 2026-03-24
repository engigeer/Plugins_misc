/*

  probe_relays.c - controls relay(s) for switching between probes using a single probe input

  Use G65P5Q<n> to select probe where <n> = 0 is for direct input, <n> = 1 is for toolsetter and <n> = 2 is for second spindle probe.

  Part of grblHAL

  Copyright (c) 2024-2025 Terje Io

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

#if PROBE_ENABLE == 2 && NGC_PARAMETERS_ENABLE

#include "grbl/hal.h"
#include "grbl/task.h"
#include "grbl/nvs_buffer.h"
#include "grbl/pin_bits_masks.h"

static on_report_options_ptr on_report_options;
static probe_select_ptr hal_probe_select;

xbar_t *probe_pins[2];

bool onProbeSelect (probe_id_t probe_id)
{
    bool ok;

    switch(probe_id) {

            case Probe_Default:
                probe_pin[0]->config(); // somehow communicate this back to expander
            break;    

        case Probe_Toolsetter:
                probe_pin[1]->config();//toolsetter_pin.pin = true; // somehow communicate this back to expander
            break;

        default: break;
    }

    return ok;
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Probe select expander", "0.01");
}

aux_ctrl_t probe_pin = { .function = Input_Probe, .port = IOPORT_UNASSIGNED, .gpio.pin = 4 };
aux_ctrl_t toolsetter_pin = { .function = Input_Toolsetter, .port = IOPORT_UNASSIGNED, .gpio.pin = 3 };

void probe_select_init (void)
{
    bool ok = true;
    xbar_t *pin;

    if((ioports_enumerate(Port_Digital, Port_Input, (pin_cap_t){.external = On, .claimable = On }, __find_in_ext, &probe_pin))
            && (probe_pins[0] = aux_ctrl_claim_port(&probe_pin)))
        ioport_set_description(Port_Digital, Port_Input, probe_pin.port, "expander multiplex");
    else 
        ok = false;

    if((ioports_enumerate(Port_Digital, Port_Input, (pin_cap_t){.external = On, .claimable = On }, __find_in_ext, &toolsetter_pin))
            && (probe_pins[1] = aux_ctrl_claim_port(&toolsetter_pin)))
        ioport_set_description(Port_Digital, Port_Input, toolsetter_pin.port, "expander multiplex");
    else 
        ok = false;
    
    if(ok && (hal.driver_cap.toolsetter = On)) {
        hal_probe_select = hal.probe.select;
        hal.probe.select = onProbeSelect;
    } else
        task_run_on_startup(report_warning, "Probe expander plugin: error claiming required ports.");

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;
}

#endif // PROBE_ENABLE == 2 && NGC_PARAMETERS_ENABLE
