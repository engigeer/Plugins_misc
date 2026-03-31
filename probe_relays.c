/*

  probe_relays.c - modified for expander probe multiplexing
  Part of grblHAL

  Copyright (c) 2024-2025 Terje Io
  Copyright (c) 2026 Mitchell Grams

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
static probe_select_ptr probe_select;
static probe_configure_ptr probe_configure;

xbar_t *probe_pins[2];
probe_id_t active_probe = Probe_Default;

aux_ctrl_t probe_pin = { .function = Input_Probe, .port = IOPORT_UNASSIGNED, .gpio.pin = 4 };
aux_ctrl_t toolsetter_pin = { .function = Input_Toolsetter, .port = IOPORT_UNASSIGNED, .gpio.pin = 3 };

bool onProbeSelect (probe_id_t probe_id)
{
    bool ok = false;

    switch(probe_id) {

        case Probe_Default:
                //probe_pin[0]->config(); // somehow communicate this back to expander
                ok = true;
                active_probe = probe_id;
                hal.probe.configure(false, false);
            break;    

        case Probe_Toolsetter:
                //probe_pin[1]->config();//toolsetter_pin.pin = true; // somehow communicate this back to expander
                ok = true;
                active_probe = probe_id;
                hal.probe.configure(false, false);
            break;

        default: break;
    }

    return ok;
}

static void probeConfigure (bool is_probe_away, bool probing)
{
    if (active_probe == Probe_Toolsetter) {
        if (settings.probe.invert_toolsetter_input != settings.probe.invert_probe_pin)
            is_probe_away = !is_probe_away;
    }
    probe_configure(is_probe_away, probing);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Probe select expander", "0.01");
}

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
        probe_select = hal.probe.select;
        hal.probe.select = onProbeSelect;

        probe_configure = hal.probe.configure;
        hal.probe.configure = probeConfigure;

    } else
        task_run_on_startup(report_warning, "Probe expander plugin: error claiming required ports.");

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;
}

#endif // PROBE_ENABLE == 2 && NGC_PARAMETERS_ENABLE
