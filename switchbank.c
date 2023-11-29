/*

  switchbank.c - plugin for binding functions to aux output pins

  Part of grblHAL

  Public domain.

  Tip: use the $pins command to check the port mapping.
*/

#include <stdio.h>
#include <string.h>

#include "grbl/hal.h"
#include "grbl/nvs_buffer.h"
#include "grbl/nuts_bolts.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/platform.h"

#ifndef GRBL_BUILD
#error "grblHAL build 20211117 or later is required for this plugin!"
#endif

#ifndef N_SWITCHBANK
#define N_SWITCHBANK 4
#endif

//the index of this array must match the radio button descriptions
typedef enum {  MCODE, 
                SPINDLE_ACTIVE, 
                COOLANT_MIST_ACTIVE, 
                COOLANT_FLOOD_ACTIVE, 
} aux_function_t;

typedef struct {
    aux_function_t function[N_SWITCHBANK];
} switchbank_settings_t;

static bool can_map_ports = false;
static uint8_t n_ports;
uint8_t port[N_SWITCHBANK];
static char max_port[4];
static nvs_address_t nvs_address;
static on_report_options_ptr on_report_options;
static switchbank_settings_t plugin_settings;
static driver_reset_ptr driver_reset;

static on_execute_realtime_ptr on_execute_realtime, on_execute_delay;
static on_override_changed_ptr on_override_changed;
static on_spindle_programmed_ptr on_spindle_programmed;
static on_unknown_accessory_override_ptr on_unknown_accessory_override;
static coolant_set_state_ptr coolant_set_state_;

static uint32_t polling_ms = 0;

// Called on a soft reset so that normal operation can be restored.
static void plugin_reset (void)
{
    driver_reset(); // Call the next reset handler in the chain.
}

static void poll_buttons (void){
    
    uint32_t ms = hal.get_elapsed_ticks();
    if(ms < polling_ms + 50)
        return;

    polling_ms = ms;    
}

static void button_poll_realtime (sys_state_t grbl_state)
{
    on_execute_realtime(grbl_state);

    //poll_buttons();
}

static void button_poll_delay (sys_state_t grbl_state)
{
    on_execute_delay(grbl_state);

    //poll_buttons();
}

static const setting_group_detail_t macro_groups [] = {
    { Group_Root, Group_AuxPorts, "Aux ports"}
};

static const setting_detail_t switchbank_settings[] = {
    { Setting_UserDefined_6, Group_AuxPorts, "Aux Output 0 Function", NULL, Format_RadioButtons,  "M62-M65,Spindle/Laser Enable(M3/M4),Flood Enable (M7),Mist Enable(M8)", NULL, NULL, Setting_NonCore, &plugin_settings.function[0], NULL, NULL },
    { Setting_UserDefined_7, Group_AuxPorts, "Aux Output 1 Function", NULL, Format_RadioButtons,  "M62-M65,Spindle/Laser Enable(M3/M4),Flood Enable (M7),Mist Enable(M8)", NULL, NULL, Setting_NonCore, &plugin_settings.function[1], NULL, NULL },
    { Setting_UserDefined_8, Group_AuxPorts, "Aux Output 2 Function", NULL, Format_RadioButtons,  "M62-M65,Spindle/Laser Enable(M3/M4),Flood Enable (M7),Mist Enable(M8)", NULL, NULL, Setting_NonCore, &plugin_settings.function[2], NULL, NULL },
    { Setting_UserDefined_9, Group_AuxPorts, "Aux Output 3 Function", NULL, Format_RadioButtons,  "M62-M65,Spindle/Laser Enable(M3/M4),Flood Enable (M7),Mist Enable(M8)", NULL, NULL, Setting_NonCore, &plugin_settings.function[3], NULL, NULL },
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t switchbank_settings_descr[] = {
    { Setting_UserDefined_6, "Connect Switchbank pin to this action" },
    { Setting_UserDefined_7, "Connect Switchbank pin to this action" },
    { Setting_UserDefined_8, "Connect Switchbank pin to this action" },
    { Setting_UserDefined_9, "Connect Switchbank pin to this action"  },

};

#endif

// Write settings to non volatile storage (NVS).
static void switchbank_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(switchbank_settings_t), true);
}

// Restore default settings and write to non volatile storage (NVS).
static void switchbank_settings_restore (void)
{
    uint_fast8_t idx = N_SWITCHBANK;

    // Register default functions.
    for(idx = 0; idx < N_SWITCHBANK; idx++) {
            plugin_settings.function[idx] = MCODE;
    };

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(switchbank_settings_t), true);
}

static void no_ports (uint_fast16_t state)
{
    report_message("Switchbankk plugin failed to claim all needed ports!", Message_Warning);
}

//*****Switchbank will always claim the first 4 aux outputs******
static void switchbank_settings_load (void)
{
    uint_fast8_t idx = N_SWITCHBANK;

    if(hal.nvs.memcpy_from_nvs((uint8_t *)&plugin_settings, nvs_address, sizeof(switchbank_settings_t), true) != NVS_TransferResult_OK)
        switchbank_settings_restore();

    // If port mapping is available try to claim ports as configured.
    if(can_map_ports && (n_ports >= N_SWITCHBANK)) {

        //just claim them explicitly so that we can have unique names.
        idx--;
        port[idx] = idx;          
        if(!ioport_claim(Port_Digital, Port_Output, &port[idx], "SwitchBank 3 pin"))   // Try to claim the port.
            port[idx] = 0xFF;                                                       // If not successful flag it as not claimed.

        idx--;
        port[idx] = idx;          
        if(!ioport_claim(Port_Digital, Port_Output, &port[idx], "SwitchBank 2 pin"))   // Try to claim the port.
            port[idx] = 0xFF;     

        idx--;
        port[idx] = idx;          
        if(!ioport_claim(Port_Digital, Port_Output, &port[idx], "SwitchBank 1 pin"))   // Try to claim the port.
            port[idx] = 0xFF;     

        idx--;
        port[idx] = idx;          
        if(!ioport_claim(Port_Digital, Port_Output, &port[idx], "SwitchBank 0 pin"))   // Try to claim the port.
            port[idx] = 0xFF;                                            
    }  else{
        protocol_enqueue_rt_command(no_ports);
    }
}

// Settings descriptor used by the core when interacting with this plugin.
static setting_details_t setting_details = {
    .groups = macro_groups,
    .n_groups = sizeof(macro_groups) / sizeof(setting_group_detail_t),
    .settings = switchbank_settings,
    .n_settings = sizeof(switchbank_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = switchbank_settings_descr,
    .n_descriptions = sizeof(switchbank_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = switchbank_settings_save,
    .load = switchbank_settings_load,
    .restore = switchbank_settings_restore
};

static void onSpindleProgrammed(spindle_ptrs_t *spindle, spindle_state_t state, float rpm, spindle_rpm_mode_t mode){

    if(on_spindle_programmed)
        on_spindle_programmed(spindle, state, rpm, mode);

    uint_fast8_t idx = N_SWITCHBANK;

    while(idx){
        idx--;
        if(plugin_settings.function[idx] == SPINDLE_ACTIVE)
            hal.port.digital_out(port[idx], state.on);
    }
}

static void onOverrideChanged(override_changed_t override){

    if(on_override_changed)
        on_override_changed(override);    

}

static void onAccessoryOverride(uint8_t cmd){

    if(on_unknown_accessory_override)
        on_unknown_accessory_override(cmd);       

}

static void onCoolantSetState (coolant_state_t state)
{
    coolant_set_state_(state);

    uint_fast8_t idx = N_SWITCHBANK;        

    while(idx){
        idx--;
        if(plugin_settings.function[idx] == COOLANT_MIST_ACTIVE)
            hal.port.digital_out(port[idx], state.mist);
    }

    idx = N_SWITCHBANK;
    while(idx){
        idx--;
        if(plugin_settings.function[idx] == COOLANT_FLOOD_ACTIVE)
            hal.port.digital_out(port[idx], state.flood);
    }      

}

// Add info about our plugin to the $I report.
static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:SwitchBank plugin v0.01]" ASCII_EOL);
}

static void warning_msg (uint_fast16_t state)
{
    report_message("SwitchBank plugin failed to initialize!", Message_Warning);
}

void switchbank_init (void)
{
    bool ok = (n_ports = ioports_available(Port_Digital, Port_Output)) > N_SWITCHBANK;

    if(ok && !(can_map_ports = ioport_can_claim_explicit())) {

        // Driver does not support explicit pin claiming, claim the highest numbered ports instead.

        uint_fast8_t idx = N_SWITCHBANK;

        do {
            idx--;
            if(!(ok = ioport_claim(Port_Digital, Port_Output, &port[idx], "Switchbank pin")))
                port[idx] = 0xFF;
        } while(idx);
    }

    // If enough free non volatile memory register our plugin with the core.
    if(ok && (nvs_address = nvs_alloc(sizeof(switchbank_settings_t)))) {

        // Register settings.
        settings_register(&setting_details);

        // Used for setting value validation.
        strcpy(max_port, uitoa(n_ports - 1));

        // Add our plugin to the $I options report.
        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        //add polling of button state to realtime and delay chains
        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = button_poll_realtime;

        on_execute_delay = grbl.on_execute_delay;
        grbl.on_execute_delay = button_poll_delay;    

        //on_override_changed = grbl.on_override_changed;             // Subscribe to the event by saving away the original
        //grbl.on_override_changed = onOverrideChanged;              // function pointer and adding ours to the chain.     

        on_spindle_programmed = grbl.on_spindle_programmed;             // Subscribe to the event by saving away the original
        grbl.on_spindle_programmed = onSpindleProgrammed;              // function pointer and adding ours to the chain.     

        //on_unknown_accessory_override = grbl.on_unknown_accessory_override;             // Subscribe to the event by saving away the original
        //grbl.on_unknown_accessory_override = onAccessoryOverride;              // function pointer and adding ours to the chain.

        coolant_set_state_ = hal.coolant.set_state;
        hal.coolant.set_state = onCoolantSetState;                                

        // Hook into the driver reset chain so we
        // can restore normal operation if a reset happens
        // when a macro is running.
        driver_reset = hal.driver_reset;
        hal.driver_reset = plugin_reset;
    } else
        protocol_enqueue_rt_command(warning_msg);
}