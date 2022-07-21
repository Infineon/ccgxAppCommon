/***************************************************************************//**
* \file pdo.c
* \version 1.1.0 
*
* PDO evaluation and handler functions 
*
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "config.h"
#include "cy_pdstack_common.h"
#include "cy_pdstack_dpm.h"
#include "cy_pdstack_utils.h"
#include "pdo.h"
#include "app.h"
#include "srom.h"

#if CCG_HPI_ENABLE
#include <hpi.h>
#endif /* (CCG_HPI_ENABLE) */

#if APP_PPS_SINK_SUPPORT
static pd_do_t app_prog_rdo[NO_OF_TYPEC_PORTS];
static bool    app_pps_snk_en[NO_OF_TYPEC_PORTS];
#endif /* APP_PPS_SINK_SUPPORT */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
#include "power_throttle.h"
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */


#if (!(CY_PD_SOURCE_ONLY))

#if (!SROM_CODE_APP_PDO)
/* PDO Variables. */
uint32_t gl_max_min_cur_pwr[NO_OF_TYPEC_PORTS];
uint32_t gl_contract_power[NO_OF_TYPEC_PORTS];
uint32_t gl_contract_voltage[NO_OF_TYPEC_PORTS];
uint32_t gl_op_cur_power[NO_OF_TYPEC_PORTS];
#endif /* (!SROM_CODE_APP_PDO) */

ATTRIBUTES_APP_PDO static uint32_t calc_power(uint32_t voltage, uint32_t current)
{
    /*
       Voltage is expressed in 50 mV units.
       Current is expressed in 10 mA units.
       Power should be expressed in 250 mW units.
       */
    return (CALL_MAP(div_round_up)(voltage * current, 500));
}

#if !DISABLE_PDO_BATTERY && !PDO_SNK_BATTERY_SUPP_DISABLE
ATTRIBUTES_APP_PDO static uint32_t calc_current(uint32_t power, uint32_t voltage)
{
    /*
       Power is expressed in 250 mW units.
       Voltage is expressed in 50 mV units.
       Current should be expressed in 10 mA units.
       */
    return (CALL_MAP(div_round_up)(power * 500, voltage));
}
#endif /* !DISABLE_PDO_BATTERY && PDO_SNK_BATTERY_SUPP */

#if CCG_UCSI_ENABLE
#if UCSI_SET_PWR_LEVEL_ENABLE
#define PWR_LVL_4P5W                    (9)         /* 4.5W in 0.5W units */
#define PWR_LVL_4P0W                    (8)         /* 4W in 0.5W units */
#define MIN_CURRENT                     (90)        /* 900mA in 10mA units */

/**
 * @brief Edit the advertised PDOs to match the OS's requirements
 *
 * @param port The port whose PDOs need editing
 * @param power Final power level that the OS wants CCG to get to
 * @param is_src Set to '1' if the Source Caps need editing. For Sink Caps, its set to '0'
 * @param &out_mask An output variable indicating a bitmask of enabled PDOs
 *
 * @return
 *    NULL If CCG should fail the command by setting the Error Indicator
 *    pd_do_t[7] A 7-element PD data object array of edited PDOs
 */
pd_do_t* ucsi_change_pdo_power(uint8_t port, uint8_t power, uint8_t is_src, uint8_t *out_mask)
{
    uint8_t i, pdo_cnt;
    uint32_t cur_power;
    const pd_do_t* cur_pdo;
    static pd_do_t pdos[MAX_NO_OF_PDO];

    /* Always enable 5V */
    *out_mask = 1;

    /* Take the number of PDOs and the list from the configuration table */
    if(is_src == 1)
    {
        pdo_cnt = get_pd_port_config(port)->src_pdo_cnt;
        cur_pdo = (pd_do_t*)get_pd_port_config(port)->src_pdo_list;
    }
    else
    {
        pdo_cnt = get_pd_port_config(port)->snk_pdo_cnt;
        cur_pdo = (pd_do_t*)get_pd_port_config(port)->snk_pdo_list;
    }

    for(i = 0; i < pdo_cnt; i++)
    {
        pdos[i].val = cur_pdo[i].val;

        /* If the OPM sets PD power level to 0 or 255, we revert to the default configuration */
        if((power == 0) || (power == 0xFF))
            *out_mask |= (1 << i);
        else
        {
            switch(pdos[i].fixed_src.supply_type)
            {
                case PDO_FIXED_SUPPLY:
                    cur_power = calc_power(pdos[i].fixed_src.voltage, pdos[i].fixed_src.max_current);
                    /* Force the max current to be within 900mA and 3A */
                    pdos[i].fixed_src.max_current = CY_USBPD_GET_MAX(MIN_CURRENT, CY_USBPD_GET_MIN((2 * power * pdos[i].fixed_src.max_current) / cur_power, CBL_CAP_3A));

                    /* Enable other PDOs only if requested power is >4.5W */
                    if(power > PWR_LVL_4P5W)
                        *out_mask |= (1 << i);
                    break;

                case PDO_VARIABLE_SUPPLY:
                    cur_power = calc_power(pdos[i].var_src.max_voltage, pdos[i].var_src.max_current);
                    pdos[i].var_src.max_current = CY_USBPD_GET_MAX(MIN_CURRENT, CY_USBPD_GET_MIN((2 * power * pdos[i].var_src.max_current) / cur_power, CBL_CAP_3A));
                    *out_mask |= (1 << i);
                    break;

                case PDO_BATTERY:
                    pdos[i].bat_src.max_power = power * 2;
                    *out_mask |= (1 << i);
                    break;
            }
        }
    }

    return pdos;
}
#endif /* UCSI_SET_PWR_LEVEL_ENABLE */
#endif /* CCG_UCSI_ENABLE */

/**
 * Checks if SRC pdo is acceptable for SNK pdo.
 * @param context PD Stack context
 * @param pdo_src pointer to current SRC PDO
 * @param snk_pdo_idx SNK PDO index
 * @return True if current src pdo is acceptable for current snk pdo
 */
ATTRIBUTES_APP_PDO static bool is_src_acceptable_snk(cy_stc_pdstack_context_t* context, cy_pd_pd_do_t* pdo_src, uint8_t snk_pdo_idx)
{
    cy_pd_pd_do_t* pdo_snk = (cy_pd_pd_do_t*)&(context->dpmStat.curSnkPdo[snk_pdo_idx]);
    uint8_t port = context->port;

    uint32_t snk_supply_type = pdo_snk->fixed_snk.supplyType;
    uint32_t fix_volt;
    uint32_t maxVolt;
    uint32_t minVolt;
    uint32_t out = false;
    uint32_t max_min_temp, compare_temp;
#if !DISABLE_PDO_BATTERY    
    uint32_t oper_cur_pwr;
#endif /* DISABLE_PDO_BATTERY */

    max_min_temp = context->dpmStat.curSnkMaxMin[snk_pdo_idx] & CY_PD_SNK_MIN_MAX_MASK;

    switch(pdo_src->fixed_src.supplyType)
    {
        case CY_PDSTACK_PDO_FIXED_SUPPLY:  /* Fixed supply PDO */
            fix_volt = pdo_src->fixed_src.voltage;

            maxVolt = div_round_up(fix_volt, 20);
            minVolt = fix_volt - maxVolt;
            maxVolt = fix_volt + maxVolt;

            switch(snk_supply_type)  /* Checking sink PDO type */
            {
                case CY_PDSTACK_PDO_FIXED_SUPPLY:
                    if(fix_volt == pdo_snk->fixed_snk.voltage)
                    {
                        compare_temp = CY_USBPD_GET_MAX (max_min_temp, pdo_snk->fixed_snk.opCurrent);
                        if (pdo_src->fixed_src.maxCurrent >= compare_temp)
                        {
#if (!POWER_BANK)
                            gl_op_cur_power[port] = pdo_snk->fixed_snk.opCurrent;
#else /* (POWER_BANK) */
                            gl_op_cur_power[port] = pdo_src->fixed_src.maxCurrent;
#endif /* (POWER_BANK) */
                            out = true;
                        }
                    }
                    break;

                case CY_PDSTACK_PDO_VARIABLE_SUPPLY:
                    if ((minVolt >= pdo_snk->var_snk.minVoltage) && (maxVolt <= pdo_snk->var_snk.maxVoltage))
                    {
                        compare_temp = CY_USBPD_GET_MAX (max_min_temp, pdo_snk->var_snk.opCurrent);
                        if (pdo_src->fixed_src.maxCurrent >= compare_temp)
                        {
#if (!POWER_BANK)
                            gl_op_cur_power[port] = pdo_snk->var_snk.opCurrent;
#else /* (POWER_BANK) */
                            gl_op_cur_power[port] = pdo_src->fixed_src.maxCurrent;
#endif /* (POWER_BANK) */
                            out = true;
                        }
                    }
                    break;

#if !DISABLE_PDO_BATTERY && !PDO_SNK_BATTERY_SUPP_DISABLE
                case CY_PDSTACK_PDO_BATTERY:
                    if ((minVolt >= pdo_snk->bat_snk.minVoltage) && (maxVolt <= pdo_snk->bat_snk.maxVoltage))
                    {
                        fix_volt = minVolt;

                        /* Calculate the operating current and min/max current values. */
                        oper_cur_pwr = calc_current(pdo_snk->bat_snk.opPower, minVolt);
                        /* Calculate the operating current and min/max current values. */
                        /* Intel: Always set operating current to 27W/fixed_pdo_voltage and max op. current to PDO voltage */
                        max_min_temp = calc_current(max_min_temp, minVolt);

                        /* Make sure the source can supply the maximum current that may be required. */
                        compare_temp = CY_USBPD_GET_MAX(max_min_temp, oper_cur_pwr);
                        if (pdo_src->fixed_src.maxCurrent >= compare_temp)
                        {
                            gl_op_cur_power[port] = oper_cur_pwr;
                            out = true;
                        }
                    }
                    break;
#endif /* DISABLE_PDO_BATTERY */
                default:
                    break;
            }

            if (out)
            {
                gl_contract_voltage[port] = fix_volt;
                gl_contract_power[port]   = calc_power (fix_volt, gl_op_cur_power[port]);
                gl_max_min_cur_pwr[port]  = max_min_temp;
            }
            break;

#if !DISABLE_PDO_BATTERY
        case CY_PDSTACK_PDO_BATTERY:   /* SRC is a battery */
            maxVolt = pdo_src->bat_src.maxVoltage;
            minVolt = pdo_src->bat_src.minVoltage;

            switch(snk_supply_type)
            {
                case CY_PDSTACK_PDO_FIXED_SUPPLY:
                    /* Battery cannot supply fixed voltage
                     * Battery voltage changes with time
                     * This contract if permitted can be un-reliable */
                    break;

                case CY_PDSTACK_PDO_VARIABLE_SUPPLY:
                    if((minVolt >= pdo_snk->var_snk.minVoltage) && (maxVolt <= pdo_snk->var_snk.maxVoltage))
                    {
                        /* Calculate the expected operating power and maximum power requirement. */
                        oper_cur_pwr = calc_power(maxVolt, pdo_snk->var_snk.opCurrent);
                        max_min_temp = calc_power(maxVolt, max_min_temp);

                        compare_temp = CY_USBPD_GET_MAX (oper_cur_pwr, max_min_temp);
                        if (pdo_src->bat_src.maxPower >= compare_temp)
                        {
#if (!POWER_BANK)
                            gl_op_cur_power[port] = oper_cur_pwr;
#else /* (POWER_BANK) */
                            gl_op_cur_power[port] = pdo_src->bat_src.maxPower;
#endif /* (POWER_BANK) */
                            out = true;
                        }
                    }
                    break;

#if !PDO_SNK_BATTERY_SUPP_DISABLE
                case CY_PDSTACK_PDO_BATTERY:
                    /* Battery connected directly to a battery
                     * This combination is unreliable */
                    if((minVolt >= pdo_snk->bat_snk.minVoltage) && (maxVolt <= pdo_snk->bat_snk.maxVoltage))
                    {
                        compare_temp = CY_USBPD_GET_MAX (max_min_temp, pdo_snk->bat_snk.opPower);
                        if (pdo_src->bat_src.maxPower >= compare_temp)
                        {
                            gl_op_cur_power[port] = pdo_snk->bat_snk.opPower;
                            out = true;
                        }
                    }
                    break;
#endif /* PDO_SNK_BATTERY_SUPP */
                default:
                    break;
            }

            if (out)
            {
                gl_contract_voltage[port] = maxVolt;
                gl_max_min_cur_pwr[port]  = max_min_temp;
                gl_contract_power[port]   = gl_op_cur_power[port];
            }
            break;
#endif /* DISABLE_PDO_BATTERY */

        case CY_PDSTACK_PDO_VARIABLE_SUPPLY:   /* Variable supply PDO */
            maxVolt = pdo_src->var_src.maxVoltage;
            minVolt = pdo_src->var_src.minVoltage;

            switch (snk_supply_type) /* Checking sink PDO type */
            {
                case CY_PDSTACK_PDO_FIXED_SUPPLY:
                    /* This connection is not feasible
                     * A variable source cannot provide a fixed voltage */
                    break;

                case CY_PDSTACK_PDO_VARIABLE_SUPPLY:
                    if((minVolt >= pdo_snk->var_snk.minVoltage) && (maxVolt <= pdo_snk->var_snk.maxVoltage))
                    {
                        compare_temp = CY_USBPD_GET_MAX (pdo_snk->var_snk.opCurrent, max_min_temp);

                        if (pdo_src->var_src.maxCurrent >= compare_temp)
                        {
#if (!POWER_BANK)
                            gl_contract_power[port] = calc_power(minVolt, pdo_snk->var_snk.opCurrent);
                            gl_op_cur_power[port]   = pdo_snk->var_snk.opCurrent;
#else /* (POWER_BANK) */
                            gl_contract_power[port] = calc_power(minVolt, pdo_src->var_src.maxCurrent);
                            gl_op_cur_power[port]   = pdo_src->var_src.maxCurrent;
#endif /* (POWER_BANK) */
                            out = true;
                        }
                    }
                    break;

#if !DISABLE_PDO_BATTERY && !PDO_SNK_BATTERY_SUPP_DISABLE
                case CY_PDSTACK_PDO_BATTERY:
                    if((minVolt >= pdo_snk->bat_snk.minVoltage) && (maxVolt <= pdo_snk->bat_snk.maxVoltage))
                    {
                        /* Convert from power to current. */
                        oper_cur_pwr = calc_current(pdo_snk->bat_snk.opPower, minVolt);
                        max_min_temp = calc_current(max_min_temp, minVolt);

                        compare_temp = CY_USBPD_GET_MAX(oper_cur_pwr, max_min_temp);
                        if (pdo_src->var_src.maxCurrent >= compare_temp)
                        {
                            gl_contract_power[port] = pdo_snk->bat_snk.opPower;
                            gl_op_cur_power[port]   = oper_cur_pwr;
                            out = true;
                        }
                    }
                    break;
#endif /* !DISABLE_PDO_BATTERY && PDO_SNK_BATTERY_SUPP */
                default:
                    break;
            }

            if (out)
            {
                gl_contract_voltage[port] = maxVolt;
                gl_max_min_cur_pwr[port]  = max_min_temp;
            }
            break;

        default:
            break;
    }

    return out;
}

ATTRIBUTES_APP_PDO static cy_pd_pd_do_t form_rdo(cy_stc_pdstack_context_t* context, uint8_t pdo_no, bool capMisMatch, bool giveBack)
{
#if (CY_PD_REV3_ENABLE)
    const cy_stc_pd_dpm_config_t* dpm = &(context->dpmConfig);
#endif /* CY_PD_REV3_ENABLE */
    cy_pd_pd_do_t snkRdo;
    uint8_t port = context->port;

    snkRdo.val = 0u;
    snkRdo.rdo_gen.noUsbSuspend = context->dpmStat.snkUsbSuspEn;
    snkRdo.rdo_gen.usbCommCap = context->dpmStat.snkUsbCommEn;
    snkRdo.rdo_gen.capMismatch = capMisMatch;
    snkRdo.rdo_gen.objPos = pdo_no;
    snkRdo.rdo_gen.giveBackFlag = (capMisMatch) ? false : giveBack;
    snkRdo.rdo_gen.opPowerCur = gl_op_cur_power[port];
    snkRdo.rdo_gen.minMaxPowerCur = gl_max_min_cur_pwr[port];

    if (
            (snkRdo.rdo_gen.giveBackFlag == false) &&
            (snkRdo.rdo_gen.opPowerCur > snkRdo.rdo_gen.minMaxPowerCur)
       )
    {
        snkRdo.rdo_gen.minMaxPowerCur = snkRdo.rdo_gen.opPowerCur;
    }

#if (CCG6_SROM_CODE_ENABLE || CY_PD_REV3_ENABLE)
    if(dpm->specRevSopLive >= CY_PD_REV3)
    {
        snkRdo.rdo_gen.unchunkSup = true;
    }
#endif /* (CCG6_SROM_CODE_ENABLE | CY_PD_REV3_ENABLE) */

    return snkRdo;
}

#if APP_PPS_SINK_SUPPORT
static pd_do_t handle_pps_sink_rdo(uint8_t port, const pd_packet_t *srcCap, pd_do_t dflt_rdo)
{
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    pd_do_t pdo;
    pd_do_t rdo = dflt_rdo;

    if (
            (app_pps_snk_en[port]) &&
            (srcCap->len >= app_prog_rdo[port].rdo_gen.objPos)
       )
    {
        /* Perform sanity checks on validity of the RDO. */
        pdo = srcCap->dat[app_prog_rdo[port].rdo_gen.objPos - 1];

        if (
                (pdo.pps_src.supplyType == PDO_AUGMENTED) &&
                (pdo.pps_src.apdoType == APDO_PPS) &&
                ((pdo.pps_src.maxVolt * 5) >= app_prog_rdo[port].rdo_pps.outVolt) &&
                ((pdo.pps_src.minVolt * 5) <= app_prog_rdo[port].rdo_pps.outVolt) &&
                (pdo.pps_src.maxCur >= app_prog_rdo[port].rdo_pps.opCur)
           )
        {
            rdo.val                    = app_prog_rdo[port].val;
            rdo.rdo_gen.noUsbSuspend = dpm_stat->snkUsbSuspEn;
            rdo.rdo_gen.usbCommCap   = dpm_stat->snkUsbCommEn;
            rdo.rdo_gen.unchunkSup    = true;
        }
        else
        {
            app_pps_snk_en[port]   = false;
            app_prog_rdo[port].val = 0;
        }
    }

    return (rdo);
}
#endif /* APP_PPS_SINK_SUPPORT */

/*
 * This function can be used to ask EC to evaluate a src cap message
 * For now evaluating here and executing the callback in this function itself
 */
ATTRIBUTES_APP_PDO void eval_src_cap(cy_stc_pdstack_context_t* context, const cy_stc_pdstack_pd_packet_t* srcCap, cy_pdstack_app_resp_cbk_t app_resp_handler)
{
    uint8_t src_pdo_index, snk_pdo_index;
    cy_pd_pd_do_t* snkPdo = (cy_pd_pd_do_t*)&context->dpmStat.curSnkPdo[0];
    uint8_t port = context->port;

    uint16_t src_vsafe5_cur = srcCap->dat[0].fixed_src.maxCurrent; /* Source max current for first PDO */
    cy_pd_pd_do_t snkRdo;
    uint32_t highest_gl_contract_power = 0u;
    bool match = false;

    for(snk_pdo_index = 0u; snk_pdo_index < context->dpmStat.curSnkPdocount; snk_pdo_index++)
    {
        for(src_pdo_index = 0u; src_pdo_index < srcCap->len; src_pdo_index++)
        {
            if(is_src_acceptable_snk(context, (cy_pd_pd_do_t*)(&srcCap->dat[src_pdo_index]), snk_pdo_index))
            {
                bool max_cond = false; 

                switch(PD_PDO_SEL_ALGO)
                {
                    case CY_PDSTACK_HIGHEST_POWER:
                        /* Contract_power is based on SRC PDO */
                        if (srcCap->dat[src_pdo_index].fixed_src.supplyType == CY_PDSTACK_PDO_FIXED_SUPPLY)
                        {
                            uint32_t temp_power = calc_power(srcCap->dat[src_pdo_index].fixed_src.voltage,
                                    srcCap->dat[src_pdo_index].fixed_src.maxCurrent);
                            if (temp_power >= highest_gl_contract_power)
                            {
                                highest_gl_contract_power = temp_power;
                                max_cond = true;
                            }
                        }
                        break;

                    case CY_PDSTACK_HIGHEST_VOLTAGE:
                        /* Only fixed pdo takes part */
                        if ((srcCap->dat[src_pdo_index].fixed_src.supplyType == CY_PDSTACK_PDO_FIXED_SUPPLY)
                                && (gl_contract_voltage[port] >= highest_gl_contract_power))
                        {
                            highest_gl_contract_power = gl_contract_voltage[port];
                            max_cond = true;
                        }
                        break;

                    case CY_PDSTACK_HIGHEST_CURRENT:
                        /* Only fixed pdo takes part */
                        if ((srcCap->dat[src_pdo_index].fixed_src.supplyType == CY_PDSTACK_PDO_FIXED_SUPPLY)
                                && (srcCap->dat[src_pdo_index].fixed_src.maxCurrent >= highest_gl_contract_power))
                        {
                            highest_gl_contract_power = srcCap->dat[src_pdo_index].fixed_src.maxCurrent;
                            max_cond = true;
                        }
                        break;

                    default:
                        /* Contract_power is calculated in is_src_acceptable_snk() */
                        if (gl_contract_power[port] >= highest_gl_contract_power)
                        {
                            highest_gl_contract_power = gl_contract_power[port];
                            max_cond = true;
                        }
                        break;
                }

                if (max_cond)
                {
                    /* Check if sink needs higher capability */
                    if ((snkPdo[0].fixed_snk.highCap) && (gl_contract_voltage[port] == (CY_PD_VSAFE_5V/CY_PD_VOLT_PER_UNIT)))
                    {
                        /* 5V contract isn't acceptable with highCap = 1 */
                        continue;
                    }

                    snkRdo = form_rdo(context, (src_pdo_index + 1u), false,
                            (context->dpmStat.curSnkMaxMin[snk_pdo_index] & CY_PD_GIVE_BACK_MASK));
                    match = true;

#if APP_PPS_SINK_SUPPORT
                    /* Update to PPS RDO if so required by user. */
                    snkRdo = handle_pps_sink_rdo (port, srcCap, snkRdo);
#endif /* APP_PPS_SINK_SUPPORT */
                }
            }
        }
    }

    if(match == false)
    {
        /* Capability mismatch: Ask for vsafe5v PDO with CapMismatch */
        gl_contract_voltage[port] = snkPdo[0].fixed_snk.voltage;
        gl_op_cur_power[port] = snkPdo[0].fixed_snk.opCurrent;
        gl_contract_power[port] = div_round_up(
                gl_contract_voltage[port] * gl_op_cur_power[port], 500u);

        if(src_vsafe5_cur < gl_op_cur_power[port])
        {
            /* SNK operation current can't be bigger than SRC maxCurrent */
            gl_op_cur_power[port] = src_vsafe5_cur;
        }

        gl_max_min_cur_pwr[port] = context->dpmStat.curSnkMaxMin[0];
        snkRdo = form_rdo(context, 1u, true, false);
    }

    /* Config table setting to always request max current instead of operation current */
#if defined(PD_REQUIRED_MAX_POWER)
    {
        uint32_t maxCurrent = srcCap->dat[snkRdo.rdo_gen.objPos - 1].fixed_src.maxCurrent;
        snkRdo.rdo_gen.opPowerCur = maxCurrent;
        snkRdo.rdo_gen.minMaxPowerCur = maxCurrent;
    }
#endif /* defined(PD_REQUIRED_MAX_POWER) */
    (app_get_resp_buf(port))->respDo = snkRdo;
    app_resp_handler(context, app_get_resp_buf(context->port));
}
#endif /* (!(CY_PD_SOURCE_ONLY)) */

#if (!CY_PD_SINK_ONLY)

/*
 * This function can be used to ask EC to evaluate a request message
 * For now evaluating here and executing the callback in this function itself
 */
ATTRIBUTES_APP_PDO void eval_rdo(cy_stc_pdstack_context_t* context, cy_pd_pd_do_t rdo, cy_pdstack_app_resp_cbk_t app_resp_handler)
{
    cy_en_pdstack_status_t retVal = CALL_MAP(Cy_PdStack_Dpm_IsRdoValid)(context, rdo);

    if (retVal == CY_PDSTACK_STAT_SUCCESS)
    {
        CALL_MAP(app_get_resp_buf)(context->port)->reqStatus = CY_PDSTACK_REQ_ACCEPT;
    }
    else
    {
        CALL_MAP(app_get_resp_buf)(context->port)->reqStatus = CY_PDSTACK_REQ_REJECT;

#if CY_PD_REV3_ENABLE
        if(retVal == CY_PDSTACK_STAT_NOT_SUPPORTED)
        {
            CALL_MAP(app_get_resp_buf)(context->port)->reqStatus = CY_PDSTACK_REQ_SEND_HARD_RESET;
        }
#endif /* CY_PD_REV3_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
        if(true == ccg_power_throttle_get_power_throttle_cmd_pending(context))
        {
            ccg_power_throttle_set_power_throttle_renegotiation_complete(context, true, false);
        }
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */
    }

    app_resp_handler(context, CALL_MAP(app_get_resp_buf)(context->port));
}
#endif /* (!CY_PD_SINK_ONLY) */

#if (CCG6_SROM_CODE_ENABLE)
void app_update_rdo (uint8_t port, const pd_packet_t* srcCap, app_resp_t *appResp)
{
    /*  Enable below code if RDO prepared by ROM code needs to be over-ridden. */
#if CCG6_ENABLE_SRC_CAP_CONFIG_PARAMS
    const dpm_status_t* dpm = CALL_MAP(dpm_get_info)(port);
    uint8_t src_pdo_index, snk_pdo_index;
    pd_do_t* snkPdo = (pd_do_t*)&dpm->curSnkPdo[0];
    uint16_t src_vsafe5_cur = srcCap->dat[0].fixed_src.maxCurrent; /* Source max current for first PDO */
    pd_do_t snkRdo;
    uint32_t highest_gl_contract_power = 0u;
    bool match = false;

    for(snk_pdo_index = 0u; snk_pdo_index < dpm->curSnkPdocount; snk_pdo_index++)
    {
        for(src_pdo_index = 0u; src_pdo_index < srcCap->len; src_pdo_index++)
        {
            if(is_src_acceptable_snk(port, (pd_do_t*)(&srcCap->dat[src_pdo_index]), snk_pdo_index))
            {
                bool max_cond = false; 

                switch (PD_GET_PTR_HOST_CFG_TBL(port)->pdo_sel_alg)
                {
                    case HIGHEST_POWER:
                        /* Contract_power is based on SRC PDO */
                        if (srcCap->dat[src_pdo_index].fixed_src.supplyType == PDO_FIXED_SUPPLY)
                        {
                            uint32_t temp_power = calc_power(srcCap->dat[src_pdo_index].fixed_src.voltage,
                                    srcCap->dat[src_pdo_index].fixed_src.maxCurrent);
                            if (temp_power >= highest_gl_contract_power)
                            {
                                highest_gl_contract_power = temp_power;
                                max_cond = true;
                            }
                        }
                        break;

                    case HIGHEST_VOLTAGE:
                        /* Only fixed pdo takes part */
                        if ((srcCap->dat[src_pdo_index].fixed_src.supplyType == PDO_FIXED_SUPPLY)
                                && (CALL_MAP(gl_contract_voltage)[port] >= highest_gl_contract_power))
                        {
                            highest_gl_contract_power = CALL_MAP(gl_contract_voltage)[port];
                            max_cond = true;
                        }
                        break;

                    case HIGHEST_CURRENT:
                        /* Only fixed pdo takes part */
                        if ((srcCap->dat[src_pdo_index].fixed_src.supplyType == PDO_FIXED_SUPPLY)
                                && (srcCap->dat[src_pdo_index].fixed_src.maxCurrent >= highest_gl_contract_power))
                        {
                            highest_gl_contract_power = srcCap->dat[src_pdo_index].fixed_src.maxCurrent;
                            max_cond = true;
                        }
                        break;

                    default:
                        /* Contract_power is calculated in is_src_acceptable_snk() */
                        if (CALL_MAP(gl_contract_power)[port] >= highest_gl_contract_power)
                        {
                            highest_gl_contract_power = CALL_MAP(gl_contract_power)[port];
                            max_cond = true;
                        }
                        break;
                }

                if(max_cond)
                {
                    /* Check if sink needs higher capability */
                    if (
                            (snkPdo[0].fixed_snk.highCap) &&
                            (CALL_MAP(gl_contract_voltage)[port] == (VSAFE_5V/PD_VOLT_PER_UNIT))
                       )
                    {
                        /* 5V contract isn't acceptable with highCap = 1 */
                        continue;
                    }

                    snkRdo = form_rdo(port, (src_pdo_index + 1u), false,
                            (dpm->curSnkMaxMin[snk_pdo_index] & GIVE_BACK_MASK));
                    match = true;

#if APP_PPS_SINK_SUPPORT
                    /* Update to PPS RDO if so required by user. */
                    snkRdo = handle_pps_sink_rdo (port, srcCap, snkRdo);
#endif /* APP_PPS_SINK_SUPPORT */
                }
            }
        }
    }

    if(match == false)
    {
        /* Capability mismatch: Ask for vsafe5v PDO with CapMismatch */
        CALL_MAP(gl_contract_voltage)[port] = snkPdo[0].fixed_snk.voltage;
        CALL_MAP(gl_op_cur_power)[port] = snkPdo[0].fixed_snk.opCurrent;
        CALL_MAP(gl_contract_power)[port] = CALL_MAP(div_round_up)(
                CALL_MAP(gl_contract_voltage)[port] * CALL_MAP(gl_op_cur_power)[port], 500u);

        if(src_vsafe5_cur < CALL_MAP(gl_op_cur_power)[port])
        {
            /* SNK operation current can't be bigger than SRC maxCurrent */
            CALL_MAP(gl_op_cur_power)[port] = src_vsafe5_cur;
        }

        CALL_MAP(gl_max_min_cur_pwr)[port] = dpm->curSnkMaxMin[0];
        snkRdo = form_rdo(port, 1u, true, false);
    }

    /* Config table setting to always request max current instead of operation current */
    if(PD_GET_PTR_HOST_CFG_TBL(port)->req_max_pwr != 0)
    {
        uint32_t maxCurrent = srcCap->dat[snkRdo.rdo_gen.objPos - 1].fixed_src.maxCurrent;
        snkRdo.rdo_gen.opPowerCur = maxCurrent;
        snkRdo.rdo_gen.minMaxPowerCur = maxCurrent;
    }

    appResp->respDo = snkRdo;
#endif /* CCG6_ENABLE_SRC_CAP_CONFIG_PARAMS */
}
#endif /* (CCG6_SROM_CODE_ENABLE) */

#if APP_PPS_SINK_SUPPORT

static void snk_recontract_timer_cb (uint8_t port, timer_id_t id);

static void snk_recontract_cb (uint8_t port, resp_status_t resp, const pd_packet_t *pkt_ptr)
{
    if (resp == SEQ_ABORTED)
    {
        /* Restart the timer so that the command can be retried. */
        timer_start (port, APP_PPS_SNK_RECONTRACT_TIMER_ID, APP_PPS_SNK_CONTRACT_RETRY_PERIOD, snk_recontract_timer_cb);
    }
    else
    {
        /* Restart the timer to attempt the sequence again after the defined period. */
        timer_start (port, APP_PPS_SNK_RECONTRACT_TIMER_ID, APP_PPS_SNK_CONTRACT_PERIOD, snk_recontract_timer_cb);
    }
}

static void snk_recontract_timer_cb (uint8_t port, timer_id_t id)
{
    const dpm_status_t *dpm_stat = dpm_get_info (port);
    dpm_pd_cmd_buf_t    param;
    ccg_status_t        stat;

    if ((app_pps_snk_en[port]) && (dpm_stat->contractExist != 0) && (dpm_stat->curPortRole == PRT_ROLE_SINK) &&
            (dpm_stat->specRevSopLive >= PD_REV3))
    {
        param.cmdSop      = SOP;
        param.noOfCmdDo = 0;
        param.timeout      = 0;
        stat = dpm_pd_command (port, DPM_CMD_SNK_CAP_CHNG, &param, snk_recontract_cb);
        if (stat != CCG_STAT_SUCCESS)
        {
            /* Restart the timer so that the command can be retried. */
            timer_start (port, id, 5, snk_recontract_timer_cb);
        }
    }
    else
    {
        app_pps_snk_en[port]   = false;
        app_prog_rdo[port].val = 0;
    }
}

uint8_t hpi_user_reg_handler(uint16_t addr, uint8_t size, uint8_t *data)
{
    const dpm_status_t *dpm_stat;
    uint8_t stat = 0x0F;
    uint8_t port = (addr >> 12) - 1;
    pd_do_t rdo;

    if ((port < NO_OF_TYPEC_PORTS) && ((addr & 0xFF) == 0x38) && (size == 4))
    {
        /* Store the user provided value in the register for debug purposes. */
        hpi_init_userdef_regs (addr, size, data);

        dpm_stat = dpm_get_info (port);
        rdo.val  = MAKE_DWORD(data[3], data[2], data[1], data[0]);

        /* Operation is only valid if we are a sink and PD contract exists. */
        if ((dpm_stat->contractExist != 0) && (dpm_stat->curPortRole == PRT_ROLE_SINK) &&
                (dpm_stat->specRevSopLive >= PD_REV3))
        {
            app_prog_rdo[port]   = rdo;
            app_pps_snk_en[port] = true;
            stat = 0x02;                        /* Success code. */

            /* Start a timer which will start re-negotiation so that PPS source can be selected. */
            timer_start (port, APP_PPS_SNK_RECONTRACT_TIMER_ID, APP_PPS_SNK_CONTRACT_RETRY_PERIOD,
                    snk_recontract_timer_cb);
        }
    }
    return (stat);
}

void app_pps_sink_disable(uint8_t port)
{
    app_pps_snk_en[port] = false;
}

#endif /* APP_PPS_SINK_SUPPORT */

/* End of File */

