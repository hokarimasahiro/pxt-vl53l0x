/**
 * makecode VL53L0X Package.
 */

/**
 * VL53L0X block
 */
//% weight=100 color=#303030 icon="\ue115" block="VL53L0X"
namespace VL53L0X {
    let I2C_ADDR = 0x29;
    let stop_variable:number;
    let measurement_timing_budget_us:number;
    let did_timeout:boolean;
    let timeout_start_ms:number;
    let io_timeout:number=0;

    // Start continuous ranging measurements. If period_ms (optional) is 0 or not
    // given, continuous back-to-back mode is used (the sensor takes measurements as
    // often as possible); otherwise, continuous timed mode is used, with the given
    // inter-measurement period in milliseconds determining how often the sensor
    // takes a measurement.
    // based on VL53L0X_StartMeasurement()
    /**
     * based on VL53L0X_StartMeasurement
     * @param period_ms interval, eg: 0
     */
    //% blockId="startContinuous" block="StartMeasurement VL53L0X interval%period_ms"
    export function startContinuous(period_ms:number)
    {
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    writeReg(0x91, stop_variable);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    if (period_ms != 0)
    {
        // continuous timed mode

        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

        let osc_calibrate_val = readReg16(regAddr.OSC_CALIBRATE_VAL);

        if (osc_calibrate_val != 0)
        {
        period_ms *= osc_calibrate_val;
        }

        writeReg32(regAddr.SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

        writeReg(regAddr.SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
    }
    else
    {
        // continuous back-to-back mode
        writeReg(regAddr.SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
    }
    }

    // Stop continuous measurements
    // based on VL53L0X_StopMeasurement()
    /**
     * based on VL53L0X_StartMeasurement
     */
    //% blockId="stopContinuous" block="StopMeasurement VL53L0X"
    export function stopContinuous()
    {
    writeReg(regAddr.SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    writeReg(0x91, 0x00);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    }

    // Returns a range reading in millimeters when continuous mode is active
    // (readRangeSingleMillimeters() also calls this function after starting a
    // single-shot range measurement)
    /**
     * based on VL53L0X_StartMeasurement
     */
    //% blockId="readRangeContinuousMillimeters" block="readRangeContinuousMillimeters"
    export function readRangeContinuousMillimeters():number
    {
    startTimeout();
    while ((readReg(regAddr.RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        if (checkTimeoutExpired())
        {
        did_timeout = true;
        return 65535;
        }
    }

    // assumptions: Linearity Corrective Gain is 1000 (default);
    // fractional ranging is not enabled
    let range = readReg16(regAddr.RESULT_RANGE_STATUS + 10);

    writeReg(regAddr.SYSTEM_INTERRUPT_CLEAR, 0x01);

    return range;
    }

    // Performs a single-shot range measurement and returns the reading in
    // millimeters
    // based on VL53L0X_PerformSingleRangingMeasurement()
    /**
     * based on VL53L0X_StartMeasurement
     */
    //% blockId="readRangeSingleMillimeters" block="readRangeSingleMillimeters"
    export function readRangeSingleMillimeters():number
    {
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    writeReg(0x91, stop_variable);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    writeReg(regAddr.SYSRANGE_START, 0x01);

    // "Wait until start bit has been cleared"
    startTimeout();
    while (readReg(regAddr.SYSRANGE_START) & 0x01)
    {
        if (checkTimeoutExpired())
        {
        did_timeout = true;
        return 65535;
        }
    }

    return readRangeContinuousMillimeters();
    }

    /**
     * initialize VL53L0X
     * @param io_2v8 running volutage at 2v8, eg: true
     */
    //% blockId="initialize" block="initialize VL53L0X at 2V8 %volts"
    export function initialize(io_2v8:boolean): boolean {

        // check model ID register (value specified in datasheet)
        if (readReg(regAddr.IDENTIFICATION_MODEL_ID) != 0xEE) { return false; }

        // VL53L0X_DataInit() begin

        // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
        if (io_2v8)
        {
            writeReg(regAddr.VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
            readReg(regAddr.VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
        }

        // "Set I2C standard mode"
        writeReg(0x88, 0x00);
        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        stop_variable = readReg(0x91);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
        writeReg(regAddr.MSRC_CONFIG_CONTROL, readReg(regAddr.MSRC_CONFIG_CONTROL) | 0x12);

        // set final range signal rate limit to 0.25 MCPS (million counts per second)
        setSignalRateLimit(0.25);

        writeReg(regAddr.SYSTEM_SEQUENCE_CONFIG, 0xFF);

        // VL53L0X_DataInit() end

        // VL53L0X_StaticInit() begin

        let spad_count:number;
        let spad_type_is_aperture:boolean;
        if (!getSpadInfo(spad_count, spad_type_is_aperture)) { return false; }

        // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
        // the API, but the same data seems to be more easily readable from
        // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
        let ref_spad_map= readBuf(regAddr.GLOBAL_CONFIG_SPAD_ENABLES_REF_0,  6);

        // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

        writeReg(0xFF, 0x01);
        writeReg(regAddr.DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
        writeReg(regAddr.DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
        writeReg(0xFF, 0x00);
        writeReg(regAddr.GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

        let first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
        let spads_enabled = 0;

        for (let i = 0; i < 48; i++)
        {
            if (i < first_spad_to_enable || spads_enabled == spad_count)
            {
            // This bit is lower than the first one that should be enabled, or
            // (reference_spad_count) bits have already been enabled, so zero this bit
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
            }
            else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
            {
            spads_enabled++;
            }
        }

        writeBuf(regAddr.GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map);

        // -- VL53L0X_set_reference_spads() end

        // -- VL53L0X_load_tuning_settings() begin
        // DefaultTuningSettings from vl53l0x_tuning.h

        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);

        writeReg(0xFF, 0x00);
        writeReg(0x09, 0x00);
        writeReg(0x10, 0x00);
        writeReg(0x11, 0x00);

        writeReg(0x24, 0x01);
        writeReg(0x25, 0xFF);
        writeReg(0x75, 0x00);

        writeReg(0xFF, 0x01);
        writeReg(0x4E, 0x2C);
        writeReg(0x48, 0x00);
        writeReg(0x30, 0x20);

        writeReg(0xFF, 0x00);
        writeReg(0x30, 0x09);
        writeReg(0x54, 0x00);
        writeReg(0x31, 0x04);
        writeReg(0x32, 0x03);
        writeReg(0x40, 0x83);
        writeReg(0x46, 0x25);
        writeReg(0x60, 0x00);
        writeReg(0x27, 0x00);
        writeReg(0x50, 0x06);
        writeReg(0x51, 0x00);
        writeReg(0x52, 0x96);
        writeReg(0x56, 0x08);
        writeReg(0x57, 0x30);
        writeReg(0x61, 0x00);
        writeReg(0x62, 0x00);
        writeReg(0x64, 0x00);
        writeReg(0x65, 0x00);
        writeReg(0x66, 0xA0);

        writeReg(0xFF, 0x01);
        writeReg(0x22, 0x32);
        writeReg(0x47, 0x14);
        writeReg(0x49, 0xFF);
        writeReg(0x4A, 0x00);

        writeReg(0xFF, 0x00);
        writeReg(0x7A, 0x0A);
        writeReg(0x7B, 0x00);
        writeReg(0x78, 0x21);

        writeReg(0xFF, 0x01);
        writeReg(0x23, 0x34);
        writeReg(0x42, 0x00);
        writeReg(0x44, 0xFF);
        writeReg(0x45, 0x26);
        writeReg(0x46, 0x05);
        writeReg(0x40, 0x40);
        writeReg(0x0E, 0x06);
        writeReg(0x20, 0x1A);
        writeReg(0x43, 0x40);

        writeReg(0xFF, 0x00);
        writeReg(0x34, 0x03);
        writeReg(0x35, 0x44);

        writeReg(0xFF, 0x01);
        writeReg(0x31, 0x04);
        writeReg(0x4B, 0x09);
        writeReg(0x4C, 0x05);
        writeReg(0x4D, 0x04);

        writeReg(0xFF, 0x00);
        writeReg(0x44, 0x00);
        writeReg(0x45, 0x20);
        writeReg(0x47, 0x08);
        writeReg(0x48, 0x28);
        writeReg(0x67, 0x00);
        writeReg(0x70, 0x04);
        writeReg(0x71, 0x01);
        writeReg(0x72, 0xFE);
        writeReg(0x76, 0x00);
        writeReg(0x77, 0x00);

        writeReg(0xFF, 0x01);
        writeReg(0x0D, 0x01);

        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x01);
        writeReg(0x01, 0xF8);

        writeReg(0xFF, 0x01);
        writeReg(0x8E, 0x01);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        // -- VL53L0X_load_tuning_settings() end

        // "Set interrupt config to new sample ready"
        // -- VL53L0X_SetGpioConfig() begin

        writeReg(regAddr.SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
        writeReg(regAddr.GPIO_HV_MUX_ACTIVE_HIGH, readReg(regAddr.GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
        writeReg(regAddr.SYSTEM_INTERRUPT_CLEAR, 0x01);

        // -- VL53L0X_SetGpioConfig() end

        measurement_timing_budget_us = getMeasurementTimingBudget();

        // "Disable MSRC and TCC by default"
        // MSRC = Minimum Signal Rate Check
        // TCC = Target CentreCheck
        // -- VL53L0X_SetSequenceStepEnable() begin

        writeReg(regAddr.SYSTEM_SEQUENCE_CONFIG, 0xE8);

        // -- VL53L0X_SetSequenceStepEnable() end

        // "Recalculate timing budget"
        setMeasurementTimingBudget(measurement_timing_budget_us);

        // VL53L0X_StaticInit() end

        // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

        // -- VL53L0X_perform_vhv_calibration() begin

        writeReg(regAddr.SYSTEM_SEQUENCE_CONFIG, 0x01);
        if (!performSingleRefCalibration(0x40)) { return false; }

        // -- VL53L0X_perform_vhv_calibration() end

        // -- VL53L0X_perform_phase_calibration() begin

        writeReg(regAddr.SYSTEM_SEQUENCE_CONFIG, 0x02);
        if (!performSingleRefCalibration(0x00)) { return false; }

        // -- VL53L0X_perform_phase_calibration() end

        // "restore the previous Sequence Config"
        writeReg(regAddr.SYSTEM_SEQUENCE_CONFIG, 0xE8);

        // VL53L0X_PerformRefCalibration() end
        return true;
    }
// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
export function setSignalRateLimit(limit_Mcps:number):boolean
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  writeReg16(regAddr.FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
  return true;
}

// Get the return signal rate limit check value in MCPS
export function getSignalRateLimit():number
{
  return readReg16(regAddr.FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
export function setMeasurementTimingBudget(budget_us:number):boolean
{
  let enables:boolean[]; //0:tcc, 1:msrc, 2:dss, 3:pre_range, 4:final_range
  let timeouts:number[];    // 0:pre_range_vcsel_period_pclks, 1:final_range_vcsel_period_pclks,
                                            // 2:msrc_dss_tcc_mclks, 3:pre_range_mclks, 4:final_range_mclks,
                                            // 5:msrc_dss_tcc_us, 6:pre_range_us, 7:final_range_us
  const StartOverhead     = 1910;
  const EndOverhead        = 960;
  const MsrcOverhead       = 660;
  const TccOverhead        = 590;
  const DssOverhead        = 690;
  const PreRangeOverhead   = 660;
  const FinalRangeOverhead = 550;

  const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  let used_budget_us = StartOverhead + EndOverhead;

  enables=getSequenceStepEnables();
  timeouts=getSequenceStepTimeouts(enables);

  if (enables[enablesType.tcc]) // tcc
  {
    used_budget_us += (timeouts[timeoutsType.msrc_dss_tcc_us] + TccOverhead);  // msrc_dss_tcc_us
  }

  if (enables[enablesType.dss])  // dss
  {
    used_budget_us += 2 * (timeouts[timeoutsType.msrc_dss_tcc_us] + DssOverhead); // msrc_dss_tcc_us
  }
  else if (enables[enablesType.msrc])    // msrc
  {
    used_budget_us += (timeouts[timeoutsType.msrc_dss_tcc_us] + MsrcOverhead);    // msrc_dss_tcc_us
  }

  if (enables[enablesType.pre_range])    // pre_range
  {
    used_budget_us += (timeouts[timeoutsType.pre_range_us] + PreRangeOverhead);   // pre_range_us
  }

  if (enables[enablesType.final_range])  // final_range
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    let final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    let final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts[timeoutsType.final_range_vcsel_period_pclks]);  // final_range_vcsel_period_pclks

    if (enables[enablesType.pre_range])  // pre_range
    {
      final_range_timeout_mclks += timeouts[timeoutsType.pre_range_mclks];    // pre_range_mclks
    }

    writeReg16(regAddr.FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
export function getMeasurementTimingBudget():number
{
  let enables:boolean[]; //0:tcc, 1:msrc, 2:dss, 3:pre_range, 4:final_range
  let timeouts:number[];    // 0:pre_range_vcsel_period_pclks, 1:final_range_vcsel_period_pclks,
                                            // 2:msrc_dss_tcc_mclks, 3:pre_range_mclks, 4:final_range_mclks,
                                            // 5:msrc_dss_tcc_us, 6:pre_range_us, 7:final_range_us
  const StartOverhead     = 1910;
  const EndOverhead        = 960;
  const MsrcOverhead       = 660;
  const TccOverhead        = 590;
  const DssOverhead        = 690;
  const PreRangeOverhead   = 660;
  const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  let budget_us = StartOverhead + EndOverhead;

  enables=getSequenceStepEnables();
  timeouts=getSequenceStepTimeouts(enables);

  if (enables[enablesType.tcc])  // tcc 
  {
    budget_us += (timeouts[timeoutsType.msrc_dss_tcc_us] + TccOverhead);  // msrc_dss_tcc_us
  }

  if (enables[enablesType.dss])  // dss
  {
    budget_us += 2 * (timeouts[timeoutsType.msrc_dss_tcc_us] + DssOverhead);  // msrc_dss_tcc_us
  }
  else if (enables[enablesType.msrc])    // msrc
  {
    budget_us += (timeouts[timeoutsType.msrc_dss_tcc_us] + MsrcOverhead); // .msrc_dss_tcc_us
  }

  if (enables[enablesType.pre_range])    // pre_range
  {
    budget_us += (timeouts[timeoutsType.pre_range_us] + PreRangeOverhead);    // pre_range_us
  }

  if (enables[enablesType.final_range])  // final_range
  {
    budget_us += (timeouts[timeoutsType.final_range_us] + FinalRangeOverhead);    // final_range_us
  }

  measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
export function setVcselPulsePeriod(type:vcselPeriodType, period_pclks:number):boolean
{
  let vcsel_period_reg = encodeVcselPeriod(period_pclks);

  let enables:boolean[]; //0:tcc, 1:msrc, 2:dss, 3:pre_range, 4:final_range
  let timeouts:number[];    // 0:pre_range_vcsel_period_pclks, 1:final_range_vcsel_period_pclks,
                                            // 2:msrc_dss_tcc_mclks, 3:pre_range_mclks, 4:final_range_mclks,
                                            // 5:msrc_dss_tcc_us, 6:pre_range_us, 7:final_range_us
  enables=getSequenceStepEnables();
  timeouts=getSequenceStepTimeouts(enables);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (type == vcselPeriodType.VcselPeriodPreRange)
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
        writeReg(regAddr.PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
        writeReg(regAddr.PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
        writeReg(regAddr.PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
        writeReg(regAddr.PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return false;
    }
    writeReg(regAddr.PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    writeReg(regAddr.PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    let new_pre_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts[6], period_pclks);  // pre_range_us

    writeReg16(regAddr.PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    let new_msrc_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts[5], period_pclks);   // msrc_dss_tcc_us

    writeReg(regAddr.MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == vcselPeriodType.VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        writeReg(regAddr.FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        writeReg(regAddr.FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(regAddr.GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        writeReg(regAddr.ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        writeReg(0xFF, 0x01);
        writeReg(regAddr.ALGO_PHASECAL_LIM, 0x30);
        writeReg(0xFF, 0x00);
        break;

      case 10:
        writeReg(regAddr.FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        writeReg(regAddr.FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(regAddr.GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(regAddr.ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        writeReg(0xFF, 0x01);
        writeReg(regAddr.ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;

      case 12:
        writeReg(regAddr.FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        writeReg(regAddr.FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(regAddr.GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(regAddr.ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        writeReg(0xFF, 0x01);
        writeReg(regAddr.ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;

      case 14:
        writeReg(regAddr.FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        writeReg(regAddr.FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(regAddr.GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(regAddr.ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        writeReg(0xFF, 0x01);
        writeReg(regAddr.ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;

      default:
        // invalid period
        return false;
    }

    // apply new VCSEL period
    writeReg(regAddr.FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    let new_final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts[7], period_pclks);    // final_range_us

    if (enables[3])  // pre_range
    {
      new_final_range_timeout_mclks += timeouts[timeoutsType.pre_range_mclks];    // pre_range_mclks
    }

    writeReg16(regAddr.FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return false;
  }

  // "Finally, the timing budget must be re-applied"

  setMeasurementTimingBudget(measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  let sequence_config = readReg(regAddr.SYSTEM_SEQUENCE_CONFIG);
  writeReg(regAddr.SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(0x0);
  writeReg(regAddr.SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return true;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
export function getVcselPulsePeriod(type:vcselPeriodType):number
{
  if (type == vcselPeriodType.VcselPeriodPreRange)
  {
    return decodeVcselPeriod(readReg(regAddr.PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == vcselPeriodType.VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(readReg(regAddr.FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
export function timeoutOccurred():boolean
{
  let tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
export function getSpadInfo(count:number, type_is_aperture:boolean):boolean
{
  let tmp;

  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);

  writeReg(0xFF, 0x06);
  writeReg(0x83, readReg(0x83) | 0x04);
  writeReg(0xFF, 0x07);
  writeReg(0x81, 0x01);

  writeReg(0x80, 0x01);

  writeReg(0x94, 0x6b);
  writeReg(0x83, 0x00);
  startTimeout();
  while (readReg(0x83) == 0x00)
  {
    if (checkTimeoutExpired()) { return false; }
  }
  writeReg(0x83, 0x01);
  tmp = readReg(0x92);

  count = tmp & 0x7f;
  type_is_aperture = (((tmp >> 7) & 0x01) == 0) ? false:true;

  writeReg(0x81, 0x00);
  writeReg(0xFF, 0x06);
  writeReg(0x83, readReg(0x83)  & ~0x04);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x01);

  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
export function getSequenceStepEnables():boolean[]
{
  let sequence_config = readReg(regAddr.SYSTEM_SEQUENCE_CONFIG);
  return[   ((sequence_config >> 4) & 0x1) == 0 ? false:true,
            ((sequence_config >> 3) & 0x1) == 0 ? false:true,
            ((sequence_config >> 2) & 0x1) == 0 ? false:true,
            ((sequence_config >> 6) & 0x1) == 0 ? false:true,
            ((sequence_config >> 7) & 0x1) == 0 ? false:true
        ]
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
/*    declare const enum timeoutsType {
        pre_range_vcsel_period_pclks=0,
        final_range_vcsel_period_pclks=1,
        msrc_dss_tcc_mclks=2,
        pre_range_mclks=3,
        final_range_mclks=4,
        msrc_dss_tcc_us=5,
        pre_range_us=6,
        final_range_us=7
    }*/
export function getSequenceStepTimeouts(enables:boolean[]):number[]
{
    let pre_range_vcsel_period_pclks = getVcselPulsePeriod(vcselPeriodType.VcselPeriodPreRange);
    let final_range_vcsel_period_pclks = getVcselPulsePeriod(vcselPeriodType.VcselPeriodFinalRange);
    let msrc_dss_tcc_mclks = readReg(regAddr.MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    let pre_range_mclks = decodeTimeout(readReg16(regAddr.PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    let final_range_mclks = decodeTimeout(readReg16(regAddr.FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    if (enables[enablesType.pre_range])
    {
        final_range_mclks -= pre_range_mclks;
    }
    let msrc_dss_tcc_us = timeoutMclksToMicroseconds(msrc_dss_tcc_mclks, pre_range_vcsel_period_pclks);
    let pre_range_us = timeoutMclksToMicroseconds(pre_range_mclks,pre_range_vcsel_period_pclks);
    let final_range_us = timeoutMclksToMicroseconds(final_range_mclks,final_range_vcsel_period_pclks);

    return [pre_range_vcsel_period_pclks,
            final_range_vcsel_period_pclks,
            msrc_dss_tcc_mclks,
            pre_range_mclks,
            final_range_mclks,
            msrc_dss_tcc_us,
            pre_range_us,
            final_range_us];
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
export function decodeTimeout(reg_val:number):number
{
  // format: "(LSByte * 2^MSByte) + 1"
  return ((reg_val & 0x00FF) <<
         ((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
export function encodeTimeout(timeout_mclks:number):number
{
  // format: "(LSByte * 2^MSByte) + 1"

  let ls_byte = 0;
  let ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
export function timeoutMclksToMicroseconds(timeout_period_mclks:number, vcsel_period_pclks:number):number
{
  let macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
export function timeoutMicrosecondsToMclks(timeout_period_us:number, vcsel_period_pclks:number):number
{
  let macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


// based on VL53L0X_perform_single_ref_calibration()
export function performSingleRefCalibration(vhv_init_byte:number):boolean
{
  writeReg(regAddr.SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  startTimeout();
  while ((readReg(regAddr.RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired()) { return false; }
  }

  writeReg(regAddr.SYSTEM_INTERRUPT_CLEAR, 0x01);

  writeReg(regAddr.SYSRANGE_START, 0x00);

  return true;
}

    /**
     * Write byte data to the specified registor
     * @param addr registor address, eg: 0x80
     * @param dat is the data will be write, eg: 0x05
     */
    //% blockId="writeReg" block="write registor address %addr|byte %dat"
    //% advanced=true
    export function writeReg(addr: number, dat: number): void {
        let buf = pins.createBuffer(2);
        buf[0] = addr >> 0;
        buf[1] = dat >> 0;
        pins.i2cWriteBuffer(I2C_ADDR, buf)
    }

    /**
     * Read byte data from the specified registor
     * @param addr registor address, eg: 0x80
     */
    //% blockId="ReadReg" block="read data from registor address %addr"
    //% advanced=true
    export function readReg(addr: number): number {
        pins.i2cWriteNumber(I2C_ADDR, addr >> 0, NumberFormat.UInt8BE);
        return pins.i2cReadNumber(I2C_ADDR, NumberFormat.UInt8BE);
    }

    /**
     * Write word data to the specified registor
     * @param addr eeprom address, eg: 0x80
     * @param dat is the data will be write, eg: 0x1234
     */
    //% blockId="WriteReg16" block="write registor address %addr|word %dat"
    //% advanced=true
    export function writeReg16(addr: number, dat: number): void {
        let buf = pins.createBuffer(3);
        buf[0] = addr >> 0;
        buf[1] = dat >> 0;
        buf[2] = dat >> 8;
        pins.i2cWriteBuffer(I2C_ADDR, buf)
    }

    /**
     * Read word data from the specified registor
     * @param addr registor address, eg: 0x80
     */
    //% blockId="ReadReg16" block="read word from registor address %addr"
    //% advanced=true
    export function readReg16(addr: number): number {
        pins.i2cWriteNumber(I2C_ADDR, addr >> 0, NumberFormat.UInt8BE);
        return pins.i2cReadNumber(I2C_ADDR, NumberFormat.UInt16BE);
    }

    /**
     * Write double word data to the specified registor
     * @param addr registor address, eg: 0x80
     * @param dat is the data will be write, eg: 0x12345678
     */
    //% blockId="WriteReg32" block="write registor address %addr|dword %dat"
    //% advanced=true
    export function writeReg32(addr: number, dat: number): void {
        let buf = pins.createBuffer(5);
        buf[0] = addr;
        buf[1] = dat >> 0;
        buf[2] = dat >> 8;
        buf[3] = dat >> 16;
        buf[4] = dat >> 24;
        pins.i2cWriteBuffer(I2C_ADDR, buf)
    }

    /**
     * Read double word data from the specified registor
     * @param addr eeprom address, eg: 0x80
     */
    //% blockId="ReadReg32" block="read dword from registor address %addr"
    //% advanced=true
    export function readReg32(addr: number): number {
        pins.i2cWriteNumber(I2C_ADDR, addr >> 0, NumberFormat.UInt8BE);
        return pins.i2cReadNumber(I2C_ADDR, NumberFormat.Int32BE);
    }

    /**
     * Write data to the specified registor
     * @param addr registor address, eg: 0x00
     * @param dat is the data will be write, eg: 5
     */
    //% blockId="WriteBuf" block="registor address %addr|write buf %dat"
    //% advanced=true
    export function writeBuf(addr: number, dat: number[]): void {
        let buf = pins.createBuffer(dat.length + 1);
        buf[0] = addr >> 0;
        for(let i=0;i<dat.length;i++){
            buf[i + 1] = dat[i] & 0xff;
        }
        pins.i2cWriteBuffer(I2C_ADDR, buf)
    }

    /**
     * Read data from the specified registor
     * @param addr registor address, eg: 0x00
     * @param size read data count, eg: 16
     */
    //% blockId="ReadBuf" block="registor address %addr|read buf %size"
    //% advanced=true
    export function readBuf(addr: number, size: number): number[] {
        let retbuf:number[]=[];

        pins.i2cWriteNumber(I2C_ADDR, addr >> 0, NumberFormat.UInt16BE);
        let buf = pins.i2cReadBuffer(I2C_ADDR, size);
        for(let i=0;i<size;i++){
            retbuf.push(buf[i]);
        }
        return retbuf;
    }

    /**
     * set i2c address
     * @param addr i2c address, eg: 0x50
     */
    //% blockId="setI2cAddress" block="i2c address set to %addr"
    //% advanced=true
    export function setI2cAddress(addr: number): void {
        writeReg(regAddr.I2C_SLAVE_DEVICE_ADDRESS, (addr >> 0)& 0x7F);
        I2C_ADDR = addr >> 0;
    }
    // Record the current time to check an upcoming timeout against
    function startTimeout() {
        timeout_start_ms = control.millis();
    }
    // Check if timeout is enabled (set to nonzero value) and has expired
    function checkTimeoutExpired():boolean{
    return (io_timeout > 0 && ((control.millis() - timeout_start_ms) > io_timeout))
    }

    // Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
    // from register value
    // based on VL53L0X_decode_vcsel_period()
    function decodeVcselPeriod(reg_val:number):number{
        return  (((reg_val) + 1) << 1)
    }

    // Encode VCSEL pulse period register value from period in PCLKs
    // based on VL53L0X_encode_vcsel_period()
    function encodeVcselPeriod(period_pclks:number):number{
        return (((period_pclks) >> 1) - 1)
    } 

    // Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
    // based on VL53L0X_calc_macro_period_ps()
    // PLL_period_ps = 1655; macro_period_vclks = 2304
    function calcMacroPeriod(vcsel_period_pclks:number):number{
        return (((2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)
    } 
}
