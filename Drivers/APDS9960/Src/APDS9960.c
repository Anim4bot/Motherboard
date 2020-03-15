#include "APDS9960.h"


gesture_data_type gesture_data_;
int gesture_ud_delta_;
int gesture_lr_delta_;
int gesture_ud_count_;
int gesture_lr_count_;
int gesture_near_count_;
int gesture_far_count_;
int gesture_state_;
int gesture_motion_;

HAL_StatusTypeDef APDS9960_Write8(uint8_t Reg, uint8_t Val)
{
	HAL_StatusTypeDef status;
	uint8_t data[2];

	data[0] = Reg;
	data[1] = Val;

	status = HAL_I2C_Mem_Write(&APDS9960_I2C_PORT, APDS9960_I2C_ADDR, Reg, 1, Val, 1, 50);
	if (status == HAL_OK)
	{
		return OK;
	}
	else
	{
		return NOK;
	}

}


uint8_t APDS9960_Read8(uint8_t Reg, uint8_t data)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&APDS9960_I2C_PORT, APDS9960_I2C_ADDR, Reg, 1, &data, 1, 50);

	return 1;
}

uint8_t APDS9960_Read(uint8_t Reg, uint8_t data, uint8_t size)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&APDS9960_I2C_PORT, APDS9960_I2C_ADDR, Reg, 1, &data, &size, 50);

	return 1;
}




uint8_t APDS9960_Init(void)
{
    uint8_t id;

    /* Read ID register and check against known values for APDS-9960 */
    APDS9960_Read8(APDS9960_ID, &id);
    if( !(id == APDS9960_ID_1 || id == APDS9960_ID_2) )
    {
        return NOK;
    }

    /* Set ENABLE register to 0 (disable all features) */
    if( !APDS9960_SetMode(ALL, OFF) ) {
        return NOK;
    }

    /* Set default values for ambient light and proximity registers */
    if( !APDS9960_Write8(APDS9960_ATIME, DEFAULT_ATIME) ) {
        return NOK;
    }
    if( !APDS9960_Write8(APDS9960_WTIME, DEFAULT_WTIME) ) {
        return NOK;
    }
    if( !APDS9960_Write8(APDS9960_PPULSE, DEFAULT_PROX_PPULSE) ) {
        return NOK;
    }
    if( !APDS9960_Write8(APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR) ) {
        return NOK;
    }
    if( !APDS9960_Write8(APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL) ) {
        return NOK;
    }
    if( !APDS9960_Write8(APDS9960_CONFIG1, DEFAULT_CONFIG1) ) {
        return NOK;
    }
    if( !APDS9960_SetLEDDrive(DEFAULT_LDRIVE) ) {
        return NOK;
    }
    if( !APDS9960_SetProximityGain(DEFAULT_PGAIN) ) {
        return NOK;
    }
    if( !APDS9960_SetAmbientLightGain(DEFAULT_AGAIN) ) {
        return NOK;
    }
    if( !APDS9960_SetProxIntLowThresh(DEFAULT_PILT) ) {
        return NOK;
    }
    if( !APDS9960_SetProxIntHighThresh(DEFAULT_PIHT) ) {
        return NOK;
    }
    if( !APDS9960_SetLightIntLowThreshold(DEFAULT_AILT) ) {
        return NOK;
    }
    if( !APDS9960_SetLightIntHighThreshold(DEFAULT_AIHT) ) {
        return NOK;
    }
    if( !APDS9960_Write8(APDS9960_PERS, DEFAULT_PERS) ) {
        return NOK;
    }
    if( !APDS9960_Write8(APDS9960_CONFIG2, DEFAULT_CONFIG2) ) {
        return NOK;
    }
    if( !APDS9960_Write8(APDS9960_CONFIG3, DEFAULT_CONFIG3) ) {
        return NOK;
    }

    /* Set default values for gesture sense registers */
    if( !APDS9960_SetGestureEnterThresh(DEFAULT_GPENTH) ) {
        return NOK;
    }
    if( !APDS9960_SetGestureExitThresh(DEFAULT_GEXTH) ) {
        return NOK;
    }
    if( !APDS9960_Write8(APDS9960_GCONF1, DEFAULT_GCONF1) ) {
        return NOK;
    }
    if( !APDS9960_SetGestureGain(DEFAULT_GGAIN) ) {
        return NOK;
    }
    if( !APDS9960_SetGestureLEDDrive(DEFAULT_GLDRIVE) ) {
        return NOK;
    }
    if( !APDS9960_SetGestureWaitTime(DEFAULT_GWTIME) ) {
        return NOK;
    }
    if( !APDS9960_Write8(APDS9960_GOFFSET_U, DEFAULT_GOFFSET) ) {
        return NOK;
    }
    if( !APDS9960_Write8(APDS9960_GOFFSET_D, DEFAULT_GOFFSET) ) {
        return NOK;
    }
    if( !APDS9960_Write8(APDS9960_GOFFSET_L, DEFAULT_GOFFSET) ) {
        return NOK;
    }
    if( !APDS9960_Write8(APDS9960_GOFFSET_R, DEFAULT_GOFFSET) ) {
        return NOK;
    }
    if( !APDS9960_Write8(APDS9960_GPULSE, DEFAULT_GPULSE) ) {
        return NOK;
    }
    if( !APDS9960_Write8(APDS9960_GCONF3, DEFAULT_GCONF3) ) {
        return NOK;
    }
    if( !APDS9960_SetGestureIntEnable(DEFAULT_GIEN) ) {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_SetMode(uint8_t mode, uint8_t enable)
{
    uint8_t reg_val;

    /* Read current ENABLE register */
    reg_val = APDS9960_GetMode();
    if( reg_val == ERROR )
    {
        return NOK;
    }

    /* Change bit(s) in ENABLE register */
    enable = enable & 0x01;
    if( mode >= 0 && mode <= 6 )
    {
        if (enable)
        {
            reg_val |= (1 << mode);
        }
        else
        {
            reg_val &= ~(1 << mode);
        }
    }
    else if( mode == ALL )
    {
        if (enable)
        {
            reg_val = 0x7F;
        }
        else
        {
            reg_val = 0x00;
        }
    }

    /* Write value back to ENABLE register */
    if( !APDS9960_Write8(APDS9960_ENABLE, reg_val) )
    {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_GetMode(void)
{
  uint8_t enable_value;

    /* Read current ENABLE register */
    if( !APDS9960_Read8(APDS9960_ENABLE, &enable_value) )
    {
        return NOK;
    }

    return enable_value;
}



uint8_t APDS9960_SetLEDDrive(uint8_t drive)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( !APDS9960_Read8(APDS9960_CONTROL, &val) )
    {
        return NOK;
    }

    /* Set bits in register to given value */
    drive &= 0x03;
    drive = drive << 6;
    val &= 0x3F;
    val |= drive;

    /* Write register value back into CONTROL register */
    if( !APDS9960_Write8(APDS9960_CONTROL, val) )
    {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_SetProximityGain(uint8_t drive)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( !APDS9960_Read8(APDS9960_CONTROL, &val) )
    {
        return NOK;
    }

    /* Set bits in register to given value */
    drive &= 0x03;
    drive = drive << 2;
    val &= 0xF3;
    val |= drive;

    /* Write register value back into CONTROL register */
    if( !APDS9960_Write8(APDS9960_CONTROL, val) )
    {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_SetAmbientLightGain(uint8_t drive)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( !APDS9960_Read8(APDS9960_CONTROL, &val) )
    {
        return NOK;
    }

    /* Set bits in register to given value */
    drive &= 0x03;
    val &= 0xFC;
    val |= drive;

    /* Write register value back into CONTROL register */
    if( !APDS9960_Write8(APDS9960_CONTROL, val) )
    {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_SetProxIntLowThresh(uint8_t threshold)
{
    if( !APDS9960_Write8(APDS9960_PILT, threshold) )
    {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_SetProxIntHighThresh(uint8_t threshold)
{
    if( !APDS9960_Write8(APDS9960_PIHT, threshold) )
    {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_SetLightIntLowThreshold(uint16_t threshold)
{
    uint8_t val_low;
    uint8_t val_high;

    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;

    /* Write low byte */
    if( !APDS9960_Write8(APDS9960_AILTL, val_low) )
    {
        return NOK;
    }

    /* Write high byte */
    if( !APDS9960_Write8(APDS9960_AILTH, val_high) )
    {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_SetLightIntHighThreshold(uint16_t threshold)
{
    uint8_t val_low;
    uint8_t val_high;

    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;

    /* Write low byte */
    if( !APDS9960_Write8(APDS9960_AIHTL, val_low) )
    {
        return NOK;
    }

    /* Write high byte */
    if( !APDS9960_Write8(APDS9960_AIHTH, val_high) )
    {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_SetGestureIntEnable(uint8_t enable)
{
    uint8_t val;

    /* Read value from GCONF4 register */
    if( !APDS9960_Read8(APDS9960_GCONF4, &val) )
    {
        return NOK;
    }

    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 1;
    val &= 0xFD;
    val |= enable;

    /* Write register value back into GCONF4 register */
    if( !APDS9960_Write8(APDS9960_GCONF4, val) )
    {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_SetGestureWaitTime(uint8_t time)
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if( !APDS9960_Read8(APDS9960_GCONF2, &val) )
    {
        return NOK;
    }

    /* Set bits in register to given value */
    time &= 0x07;
    val &= 0xF8;
    val |= time;

    /* Write register value back into GCONF2 register */
    if( !APDS9960_Write8(APDS9960_GCONF2, val) )
    {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_SetGestureLEDDrive(uint8_t drive)
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if( !APDS9960_Read8(APDS9960_GCONF2, &val) ) {
        return NOK;
    }

    /* Set bits in register to given value */
    drive &= 0x03;
    drive = drive << 3;
    val &= 0xE7;
    val |= drive;

    /* Write register value back into GCONF2 register */
    if( !APDS9960_Write8(APDS9960_GCONF2, val) ) {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_SetGestureGain(uint8_t gain)
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if( !APDS9960_Read8(APDS9960_GCONF2, &val) ) {
        return NOK;
    }

    /* Set bits in register to given value */
    gain &= 0x03;
    gain = gain << 5;
    val &= 0x9F;
    val |= gain;

    /* Write register value back into GCONF2 register */
    if( !APDS9960_Write8(APDS9960_GCONF2, val) ) {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_SetGestureExitThresh(uint8_t threshold)
{
    if( !APDS9960_Write8(APDS9960_GEXTH, threshold) ) {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_SetGestureEnterThresh(uint8_t threshold)
{
    if( !APDS9960_Write8(APDS9960_GPENTH, threshold) ) {
        return NOK;
    }

    return OK;
}


uint8_t enableGestureSensor(int interrupts)
{

    /* Enable gesture mode
       Set ENABLE to 0 (power off)
       Set WTIME to 0xFF
       Set AUX to LED_BOOST_300
       Enable PON, WEN, PEN, GEN in ENABLE
    */
	APDS9960_ResetGestureParameters();
    if( !APDS9960_Write8(APDS9960_WTIME, 0xFF) ) {
        return NOK;
    }
    if( !APDS9960_Write8(APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE) ) {
        return NOK;
    }
    if( !APDS9960_SetLEDBoost(LED_BOOST_300) ) {
        return NOK;
    }
    if( interrupts ) {
        if( !APDS9960_SetGestureIntEnable(1) ) {
            return NOK;
        }
    } else {
        if( !APDS9960_SetGestureIntEnable(0) ) {
            return NOK;
        }
    }
    if( !APDS9960_SetGestureMode(1) ) {
        return NOK;
    }
    if( !APDS9960_EnablePower() ){
        return NOK;
    }
    if( !APDS9960_SetMode(WAIT, 1) ) {
        return NOK;
    }
    if( !APDS9960_SetMode(PROXIMITY, 1) ) {
        return NOK;
    }
    if( !APDS9960_SetMode(GESTURE, 1) ) {
        return NOK;
    }

    return OK;
}


void APDS9960_ResetGestureParameters(void)
{
    gesture_data_.index = 0;
    gesture_data_.total_gestures = 0;

    gesture_ud_delta_ = 0;
    gesture_lr_delta_ = 0;

    gesture_ud_count_ = 0;
    gesture_lr_count_ = 0;

    gesture_near_count_ = 0;
    gesture_far_count_ = 0;

    gesture_state_ = 0;
    gesture_motion_ = DIR_NONE;
}


uint8_t APDS9960_SetLEDBoost(uint8_t boost)
{
    uint8_t val;

    /* Read value from CONFIG2 register */
    if( !APDS9960_Read8(APDS9960_CONFIG2, &val) )
    {
        return NOK;
    }

    /* Set bits in register to given value */
    boost &= 0x03;
    boost = boost << 4;
    val &= 0xCF;
    val |= boost;

    /* Write register value back into CONFIG2 register */
    if( !APDS9960_Write8(APDS9960_CONFIG2, val) )
    {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_SetGestureMode(uint8_t mode)
{
    uint8_t val;

    /* Read value from GCONF4 register */
    if( !APDS9960_Read8(APDS9960_GCONF4, &val) )
    {
        return NOK;
    }

    /* Set bits in register to given value */
    mode &= 0x01;
    val &= 0xFE;
    val |= mode;

    /* Write register value back into GCONF4 register */
    if( !APDS9960_Write8(APDS9960_GCONF4, val) )
    {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_EnablePower(void)
{
    if( !APDS9960_SetMode(POWER, 1) )
    {
        return NOK;
    }

    return OK;
}


uint8_t isGestureAvailable(void)
{
    uint8_t val;
    /* Read value from GSTATUS register */
    if( !APDS9960_Read8(APDS9960_GSTATUS, &val) )
    {
        return ERROR;
    }

    /* Shift and mask out GVALID bit */
    val &= APDS9960_GVALID;

    /* Return OK/NOK based on GVALID bit */
    if( val == 1)
    {
        return OK;
    }
    else
    {
        return NOK;
    }
}


uint8_t APDS9960_ReadGesture(void)
{
    uint8_t fifo_level = 0;
    int bytes_read = 0;
    uint8_t fifo_data[128];
    uint8_t gstatus;
    int motion;
    int i;

    /* Make sure that power and gesture is on and data is valid */
    if( !APDS9960_IsGestureAvailable() || !(APDS9960_GetMode() & 0x41) )
    {
        return DIR_NONE;
    }

    /* Keep looping as long as gesture data is valid */
    while(1) {

        /* Wait some time to collect next batch of FIFO data */
        delayms(FIFO_PAUSE_TIME);

        /* Get the contents of the STATUS register. Is data still valid? */
        if( !APDS9960_Read8(APDS9960_GSTATUS, &gstatus) )
        {
            return ERROR;
        }

        /* If we have valid data, read in FIFO */
        if( (gstatus & APDS9960_GVALID) == APDS9960_GVALID )
        {

            /* Read the current FIFO level */
            if( !APDS9960_Read8(APDS9960_GFLVL, &fifo_level) )
            {
                return ERROR;
            }

            /* If there's stuff in the FIFO, read it into our data block */
            if( fifo_level > 0)
            {
                bytes_read = APDS9960_Read(APDS9960_GFIFO_U, fifo_data, (fifo_level * 4) );
                if( bytes_read == -1 )
                {
                    return ERROR;
                }

                /* If at least 1 set of data, sort the data into U/D/L/R */
                if( bytes_read >= 4 )
                {
                    for( i = 0; i < bytes_read; i += 4 )
                    {
                        gesture_data_.u_data[gesture_data_.index] = fifo_data[i + 0];
                        gesture_data_.d_data[gesture_data_.index] = fifo_data[i + 1];
                        gesture_data_.l_data[gesture_data_.index] = fifo_data[i + 2];
                        gesture_data_.r_data[gesture_data_.index] = fifo_data[i + 3];
                        gesture_data_.index++;
                        gesture_data_.total_gestures++;
                    }

                    /* Reset data */
                    gesture_data_.index = 0;
                    gesture_data_.total_gestures = 0;
                }
            }
        }
        else
        {

            /* Determine best guessed gesture and clean up */
        	osDelay(FIFO_PAUSE_TIME);
        	APDS9960_DecodeGesture();
            motion = gesture_motion_;

            APDS9960_ResetGestureParameters();
            return motion;
        }
    }
}


uint8_t APDS9960_ProcessGestureData(void)
{
    uint8_t u_first = 0;
    uint8_t d_first = 0;
    uint8_t l_first = 0;
    uint8_t r_first = 0;
    uint8_t u_last = 0;
    uint8_t d_last = 0;
    uint8_t l_last = 0;
    uint8_t r_last = 0;
    int ud_ratio_first;
    int lr_ratio_first;
    int ud_ratio_last;
    int lr_ratio_last;
    int ud_delta;
    int lr_delta;
    int i;

    /* If we have less than 4 total gestures, that's not enough */
    if( gesture_data_.total_gestures <= 4 )
    {
        return NOK;
    }

    /* Check to make sure our data isn't out of bounds */
    if( (gesture_data_.total_gestures <= 32) && (gesture_data_.total_gestures > 0) )
    {
        /* Find the first value in U/D/L/R above the threshold */
        for( i = 0; i < gesture_data_.total_gestures; i++ )
        {
            if( (gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) )
            {

                u_first = gesture_data_.u_data[i];
                d_first = gesture_data_.d_data[i];
                l_first = gesture_data_.l_data[i];
                r_first = gesture_data_.r_data[i];
                break;
            }
        }

        /* If one of the _first values is 0, then there is no good data */
        if( (u_first == 0) || (d_first == 0) || (l_first == 0) || (r_first == 0) )
        {

            return NOK;
        }
        /* Find the last value in U/D/L/R above the threshold */
        for( i = gesture_data_.total_gestures - 1; i >= 0; i-- )
        {
            if( (gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) )
            {

                u_last = gesture_data_.u_data[i];
                d_last = gesture_data_.d_data[i];
                l_last = gesture_data_.l_data[i];
                r_last = gesture_data_.r_data[i];
                break;
            }
        }
    }

    /* Calculate the first vs. last ratio of up/down and left/right */
    ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
    lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
    ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last);
    lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last);


    /* Determine the difference between the first and last ratios */
    ud_delta = ud_ratio_last - ud_ratio_first;
    lr_delta = lr_ratio_last - lr_ratio_first;

    /* Accumulate the UD and LR delta values */
    gesture_ud_delta_ += ud_delta;
    gesture_lr_delta_ += lr_delta;


    /* Determine U/D gesture */
    if( gesture_ud_delta_ >= GESTURE_SENSITIVITY_1 )
    {
        gesture_ud_count_ = 1;
    }
    else if( gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1 )
    {
        gesture_ud_count_ = -1;
    }
    else
    {
        gesture_ud_count_ = 0;
    }

    /* Determine L/R gesture */
    if( gesture_lr_delta_ >= GESTURE_SENSITIVITY_1 )
    {
        gesture_lr_count_ = 1;
    }
    else if( gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1 )
    {
        gesture_lr_count_ = -1;
    }
    else
    {
        gesture_lr_count_ = 0;
    }

    /* Determine Near/Far gesture */
    if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 0) )
    {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && (ABS(lr_delta) < GESTURE_SENSITIVITY_2) )
        {

            if( (ud_delta == 0) && (lr_delta == 0) )
            {
                gesture_near_count_++;
            }
            else if( (ud_delta != 0) || (lr_delta != 0) )
            {
                gesture_far_count_++;
            }

            if( (gesture_near_count_ >= 10) && (gesture_far_count_ >= 2) )
            {
                if( (ud_delta == 0) && (lr_delta == 0) )
                {
                    gesture_state_ = NEAR_STATE;
                }
                else if( (ud_delta != 0) && (lr_delta != 0) )
                {
                    gesture_state_ = FAR_STATE;
                }
                return OK;
            }
        }
    }
    else
    {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && (ABS(lr_delta) < GESTURE_SENSITIVITY_2) )
        {

            if( (ud_delta == 0) && (lr_delta == 0) )
            {
                gesture_near_count_++;
            }

            if( gesture_near_count_ >= 10 )
            {
                gesture_ud_count_ = 0;
                gesture_lr_count_ = 0;
                gesture_ud_delta_ = 0;
                gesture_lr_delta_ = 0;
            }
        }
    }

    return NOK;
}


uint8_t APDS9960_DecodeGesture(void)
{
    /* Return if near or far event is detected */
    if( gesture_state_ == NEAR_STATE )
    {
        gesture_motion_ = DIR_NEAR;
        return OK;
    }
    else if ( gesture_state_ == FAR_STATE )
    {
        gesture_motion_ = DIR_FAR;
        return OK;
    }

    /* Determine swipe direction */
    if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 0) )
    {
        gesture_motion_ = DIR_UP;
    }
    else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 0) )
    {
        gesture_motion_ = DIR_DOWN;
    }
    else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 1) )
    {
        gesture_motion_ = DIR_RIGHT;
    }
    else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == -1) )
    {
        gesture_motion_ = DIR_LEFT;
    }
    else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 1) )
    {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) )
        {
            gesture_motion_ = DIR_UP;
        }
        else
        {
            gesture_motion_ = DIR_RIGHT;
        }
    }
    else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == -1) )
    {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) )
        {
            gesture_motion_ = DIR_DOWN;
        }
        else
        {
            gesture_motion_ = DIR_LEFT;
        }
    }
    else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == -1) )
    {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) )
        {
            gesture_motion_ = DIR_UP;
        }
        else
        {
            gesture_motion_ = DIR_LEFT;
        }
    }
    else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 1) )
    {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) )
        {
            gesture_motion_ = DIR_DOWN;
        }
        else
        {
            gesture_motion_ = DIR_RIGHT;
        }
    }
    else
    {
        return NOK;
    }

    return OK;
}


uint8_t APDS9960_ReadSensor(void)
{
	int Gesture = 0;

	if(APDS9960_IsGestureAvailable())
	{
		Gesture = APDS9960_ReadGesture();
	}
	return Gesture;
}
