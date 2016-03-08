/*******************************************************************************
 * Copyright (C) 2014 HiDeep, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *******************************************************************************/

#include "ist520e.h"    // TODO : rename name..

#define  IST520E_UC  "crimson.bin"

struct ist510e *g_istxxxx_data;
extern struct ts_data g_ts_data;

//0x00: menu key
//0x01: home key
//0x02: back key
#ifdef IST510E_KEYBUTTON
static int tsp_keycodes[] = {KEY_MENU, KEY_HOMEPAGE, KEY_BACK};
#endif

#define CHECK_ENABLE_GESTURE(A)	((u32)(1<<A))
int huawei_ts_i2c_read(u8 *reg_addr, u16 reg_len, u8 *buf, u16 len)
{
	int count = 0;
	int ret;
	struct i2c_msg xfer[2];

	/*register addr*/
	xfer[0].addr = g_ts_data.client->addr;
	xfer[0].flags = 0;
	xfer[0].len = reg_len;
	xfer[0].buf = reg_addr;

	/* Read data */
	xfer[1].addr = g_ts_data.client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = buf;

	count = 0;
	do {
		ret = i2c_transfer(g_ts_data.client->adapter, &xfer[0], 1);
		if (ret == 1){
			goto reading_data;
		}
		msleep(I2C_WAIT_TIME);
	} while (++count < I2C_RW_TRIES);

reading_data:
	count = 0;
	do {
		ret = i2c_transfer(g_ts_data.client->adapter, &xfer[1], 1);
		if (ret == 1) {
			return NO_ERR;
		}
do_retry:
		msleep(I2C_WAIT_TIME);
	} while (++count < I2C_RW_TRIES);
	TS_LOG_ERR("ts_i2c_read failed\n");
	return -EIO;
}

int hideep_i2c_read(struct ist510e *ts, u16 addr, u16 len, u8 *buf)
{
	int ret = -1;
	int i;

	TS_LOG_DEBUG("hideep_i2c_read:addr = 0x%04x,len = %d\n",addr,len);
	mutex_lock(&ts->i2c_mutex);
	ret = huawei_ts_i2c_read((u8*)&addr, 2, buf, len);
	//ret = ts->huawei_ts_data->bops->bus_read();
	if(ret < 0)
	    goto i2c_err;

	mutex_unlock(&ts->i2c_mutex);
	return  0;

i2c_err:
	mutex_unlock(&ts->i2c_mutex);
	TS_LOG_ERR("i2c read error, ret = %d\n,ret");
	return -1;
}

int hideep_i2c_write(struct ist510e *ts, u16 addr, u16 len, u8 *buf)
{
	int ret = -1;
	int i;

	TS_LOG_DEBUG("hideep_i2c_write:addr = 0x%04x,len = %d\n",addr,len);
	if(g_ts_log_cfg>=1){
		for(i=0;i<len;i++){
			if(i%16==0){
				if(i!=0)
					printk("\n");
				printk("%s:%d:buf[0x%04x]\n",__FUNCTION__,__LINE__,i);
			}
				printk("0x%02x\n",buf[i]);
		}
			printk("\n");
	}
	mutex_lock(&ts->i2c_mutex);
	ts->seg_buff[0] = (addr >> 0) & 0xFF;
	ts->seg_buff[1] = (addr >> 8) & 0xFF;
	memcpy(ts->seg_buff+2, buf, len);
	ret = ts->huawei_ts_data->bops->bus_write(ts->seg_buff, 2+len);
	if(ret < 0){
		TS_LOG_ERR("i2c write address error\n");
		goto i2c_err;
	}
	mutex_unlock(&ts->i2c_mutex);
	return  0;

i2c_err:
	mutex_unlock(&ts->i2c_mutex);
	TS_LOG_ERR("i2c write error, ret = %d\n",ret);    
	return -1;
}

static int hideep_item_switch(u8 rw, u32 addr, u32 len, u8* buffer)
{
	s32 ret = 0;
	struct ist510e *ts   = g_istxxxx_data;

	switch (rw) 
	{
		case TS_ACTION_WRITE:
			ret = hideep_i2c_write(ts, addr, len, buffer);
			break;
		case TS_ACTION_READ :
			ret = hideep_i2c_read (ts, addr, len, buffer);
			break;
		default:
			ret = -EINVAL;
			break;
	}

	return ret ;
}

static s32 hideep_get_event(struct ist510e *ts)
{
	s32 ret;
	u8 i2c_buff[2];
	s32 touch_count = 0;
	uint32_t info_size;
	
	TS_LOG_DEBUG("hideep_get_event enter\n");
#ifdef HIDEEP_READ_I2C_CNT_DATA
	ret = hideep_i2c_read(ts, TOUCH_COUNT_ADDR, TOUCH_MAX_COUNT*sizeof(struct ist510e_touch_evt)+2, (u8*)ts->i2c_buf);
	memcpy((u8*)i2c_buff,(u8*)ts->i2c_buf,2);
#else
	ret = hideep_i2c_read(ts, TOUCH_COUNT_ADDR, 2, (u8*)&i2c_buff);
#endif
	if(ret < 0)
		goto i2c_err;
	ts->input_touch_nr = i2c_buff[0];
	ts->input_key_nr = i2c_buff[1]&0x0f;
	ts->input_event_nr = ((i2c_buff[1]&0xf0)>>4);
	TS_LOG_DEBUG("enter buf[0] = 0x%02x, buf[1] = 0x%02x\n", i2c_buff[0], i2c_buff[1]);
	TS_LOG_DEBUG("touch = %d, key = %d, event = %d \n", ts->input_touch_nr, ts->input_key_nr, ts->input_event_nr);
	if( ts->input_touch_nr >TOUCH_MAX_COUNT || ts->input_key_nr >KEYS_MAX_COUNT){
		TS_LOG_ERR("over size error touch = %d, key = %d\n",ts->input_touch_nr,ts->input_key_nr);
		ts->input_touch_nr = 0;
		ts->input_key_nr = 0;
	}
	touch_count = ts->input_touch_nr + ts->input_key_nr;
	if (ts->input_touch_nr > 0){
		info_size = ts->input_touch_nr *sizeof(struct ist510e_touch_evt);
#ifdef HIDEEP_READ_I2C_CNT_DATA
		memcpy((u8*)ts->input_evt,(u8*)(ts->i2c_buf+2),ts->input_touch_nr *sizeof(struct ist510e_touch_evt));
#else
		ret = hideep_i2c_read(ts, TOUCH_READ_START_ADDR, info_size, (u8*)ts->input_evt);
#endif
		if (ret < 0){
			goto i2c_err;
		}
#ifdef HIDEEP_TRACE_TOUCH_EVENT	//for debug for ghost and missing point
		if(ts->debug_evt){
			ts->debug_index = ts->debug_index>=MAX_DEBUG_COUNT?0:ts->debug_index;
			ts->debug_evt[ts->debug_index].count = ts->input_touch_nr;
			ktime_get_ts(&ts->event_time[ts->debug_index]);
			memcpy(&ts->debug_evt[ts->debug_index].events[0],(u8*)ts->input_evt,info_size);
			ts->debug_index++;
			ts->debug_total++;
			ts->debug_total = ts->debug_total>2000000?2000000:ts->debug_total;
		}
#endif
	}
	if (ts->input_key_nr > 0){
		uint32_t info_size = ts->input_key_nr*sizeof(struct ist510e_touch_key);
		ret = hideep_i2c_read(ts, KEY_READ_START_ADDR, info_size, (u8*)ts->input_key);
		if (ret < 0){
			goto i2c_err;
		}
	}
	return touch_count;
i2c_err:
	TS_LOG_ERR("hideep_get_event i2c err\n");
	return -1;
}

#ifdef ISTCORE_IF_DEVICE
static s32 hideep_get_image(struct ist510e *ts)
{
	s32 ret = 0;
	int i;
	struct i2c_client *client = ts->client;
	struct ist510e_debug_dev  *debug_dev = &ts->debug_dev;

	TS_LOG_DEBUG("hideep_get_image enter\n");
	ret = hideep_i2c_read(ts, VR_ADDR_IMAGE, debug_dev->im_size, debug_dev->im_buff);
	if(ret < 0)
		goto i2c_err;
	TS_LOG_DEBUG( "load image from sensor(%d)\n", debug_dev->im_size);

	return ret;
i2c_err:
	TS_LOG_ERR("error!\n");
	return ret;
}
#endif

static int hideep_capability(struct input_dev *input_dev, struct ts_device_data *ts_dev_data)
{
	u32 i = 0;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, input_dev->keybit);

	set_bit(TS_DOUBLE_CLICK, input_dev->keybit);
	set_bit(TS_SLIDE_L2R, input_dev->keybit);
	set_bit(TS_SLIDE_R2L, input_dev->keybit);
	set_bit(TS_SLIDE_T2B, input_dev->keybit);
	set_bit(TS_SLIDE_B2T, input_dev->keybit);
	set_bit(TS_CIRCLE_SLIDE, input_dev->keybit);
	set_bit(TS_LETTER_c, input_dev->keybit);
	set_bit(TS_LETTER_e, input_dev->keybit);
	set_bit(TS_LETTER_m, input_dev->keybit);
	set_bit(TS_LETTER_w, input_dev->keybit);
	set_bit(TS_PALM_COVERED, input_dev->keybit);

	set_bit(TS_TOUCHPLUS_KEY0, input_dev->keybit);
	set_bit(TS_TOUCHPLUS_KEY1, input_dev->keybit);
	set_bit(TS_TOUCHPLUS_KEY2, input_dev->keybit);
	set_bit(TS_TOUCHPLUS_KEY3, input_dev->keybit);
	set_bit(TS_TOUCHPLUS_KEY4, input_dev->keybit);

#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#endif

#ifdef IST510E_KEYBUTTON
	for (i = 0 ; i < ARRAY_SIZE(tsp_keycodes); i++)
		set_bit(tsp_keycodes[i], input_dev->keybit);
#endif
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,  0, ts_dev_data->x_max_mt, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,  0, ts_dev_data->y_max_mt, 0, 0);
	// z value's range.
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,  0, 65535,0, 0);
	 // w value's range.
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255,0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, TOUCH_MAX_COUNT ,0, 0);
	return 0;
}

static int hideep_get_version(struct ist510e *ts)
{
	s32 ret = 0;
	DWZ_INFO_T  *dmz_info = &ts->dwz_info;

	hideep_reset_ic();
	mdelay(100);
	ret = hideep_i2c_read(ts, VR_VERSION_ADDR, 2, &dmz_info->pannel.version);	//ic fw version
	if(ret < 0)
		goto i2c_err;
	TS_LOG_INFO("vr version : %04x\n", dmz_info->pannel.version);
	
	return 0;
i2c_err:
	TS_LOG_ERR("hideep_get_version error\n");
	return ret;
}

static int hideep_regulator_vci_enable(struct ist510e *ts_drv)
{
	int ret = 0;
	int vol_vlaue;

	vol_vlaue = HIDEEP_VCI_LDO_JDI_VALUE;
	ret = regulator_set_voltage(ts_drv->tp_vci, vol_vlaue, vol_vlaue);
	if(ret < 0) {
		TS_LOG_ERR("tp_vci voltage error\n");
		return -EINVAL;
	}
	ret = regulator_enable(ts_drv->tp_vci);
	if (ret < 0) {
		TS_LOG_ERR("tp_vci enable  error\n");
		return -EINVAL;
	}

	return NO_ERR;
}


/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static void hideep_regulator_put(struct ist510e *ts_drv)
{
	struct ts_device_data* dat = ts_drv->huawei_ts_data;

	TS_LOG_INFO("hideep_regulator_put enter.\n");
	if (dat->vci_regulator_type  == 1 && !IS_ERR(ts_drv->tp_vci )) {
		regulator_disable(ts_drv->tp_vci);
		regulator_put(ts_drv->tp_vci);
	}
	if (dat->vddio_regulator_type  == 1 && !IS_ERR(ts_drv->tp_vddio)) 	{            
		regulator_disable(ts_drv->tp_vddio);
		regulator_put(ts_drv->tp_vddio);
	}

	return;
}

static int hideep_regulator_get(struct ist510e *ts_drv)
{
	s32 ret = 0;
	struct ts_device_data* dat = ts_drv->huawei_ts_data;
	struct platform_device *platform_dev = ts_drv->huawei_ts_dev;

	TS_LOG_INFO("hideep_regulator_get enter.\n");
	// VCI
	if (dat->vci_regulator_type == 1) {
		ts_drv->tp_vci = regulator_get(&platform_dev->dev, HIDEEP_VDD);
		if (IS_ERR(ts_drv->tp_vci)) {
			TS_LOG_ERR("regulator_get:tp_vci error\n");
			goto err;
		}
		ret = hideep_regulator_vci_enable(ts_drv);
		if(ret < 0){
			TS_LOG_ERR("regulator_enable:tp_vci error\n");
			goto err;
		}
	}
	mdelay(5);
	// VDDIO
	if (dat->vddio_regulator_type == 1) {
		ts_drv->tp_vddio = regulator_get(&platform_dev->dev, HIDEEP_VBUS);
		if (IS_ERR(ts_drv->tp_vddio)) {
			TS_LOG_ERR("regulator_get :tp_vddio error\n");
			goto err;
		}
		ret = regulator_enable(ts_drv->tp_vddio);
		if (ret < 0) {
			TS_LOG_ERR("regulator_enable :tp_vddio error\n");        
			goto err;
		}
	}
	return NO_ERR;    
err:
	return ret;
}

static int hideep_pinctrl_get(struct ist510e *ts_drv)
{
	int ret = 0;

	ts_drv->pctrl = devm_pinctrl_get(&ts_drv->huawei_ts_dev->dev);
	if (IS_ERR(ts_drv->pctrl)) {
		TS_LOG_ERR("failed to devm pinctrl get\n");
		ret = -EINVAL;
		return ret;
	}

	ts_drv->pins_default = pinctrl_lookup_state(ts_drv->pctrl, "default");
	if (IS_ERR(ts_drv->pins_default)) {
		TS_LOG_ERR("failed to pinctrl lookup state default\n");
		ret = -EINVAL;
		goto err_pinctrl_put;
	}

	ts_drv->pins_idle  = pinctrl_lookup_state(ts_drv->pctrl, "idle");
	if (IS_ERR(ts_drv->pins_idle)) {
		TS_LOG_ERR("failed to pinctrl lookup state idle\n");
		ret = -EINVAL;
		goto err_pinctrl_put;
	}

	return 0;
err_pinctrl_put:
	devm_pinctrl_put(ts_drv->pctrl);
	return ret;
}

static void hideep_gpio_put(struct ist510e *ts_drv)
{
	struct ts_device_data *dat   = ts_drv->huawei_ts_data;

	gpio_free(dat->reset_gpio);
	gpio_free(dat->irq_gpio);
	gpio_free(dat->vci_gpio_ctrl);
	gpio_free(dat->vddio_gpio_ctrl);
	if(!IS_ERR(ts_drv->pins_default))
		devm_pinctrl_put(ts_drv->pctrl);

	return;
}
static int hideep_gpio_get(struct ist510e *ts_drv)
{
	s32 ret = 0;
	struct ts_device_data *dat   = ts_drv->huawei_ts_data;

	// request  gpio resources
	ret = gpio_request(dat->reset_gpio, "ts_reset_gpio\n");
	if (ret < 0) {
		TS_LOG_ERR("gpio : ts_reset_gpio\n");
		goto err;
	}
	ret = gpio_request(dat->irq_gpio, "ts_irq_gpio\n");
	if (ret < 0) {
		TS_LOG_ERR("gpio : ts_irq_gpio\n");
		goto err;
	}
	ret = gpio_request(dat->vci_gpio_ctrl, "ts_vci_gpio\n");
	if (ret < 0) {
		TS_LOG_ERR("gpio : ts_vci_gpio\n");
		goto err;
	}
	if(dat->vci_gpio_ctrl != dat->vddio_gpio_ctrl){
		ret = gpio_request(dat->vddio_gpio_ctrl, "ts_vddio_gpio\n");
		if (ret < 0) {
			TS_LOG_ERR("gpio : ts_vddio_gpio\n");
			goto err;
		}
	}
	ret = hideep_pinctrl_get(ts_drv);
	if(ret <0){
		goto err;
	}
	
	return NO_ERR;
err:
	hideep_gpio_put(ts_drv);
	return ret;
}

static void hideep_power_on(struct ist510e *ts_drv)
{
	s32 ret;
	struct ts_device_data *dat = ts_drv->huawei_ts_data;

	TS_LOG_INFO("hideep_power_on enter\n");
	ret = pinctrl_select_state(ts_drv->pctrl, ts_drv->pins_default);
	if (ret < 0){
		TS_LOG_ERR("set iomux normal error, %d\n", ret);
	}

	gpio_direction_input(dat->irq_gpio);
	gpio_direction_output(dat->reset_gpio, 1);
	if(dat->vci_gpio_type == 1){
		TS_LOG_INFO("VCI type = 1\n");
		gpio_direction_output(dat->vci_gpio_ctrl, 1);
		msleep(5);
	}

	if(dat->vddio_gpio_type == 1){
		TS_LOG_INFO("VDDIO type = 1\n");
		gpio_direction_output(dat->vddio_gpio_ctrl, 1);
		msleep(5);
	}
	return;
}

static void hideep_power_off(struct ist510e *ts_drv)
{
	s32 ret;
	struct ts_device_data *dat = ts_drv->huawei_ts_data;

	TS_LOG_INFO("hideep_power_off enter\n");
	ret = pinctrl_select_state(ts_drv->pctrl, ts_drv->pins_idle);
	if (ret < 0){
		TS_LOG_ERR("set iomux idel error, %d\n", ret);
	}
	gpio_direction_input(dat->irq_gpio);
	gpio_direction_output(dat->reset_gpio, 1);
	if(dat->vci_gpio_type == 1){
		gpio_direction_output(dat->vci_gpio_ctrl , 0);
		msleep(5);
	}
	if(dat->vddio_gpio_type == 1){
		gpio_direction_output(dat->vddio_gpio_ctrl, 0);
		msleep(5);
	}

	return;
}

void hideep_reset_ic(void)
{
	struct ist510e *ts_drv = g_istxxxx_data;
	struct ts_device_data *dat  = ts_drv->huawei_ts_data;

	gpio_direction_output(dat->reset_gpio, 1);
	mdelay(20);
	gpio_direction_output(dat->reset_gpio, 0);
	mdelay(20);
	gpio_direction_output(dat->reset_gpio, 1);

	return ;
}
static void hideep_uninit_hw(struct ist510e *ts_drv)
{
	hideep_gpio_put(ts_drv);
	hideep_regulator_put(ts_drv);
}

static int hideep_init_hw(struct ist510e *ts_drv)
{
	s32 ret = 0;

	TS_LOG_INFO("hideep_init_hw\n");

	ret = hideep_regulator_get(ts_drv);
	if (ret < 0) {
		TS_LOG_ERR("regulator error\n"); 
		goto err_rgt;
	}

	ret = hideep_gpio_get(ts_drv);
	if (ret < 0) {
		TS_LOG_ERR("gpio get  error\n"); 
		goto err_gpio;
	}

	hideep_power_on(ts_drv);
	hideep_reset_ic();

	return NO_ERR;
err_gpio:
	hideep_gpio_put(ts_drv);
err_rgt:
	hideep_regulator_put(ts_drv);
	return ret;
}

static void hideep_dump_config(struct ts_device_data *data)
{
	TS_LOG_INFO("irq_gpio             : %d\n",data->irq_gpio  );
	TS_LOG_INFO("reset_gpio           : %d\n",data->reset_gpio);
	TS_LOG_INFO("vci_gpio_ctrl        : %d\n",data->vci_gpio_ctrl);
	TS_LOG_INFO("vddio_gpio_ctrl      : %d\n",data->vddio_gpio_ctrl);    
	TS_LOG_INFO("vci_regulator_type   : %d\n",data->vci_regulator_type);
	TS_LOG_INFO("vddio_regulator_type : %d\n",data->vddio_regulator_type);
	TS_LOG_INFO("x_mt            : %d\n",data->x_max_mt);
	TS_LOG_INFO("y_mt            : %d\n",data->y_max_mt);
	TS_LOG_INFO("ic_type         : %d\n",data->ic_type);

	return;
}

static int hideep_parse_config(struct device_node *device, struct ts_device_data *data)
{
	s32 ret = 0;

	TS_LOG_INFO("hideep_parse_config enter\n");
	ret  = of_property_read_u32(device, HIDEEP_IRQ_CFG, &data->irq_config);
	ret |= of_property_read_u32(device, HIDEEP_ALGO_ID, &data->algo_id);
	ret |= of_property_read_u32(device, HIDEEP_IC_TYPES,  &data->ic_type);
	ret |= of_property_read_u32(device, HIDEEP_X_MAX_MT,  &data->x_max_mt);
	ret |= of_property_read_u32(device, HIDEEP_Y_MAX_MT, &data->y_max_mt);
	ret |= of_property_read_u32(device, HIDEEP_X_MAX, &data->x_max);
	ret |= of_property_read_u32(device, HIDEEP_Y_MAX,	&data->y_max);
	ret |= of_property_read_u32(device, HIDEEP_VCI_GPIO_TYPE, &data->vci_gpio_type);
	ret |= of_property_read_u32(device, HIDEEP_VCI_REGULATOR_TYPE, &data->vci_regulator_type);
	ret |= of_property_read_u32(device, HIDEEP_VDDIO_GPIO_TYPE, &data->vddio_gpio_type);
	ret |= of_property_read_u32(device, HIDEEP_VDDIO_REGULATOR_TYPE, &data->vddio_regulator_type);

	if (ret) {
		goto err;
	}
	if(data->vci_gpio_type == 1) {
		data->vci_gpio_ctrl = of_get_named_gpio(device, HIDEEP_VCI_GPIO_CTRL, 0);
		if (!gpio_is_valid(data->vci_gpio_ctrl)) {
			goto err;
		}
	}
	if(data->vddio_gpio_type == 1) 	{
		data->vddio_gpio_ctrl = of_get_named_gpio(device, HIDEEP_VDDIO_GPIO_CTRL, 0);
		if (!gpio_is_valid(data->vddio_gpio_ctrl)) {
			goto err;
		}
	}
	data->irq_gpio = of_get_named_gpio(device,   HIDEEP_IRQ_GPIO, 0);
	if (!gpio_is_valid(data->irq_gpio)) {
		goto err;
	}
	data->reset_gpio = of_get_named_gpio(device, HIDEEP_RST_GPIO, 0);
	if (!gpio_is_valid(data->reset_gpio)) {
		goto err;
	}
	hideep_dump_config(data);
	TS_LOG_INFO(" hideep_parse_config OK.\n");

	return NO_ERR;
err:
	TS_LOG_ERR("hideep_parse_config failed\n");    
	return -EINVAL;;
}

static int hideep_input_config(struct input_dev *input_dev)
{
	struct ist510e *ts_drv =  g_istxxxx_data;

	TS_LOG_INFO("hideep_input_config enter\n");
	ts_drv->input_dev = input_dev;
	hideep_capability(ts_drv->input_dev, ts_drv->huawei_ts_data);

	return NO_ERR;
}

static int hideep_chip_detect(struct device_node *device, struct ts_device_data *ts_data, struct platform_device *ts_dev)
{
	s32 ret = 0;

	TS_LOG_INFO("hideep_chip_detect enter\n");

	g_istxxxx_data = kzalloc(sizeof(struct ist510e), GFP_KERNEL);
	if(g_istxxxx_data == NULL) {
		TS_LOG_ERR("g_istxxxx_data alloc error\n");
		goto err;
	}

	g_istxxxx_data->huawei_ts_data = ts_data;
	g_istxxxx_data->huawei_ts_dev = ts_dev ;
	g_istxxxx_data->huawei_ts_dev->dev.of_node = device;                //for dsti reference
	g_istxxxx_data->client = ts_data->client;
	g_istxxxx_data->dev_state = ISTCORE_PWR_NORMAL;
	g_istxxxx_data->irq = gpio_to_irq(ts_data->irq_gpio);
	g_istxxxx_data->tch_bit = 0;
	g_istxxxx_data->huawei_ts_data->easy_wakeup_info.sleep_mode = TS_POWER_OFF_MODE;
	g_istxxxx_data->huawei_ts_data->easy_wakeup_info.easy_wakeup_gesture = false;
	g_istxxxx_data->huawei_ts_data->easy_wakeup_info.easy_wakeup_flag = false;
	g_istxxxx_data->huawei_ts_data->easy_wakeup_info.palm_cover_flag = false;
	g_istxxxx_data->huawei_ts_data->easy_wakeup_info.palm_cover_control = false;
	g_istxxxx_data->huawei_ts_data->easy_wakeup_info.off_motion_on = false;
	g_istxxxx_data->glove_mode = 0;
	g_istxxxx_data->holster_mode = 0;

	mutex_init(&g_istxxxx_data->i2c_mutex);
	mutex_init(&g_istxxxx_data->dev_mutex);

	// prepare hardware for ic detect
	ret = hideep_init_hw(g_istxxxx_data);
	if(ret < 0) {
		TS_LOG_ERR("hardware init error\n");
		goto err;
	}
#ifdef CRIMSON	//ic name
	ret = hideep_get_version(g_istxxxx_data);
	if(ret < 0) {
		TS_LOG_ERR("no ic is detected\n");
		goto err_aux;
	}
#endif

    // add debug device interface..
	TS_LOG_INFO("detect ok.\n");
	return NO_ERR;
err_aux:
	hideep_uninit_hw(g_istxxxx_data);
err:    
	if (g_istxxxx_data)
		kfree(g_istxxxx_data);
	return ret;
}

static int hideep_reset(void)
{
	hideep_reset_ic();

	return NO_ERR;
}

void hideep_shutdown(void)
{
	struct ist510e *ts = g_istxxxx_data;

	hideep_power_off(ts);
	hideep_gpio_put(ts);
	return;
}

static int hideep_init(void)
{
	struct ist510e *ts_drv =  g_istxxxx_data;
	s32 ic_type  =  ts_drv->huawei_ts_data->ic_type;
	int ret = 0;

	TS_LOG_INFO("hideep_init enter\n");
	switch(ic_type)
	{
		case IST_GREEN:
			break;
		case IST_CRIMSON:
#ifdef ISTCORE_IF_DEVICE	//debug device
			ret = hideep_iface_init(g_istxxxx_data);
			if(ret < 0) {
				TS_LOG_ERR("debug interface device error\n");    
				goto err_deivce;
			}

			ret = hideep_sysfs_init(g_istxxxx_data);
			if (ret != 0)
				goto err_deivce;
			break;
#endif
		case IST_INDIGO :
			break;
		default:
			break;
	}
	return NO_ERR;

#ifdef ISTCORE_IF_DEVICE
err_deivce:
	hideep_iface_uninit(g_istxxxx_data);
	return ret;
#endif
}

static int hideep_irq_top_half(struct ts_cmd_node *cmd)
{
	cmd->command = TS_INT_PROCESS;
	return NO_ERR;
}

static int hideep_irq_bottom_half(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	s32 t_evt;
	s32 ret = 0;
	u32 x,y;
	u8 gesture[25];
	u8 command;
	s32 report_coordinate;
	struct ist510e *ts =  g_istxxxx_data;
#ifdef ISTCORE_IF_DEVICE	//for debug
	struct ist510e_debug_dev  *debug_dev = &ts->debug_dev;
#endif
	struct ts_fingers *info = &out_cmd->cmd_param.pub_params.algo_param.info;
	s32 i;
	s32 id;
	s16 * ps;
	bool  btn_up ;
	bool  btn_dn ;
	bool  btn_mv ;
	struct ist510e_touch_evt *finger;
	struct ts_easy_wakeup_info *gesture_info;

	out_cmd->cmd_param.pub_params.algo_param.algo_order = ts->huawei_ts_data->algo_id;
	if(ts->dev_state != ISTCORE_PWR_NORMAL){
		out_cmd->command = TS_INVAILD_CMD;
		goto ret;
	}

#ifdef ISTCORE_IF_DEVICE
	if(debug_dev->im_r_en == 1) {
		mutex_lock  (&ts->dev_mutex);
		out_cmd->command = TS_INVAILD_CMD;
		hideep_get_image(ts);
		ts->debug_dev.i_rdy = 1;                                                  // TODO : need semaphore..
		wake_up_interruptible(&ts->debug_dev.i_packet);
		mutex_unlock(&ts->dev_mutex);
		return NO_ERR;
	}
#endif
	t_evt = hideep_get_event(ts);			//event count
	gesture_info = &g_istxxxx_data->huawei_ts_data->easy_wakeup_info;
	if(TS_GESTURE_MODE == gesture_info->sleep_mode){
		TS_LOG_DEBUG("ts->input_event_nr = %d\n",ts->input_event_nr);
		if (true == gesture_info->off_motion_on){
			if(ts->input_event_nr !=1){
				goto ret;
			}
			info->gesture_wakeup_value = 0;
			ret = hideep_i2c_read(ts, TOUCH_READ_START_ADDR+TOUCH_MAX_COUNT*sizeof(struct ist510e_touch_evt)+12, 26, (u8*)gesture);
			TS_LOG_DEBUG("gesture = 0x%02x\n",gesture[0]);
			switch(gesture[0]){
			case HIDEEP_GES_DOUBLE_TAP:
				if (CHECK_ENABLE_GESTURE(GESTURE_DOUBLE_CLICK) & gesture_info->easy_wakeup_gesture) {
					TS_LOG_INFO("double tap wake up.\n");
					info->gesture_wakeup_value = TS_DOUBLE_CLICK;
				}
				report_coordinate = 2;
				break;
#ifdef HIDEEP_LPM_CHARACTOR
			case HIDEEP_GES_CHAR:
				TS_LOG_INFO("character wake up.\n");
				report_coordinate = 6;
				switch(gesture[1]){
					case HIDEEP_CHAR_C:
						if (CHECK_ENABLE_GESTURE(GESTURE_LETTER_c) & gesture_info->easy_wakeup_gesture) {
							info->gesture_wakeup_value = TS_LETTER_c;
						}
						break;
					case HIDEEP_CHAR_E:
						if (CHECK_ENABLE_GESTURE(GESTURE_LETTER_e) & gesture_info->easy_wakeup_gesture) {
							info->gesture_wakeup_value = TS_LETTER_e;
						}
						break;
					case HIDEEP_CHAR_M:
						if (CHECK_ENABLE_GESTURE(GESTURE_LETTER_m) & gesture_info->easy_wakeup_gesture) {
							info->gesture_wakeup_value = TS_LETTER_m;
						}
						break;
					case HIDEEP_CHAR_W:
						if (CHECK_ENABLE_GESTURE(GESTURE_LETTER_w) & gesture_info->easy_wakeup_gesture) {
							info->gesture_wakeup_value = TS_LETTER_w;
						}
						break;
					default:
						TS_LOG_ERR("incorrect character.\n");
						break;
				}
				TS_LOG_INFO("character wake up.\n");
				break;
#endif
			default:
				TS_LOG_ERR("incorrect ID.\n");
				break;
			}
			TS_LOG_INFO("info->gesture_wakeup_value = %d\n",info->gesture_wakeup_value);
			if(info->gesture_wakeup_value!=0){
				gesture_info->off_motion_on = false;
				ps = &gesture[2];
				for(i = 0;i<report_coordinate;i++){	
					x = *ps++;
					y = *ps++;
					TS_LOG_INFO("x[%d] = %d, y[%d] = %d\n",i,x,i,y);
					gesture_info->easywake_position[i] = (x<<16)+y;
				}
				//command = 0x10;
				//hideep_item_switch(TS_ACTION_WRITE, VR_LPWU, 1, &command);
				ts->input_touch_nr = 0;
			}else{
				//incorrect gesture wake up ID. let IC sleep again.
				out_cmd->command = TS_INVAILD_CMD;
				command = 1;
				hideep_item_switch(TS_ACTION_WRITE, VR_LPWU, 1, &command);
				command = 0;
				hideep_i2c_write(ts, IST_ENTER_SANDMAN, 1, &command);
				goto ret;
			}
		}
	}

	for(i = 0; i < TOUCH_MAX_COUNT; i++) {		// memset 0
		info->fingers[i].status = 0;
	}

	finger = ts->input_evt;
	info->cur_finger_number = ts->input_touch_nr;
	for(i = 0; i < info->cur_finger_number; i++){
		id = (finger[i].index >> 0) & 0x0F;
		btn_up  = (finger[i].flag  >> EV_RELEASED) & 0x01;
		btn_dn  = (finger[i].flag  >> EV_FIRST_CONTACT) & 0x01;
		btn_mv  = (finger[i].flag  >> EV_DRAG_MOVE) & 0x01;
		if((id>=TOUCH_MAX_COUNT)||(id<0))
			continue;
		if(finger[i].z == 0)
			if(btn_dn | btn_mv)
				finger[i].z = 2;
		if(btn_up){
			info->fingers[id].status = TS_FINGER_RELEASE;
			info->fingers[id].x = finger[i].x;
			info->fingers[id].y = finger[i].y;
		}else if(btn_dn | btn_mv){
			info->fingers[id].status = TS_FINGER_PRESS;
			info->fingers[id].x = finger[i].x;
			info->fingers[id].y = finger[i].y;
			info->fingers[id].major = finger[i].w;
			info->fingers[id].pressure = finger[i].z;
		}
	}
	out_cmd->command = TS_INPUT_ALGO;
ret:    
	return NO_ERR;
}

static int hideep_suspend(void)
{
	struct ist510e *ts = g_istxxxx_data;
	u8 sleep_cmd = 0;
	u8 gesture = 0;
	struct ts_easy_wakeup_info *gesture_info;

	TS_LOG_INFO("suspend\n");
	gesture_info = &g_istxxxx_data->huawei_ts_data->easy_wakeup_info;
	switch (gesture_info->sleep_mode) {
		case TS_POWER_OFF_MODE:
			if(ts->dev_state == ISTCORE_PWR_SLEEP)
				goto ret;
			ts->dev_state = ISTCORE_PWR_SLEEP;
			break;
		/*for gesture wake up mode suspend.*/
		case TS_GESTURE_MODE:
			TS_LOG_INFO("enter gesture mode.\n");
			if (false == gesture_info->off_motion_on)
				gesture_info->off_motion_on = true;
			gesture = 1;
			hideep_item_switch(TS_ACTION_WRITE, VR_LPWU, 1, &gesture);
			gesture_info->easy_wakeup_flag = true;
			break;
		default:
			TS_LOG_ERR("no suspend.\n");
			return -EINVAL;
	}
	TS_LOG_INFO("suspend end.\n");
#ifdef CRIMSON
	hideep_i2c_write(ts, IST_ENTER_SANDMAN, 1, &sleep_cmd);
#endif  
ret:
	return NO_ERR;
}

static int hideep_resume(void)
{
	struct ist510e *ts = g_istxxxx_data;
	u8 gesture;
	struct ts_easy_wakeup_info *gesture_info;

	TS_LOG_INFO("hideep_resume\n");
	gesture_info = &g_istxxxx_data->huawei_ts_data->easy_wakeup_info;
	switch (gesture_info->sleep_mode) {
		case TS_POWER_OFF_MODE:
			if(ts->dev_state == ISTCORE_PWR_NORMAL)
				goto ret;
			ts->dev_state = ISTCORE_PWR_NORMAL  ;
#ifdef CRIMSON
			hideep_reset_ic(); 
			msleep(50);
#endif
			break;
		case TS_GESTURE_MODE:
			TS_LOG_INFO("exit gesture mode.\n");
			hideep_reset_ic(); 
			msleep(50);
			gesture = 0;
			hideep_item_switch(TS_ACTION_WRITE, VR_LPWU, 1, &gesture);
			if(gesture_info->off_motion_on)
				gesture_info->off_motion_on = false;
			gesture_info->easy_wakeup_flag = false;
			break;
		default:
			TS_LOG_ERR("no resume.\n");
			return -EINVAL;
	}
	TS_LOG_INFO("resume end.\n");
	hideep_item_switch(TS_ACTION_WRITE, VR_GLOVE, 1, &ts->glove_mode);
	//---------------------------------
ret:
	return NO_ERR;
}


static int hideep_fw_update_sd(void)
{
	s32 ret = 0;
	struct ist510e *ts = g_istxxxx_data;

	TS_LOG_INFO("enter %s\n", __FUNCTION__);
	ret = hideep_load_ucode(ts, IST520E_UC,0);
	if(ret < 0){
		TS_LOG_ERR("fw update failed\n");    
		goto err;
	}
	TS_LOG_INFO("fw update success\n");

	return NO_ERR;
err:
	return -EINVAL;
}

static int hideep_fw_update_boot(char *file_name)
{
	s32 ret;
	const struct firmware *fw_entry;    
	struct ist510e *ts_drv = g_istxxxx_data;
	struct device *dev = &ts_drv->huawei_ts_dev->dev;
	s32 load = 0;
	u16 version;
	size_t file_name_size = strlen(file_name)+strlen("ist510e");
	char firmware_name[PRODUCT_ID_LENGTH + file_name_size +1];
	snprintf(firmware_name, sizeof(firmware_name), "ts/%s%s.bin", file_name, "ist510e");
	firmware_name[PRODUCT_ID_LENGTH + file_name_size] = 0;

	TS_LOG_INFO("hideep_fw_update_boot enter, firmware name is %s\n", firmware_name);
	ret = request_firmware(&fw_entry, firmware_name, dev);
	if(ret != 0){
		TS_LOG_ERR("request_firmware(%s) failed\n", firmware_name);
		goto err;
	}
	release_firmware(fw_entry);
	ret = hideep_i2c_read(ts_drv, VR_VERSION_ADDR, 2, &version);
	if(ret < 0){
		TS_LOG_ERR("read i2c fail.\n"); 
		goto err;
	}
	TS_LOG_INFO("vr version : %04x\n", version);
	ret = hideep_load_ucode(ts_drv, firmware_name, version);
	if(ret < 0){
		TS_LOG_ERR("fw update failed\n");    
		goto err;
	}
	TS_LOG_INFO("fw update success\n");
	return NO_ERR;
err:
	return  ret;
}

static int hideep_get_info(struct ts_chip_info_param *info)
{
	struct ist510e *ts   = g_istxxxx_data;
	DWZ_INFO_T     *dmz_info = &ts->dwz_info ;

	TS_LOG_INFO("hideep_get_info enter\n");
	hideep_get_version(g_istxxxx_data);
	snprintf(info->ic_vendor, PAGE_SIZE, "hideep-ist510e-");
	snprintf(info->mod_vendor, PAGE_SIZE, "samsung-");
	snprintf(info->fw_vendor, PAGE_SIZE,"%04x\n",  dmz_info->pannel.version);

	return NO_ERR;
}

int hideep_get_image_page(struct ts_rawdata_info *info,u16 command,u16 offset)
{
	struct ist510e *ts   = g_istxxxx_data;
	struct ist510e_debug_dev *dev = &ts->debug_dev;
	u8 vr_buff;
	s32 ret = 0;
	int time_out;
	s32 i,j,retry;
	DWZ_INFO_T  *dmz_info = &ts->dwz_info ;
	u16 p_aulu_buf[31];
	s16 *ps16;
	s32 *ps32;

	TS_LOG_INFO("hideep_get_image_page enter\n");
	dev->i_rdy = 0;
	dev->vr_addr = 0x0;
	dev->vr_size = 0x1;
	vr_buff = command;
	ret = hideep_i2c_write(ts, dev->vr_addr, dev->vr_size, &vr_buff);
	if(ret < 0)
		goto hideep_get_image_page_command_err;

	dev->im_size = 2*info->buff[0]*info->buff[1]+8;
	dev->im_buff = kmalloc(dev->im_size, GFP_KERNEL);
	if(dev->im_buff == NULL){
		TS_LOG_ERR("can't alloc memory\n");
		goto hideep_get_image_page_command_err;
	}
	for(retry = 0;retry <5;retry ++){
		time_out = 0;
		do{
			hideep_get_image(ts);
			mdelay(10);
			time_out++;
			TS_LOG_DEBUG("time count %d, dev->im_buff[0] = 0x%02x\n",time_out,dev->im_buff[0]);
			if(time_out>10000)
				goto hideep_get_image_page_timeout_err;
		}while(dev->im_buff[0]!='G');
	}
	dev->i_rdy = 0;
	//copy data to target buff.
	ps16 = dev->im_buff+8;
	ps32 = info->buff+offset;
	for(i = 0; i < info->buff[1]; i++){
		for(j = 0; j < info->buff[0]; j++)
			*(ps32+i*info->buff[0]+j) =(s32)( *(ps16+i*info->buff[0]+j));
	}
	if (dev->im_buff)
		kfree(dev->im_buff);
	TS_LOG_INFO("hideep_get_image_page end\n");
	return 0;
hideep_get_image_page_timeout_err:
	if (dev->im_buff)
		kfree(dev->im_buff);
hideep_get_image_page_command_err:
	TS_LOG_ERR("hideep_get_image_page error\n");
	return -1;
}

int hideep_get_aulura_image_page(struct ts_rawdata_info *info,u16 command,u16 offset)
{
	struct ist510e *ts   = g_istxxxx_data;
	struct ist510e_debug_dev *dev = &ts->debug_dev;
	u8 vr_buff;
	s32 ret = 0;
	int time_out;
	s32 i,j,retry;
	u8 *ps8;
	s16 *ps16;
	s32 *ps32;
	
	TS_LOG_INFO("hideep_get_aulura_image_page enter cmd = 0x%04x\n",command);
	//set op_mode as AULU diff mode.
	dev->i_rdy = 0;
	dev->vr_addr = 0x0;
	dev->vr_size = 0x1;
	vr_buff = command;
	ret = hideep_i2c_write(ts, dev->vr_addr, dev->vr_size, &vr_buff);
	if(ret < 0){
		goto hideep_get_aulura_image_page_command_err;
	}

	if(command == OPM_AULU_DIFF){
		dev->im_size = info->buff[1]+8;
	}else{
		dev->im_size = info->buff[1]*2+8;
	}
	dev->im_buff = kmalloc(dev->im_size, GFP_KERNEL);
	if(dev->im_buff == NULL){
		TS_LOG_ERR("can't alloc memory\n");
		goto hideep_get_aulura_image_page_command_err;
	}
	for(retry = 0;retry <5;retry ++){
		time_out = 0;
		do{
			hideep_get_image(ts);
			mdelay(10);
			time_out++;
			TS_LOG_DEBUG("time count %d, dev->im_buff[0] = 0x%02x\n",time_out,dev->im_buff[0]);
			if(time_out>10000)
				goto hideep_get_aulura_image_page_timeout_err;
		}while(dev->im_buff[0]!='G');
	}
	dev->i_rdy = 0;
	ps32 = info->buff+offset;
    	for(i = 0; i < info->buff[1]; i++){
		//save aulu in the target buf.
		if(command == OPM_AULU_DIFF){
			TS_LOG_DEBUG("buf[%d] = %d\n",i,((s8*)(dev->im_buff+8))[i]);
			*(ps32+(i+1)*info->buff[0]-1) = ((s8*)(dev->im_buff+8))[i];
		}else{
			TS_LOG_DEBUG("buf[%d] = %d\n",i,((s16*)(dev->im_buff+8))[i]);
			*(ps32+(i+1)*info->buff[0]-1) = ((s16*)(dev->im_buff+8))[i];
		}
	}

	if (dev->im_buff)
		kfree(dev->im_buff);
	TS_LOG_INFO("hideep_get_aulura_image_page end.\n");

	return 0;
hideep_get_aulura_image_page_timeout_err:
	if (dev->im_buff)
		kfree(dev->im_buff);
hideep_get_aulura_image_page_command_err:
	TS_LOG_ERR("hideep_get_aulura_image_page error\n");
	return -1;
}

static void hideep_rawdata_image_data_test(struct ts_rawdata_info *info)
{
	int i, j;
	int index = 0;
	int rx = info->buff[0];
	int tx = info->buff[1];
	u32 result_count = 0;
	u16 rawdata_max[(HIDEEP_RX_COUNT-1)*HIDEEP_TX_COUNT] = {5000};
	u16 rawdata_min[(HIDEEP_RX_COUNT-1)*HIDEEP_TX_COUNT] = {0};
	u16 temp_buffer[(HIDEEP_RX_COUNT-1)*HIDEEP_TX_COUNT] = {0};

	TS_LOG_INFO("hideep_rawdata_image_data_test\n");

	for (i = 0; i < (HIDEEP_RX_COUNT-1)*HIDEEP_TX_COUNT; i ++) {
		rawdata_max[i] = 5000;
		rawdata_min[i] = 0;
	}

	for(i = 0; i < tx; i++) {
		for (j = 0; j < rx - 1; j ++) {
			temp_buffer[index] = info->buff[i*rx+j+2];
			index ++;
		}
	}
#if 1
	j = 0;
	/*print temp buffer data*/
	for(i = 0; i < tx*(rx-1); i++){
		printk("%d ", temp_buffer[i]);
		j ++;
		if(j == rx-1){
			j = 0;
			printk("\n");
		}
	}
	printk("\n");
#endif
	for(i = 0; i < tx*(rx-1); i++) {
		if((temp_buffer[i] < rawdata_max[i]) && (temp_buffer[i] > rawdata_min[i]))
			result_count++;
		else
			TS_LOG_INFO("over limit, buffer[%d]:%d\n", i, temp_buffer[i]);
	}

	TS_LOG_INFO("result_count = %d\n", result_count);
	if(result_count >= tx*(rx-1))
		strncat(info->result, "-1P", MAX_STR_LEN);
	else
		strncat(info->result, "-1F", MAX_STR_LEN);
}

static void hideep_rawdata_image_z_test(struct ts_rawdata_info *info)
{
	int i;
	int index = 0;
	int rx = info->buff[0];
	int tx = info->buff[1];
	u32 result_count = 0;
	u16 rawdata_z_max[HIDEEP_TX_COUNT] = {0};
	u16 rawdata_z_min[HIDEEP_TX_COUNT] = {0};
	u16 temp_z_buffer[HIDEEP_TX_COUNT] = {0};

	TS_LOG_INFO("hideep_rawdata_image_z_test\n");
	for (i = 0; i < HIDEEP_TX_COUNT; i ++) {
		rawdata_z_max[i] = 10000;
		rawdata_z_min[i] = 0;
	}

	for(i = 0; i < tx; i++) {
		temp_z_buffer[index] = info->buff[i*rx+rx-1+2];
		index ++;
	}
#if 1
	/*print temp buffer data*/
	for(i = 0; i < tx; i++){
		printk("%d\n", temp_z_buffer[i]);
	}
#endif
	for(i = 0; i < tx; i++) {
		if((temp_z_buffer[i] < rawdata_z_max[i]) && (temp_z_buffer[i] > rawdata_z_min[i]))
			result_count++;
		else
			TS_LOG_INFO("over limit, z_buffer[%d]:%d\n", i, temp_z_buffer[i]);
	}

	TS_LOG_INFO("result_count = %d\n", result_count);

	if(result_count >= tx)
		strncat(info->result, "-2P", MAX_STR_LEN);
	else 
		strncat(info->result, "-2F", MAX_STR_LEN);
}

static void hideep_delta_image_data_test(struct ts_rawdata_info *info)
{
	int i, j;
	int index = 0;
	int rx = info->buff[0];
	int tx = info->buff[1];
	u32 result_count = 0;
	s32 rawdata_max[(HIDEEP_RX_COUNT-1)*HIDEEP_TX_COUNT] = {0};
	s32 rawdata_min[(HIDEEP_RX_COUNT-1)*HIDEEP_TX_COUNT] = {0};
	s32 temp_buffer[(HIDEEP_RX_COUNT-1)*HIDEEP_TX_COUNT] = {0};

	TS_LOG_INFO("hideep_delta_image_data_test\n");

	for (i = 0; i < (HIDEEP_RX_COUNT-1)*HIDEEP_TX_COUNT; i ++) {
		rawdata_max[i] = 500;
		rawdata_min[i] = -500;
	}

	for(i = 0; i < tx; i++) {
		for (j = 0; j < rx - 1; j ++) {
			temp_buffer[index] = info->buff[2+rx*tx+i*rx+j];
			index ++;
		}
	}
#if 1
	j = 0;
	/*print temp buffer data*/
	for(i = 0; i < tx*(rx-1); i++){
		printk("%d ", temp_buffer[i]);
		j ++;
		if(j == rx-1){
			j = 0;
			printk("\n");
		}
	}
	printk("\n");
#endif
	for(i = 0; i < tx*(rx-1); i++) {
		if((temp_buffer[i] < rawdata_max[i]) && (temp_buffer[i] > rawdata_min[i]))
			result_count++;
		else
			TS_LOG_INFO("over limit, buffer[%d]:%d\n", i, temp_buffer[i]);
	}

	TS_LOG_INFO("result_count = %d\n", result_count);

	if(result_count >= tx*(rx-1))
		strncat(info->result, "-3P", MAX_STR_LEN);
	else
		strncat(info->result, "-3F", MAX_STR_LEN);
}

static void hideep_delta_image_z_test(struct ts_rawdata_info *info)
{
	int i;
	int index = 0;
	int rx = info->buff[0];
	int tx = info->buff[1];
	u32 result_count = 0;
	s32 rawdata_z_max[HIDEEP_TX_COUNT] = {0};
	s32 rawdata_z_min[HIDEEP_TX_COUNT] = {0};
	s32 temp_z_buffer[HIDEEP_TX_COUNT] = {0};

	TS_LOG_INFO("hideep_delta_image_z_test\n");

	for (i = 0; i < HIDEEP_TX_COUNT; i ++) {
		rawdata_z_max[i] = 500;
		rawdata_z_min[i] = -500;
	}

	for(i = 0; i < tx; i++) {
		temp_z_buffer[index] = info->buff[2+tx*rx+i*rx+rx-1];
		index ++;
	}
#if 1
	/*print temp buffer data*/
	for(i = 0; i < tx; i++){
		printk("%d\n", temp_z_buffer[i]);
	}
#endif
	for(i = 0; i < tx; i++) {
		if((temp_z_buffer[i] < rawdata_z_max[i]) && (temp_z_buffer[i] > rawdata_z_min[i]))
			result_count++;
		else
			TS_LOG_INFO("over limit, z_buffer[%d]:%d\n", i, temp_z_buffer[i]);
	}

	TS_LOG_INFO("result_count = %d\n", result_count);

	if(result_count >= tx)
		strncat(info->result, "-4P", MAX_STR_LEN);
	else 
		strncat(info->result, "-4F", MAX_STR_LEN);
}

static int hideep_get_rawdata(struct ts_rawdata_info *info, struct ts_cmd_node *out_cmd)
{
	struct ist510e *ts   = g_istxxxx_data;
	struct ist510e_debug_dev *dev = &ts->debug_dev;
	DWZ_INFO_T *dmz_info = &ts->dwz_info;
	s32 ret = 0;
	s32 i;
	u8 vr_buff;
	s32	max,min,sum;
	s32 *ps32;
	char rawdata_result[2] = {0};
	char rawdata_z_result[2] = {0};
	char deltdata_result[2] = {0};
	char deltdata_z_result[2] = {0};
 
	TS_LOG_INFO("hideep_get_rawdata enter\n");

	dmz_info->pannel.rx = HIDEEP_RX_COUNT;
	dmz_info->pannel.tx = HIDEEP_TX_COUNT;
	mutex_lock(&ts->dev_mutex);
	info->buff[0] = dmz_info->pannel.rx;
	info->buff[1] = dmz_info->pannel.tx;
	info->used_size = info->buff[0]*info->buff[1]*2+2;
	dev->im_r_en = 1;
	if(dev->im_buff == NULL) {
		TS_LOG_INFO("hideep_get_rawdata im_buff is NULL\n");
		memcpy(info->result,"0F-1F-2F-3F-4F\n",strlen("0F-1F-2F-3F-4F")+1);
		goto hideep_get_rawdata_alloc_err;
	}
	//read touch area rawdata...
	if(hideep_get_image_page(info,OPM_DEMOD,2)!=0) {
		memcpy(info->result,"0F-1F-2F-3F-4F\n",strlen("0F-1F-2F-3F-4F")+1);
		goto hideep_get_rawdata_rd_err;
	} else {
		memcpy(info->result,"0P",strlen("0P"));
	}
	//read Z-sensor rawdata...
	if(hideep_get_aulura_image_page(info,OPM_AULU_DATA,2)!=0)
		goto hideep_get_rawdata_rd_err;
	//read touch area diff...
	if(hideep_get_image_page(info,OPM_DIFF,2+(info->buff[0]*info->buff[1]))!=0)
		goto hideep_get_rawdata_df_err;
	//read Z-sensor area diff...
	if(hideep_get_aulura_image_page(info,OPM_AULU_DIFF,2+(info->buff[0]*info->buff[1]))!=0)
		goto hideep_get_rawdata_df_err;

	/*rawdata and delta data testing*/
	hideep_rawdata_image_data_test(info);
	hideep_rawdata_image_z_test(info);
	hideep_delta_image_data_test(info);
	hideep_delta_image_z_test(info);

	/*rawdata image data*/
	max = 0;
	min = 0xffff;
	sum = 0;
	ps32 = info->buff+2;
	for(i = 0;i<(info->buff[0]*info->buff[1]);i++){
		max = *(ps32+i)>max?*(ps32+i):max;
		min = *(ps32+i)<min?*(ps32+i):min;
		sum+= *(ps32+i);
	}

	i = strlen(info->result);
	snprintf(info->result+i,50,"[%4d,%4d,%4d] ",sum/(info->buff[0]*info->buff[1]),max,min);

	/*delta image data*/
	max = 0;
	min = 0xffff;
	sum = 0;
	ps32 = info->buff+2+info->buff[0]*info->buff[1];
	for(i = 0;i<(info->buff[0]*info->buff[1]);i++){
		max = *(ps32+i)>max?*(ps32+i):max;
		min = *(ps32+i)<min?*(ps32+i):min;
		sum+= *(ps32+i);
	}
	i = strlen(info->result);
	snprintf(info->result+i,50,"[%4d,%4d,%4d]\n",sum/(info->buff[0]*info->buff[1]),max,min);
	dev->i_rdy = 0;
	dev->vr_addr = 0x0;
	dev->vr_size = 0x1;
	vr_buff = OPM_TOUCH_A;
	ret = hideep_i2c_write(ts, dev->vr_addr, dev->vr_size, &vr_buff);
	dev->im_r_en = 0;
	out_cmd->command = TS_INVAILD_CMD;
	mutex_unlock  (&ts->dev_mutex);
	TS_LOG_INFO("finish.\n");

	return NO_ERR;
hideep_get_rawdata_rd_err:
	TS_LOG_ERR("rawdata\n");
	goto err_quit;
hideep_get_rawdata_df_err:
	TS_LOG_ERR("diff.\n");
	goto err_quit;
hideep_get_rawdata_alloc_err:
	TS_LOG_ERR("alloc memory error.\n");
	goto err_quit;
err_quit:
	dev->i_rdy = 0;
	dev->vr_addr = 0x0;
	dev->vr_size = 0x1;
	vr_buff = OPM_TOUCH_A;
	ret = hideep_i2c_write(ts, dev->vr_addr, dev->vr_size,&vr_buff);
	dev->im_r_en = 0;
	out_cmd->command = TS_INVAILD_CMD;
	mutex_unlock(&ts->dev_mutex);
	return -EINVAL;
}

static int hideep_holster_switch(struct ts_holster_info *info)
{
	int ret = NO_ERR;
	struct ist510e *ts = g_istxxxx_data;
	u8 cmd_buf[5]={0};
	int x0 = g_ts_data.feature_info.window_info.top_left_x0;
	int y0 = g_ts_data.feature_info.window_info.top_left_y0;
	int x1 = g_ts_data.feature_info.window_info.bottom_right_x1;
	int y1 = g_ts_data.feature_info.window_info.bottom_right_y1;
	u16 *p16;

	TS_LOG_INFO("switch = %d, Holster window_info is (x0=%d,y0=%d)(x1=%d,y1=%d)\n",\
		info->holster_switch,x0,y0,x1,y1);
	switch(info->op_action){
		case TS_ACTION_WRITE:
			if(!info->holster_switch){
				TS_LOG_INFO("disable holster mode.\n");
				cmd_buf[0] = 0x00;
				ret = hideep_i2c_write(ts, VR_HOLSTER, 1, cmd_buf);
				if(ret < 0){
					TS_LOG_ERR("i2c write error.\n");
					break;
				}
			}else{
				//send the sensitivity;
				cmd_buf[0] = 0x10;
				cmd_buf[1] = info->holster_switch;
				ret = hideep_i2c_write(ts, VR_HOLSTER, 2, cmd_buf);
				if(ret < 0){
					TS_LOG_ERR("i2c write error.\n");
					break;
				}
				//send the top_left x,y;
				cmd_buf[0] = 0x11;
				p16 = &cmd_buf[1];
				*p16++ = x0;
				*p16++ = y0;
				ret = hideep_i2c_write(ts, VR_HOLSTER, 5, cmd_buf);
				if(ret < 0){
					TS_LOG_ERR("i2c write error.\n");
					break;
				}
				//send the right_bottom x,y;
				cmd_buf[0] = 0x12;
				p16 = &cmd_buf[1];
				*p16++ = x1;
				*p16++ = y1;
				ret = hideep_i2c_write(ts, VR_HOLSTER, 5, cmd_buf);
				if(ret < 0){
					TS_LOG_ERR("i2c write error.\n");
					break;
				}
			}
			g_istxxxx_data->holster_mode = info->holster_switch;
			break;
		case TS_ACTION_READ:
			info->holster_switch = g_istxxxx_data->holster_mode;
			break;
		default:
			TS_LOG_ERR("no this action\n");
			break;
	}

	return ret;
}

static int hideep_glove_switch(struct ts_glove_info *info)
{
	int ret = NO_ERR;

	TS_LOG_INFO("enter, action = %d, switch = %d\n",info->op_action , info->glove_switch);
	if(info->op_action == TS_ACTION_WRITE){
		g_istxxxx_data->glove_mode = info->glove_switch;
		ret = hideep_item_switch(info->op_action, VR_GLOVE,   1, &info->glove_switch);
	}else if(info->op_action == TS_ACTION_READ){
		info->glove_switch = g_istxxxx_data->glove_mode;
		ret = g_istxxxx_data->glove_mode;
	}

	return ret;
}

static int hideep_palm_switch(struct ts_palm_info *info)
{
	int ret = NO_ERR;

	TS_LOG_INFO("hideep_palm_switch enter\n");
	ret = hideep_item_switch(info->op_action, VR_GLOVE,   1, &info->palm_switch);
	return ret;
}

struct ts_device_ops ts_hideep_ops =
{
    .chip_parse_config = hideep_parse_config,
    .chip_input_config  = hideep_input_config,
    .chip_detect = hideep_chip_detect ,
    .chip_init = hideep_init,
    .chip_reset = hideep_reset,
    .chip_hw_reset = hideep_reset,
    .chip_shutdown = hideep_shutdown,

    .chip_irq_top_half = hideep_irq_top_half,
    .chip_irq_bottom_half = hideep_irq_bottom_half,
    .chip_suspend = hideep_suspend,
    .chip_resume = hideep_resume,

    .chip_fw_update_sd = hideep_fw_update_sd,
    .chip_fw_update_boot = hideep_fw_update_boot,
    .chip_get_info = hideep_get_info,
    .chip_get_rawdata = hideep_get_rawdata,

    .chip_holster_switch = hideep_holster_switch,
    .chip_glove_switch = hideep_glove_switch,
    .chip_palm_switch = hideep_palm_switch,
}; 

void hideep_power_gpio_enable(void)
{
	struct ist510e *ts  = g_istxxxx_data;
	struct ts_device_data *dat  = ts->huawei_ts_data;
	gpio_direction_output(dat->vddio_gpio_ctrl, 1);
}

void hideep_power_gpio_disable(void)
{
	struct ist510e *ts  = g_istxxxx_data;
	struct ts_device_data *dat   = ts->huawei_ts_data;
	gpio_direction_output(dat->vddio_gpio_ctrl, 0);
}

EXPORT_SYMBOL(hideep_power_gpio_enable);
EXPORT_SYMBOL(hideep_power_gpio_disable);


