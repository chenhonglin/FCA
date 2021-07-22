/*
 * crl_ar0143_cfg.c
 * Copyright (C) 2016 Harman International Ltd,
 *
 * Author: Sreeju Arumugan Selvaraj <sreeju.selvaraj@harman.com>
 * Created on: 03-01-2016
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include "crlmodule.h"
#include "crlmodule-regs.h"
#include "ar0143-reg.h"

static int ap0101_i2c_write(struct crl_sensor *sensor,
			u16 i2c_addr,
			u16 reg,
			u8 len,
			u16 val)
{
	return crlmodule_write_reg(sensor, i2c_addr, reg,
					 len, 0xFFFF, val);
}

static int ap0101_i2c_read(struct crl_sensor *sensor,
			u16 i2c_addr,
			u16 reg,
			u8 len,
			u32 *val)
{
	struct crl_register_read_rep read_reg;

	read_reg.address = reg;
	read_reg.len = len; /* 8/16bit wide */
	read_reg.dev_i2c_addr = i2c_addr;
	return crlmodule_read_reg(sensor, read_reg, val);
}


static int check_sysmgr_cmd_status(struct crl_sensor *sensor, u16 i2c_addr,
									u16 *sts, u16 *comp_id, u16 *failure_id)
{
	u32 val;
	int retry;
	int ret = -1;
	int timeout = 20;

	*sts = 0xFFFF;
	*comp_id = 0xFFFF;
	*failure_id = 0xFFFF;

	/* Check for  cmd status */
	for (retry = 0; retry < timeout; retry++) {
		ret = ap0101_i2c_read(sensor, i2c_addr, SYSMGR_CMD_STATUS_REG,
											CRL_REG_LEN_08BIT, &val);
		if (!ret && !val) {
			*sts = val;
			break;
		} else {
			if (!ret)
				*sts = val & 0xFF;
			pr_err("%s: Retrying: return value: %d, command response: 0x%02x\n",
					__func__, ret, val);
			msleep(10);
		}
	}

	if (ret) {
		pr_err("%s: Reading cmd status failed\n",
					__func__, ret, val);
		goto err;
	}

	ret = ap0101_i2c_read(sensor, i2c_addr, SYSMGR_CMD_COMP_ID_REG,
											CRL_REG_LEN_08BIT, &val);
	if (ret) {
		pr_err("%s: Reading cmd comp id failed\n",
					__func__, ret, val);
		goto err;
	}
	*comp_id = val & 0xFF;

	ret = ap0101_i2c_read(sensor, i2c_addr, SYSMGR_CMD_COMP_FAIL_ID_REG,
												CRL_REG_LEN_16BIT, &val);
	if (ret) {
		pr_err("%s: Reading comp failure id failed\n",
					__func__, ret, val);
	}
	*failure_id = val & 0xFFFF;

	if (*sts || *comp_id || *failure_id)
		ret = -1;

err:
	return ret;
}

static int check_dorebell_bit(struct crl_sensor *sensor, u16 i2c_addr, u16 *sts)
{
	u32 val;
	int ret, retry;
	int timeout = 20;
	*sts = 0xFFFF;

	/* Check for dore bell bit cleared */
	for (retry = 0; retry < timeout; retry++) {
		ret = ap0101_i2c_read(sensor, i2c_addr, CMD_REG,
								CRL_REG_LEN_16BIT, &val);
		if (!ret && (!(val & DOORBELL_BIT) || !val)) {
			*sts = val;
			return 0;
		} else {
			if (!ret)
				*sts = val & 0xFFFF;
			pr_err("%s: Retrying: return value: %d, command response: 0x%04x\n",
					__func__, ret, val);
			msleep(10);
			ret = -1;
		}
	}

	if (!ret && *sts)
		ret = -1;

	return ret;
}

static int ap0101_cmd_handler(struct crl_sensor *sensor,
								u16 i2c_addr, u32 cmd)
{
	u16 result, status, comp_id, failure_id;
	int ret;

	pr_err("%s: handling cmd: 0x04%x\n", __func__, cmd);

	/* Check dorebell bit */
	ret = check_dorebell_bit(sensor, i2c_addr, &result);
	if (ret) {
		pr_err("%s:dorebell bit error case 1: return value: %d, command response: 0x%04x\n",
				__func__, ret, result);
		goto err;
	}

	/* Issue the command */
	ret = ap0101_i2c_write(sensor, i2c_addr, CMD_REG,
								CRL_REG_LEN_16BIT, cmd);
	if (ret) {
		pr_err("%s:cmd write error: return value: %d, writing: 0x%04x at 0x%04x failed\n",
				__func__, cmd, CMD_REG);
		goto err;
	}

	msleep(10);

	/* Check dorebell bit */
	ret = check_dorebell_bit(sensor, i2c_addr, &result);
	if (ret) {
		pr_err("%s:dorebell bit error case 2 return value: %d, command response: 0x%04x\n",
				__func__, ret, result);
		goto err;
	}

	switch (cmd) {
		case CMD_SYSMGR_SET_STATE:
			ret = check_sysmgr_cmd_status(sensor, i2c_addr, &status,
											&comp_id, &failure_id);
			if (ret) {
				pr_err("%s: cmd failed: sysmgr_cmd_status: 0x%02x comp_id: 0x%04x failure_id: 0x%04x\n",
						__func__, status, comp_id, failure_id);
			} else {
				pr_err("%s: cmd success: sysmgr_cmd_status: 0x%02x comp_id: 0x%04x failure_id: 0x%04x\n",
						__func__, status, comp_id, failure_id);
			}
			break;
		case CMD_SEQ_REFRESH:
			/* TODO: */
			break;
		default:
			break;
	}

err:
	return ret;
}

int ap0101_reg_read(struct crl_sensor *sensor,
							u16 i2c_addr,
							u16 reg,
							u16 *val)
{
	int ret;
	u32 rdval;

	ret = ap0101_i2c_read(sensor, i2c_addr, reg,
							CRL_REG_LEN_16BIT, &rdval);
	if (!ret)
		*val = (rdval & 0xFFFF);

	return ret;

}

/* HCI for ap0101 reg write */
static int ap0101_reg_write(struct crl_sensor *sensor,
							u16 i2c_addr,
							u16 reg,
							u8 len,
							u32 val)
{
	int ret = -1;

	switch (reg) {
		case CMD_REG:
			ret = ap0101_cmd_handler(sensor, i2c_addr, val);
			break;
		default:
			ret = ap0101_i2c_write(sensor, i2c_addr, reg, len, val);
			break;
	}

	return ret;
}


/* HCI for ar0143 reg write */
static int ar0143_reg_write(struct crl_sensor *sensor,
							u16 i2c_addr,
							u16 reg,
							u32 val)
{
	int ret;

	/* command: GET LOCK */
	ret = ap0101_cmd_handler(sensor, i2c_addr, CMD_CCIMGR_GET_LOCK);
	if (ret)
		goto err;
	/* command: GET LOCK STATUS */
	ret = ap0101_cmd_handler(sensor, i2c_addr, CMD_CCIMGR_LOCK_STATUS);
	if (ret)
		goto rel_lock;
	/* Fill parameters */
	/* reg add to be read */
	ret = ap0101_i2c_write(sensor, i2c_addr, PARAMS_POOL_0, CRL_REG_LEN_16BIT, reg);
	if (ret)
		goto rel_lock;
	/* No of bytes to be written and values */
	ret = ap0101_i2c_write(sensor, i2c_addr, PARAMS_POOL_1, CRL_REG_LEN_16BIT, 0x0200 | ((val & 0xFF00) >> 8));
	if (ret)
		goto rel_lock;
	ret = ap0101_i2c_write(sensor, i2c_addr, PARAMS_POOL_2, CRL_REG_LEN_16BIT, (val & 0x00FF) << 8);
	if (ret)
		goto rel_lock;
	/*Command: WRITE REG */
	ret = ap0101_cmd_handler(sensor, i2c_addr, CMD_CCIMGR_WRITE);
	if (ret)
		goto rel_lock;
	/* Command: CCI Status */
	ret = ap0101_cmd_handler(sensor, i2c_addr, CMD_CCIMGR_STATUS);
	if (ret)
		goto rel_lock;
	else
		pr_err("%s: success, now release the lock\n", __func__);

rel_lock:
	/* Command: RELEASE LOCK */
	ret = ap0101_cmd_handler(sensor, i2c_addr, CMD_CCIMGR_RELEASE_LOCK);
err:
	return ret;
}

int ar0143_reg_read(struct crl_sensor *sensor,
							u16 i2c_addr,
							u16 reg,
							u16 *val)
{
	int ret;
	u32 rdval;

	/* command: GET LOCK */
	ret = ap0101_cmd_handler(sensor, i2c_addr, CMD_CCIMGR_GET_LOCK);
	if (ret)
		goto err;
	/* command: GET LOCK STATUS */
	ret = ap0101_cmd_handler(sensor, i2c_addr, CMD_CCIMGR_LOCK_STATUS);
	if (ret)
		goto rel_lock;
	/* Fill parameters */
	/* reg add to be read */
	ret = ap0101_i2c_write(sensor, i2c_addr, PARAMS_POOL_0, CRL_REG_LEN_16BIT, reg);
	if (ret)
		goto rel_lock;
	/* No of bytes to be read */
	ret = ap0101_i2c_write(sensor, i2c_addr, PARAMS_POOL_1, CRL_REG_LEN_16BIT, 0x0200);
	if (ret)
		goto rel_lock;
	/*Command: READ REG */
	ret = ap0101_cmd_handler(sensor, i2c_addr, CMD_CCIMGR_READ);
	if (ret)
		goto rel_lock;
	/* Command: CCI Status */
	ret = ap0101_cmd_handler(sensor, i2c_addr, CMD_CCIMGR_STATUS);
	if (ret)
		goto rel_lock;
	ret = ap0101_i2c_read(sensor, i2c_addr, PARAMS_POOL_0,
									CRL_REG_LEN_16BIT, &rdval);
	if (!ret) {
		*val = (rdval & 0xFFFF);
		pr_err("%s: success, now release the lock\n", __func__);
	}


rel_lock:
	/* Command: RELEASE LOCK */
	ret = ap0101_cmd_handler(sensor, i2c_addr, CMD_CCIMGR_RELEASE_LOCK);
err:
	return ret;
}

int ap0101_write_regs(struct crl_sensor *sensor,
		             const struct crl_register_write_rep *regs, int len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = ap0101_reg_write(sensor, regs[i].dev_i2c_addr,
								regs[i].address, regs[i].len, regs[i].val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

int ar0143_write_regs(struct crl_sensor *sensor,
		             const struct crl_register_write_rep *regs, int len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = ar0143_reg_write(sensor, regs[i].dev_i2c_addr,
								regs[i].address, regs[i].val);
		if (ret < 0)
			return ret;
	}

	return 0;
}
