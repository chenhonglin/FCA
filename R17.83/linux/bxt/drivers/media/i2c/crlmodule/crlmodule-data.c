/*
 * Copyright (c) 2014--2017 Intel Corporation.
 *
 * Author: Vinod Govindapillai <vinod.govindapillai@intel.com>
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

#include "crlmodule.h"
#include "crl_ov13858_configuration.h"
#include "crl_imx132_configuration.h"
#include "crl_imx214_configuration.h"
#include "crl_imx135_configuration.h"
#include "crl_imx230_configuration.h"
#include "crl_imx318_configuration.h"
#include "crl_ov8858_configuration.h"
#include "crl_ov13860_configuration.h"
#include "crl_adv7481_cvbs_configuration.h"
#ifdef CONFIG_INTEL_IPU4_ADV7281
#include "crl_adv7281_cvbs_configuration.h"
#endif
#include "crl_adv7481_hdmi_configuration.h"
#include "crl_adv7481_eval_configuration.h"
#include "crl_imx185_configuration.h"
#include "crl_ov10635_configuration.h"
#include "crl_ov10640_configuration.h"
#include "crl_imx477_master_configuration.h"
#include "crl_imx477_slave_configuration.h"
#include "crl_imx274_configuration.h"
#include "crl_ov5670_configuration.h"
#include "crl_imx290_configuration.h"
#include "crl_pixter_stub_configuration.h"
#include "crl_ov2740_configuration.h"
#include "crl_magna_configuration.h"
#include "crl_bjev_face_detect_configuration.h"

#ifdef CONFIG_INTEL_IPU4_DS90UB954_MOPAR
#include "crl_ti954_mopar_configuration.h"
#endif

#ifdef CONFIG_INTEL_IPU4_TI953_SENSOR
#include "crl_ti953_sensor_configuration.h"
#endif

#ifdef CONFIG_INTEL_IPU4_APTINA_RVC
#include "crl_aptina_configuration_rvc.h"
#endif

#ifdef CONFIG_INTEL_IPU4_AR0143_RVC
#include "crl_ar0143_configuration_rvc.h"
#endif
#ifdef CONFIG_INTEL_IPU4_AR0143_AVM
#include "crl_ar0143_configuration_avm.h"
#endif

static const struct crlmodule_sensors supported_sensors[] = {
	{ "i2c-OVTIF858:00", "ov13858", &ov13858_crl_configuration},
	{ "OV13858", "ov13858", &ov13858_crl_configuration},
	{ "OV13858-2", "ov13858", &ov13858_crl_configuration},
	{ "i2c-INT3474:00", "ov2740", &ov2740_crl_configuration },
	{ "OV2740", "ov2740", &ov2740_crl_configuration },
	{ "i2c-SONY214A:00", "imx214", &imx214_crl_configuration },
	{ "IMX214", "imx214", &imx214_crl_configuration },
	{ "i2c-SONY132A:00", "imx132", &imx132_crl_configuration },
	{ "i2c-INT3471:00", "imx135", &imx135_crl_configuration },
	{ "i2c-SONY230A:00", "imx230", &imx230_crl_configuration },
	{ "i2c-INT3477:00", "ov8858", &ov8858_crl_configuration },
	{ "i2c-OV5670AA:00", "ov5670", &ov5670_crl_configuration },
	{ "IMX185", "imx185", &imx185_crl_configuration },
	{ "IMX477-MASTER", "imx477", &imx477_master_crl_configuration },
	{ "IMX477-SLAVE-1", "imx477", &imx477_slave_crl_configuration },
	{ "OV13860", "ov13860", &ov13860_crl_configuration },
	{ "ADV7481 CVBS", "adv7481_cvbs", &adv7481_cvbs_crl_configuration },
#ifdef CONFIG_INTEL_IPU4_DS90UB954_MOPAR
           { "DS90UB954", "ds90ub954", &ti954_mopar_crl_configuration },
#endif

#ifdef CONFIG_INTEL_IPU4_ADV7281
	{ "ADV7281 CVBS", "adv7281_cvbs", &adv7281_cvbs_crl_configuration },
#endif
	{ "ADV7481 HDMI", "adv7481_hdmi", &adv7481_hdmi_crl_configuration },
	{ "ADV7481_EVAL", "adv7481_eval", &adv7481_eval_crl_configuration },
	{ "ADV7481B_EVAL", "adv7481b_eval", &adv7481b_eval_crl_configuration },
	{ "SONY318A", "imx318", &imx318_crl_configuration },
	{ "OV10635", "ov10635", &ov10635_crl_configuration },
	{ "OV10640", "ov10640", &ov10640_crl_configuration },
	{ "IMX274", "imx274", &imx274_crl_configuration },
	{ "OV5670", "ov5670", &ov5670_crl_configuration },
	{ "IMX290", "imx290", &imx290_crl_configuration},
	{ "PIXTER_STUB", "pixter_stub", &pixter_stub_crl_configuration},
	{ "PIXTER_STUB_B", "pixter_stub_b", &pixter_stub_b_crl_configuration},
	{ "INT3474", "ov2740", &ov2740_crl_configuration },
	{ "MAGNA", "magna", &magna_crl_configuration },
	{ "BJEV_FACE_DETECT", "bjev_face_detect", &bjev_face_detect_crl_configuration },
#ifdef CONFIG_INTEL_IPU4_AR0143_RVC
	{ "AR0143_RVC", "ar0143_rvc", &ar0143_rvc_crl_configuration },
#endif
#ifdef CONFIG_INTEL_IPU4_AR0143_AVM
	{ "AR0143_AVM", "ar0143_avm", &ar0143_avm_crl_configuration },
#endif
#ifdef CONFIG_INTEL_IPU4_TI953_SENSOR
	{ "TI953_SENSOR", "ti953_sensor", &ti953_sensor_crl_configuration },
#endif
#ifdef CONFIG_INTEL_IPU4_APTINA_RVC
	{ "APTINA_RVC", "aptina_rvc", &aptina_rvc_crl_configuration },
#endif
};

/*
 * Function to populate the CRL data structure from the sensor configuration
 * definition file
 */
int crlmodule_populate_ds(struct crl_sensor *sensor, struct device *dev)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_sensors); i++) {
		/* Check the ACPI supported modules */
		if (!strcmp(dev_name(dev), supported_sensors[i].pname)) {
			sensor->sensor_ds = supported_sensors[i].ds;
			dev_info(dev, "%s %s selected\n",
				 __func__, supported_sensors[i].name);
			return 0;
		}

		/* Check the non ACPI modules */
		if (!strcmp(sensor->platform_data->module_name,
			    supported_sensors[i].pname)) {
			sensor->sensor_ds = supported_sensors[i].ds;
			dev_info(dev, "%s %s selected\n",
				 __func__, supported_sensors[i].name);
			return 0;
		}
	}

	dev_err(dev, "%s No suitable configuration found for %s\n",
		     __func__, dev_name(dev));
	return -EINVAL;
}

/*
 * Function validate the contents CRL data structure to check if all the
 * required fields are filled and are according to the limits.
 */
int crlmodule_validate_ds(struct crl_sensor *sensor)
{
	/* TODO! Revisit this. */
	return 0;
}

/* Function to free all resources allocated for the CRL data structure */
void crlmodule_release_ds(struct crl_sensor *sensor)
{
	/*
	 * TODO! Revisit this.
	 * Place for cleaning all the resources used for the generation
	 * of CRL data structure.
	 */
}

/* Search for sensor->sensor_ds in support sensor list to get the sensor name */
const char* crlmodule_sensor_name(struct crl_sensor *sensor)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(supported_sensors); i++) {
		if (sensor->sensor_ds == supported_sensors[i].ds){
			return supported_sensors[i].name;
		}
	}
	return "<unknown>";
}
