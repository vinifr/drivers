/*
 * pmic-consumer.c
 *
 * Copyright 2015 Embarcados.
 *
 * Author: Vinicius Maciel <viniciusfre@gmail.com>
 *
 * Based of userspace-consumer driver:
 *   Copyright 2009 CompuLab, Ltd.
 *   Author: Mike Rapoport <mike@compulab.co.il>
 * 
 *   Copyright 2008 Wolfson Microelectronics PLC.
 *   Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/userspace-consumer.h>
#include <linux/slab.h>

struct pmic_consumer_data {
	const char *name;

	struct mutex lock;
	bool enabled;

	int num_supplies;
	struct regulator_bulk_data *supplies;
};

static ssize_t reg_show_name(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct pmic_consumer_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", data->name);
}

static ssize_t reg_show_state(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct pmic_consumer_data *data = dev_get_drvdata(dev);

	if (data->enabled)
		return sprintf(buf, "enabled\n");

	return sprintf(buf, "disabled\n");
}

static ssize_t reg_set_state(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct pmic_consumer_data *data = dev_get_drvdata(dev);
	bool enabled;
	int ret;

	/*
	 * sysfs_streq() doesn't need the \n's, but we add them so the strings
	 * will be shared with show_state(), above.
	 */
	if (sysfs_streq(buf, "enabled\n") || sysfs_streq(buf, "1"))
		enabled = true;
	else if (sysfs_streq(buf, "disabled\n") || sysfs_streq(buf, "0"))
		enabled = false;
	else {
		dev_err(dev, "Configuring invalid mode\n");
		return count;
	}

	mutex_lock(&data->lock);
	if (enabled != data->enabled) {
		if (enabled)
			ret = regulator_bulk_enable(data->num_supplies,
						    data->supplies);
		else
			ret = regulator_bulk_disable(data->num_supplies,
						     data->supplies);

		if (ret == 0)
			data->enabled = enabled;
		else
			dev_err(dev, "Failed to configure state: %d\n", ret);
	}
	mutex_unlock(&data->lock);

	return count;
}

static ssize_t reg_show_voltage(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int voltage = 0;	
	struct pmic_consumer_data *data = dev_get_drvdata(dev);
	struct regulator *regulator = data->supplies->consumer;
	
	if (regulator_is_enabled(regulator) > 0)
		voltage = regulator_get_voltage(regulator);
	
	return sprintf(buf, "uV:%d\n", voltage);
}

static ssize_t reg_set_voltage(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret = 0;
	int voltage = 0;
	long int value;
	struct pmic_consumer_data *data = dev_get_drvdata(dev);
	struct regulator *regulator = data->supplies->consumer;
	
	if (!kstrtol(buf, 10, &value)) {
		voltage = regulator_list_voltage(regulator, value);
	}

	if (voltage) {
		mutex_lock(&data->lock);			
		if ((ret = regulator_set_voltage(regulator, voltage,
			voltage)) < 0)
			dev_err(dev, "Failed to configure voltage: %d\n", ret);
		mutex_unlock(&data->lock);
	}

	return ret;
}

static ssize_t reg_show_current(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int current_u = 0;
	struct pmic_consumer_data *data = dev_get_drvdata(dev);
	struct regulator *regulator = data->supplies->consumer;
	
	if (regulator_is_enabled(regulator) > 0)
		current_u = regulator_get_current_limit(regulator);
	
	return sprintf(buf, "uA:%d\n", current_u);
}

static ssize_t reg_set_current(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret = 0;
	int voltage = 0;
	long int value;
	struct pmic_consumer_data *data = dev_get_drvdata(dev);
	struct regulator *regulator = data->supplies->consumer;
	
	if (!kstrtol(buf, 10, &value)) {
		voltage = regulator_list_voltage(regulator, value);
	}

	if (voltage) {
		mutex_lock(&data->lock);			
		if ((ret = regulator_set_voltage(regulator, voltage,
			voltage)) < 0)
			dev_err(dev, "Failed to configure voltage: %d\n", ret);		
		mutex_unlock(&data->lock);
	}

	return ret;
}

static DEVICE_ATTR(name, 0444, reg_show_name, NULL);
static DEVICE_ATTR(state, 0644, reg_show_state, reg_set_state);
static DEVICE_ATTR(voltage, 0644, reg_show_voltage, reg_set_voltage);
static DEVICE_ATTR(current, 0644, reg_show_current, reg_set_current);

static struct attribute *attributes[] = {
	&dev_attr_name.attr,
	&dev_attr_state.attr,
	&dev_attr_voltage.attr,
	&dev_attr_current.attr,
	NULL,
};

static const struct attribute_group attr_group = {
	.attrs	= attributes,
};

static int pmic_consumer_probe(struct platform_device *pdev)
{
	struct regulator_userspace_consumer_data *pdata;
	struct pmic_consumer_data *drvdata;
	int ret;

	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata)
		return -EINVAL;

	drvdata = devm_kzalloc(&pdev->dev,
			       sizeof(struct pmic_consumer_data),
			       GFP_KERNEL);
	if (drvdata == NULL)
		return -ENOMEM;

	drvdata->name = pdata->name;
	drvdata->num_supplies = pdata->num_supplies;
	drvdata->supplies = pdata->supplies;

	mutex_init(&drvdata->lock);

	ret = devm_regulator_bulk_get(&pdev->dev, drvdata->num_supplies,
				      drvdata->supplies);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get supplies: %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &attr_group);
	if (ret != 0)
		return ret;

	if (pdata->init_on) {
		ret = regulator_bulk_enable(drvdata->num_supplies,
					    drvdata->supplies);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to set initial state: %d\n", ret);
			goto err_enable;
		}
	}

	drvdata->enabled = pdata->init_on;
	platform_set_drvdata(pdev, drvdata);

	return 0;

err_enable:
	sysfs_remove_group(&pdev->dev.kobj, &attr_group);

	return ret;
}

static int pmic_consumer_remove(struct platform_device *pdev)
{
	struct pmic_consumer_data *data = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &attr_group);

	if (data->enabled)
		regulator_bulk_disable(data->num_supplies, data->supplies);

	return 0;
}

static struct platform_driver pmic_consumer_driver = {
	.probe		= pmic_consumer_probe,
	.remove		= pmic_consumer_remove,
	.driver		= {
		.name		= "pmic-consumer",
	},
};

module_platform_driver(pmic_consumer_driver);

MODULE_AUTHOR("Vinicius Maciel <viniciusfre@gmail.com>");
MODULE_DESCRIPTION("PMIC consumer for tps65217 regulator");
MODULE_LICENSE("GPL");