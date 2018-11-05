#include <sensor.h>

#include "devices.h"
#include "device_composition.h"

void sensors_get_env(struct environment_state *state)
{
        struct sensor_value temp, press, humidity, co2, voc;

	sensor_sample_fetch(bme280);
	sensor_channel_get(bme280, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        sensor_channel_get(bme280, SENSOR_CHAN_PRESS, &press);
        sensor_channel_get(bme280, SENSOR_CHAN_HUMIDITY, &humidity);
	printk("Temp: %d.%06d; Press: %d.%06d; Humidity: %d.%06d\n",
		    temp.val1, temp.val2, press.val1, press.val2,
		    humidity.val1, humidity.val2);

	sensor_sample_fetch(ccs811);
	sensor_channel_get(ccs811, SENSOR_CHAN_CO2, &co2);
        sensor_channel_get(ccs811, SENSOR_CHAN_VOC, &voc);
	printk("Co2: %d.%06dppm; VOC: %d.%06dppb\n", co2.val1, co2.val2,
            voc.val1, voc.val2);

        state->temp = sensor_value_to_double(&temp);
        state->press = sensor_value_to_double(&press);
        state->humidity = sensor_value_to_double(&humidity);
        state->co2 = sensor_value_to_double(&co2);
        state->voc = sensor_value_to_double(&voc);
}