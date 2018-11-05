#include <logging/log.h>
#define LOG_MODULE_NAME mesh_module
#define LOG_LEVEL CONFIG_MESH_MODULE_LOG_LEVEL
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include "mesh.h"
#include "device_composition.h"

#ifdef OOB_AUTH_ENABLE

static int output_number(bt_mesh_output_action_t action, u32_t number)
{
	LOG_DBG("OOB Number: %u", number);
	return 0;
}

static int output_string(const char *str)
{
	LOG_DBG("OOB String: %s", str);
	return 0;
}

#endif

static void prov_complete(u16_t net_idx, u16_t addr)
{
        LOG_INF("Provisioning complete");
}

static void prov_reset(void)
{
        LOG_DBG("Provisioning reset");
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

static u8_t dev_uuid[16] = { 0xdd, 0xdd };

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,

#ifdef OOB_AUTH_ENABLE

	.output_size = 6,
	.output_actions = BT_MESH_DISPLAY_NUMBER | BT_MESH_DISPLAY_STRING,
	.output_number = output_number,
	.output_string = output_string,

#endif

	.complete = prov_complete,
	.reset = prov_reset,
};

void bt_ready(int err)
{
	struct bt_le_oob oob;

	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}

	LOG_DBG("Bluetooth initialised");

	err = bt_mesh_init(&prov, &comp);
	if (err) {
		LOG_ERR("Initialising mesh failed (err %d)", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	/* Use identity address as device UUID */
	if (bt_le_oob_get_local(BT_ID_DEFAULT, &oob)) {
		LOG_ERR("Identity Address unavailable");
	} else {
		memcpy(dev_uuid, oob.addr.a.val, 6);
	}

	bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);

	LOG_INF("Mesh initialised");
}