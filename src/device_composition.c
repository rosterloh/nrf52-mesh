#include <gpio.h>
//#include <math.h>

#include "mesh.h"
#include "device_composition.h"
#include "sensors.h"
#include "devices.h"

static struct bt_mesh_cfg_srv cfg_srv = {
	.relay = BT_MESH_RELAY_DISABLED,
	.beacon = BT_MESH_BEACON_ENABLED,

#if defined(CONFIG_BT_MESH_FRIEND)
	.frnd = BT_MESH_FRIEND_ENABLED,
#else
	.frnd = BT_MESH_FRIEND_NOT_SUPPORTED,
#endif

#if defined(CONFIG_BT_MESH_GATT_PROXY)
	.gatt_proxy = BT_MESH_GATT_PROXY_ENABLED,
#else
	.gatt_proxy = BT_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif

	.default_ttl = 7,

	/* 3 transmissions with 20ms interval */
	.net_transmit = BT_MESH_TRANSMIT(2, 20),
	.relay_retransmit = BT_MESH_TRANSMIT(2, 20),
};

static struct bt_mesh_health_srv health_srv = {
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

BT_MESH_MODEL_PUB_DEFINE(env_srv_pub_root, NULL, 2 + 1);
BT_MESH_MODEL_PUB_DEFINE(env_cli_pub_root, NULL, 2 + 1);

BT_MESH_MODEL_PUB_DEFINE(vnd_pub, NULL, 3 + 6);

/* Definitions of models user data */
struct environment_state env_srv_root_user_data;

struct vendor_state vnd_user_data;

static struct bt_mesh_elem elements[];

/* Environment Server message handlers */
static void env_get(struct bt_mesh_model *model,
		    struct bt_mesh_msg_ctx *ctx,
		    struct net_buf_simple *buf)
{
	struct net_buf_simple *msg = NET_BUF_SIMPLE(2 + 1 + 4);
	struct environment_state *state = model->user_data;

        sensors_get_env(state);

	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_ENV_STATUS);
	net_buf_simple_add_u8(msg, (u8_t) /*round(*/state->temp/*)*/);

	if (bt_mesh_model_send(model, ctx, msg, NULL, NULL)) {
		printk("Unable to send ENV_SRV Status response\n");
	}
}

/* Environment Client message handlers */
static void env_status(struct bt_mesh_model *model,
		       struct bt_mesh_msg_ctx *ctx,
		       struct net_buf_simple *buf)
{
	printk("Acknownledgement from ENV_SRV\n");
	printk("Temp = %02x\n", net_buf_simple_pull_u8(buf));
}

void env_publisher(struct bt_mesh_model *model)
{
	struct net_buf_simple *msg = model->pub->msg;
	struct environment_state *state = model->user_data;

	if (model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
		int err;
                
                sensors_get_env(state);

                bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_ENV_STATUS);

                err = bt_mesh_model_publish(model);
		if (err) {
			printk("bt_mesh_model_publish err %d\n", err);
		}
        }
}

/* Vendor Model message handlers*/
static void vnd_get(struct bt_mesh_model *model,
		    struct bt_mesh_msg_ctx *ctx,
		    struct net_buf_simple *buf)
{
	struct net_buf_simple *msg = NET_BUF_SIMPLE(3 + 6 + 4);
	struct vendor_state *state = model->user_data;

	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_3(0x04, CID_ZEPHYR));
	net_buf_simple_add_le16(msg, state->current);
	net_buf_simple_add_le32(msg, state->response);

	if (bt_mesh_model_send(model, ctx, msg, NULL, NULL)) {
		printk("Unable to send VENDOR Status response\n");
	}
}

static void vnd_set_unack(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{
	u8_t tid;
	int current;
	s64_t now;
	struct vendor_state *state = model->user_data;

	current = net_buf_simple_pull_le16(buf);
	tid = net_buf_simple_pull_u8(buf);

	now = k_uptime_get();
	if (state->last_tid == tid && state->last_tx_addr == ctx->addr &&
	    (now - state->last_msg_timestamp <= 6000)) {
		return;
	}

	state->last_tid = tid;
	state->last_tx_addr = ctx->addr;
	state->last_msg_timestamp = now;
	state->current = current;

	/* This is dummy response for demo purpose */
	state->response = 0xA578FEB3;

	printk("Vendor model message = %04x\n", state->current);

	if (state->current == STATE_ON) {
		/* LED On */
		gpio_pin_write(led_device[1], LED1_GPIO_PIN, 0);
	} else {
		/* LED Off */
		gpio_pin_write(led_device[1], LED1_GPIO_PIN, 1);
	}
}

static void vnd_set(struct bt_mesh_model *model,
		    struct bt_mesh_msg_ctx *ctx,
		    struct net_buf_simple *buf)
{
	vnd_set_unack(model, ctx, buf);
	vnd_get(model, ctx, buf);
}

static void vnd_status(struct bt_mesh_model *model,
		       struct bt_mesh_msg_ctx *ctx,
		       struct net_buf_simple *buf)
{
	printk("Acknownledgement from Vendor\n");
	printk("cmd = %04x\n", net_buf_simple_pull_le16(buf));
	printk("response = %08x\n", net_buf_simple_pull_le32(buf));
}

/* Mapping of message handlers for Environment Server (0x1000) */
static const struct bt_mesh_model_op env_srv_op[] = {
	{ BT_MESH_MODEL_OP_2(0x82, 0x01), 0, env_get },
	BT_MESH_MODEL_OP_END,
};

/* Mapping of message handlers for Environment Client (0x1001) */
static const struct bt_mesh_model_op env_cli_op[] = {
	{ BT_MESH_MODEL_OP_2(0x82, 0x1), 1, env_status },
	BT_MESH_MODEL_OP_END,
};

/* Mapping of message handlers for Vendor (0x4321) */
static const struct bt_mesh_model_op vnd_ops[] = {
	{ BT_MESH_MODEL_OP_3(0x01, CID_ZEPHYR), 0, vnd_get },
	{ BT_MESH_MODEL_OP_3(0x02, CID_ZEPHYR), 3, vnd_set },
	{ BT_MESH_MODEL_OP_3(0x03, CID_ZEPHYR), 3, vnd_set_unack },
	{ BT_MESH_MODEL_OP_3(0x04, CID_ZEPHYR), 6, vnd_status },
	BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_model root_models[] = {
        BT_MESH_MODEL_CFG_SRV(&cfg_srv),
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),

	BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_SRV,
                      env_srv_op, &env_srv_pub_root,
		      &env_srv_root_user_data),
        BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_CLI,
                      env_cli_op, &env_cli_pub_root,
                      NULL),
};

static struct bt_mesh_model vnd_models[] = {
	BT_MESH_MODEL_VND(CID_ZEPHYR, 0x4321, vnd_ops,
			  &vnd_pub, &vnd_user_data),
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, vnd_models),
};

const struct bt_mesh_comp comp = {
	.cid = CID_ZEPHYR,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};