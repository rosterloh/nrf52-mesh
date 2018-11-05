#pragma once

#define CID_ZEPHYR 0x0002

#define STATE_OFF	0x00
#define STATE_ON	0x01
#define STATE_DEFAULT	0x01
#define STATE_RESTORE	0x02

struct environment_state {
        double temp;
        double press;
        double humidity;
        double co2;
        double voc;
};

struct vendor_state {
	int current;
	u32_t response;
	u8_t last_tid;
	u16_t last_tx_addr;
	s64_t last_msg_timestamp;
};

extern struct environment_state env_srv_root_user_data;

//extern struct bt_mesh_model root_models[];
//extern struct bt_mesh_model vnd_models[];

extern const struct bt_mesh_comp comp;

void env_publisher(struct bt_mesh_model *model);