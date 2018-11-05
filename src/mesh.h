#pragma once

#include <settings/settings.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/hci.h>
#include <bluetooth/mesh.h>

/* Model Operation Codes */
#define BT_MESH_MODEL_OP_ENV_GET		BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_ENV_SET		BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_ENV_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_ENV_STATUS		BT_MESH_MODEL_OP_2(0x82, 0x04)

void bt_ready(int err);