import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.const import CONF_ID
from esphome.components.zigbee.const import KEY_ZIGBEE, ZigbeeComponent
from esphome.components.zigbee.const_esp32 import KEY_ZIGBEE_EP
from esphome.core import CORE


DEPENDENCIES = ["zigbee"]

CONF_ZIGBEE_ID = "zigbee_id"
CONF_ENDPOINT = "endpoint"
CONF_DESTINATION_ENDPOINT = "destination_endpoint"
CONF_CLUSTER_ID = "cluster_id"
CONF_DIAGNOSTIC_CLUSTER_ID = "diagnostic_cluster_id"
CONF_COMMAND_ID = "command_id"


zigbee_batch_ns = cg.esphome_ns.namespace("zigbee_batch")

ZigbeeBatchComponent = zigbee_batch_ns.class_(
    "ZigbeeBatchComponent",
    cg.Component,
)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ZigbeeBatchComponent),

            cv.Required(CONF_ZIGBEE_ID):
                cv.use_id(ZigbeeComponent),

            cv.Optional(CONF_ENDPOINT, default=1):
                cv.int_range(min=1, max=240),

            cv.Optional(CONF_DESTINATION_ENDPOINT, default=1):
                cv.int_range(min=1, max=240),

            # Keep it strictly above the custom-cluster minimum boundary.
            cv.Optional(CONF_CLUSTER_ID, default=0xFC01):
                cv.int_range(min=0xFC01, max=0xFFFF),

            cv.Optional(CONF_DIAGNOSTIC_CLUSTER_ID, default=0xFC02):
                cv.int_range(min=0xFC01, max=0xFFFF),

            cv.Optional(CONF_COMMAND_ID, default=0x00):
                cv.int_range(min=0, max=0xFF),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
)


async def to_code(config):
    # All final validators have run by code generation, including Zigbee's
    # endpoint allocation. Checking earlier can miss automatically assigned IDs.
    endpoints = CORE.data.get(KEY_ZIGBEE, {}).get(KEY_ZIGBEE_EP, {})
    if config[CONF_ENDPOINT] not in endpoints:
        raise cv.Invalid(
            f"zigbee_batch endpoint {config[CONF_ENDPOINT]} does not exist. "
            "Select an endpoint declared in the Zigbee configuration "
            f"(available: {sorted(endpoints)})."
        )

    if config[CONF_CLUSTER_ID] == config[CONF_DIAGNOSTIC_CLUSTER_ID]:
        raise cv.Invalid(
            "zigbee_batch data and diagnostic cluster IDs must differ"
        )

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    zb = await cg.get_variable(config[CONF_ZIGBEE_ID])

    cg.add(var.set_zigbee(zb))
    cg.add(var.set_endpoint(config[CONF_ENDPOINT]))
    cg.add(
        var.set_destination_endpoint(
            config[CONF_DESTINATION_ENDPOINT]
        )
    )
    cg.add(var.set_cluster_id(config[CONF_CLUSTER_ID]))
    cg.add(
        var.set_diagnostic_cluster_id(
            config[CONF_DIAGNOSTIC_CLUSTER_ID]
        )
    )
    cg.add(var.set_command_id(config[CONF_COMMAND_ID]))

    # The selected endpoint is created by the Zigbee component before runtime
    # setup registers the device descriptor with the stack.
    cg.add(
        zb.add_cluster(
            config[CONF_ENDPOINT],
            config[CONF_CLUSTER_ID],
            cg.RawExpression("EZB_ZCL_CLUSTER_SERVER"),
        )
    )
    cg.add(
        zb.add_cluster(
            config[CONF_ENDPOINT],
            config[CONF_DIAGNOSTIC_CLUSTER_ID],
            cg.RawExpression("EZB_ZCL_CLUSTER_SERVER"),
        )
    )
