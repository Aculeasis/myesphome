import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.const import CONF_ID
from esphome.components import esp32
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
CONF_DATA = "data"
CONF_NONCRITICAL_ERROR_LIMIT = "noncritical_error_limit"
CONF_WAKEUP_PIN = "wakeup_pin"

MAX_DATA_FIELDS = 21


zigbee_batch_ns = cg.esphome_ns.namespace("zigbee_batch")

ZigbeeBatchComponent = zigbee_batch_ns.class_(
    "ZigbeeBatchComponent",
    cg.Component,
)
DataType = zigbee_batch_ns.enum("DataType", is_class=True)

DATA_TYPES = {
    "uint8_t": DataType.UINT8,
    "uint16_t": DataType.UINT16,
    "uint32_t": DataType.UINT32,
    "int8_t": DataType.INT8,
    "int16_t": DataType.INT16,
    "int32_t": DataType.INT32,
}


def _validate_cluster_ids(config):
    if config[CONF_CLUSTER_ID] == config[CONF_DIAGNOSTIC_CLUSTER_ID]:
        raise cv.Invalid(
            "zigbee_batch data and diagnostic cluster IDs must differ"
        )
    return config


def _validate_data_fields(value):
    fields = cv.ensure_list(cv.enum(DATA_TYPES, lower=True))(value)
    if len(fields) > MAX_DATA_FIELDS:
        raise cv.Invalid(
            f"zigbee_batch supports at most {MAX_DATA_FIELDS} data fields"
        )
    return fields


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

            cv.Optional(CONF_DATA, default=[]): _validate_data_fields,

            cv.Optional(CONF_NONCRITICAL_ERROR_LIMIT, default=5):
                cv.int_range(min=1, max=16),

            cv.Optional(CONF_WAKEUP_PIN): cv.int_range(min=0, max=30),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    _validate_cluster_ids,
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
    cg.add(var.set_data_field_count(len(config[CONF_DATA])))
    for index, data_type in enumerate(config[CONF_DATA]):
        cg.add(var.set_data_type(index, data_type))
    cg.add(
        var.set_noncritical_error_limit(
            config[CONF_NONCRITICAL_ERROR_LIMIT]
        )
    )
    if CONF_WAKEUP_PIN in config:
        cg.add(var.set_wakeup_pin(config[CONF_WAKEUP_PIN]))

    # Zigbee v2 uses ESP-IDF tickless idle for stack-aware light sleep. This
    # lets its keepalive deadline wake the chip independently of measurements.
    esp32.add_idf_sdkconfig_option("CONFIG_PM_ENABLE", True)
    esp32.add_idf_sdkconfig_option(
        "CONFIG_FREERTOS_USE_TICKLESS_IDLE", True
    )

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
