import * as exposes from 'zigbee-herdsman-converters/lib/exposes';

const e = exposes.presets;
const ea = exposes.access;

const BATCH_CLUSTER_ID = 0xfc01;
const DIAGNOSTIC_CLUSTER_ID = 0xfc02;
const BATCH_COMMAND_ID = 0x00;

const ERROR_NAMES = new Map([
    [60001, 'ERROR_COMPONENT_NULL'],
    [60002, 'ERROR_STACK_NOT_STARTED'],
    [60003, 'ERROR_NOT_JOINED'],
    [60004, 'ERROR_TX_BUSY'],
    [60005, 'ERROR_LOCK_FAILED'],
    [60006, 'ERROR_QUEUE_FAILED'],
    [60007, 'ERROR_CONFIRM_TIMEOUT'],
    [60008, 'ERROR_CONFIRM_NULL'],
    [60009, 'ERROR_SLEEP_FAILED'],
    [60010, 'ERROR_AWAKE_GUARD'],
    [60011, 'ERROR_PARENT_LINK_FAILURE'],
    [60012, 'ERROR_DEVICE_REBOOT_FAILED'],
    [60013, 'ERROR_CONFIG_LOCK_FAILED'],
    [60014, 'ERROR_DIAGNOSTIC_QUEUE_FAILED'],
]);

function payloadBytes(msg) {
    const data = msg.data;

    if (Buffer.isBuffer(data)) return data;
    if (Array.isArray(data)) return Buffer.from(data);
    if (data && Buffer.isBuffer(data.data)) return data.data;
    if (data && Array.isArray(data.data)) return Buffer.from(data.data);

    // Keep compatibility with raw payloads represented as an object whose
    // enumerable keys are byte offsets.
    if (data && typeof data === 'object') {
        const keys = Object.keys(data)
            .filter((key) => /^\d+$/.test(key))
            .sort((a, b) => Number(a) - Number(b));

        if (keys.length > 0) return Buffer.from(keys.map((key) => data[key]));
    }

    return null;
}

function batchPayload(data) {
    if (!data) return null;
    if (data.length === 15 && data[0] === 1) return data;

    // An unknown command can be delivered by zigbee-herdsman as raw ZCL
    // bytes: frame control, transaction sequence, command id, payload.
    if (data.length >= 18 && data[2] === BATCH_COMMAND_ID) {
        const payload = data.subarray(3, 18);
        if (payload[0] === 1) return payload;
    }

    return null;
}

function diagnosticPayload(data) {
    if (!data) return null;
    if (data.length === 3 && data[0] === 1) return data;
    if (data.length === 7 && data[0] === 2) return data;

    if (data.length >= 6 && data[2] === BATCH_COMMAND_ID) {
        const version = data[3];
        const payloadLength = version === 1 ? 3 : version === 2 ? 7 : 0;
        if (payloadLength !== 0 && data.length >= 3 + payloadLength) {
            return data.subarray(3, 3 + payloadLength);
        }
    }

    return null;
}

function clusterMatches(cluster, id, hex) {
    return [id, hex, `0x${hex}`, String(id)].includes(cluster);
}

function decodeBatch(model, msg) {
    if (msg.endpoint?.ID !== 1) return;
    if (!clusterMatches(msg.cluster, BATCH_CLUSTER_ID, 'fc01')) return;

    if (msg.type !== 'raw') return;

    const data = batchPayload(payloadBytes(msg));
    if (!data) return;

    const flags = data[1];
    const result = {};

    if (flags & (1 << 0)) result.temperature = data.readInt16LE(2) / 100.0;
    if (flags & (1 << 1)) result.humidity = data.readUInt16LE(4) / 100.0;
    if (flags & (1 << 2)) result.battery_voltage = data.readUInt16LE(6) / 1000.0;
    if (flags & (1 << 3)) result.battery = data.readUInt8(8);
    if (flags & (1 << 4)) result.uptime = data.readUInt32LE(9);
    if (flags & (1 << 5)) result.previous_tx_wait = data.readUInt16LE(13);

    return Object.keys(result).length > 0 ? result : undefined;
}

function decodeDiagnostic(model, msg) {
    if (msg.endpoint?.ID !== 1) return;
    if (!clusterMatches(msg.cluster, DIAGNOSTIC_CLUSTER_ID, 'fc02')) return;
    if (msg.type !== 'raw') return;

    const data = diagnosticPayload(payloadBytes(msg));
    if (!data) return;

    const code = data.readUInt16LE(1);
    const name = ERROR_NAMES.get(code) ??
        (code >= 61001 && code <= 61255
            ? `ERROR_TX_CONFIRM_0x${(code - 61000).toString(16).padStart(2, '0').toUpperCase()}`
            : `ERROR_UNKNOWN_${code}`);

    const result = {error: name};
    if (data[0] === 2) result.error_sequence = data.readUInt32LE(3);
    return result;
}

const definition = {
    zigbeeModel: ['ptvo.switch'],
    model: 'ptvo.switch',
    vendor: 'ESPHome',
    description: 'ESP32-C6 Temp, Humidity & Battery Sensor',

    extend: [],

    fromZigbee: [
        {
            cluster: BATCH_CLUSTER_ID,
            type: ['raw'],
            convert: decodeBatch,
        },
        {
            cluster: DIAGNOSTIC_CLUSTER_ID,
            type: ['raw'],
            convert: decodeDiagnostic,
        },
    ],
    toZigbee: [],
    exposes: [
        e.temperature().withDescription('Measured temperature'),
        e.humidity().withDescription('Measured relative humidity'),
        e.battery()
            .withDescription('Remaining battery percentage')
            .withCategory('diagnostic'),
        e.numeric('battery_voltage', ea.STATE)
            .withUnit('V')
            .withDescription('Battery voltage')
            .withCategory('diagnostic'),
        e.numeric('uptime', ea.STATE)
            .withUnit('s')
            .withDescription('Device uptime in seconds')
            .withCategory('diagnostic'),
        e.numeric('previous_tx_wait', ea.STATE)
            .withUnit('ms')
            .withDescription('Previous batch transmission wait time')
            .withCategory('diagnostic'),
        e.text('error', ea.STATE)
            .withDescription('First persisted device error')
            .withCategory('diagnostic'),
        e.numeric('error_sequence', ea.STATE)
            .withDescription('Persistent diagnostic event sequence')
            .withCategory('diagnostic'),
    ],
};

export default definition;
