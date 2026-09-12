import * as exposes from 'zigbee-herdsman-converters/lib/exposes';

const e = exposes.presets;
const ea = exposes.access;

const BATCH_CLUSTER_ID = 0xfc01;
const DIAGNOSTIC_CLUSTER_ID = 0xfc02;
const BATCH_COMMAND_ID = 0x00;

// Must match zigbee_batch.data in the ESPHome sketch. Flag bits are assigned
// in this order; only values whose bit is set occupy bytes in the payload.
const DATA_FIELDS = [
    {name: 'temperature', type: 'int16', divisor: 100},
    {name: 'humidity', type: 'uint16', divisor: 100},
    {name: 'battery_voltage', type: 'uint16', divisor: 1000},
    {name: 'battery', type: 'uint8', divisor: 1},
    {name: 'uptime', type: 'uint32', divisor: 1},
    {name: 'previous_tx_wait', type: 'uint16', divisor: 1},
];

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
    [60015, 'ERROR_PAYLOAD_INCOMPLETE'],
    [60016, 'ERROR_TX_CONTEXT_EXHAUSTED'],
    [60017, 'ERROR_REJOIN_FAILED'],
    [60018, 'ERROR_PM_CONFIG_FAILED'],
    [60019, 'ERROR_BUTTON_IRQ_FAILED'],
    [61008, 'ERROR_TARGET_ADDRESS_UNALLOCATED'],
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

function typeSize(type) {
    if (type === 'uint8' || type === 'int8') return 1;
    if (type === 'uint16' || type === 'int16') return 2;
    return 4;
}

function readValue(data, offset, type) {
    if (type === 'uint8') return data.readUInt8(offset);
    if (type === 'int8') return data.readInt8(offset);
    if (type === 'uint16') return data.readUInt16LE(offset);
    if (type === 'int16') return data.readInt16LE(offset);
    if (type === 'uint32') return data.readUInt32LE(offset);
    if (type === 'int32') return data.readInt32LE(offset);
    return null;
}

function decodeFlexiblePayload(data) {
    if (!data || data.length < 1) return null;

    const flags = [];
    let offset = 0;
    do {
        if (offset >= data.length || flags.length >= 3) return null;
        flags.push(data[offset++]);
    } while ((flags[flags.length - 1] & 0x80) !== 0);

    if (flags.length !== Math.max(1, Math.ceil(DATA_FIELDS.length / 7))) {
        return null;
    }

    const result = {};
    for (let index = 0; index < DATA_FIELDS.length; index++) {
        const present = (flags[Math.floor(index / 7)] &
            (1 << (index % 7))) !== 0;
        if (!present) continue;

        const field = DATA_FIELDS[index];
        const size = typeSize(field.type);
        if (offset + size > data.length) return null;
        result[field.name] = readValue(data, offset, field.type) /
            field.divisor;
        offset += size;
    }

    return offset === data.length ? result : null;
}

function diagnosticPayload(data) {
    if (!data || data.length < 6) return null;
    if (data[2] !== BATCH_COMMAND_ID) return null;

    const version = data[3];
    const payloadLength = version === 1 ? 3 : version === 2 ? 7 : 0;
    if (payloadLength !== 0 && data.length >= 3 + payloadLength) {
        return data.subarray(3, 3 + payloadLength);
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

    const bytes = payloadBytes(msg);
    if (!bytes || bytes.length < 4) return;
    if (bytes[2] !== BATCH_COMMAND_ID) return;

    const result = decodeFlexiblePayload(bytes.subarray(3));
    if (!result) return;

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
