import * as exposes from 'zigbee-herdsman-converters/lib/exposes';

const e = exposes.presets;
const ea = exposes.access;

const BATCH_CLUSTER_ID = 0xfc01;
const BATCH_COMMAND_ID = 0x00;

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

function decodeBatch(model, msg) {
    if (msg.endpoint?.ID !== 1) return;
    if (![BATCH_CLUSTER_ID, 'fc01', '0xfc01', String(BATCH_CLUSTER_ID)].includes(msg.cluster)) return;

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
    if (flags & (1 << 5)) result.previous_tx_wait_ms = data.readUInt16LE(13);

    return Object.keys(result).length > 0 ? result : undefined;
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
        e.numeric('previous_tx_wait_ms', ea.STATE)
            .withUnit('ms')
            .withDescription('Previous batch transmission wait time')
            .withCategory('diagnostic'),
    ],
};

export default definition;
