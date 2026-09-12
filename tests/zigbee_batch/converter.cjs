// Execute the actual converter; only the exposes UI builder is stubbed.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.resolve(__dirname, '../..');
const source = fs.readFileSync(path.join(root, 'z2m-converters/esp32_c6_th.mjs'), 'utf8');
const expose = new Proxy({}, {get: () => () => expose});
const context = vm.createContext({Buffer, exposes: {presets: expose, access: {STATE: 1}}});
vm.runInContext(source.replace(/^import .*;\r?\n/m, '').replace('export default definition;', 'globalThis.definition = definition;'), context);
const definition = context.definition;
assert.equal(definition.zigbeeModel, undefined);
assert.equal(definition.model, 'ptvo.switch');
function matches(modelID, manufacturerName) {
    return definition.fingerprint.some(f => f.modelID === modelID && f.manufacturerName === manufacturerName);
}
assert(matches('ptvo.switch', 'esphome'));
assert(!matches('ptvo.switch', 'ptvo'));
assert(!matches('ptvo.switch', undefined));
assert(!matches('another.model', 'esphome'));
function message(cluster, data) { return {endpoint: {ID: 1}, type: 'raw', cluster, data: Buffer.from(data)}; }
const decode = definition.fromZigbee[0].convert;
assert.equal(decode(null, message(0xfc01, [0x19, 1, 0, 1, 0x0c, 0xfe])).temperature, -5);
assert.equal(decode(null, message(0xfc01, [0x19, 1, 0, 1, 0x0c])), undefined);
for (const uptime of [16777217, 0xffffffff]) {
    const frame = Buffer.from([0x19, 1, 0, 0x10, 0, 0, 0, 0]);
    frame.writeUInt32LE(uptime, 4);
    assert.equal(decode(null, message(0xfc01, frame)).uptime, uptime);
}
const diagnostic = definition.fromZigbee[1].convert(null,
    message(0xfc02, [0x19, 1, 0, 2, 0x67, 0xea, 9, 0, 0, 0]));
assert.equal(diagnostic.error, 'ERROR_CONFIRM_TIMEOUT');
assert.equal(diagnostic.error_sequence, 9);
console.log('PASS: converter selection, exact uint32 decoding, temperature, malformed frame, diagnostics');
