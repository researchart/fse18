#!/usr/bin/env node

const IS_LIN_COMPAT = require('./lib/is-linear-engine-compatible.js'),
      fs = require('fs');

if (process.argv.length !== 3) {
	console.log(`Usage: ${process.argv[1]} pattern.json`);
	process.exit(1);
}

const file = process.argv[2];
const content = fs.readFileSync(file);
const obj = JSON.parse(content);

try {
	const isCompat = IS_LIN_COMPAT(obj.pattern);
	obj.isLinearEngineCompatible = isCompat ? 1 : 0;
}
catch (e) {
  console.error(e);
	obj.isLinearEngineCompatible = 'UNKNOWN';
}

console.log(JSON.stringify(obj));
process.exit(0);
