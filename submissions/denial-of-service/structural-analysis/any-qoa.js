#!/usr/bin/env node

const anyQuantifiedOverlappingAdjacent = require('./lib/quantified-overlapping-adjacent.js');
      fs = require('fs');

if (process.argv.length !== 3) {
	console.log(`Usage: ${process.argv[1]} pattern.json (keys: pattern [options -- keys: minimumRepetitionUpperLimit])`);
	process.exit(1);
}

const file = process.argv[2];
const content = fs.readFileSync(file);
const obj = JSON.parse(content);

try {
	const anyQOA = anyQuantifiedOverlappingAdjacent(obj.pattern, obj.options);
	obj.anyQOA = anyQOA ? 1 : 0;
}
catch (e) {
	console.error(e);
	obj.anyQOA = 'UNKNOWN';
}

console.log(JSON.stringify(obj));
process.exit(0);
