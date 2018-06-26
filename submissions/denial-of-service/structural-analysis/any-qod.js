#!/usr/bin/env node

const anyQuantifiedOverlappingDisjunction= require('./lib/quantified-overlapping-disjunction.js'),
      fs = require('fs');

if (process.argv.length !== 3) {
	console.log(`Usage: ${process.argv[1]} pattern.json (keys: pattern [options -- keys: countQuestionMarks minimumRepetitionUpperLimit])`);
	process.exit(1);
}

const file = process.argv[2];
const content = fs.readFileSync(file);
const obj = JSON.parse(content);

try {
	const anyQOD = anyQuantifiedOverlappingDisjunction(obj.pattern, obj.options);
	obj.anyQOD = anyQOD ? 1 : 0;
}
catch (e) {
	console.error(e);
	obj.anyQOD = 'UNKNOWN';
}

console.log(JSON.stringify(obj));
process.exit(0);
