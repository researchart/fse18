#!/usr/bin/env node

const countStarHeight = require('./lib/count-star-height.js'),
      fs = require('fs');

if (process.argv.length !== 3) {
	console.log(`Usage: ${process.argv[1]} pattern.json (keys: pattern [options -- keys: countQuestionMarks minimumRepetitionUpperLimit])`);
	process.exit(1);
}

const file = process.argv[2];
const content = fs.readFileSync(file);
const obj = JSON.parse(content);

try {
	const starHeight = countStarHeight(obj.pattern, obj.options);
	obj.starHeight = starHeight;
}
catch (e) {
  console.error(e);
	obj.starHeight = 'UNKNOWN';
}

console.log(JSON.stringify(obj));
process.exit(0);
