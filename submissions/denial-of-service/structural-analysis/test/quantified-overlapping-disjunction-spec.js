const QOD = require('../lib/quantified-overlapping-disjunction.js');

it('identifies simple qods', function () {
	const samples = [/(a|a)*/, 
		               /(a|a)+/,
		               /  (a|a)+  /,
		               /(a|b|a)+/,
		               /(b|a|b|a)+/,
									];

  samples.forEach((s) => {
		expect(QOD(s)).toBe(true, `Expected QOD for /${s.source}/`);
	});
});

it('is not fooled by non-overlapping quantified disjunctions (qds, not qods)', function () {
	const samples = [/(a|b)*/, 
		               /(a|b)+/,
									];

  samples.forEach((s) => {
		expect(QOD(s)).toBe(false, `Expected no QOD for /${s.source}/`);
	});
});

it('identifies complex qods', function () {
	const samples = [/(a|\w)*/,         // a overlaps with \w
		               /(a|.)+/,          // a overlaps with '.'
		               /((a|a))+/,        // quantified but only nested-ly so
		               /([a-c]|[c-e])+/,  // [a-c] and [c-e] overlap
		               /([a-bb-c]|[c-e])+/,  // [a-bb-c] and [c-e] overlap
		               /([\x00-\x10]|\x05)+/,  // overlapping hex
		               /([\u0000-\u0000]|\u0000)+/,  // overlapping unicode
		               /(\x61|a)*/,       // equivalent hex and ascii
		               /(\u0061|a)*/,     // equivalent unicode and ascii
		               /(\w|[a-z])+/,     // \w overlaps with [a-z]
									 /(\w|\S)+/,        // \w overlaps with \S
									 /((\w)?|((?:a)))*/, // Tons of nesting

									 '^(\\s|\\n\\t)+$',
									 '(\\d|\\d\\.|\\.\\d)+', // OK, we'll never get this one, just a reference point.
									];

  samples.forEach((s) => {
		const source = s.source || s;
		expect(QOD(s)).toBe(true, `Expected QOD for /${source}/`);
	});
});

it('identifies qods even if they aren\'t dangerous', function () {
	const samples = [/(abc(d|d))+/, // has QOD but it's not pumpable
									];

  samples.forEach((s) => {
		expect(QOD(s)).toBe(true, `Expected QOD for /${s.source}/`);
	});
});

it('does not flag non-disjunctions', function () {
	const samples = [/(a)*/, 
		               /(a+)+/,
									];

  samples.forEach((s) => {
		expect(QOD(s)).toBe(false, `Expected no QOD for /${s.source}/`);
	});
});

it('honors its options: countQuestionMarks', function () {
	const maybeVulns = [/(a|a)?/,
										 ];

  maybeVulns.forEach((s) => {
		expect(QOD(s)).toBe(false, `Expected no QOD for /${s.source}/ with countQuestionMarks default (false)`);
		expect(QOD(s, {countQuestionMarks: false})).toBe(false, `Expected no QOD for /${s.source}/ with countQuestionMarks false`);
		expect(QOD(s, {countQuestionMarks: true})).toBe(true, `Expected QOD for /${s.source}/ with countQuestionMarks true`);
	});
});

it('honors its options: minimumRepetitionUpperLimit', function () {
	const samples = [/(a|a){0,100}/, 
									];
	
	const alwaysVuln = /(a|a){0,}/;

	expect(QOD(alwaysVuln)).toBe(true, `Expected QOD for /${alwaysVuln.source}/`);

	const maybeVulns = [/(a|a){0,10}/,
	                    /(a|a){5,10}/,
										 ];

  maybeVulns.forEach((s) => {
		expect(QOD(s)).toBe(true, `Expected QOD for /${s.source}/ with limit default (0)`);
		expect(QOD(s, {minimumRepetitionUpperLimit: 100})).toBe(false, `Expected no QOD for /${s.source}/ with limit 100`);
		expect(QOD(s, {minimumRepetitionUpperLimit: 5})).toBe(true, `Expected QOD for /${s.source}/ with limit 5`);
	});
});
