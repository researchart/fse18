const QOA = require('../lib/quantified-overlapping-adjacent.js');

it('identifies simple qoas', function () {
	const samples = [/\d+\d+/,
		               /a*a*/,
		               /\w*a+/,
		               /\s*\n+/,
		               /.+a*/,
		               /.*\S*/,
									];

  samples.forEach((s) => {
		expect(QOA(s)).toBe(true, `Expected QOA for /${s.source}/`);
	});
});

it('rejects random regexes', function () {
	const samples = [/\d/,
		               /abc def ghi j/,
		               /\d+   \w+/,
		               /\d+111111111111111a+/,
		               /\d+11(abc)11\d+/,  // The (abc) is a wall
		               /.*\w\w\w    (a|b|c)?    [a-c] \n+ .*/, // The \n+ is a wall
		               /.*\w\w\w    (a|b|c)?    [a-c] \n{1,} .*/, // The \n{1,} is a wall
									 '\\d+\\.\\d+?',   // . is a wall
									 '\\d+(\\.\\d+)?', // common refactoring to avoid QOA
									];

  samples.forEach((s) => {
		expect(QOA(s)).toBe(false, `Expected no QOA for /${s.source}/`);
	});
});

it('is not fooled by non-overlapping quantified disjunctions (qas, not qoas)', function () {
	const samples = [/a+b+/,
		               /\s*a*/,
		               /\s+\S*/,
									];

  samples.forEach((s) => {
		expect(QOA(s)).toBe(false, `Expected no QOA for /${s.source}/`);
	});
});

it('identifies complex qoas', function () {
	const samples = [/\d+\.?\d+/,       // separated by an optional period
		               /\d+(abc)?\d+/,    // separated by an optional group
		               /\d+[eE]?\d+/,     // separated by an optional char class
		               /\d+(abc)*\d+/,    // separated by a 0-or-more group
		               /\d+[eE]*\d+/,     // separated by a 0-or-more class (*)
		               /\d+[eE]{0,}\d+/,  // separated by a 0-or-more class ({0,})
		               /\d+[eE]{0,5}\d+/, // separated by a 0-to-5 class
		               /\d+\d\d+/,        // the intervening character is part of the class
		               /\d+.\d+/,         // the intervening character is part of the class
		               /\d+[^\D]*/,       // adjacent but very tricky
		               /[a-c]*[c-d]+/,    // adjacent overlapping char classes
		               /[a-c]*.[c-d]+/,   // adjacent overlapping char classes, the intervening character is part of the class
		               /[a-c]*.?[c-d]+/,   // adjacent overlapping char classes, the intervening character is optional
		               /\d+ \s* \s+ [a-z]* ? ? 1+/, // big gap, \d+ to 1+
		               /\d+ \s* \s+ [a-z]* ? ? \d+/, // big gap, \d+ to \d+
		               /.*\w\w\w    \d\d\d    [a-c] \s*/, // Lots of in-betweens to match
		               /.*\w\w\w    (abc)?    [a-c] \s*/, // Lots of in-betweens to match
		               /.*\w\w\w    (a|b|c)?    [a-c] \s*/, // Lots of in-betweens to match
		               /\d+1111111111111111+/,
		               /.*\w\w\w    (a|b|c)?    [a-c] \n* .*/, // The \n* is not a wall
									 /<!--[\S\s]*?-->|<script([\s\S]*?)>([\s\S]*?)<\/script>/, // Quantification in groups is problematic, but have to peel Groups to find it.


									 // As strings.
									 '\\d+.\\d+?',
									 '\\d+\\.?\\d+?',
									 '([0 +\\-\\#]*)(\\*|\\d+)?', // Ouch.
									 '(\\d+),?(\\d+)?',
									 '(.*)?:(.*)?\\|ms',
									 '(jQuery\\.inArray\\(\\s*)(.+?)(\\s*,\\s*)(.+?)(\\s*[,\\)])',
									 '^refs\\/heads\\/[A-Za-z-]+([A-Za-z-0-9\\/]?[A-Za-z-])*$',
									];

  samples.forEach((s) => {
		const source = s.source || s;
		expect(QOA(s)).toBe(true, `Expected QOA for /${source}/`);
	});
});
