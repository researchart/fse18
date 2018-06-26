const createRegExp = require('../lib/create-regexp.js');

it('should handle JS RegExp objects', function () {
	const samples = [/abc/,    // Plain string
		               /a+/,     // + Greedy
		               /a+?/,    // + Non-greedy
		               /a*/,     // * Greedy
		               /a*?/,    // + Non-greedy
		               /(a+)+/,  // Groups but no reference
		               /(a|b)*/, // Groups but no reference
		               /^z+(caught)(?:notcaught).*[aeiou].[a-z][^:]\s(a|b|[cde].*?(f|g)h)\d\wz?z+?acz{8}\Sz{3,8}(yz)a\b\Wz{15,}a(bc)yzbc\Z\D\v\B[\b]\cA\t\0\x25\u1234$/, // (Almost?) every feature
									 /\\1/,    // A backslash and a 1, not \1
		              ];

  samples.forEach((s) => {
		const result = createRegExp(s);
		expect(result.source).toBe(s.source, `Error, expected /${s.source}/ got /${result.source}/`);
	});
	return;
});

it('should handle JS regexp strings', function () {
	const samples = [/abc/.source,    // Plain string
		               /a+/.source,     // + Greedy
		               /a+?/.source,    // + Non-greedy
		               /a*/.source,     // * Greedy
		               /a*?/.source,    // + Non-greedy
		               /(a+)+/.source,  // Groups but no reference
		               /(a|b)*/.source, // Groups but no reference
		               /^z+(caught)(?:notcaught).*[aeiou].[a-z][^:]\s(a|b|[cde].*?(f|g)h)\d\wz?z+?acz{8}\Sz{3,8}(yz)a\b\Wz{15,}a(bc)yzbc\Z\D\v\B[\b]\cA\t\0\x25\u1234$/.source, // (Almost?) every feature
									 /\\1/.source,    // A backslash and a 1, not \1
		              ];

  samples.forEach((s) => {
		const result = createRegExp(s);
		expect(result.source).toBe(s, `Error, expected /${s}/ got /${result.source}/`);
	});
	return;
});

it('should handle Python regexp strings', function () {
	const samples = [{'pattern': '(?P<value>a)\\k<value>\\1',   'exp': '(a)\\1\\1'},          // Python named group. Use string encoding because this is not a valid JS regex.
		               {'pattern': '(?#comment)<-- nothing left', 'exp': '<-- nothing left'}, // Python comment.
		              ];

  samples.forEach((s) => {
		const result = createRegExp(s.pattern);
		expect(result.source).toBe(s.exp, `Error, expected /${s.exp}/ from /${s.pattern}/, but I got /${result.source}/`);
	});
	return;
});
