const IS_LIN_COMPAT = require('../lib/is-linear-engine-compatible.js');

it('should label linear correctly', function () {
	const samples = [/abc/,    // Plain string
		               /a+/,     // + Greedy
		               /a+?/,    // + Non-greedy
		               /a*/,     // * Greedy
		               /a*?/,    // + Non-greedy
		               /(a+)+/,  // Groups but no reference
		               /(a|b)*/, // Groups but no reference
		               /^z+(caught)(?:notcaught).*[aeiou].[a-z][^:]\s(a|b|[cde].*?(f|g)h)\d\wz?z+?acz{8}\Sz{3,8}(yz)a\b\Wz{15,}a(bc)yzbc\Z\D\v\B[\b]\cA\t\0\x25\u1234$/, // (Almost?) every feature
									 /\\1/,    // A backslash and a 1, not \1
									 '(?<value>a)', // Named group but no reference
									 '(?#ignore me \\1)<-- should be nothing to the left', // Backreference but in a Python comment

	                 // As a string -- contains different escaping
	                 '/$',
	                 '(Pre|Pixi)/\\d+\\.\\d+',
	                 '<p>Footer HTML</p>',
									 'https://img.shields.io/.*',
									 ':[a-zA-Z]*/',
									 '; *(Zio|Hydro|Torque|Event|EVENT|Echo|Milano|Rise|URBANO PROGRESSO|WX04K|WX06K|WX10K|KYL21|101K|C5[12]\\d{2}) Build/',
									 '^((\\+\\+)|(\\+\\=)|(\\-\\-)|(\\-\\=)|(\\*\\*)|(\\*\\=)|(\\/\\/)|(\\/\\=)|(==)|(!=)|(<=)|(>=)|(<>)|(<<)|(>>)|(//))',
									 '/library',
									 '/\\*[\\W]*pragma:[\\W]*[^_]*_START',
									 '(.*)/node_modules/([^/]*)/(.*)',
		              ];

  samples.forEach((s) => {
		expect(IS_LIN_COMPAT(s)).toBe(true);
	});
	return;
});

it('should label non-linear correctly', function () {
	const samples = [/(a) \1/, // Backreference
	                 '(?P<value>a)\\k<value>\\1', // Python named group. Use string encoding because this is not a valid JS regex.
									 '(?=a)bc',  // Look-ahead (positive)
									 '(?!a)bc',  // Look-ahead (negative)
									 '(?<=a)bc', // Look-behind (positive) -- not JS legal
									 '(?<!a)bc', // Look-behind (negative) -- not JS legal
		              ];

  samples.forEach((s) => {
		expect(IS_LIN_COMPAT(s)).toBe(false);
	});
	return;
});
