const countStarHeight = require('../lib/count-star-height.js');

it('should label star height zero', function () {
	const samples = [/abc/,     // Plain string
	                 /(foo)/,   // Group!
	                 /(foo)\*/, // Escaped * is not repetition
	                 /(foo)\+/, // Escaped + is not repetition

	                 // As a string -- contains different escaping
	                 '/$',
	                 '<p>Footer HTML</p>',
									 '^((\\+\\+)|(\\+\\=)|(\\-\\-)|(\\-\\=)|(\\*\\*)|(\\*\\=)|(\\/\\/)|(\\/\\=)|(==)|(!=)|(<=)|(>=)|(<>)|(<<)|(>>)|(//))',
									 '/library',
		              ];

  samples.forEach((s) => {
		expect(countStarHeight(s)).toBe(0, `expected star height 0 for pattern /${s}/`);
	});
	return;
});

it('should label star height one', function () {
	const samples = [/a+/,     // + Greedy
		               /a+?/,    // + Non-greedy
		               /a*/,     // * Greedy
		               /a*?/,    // + Non-greedy
		               /(a|b)*/, // Groups but one level

	                 // As a string -- contains different escaping
	                 '(Pre|Pixi)/\\d+\\.\\d+',
									 'https://img.shields.io/.*',
									 ':[a-zA-Z]*/',
									 '; *(Zio|Hydro|Torque|Event|EVENT|Echo|Milano|Rise|URBANO PROGRESSO|WX04K|WX06K|WX10K|KYL21|101K|C5[12]\\d{2}) Build/',
									 '/\\*[\\W]*pragma:[\\W]*[^_]*_START',
									 '(.*)/node_modules/([^/]*)/(.*)',
									 '((\\w)?|((?:a)))*', // ? is not star height
									 '^([\\w:](-?|\\.?))+$', // This is vulnerable anyway. Beats me why. 
		              ];

  samples.forEach((s) => {
		expect(countStarHeight(s)).toBe(1, `expected star height 1 for pattern /${s}/`);
	});
	return;
});

it('should label star height two', function () {
	const samples = [/(a+)+/,     // Easy
	                 /(a+){0,}/,  // Same as *
	                 /(a+){10,}/, // Like +
	                 /(a+|b)+/,   // Internal OR is nested
									 /(a|b+)+/,   // Same as previous but the other OR
	                 /(a*|b)+$/,  // Same as previous 2 -- this is the failing examples from safe-regex. https://github.com/substack/safe-regex/issues/8
									 /(a|(b+)+)/, // +1 +1
									 /(a|(ab+c))+/, // +1 0 +1
									 /(ab{0,40}c)*/, // inner is 
									 /(?:ab{0,40}c)*/, // non-capturing group is irrelevant
									 /(?:ab+?)*/, // non-greedy is irrelevant in the inner
									 /(?:ab+)*?/, // non-greedy is irrelevant in the outer
									 /([abc]+)*?/, // quantified char class

	                 // As a string -- contains different escaping
									 '(\\d+)+',
									 '(/*)+',
		              ];

  samples.forEach((s) => {
		expect(countStarHeight(s)).toBe(2, `expected star height 2 for pattern /${s}/`);
	});
	return;
});

it('should label star height three', function () {
	const samples = [/((a+)*)+/,
		              ];

  samples.forEach((s) => {
		expect(countStarHeight(s)).toBe(3, `expected star height 3 for pattern /${s}/`);
	});
	return;
});

it('should label star height four', function () {
	const samples = [/(((a{0,100})*)+?){0,100}/,
		              ];

  samples.forEach((s) => {
		expect(countStarHeight(s)).toBe(4, `expected star height 4 for pattern /${s}/`);
	});
	return;
});

it('should allow optional question mark handling', function () {
	const samples = [/^(foo-?bar)?side/,
	                ];
	
	samples.forEach((s) => {
		expect(countStarHeight(s)).toBe(0, `expected star height 0 for pattern /${s}/ with default question mark behavior (disabled)`);
		expect(countStarHeight(s, { countQuestionMarks: true } )).toBe(2, `expected star height 2 for pattern /${s}/ with question marks enabled`);
	});
	return;
});

it('should support variable repetition upper bound preferences', function () {
	const samples = [/(a+){0,5}/,
	                ];
	
	samples.forEach((s) => {
		expect(countStarHeight(s)).toBe(2, `expected star height 2 for pattern /${s}/ with default minimum upper limit (any)`);
		expect(countStarHeight(s, { minimumRepetitionUpperLimit: 10 } )).toBe(1, `expected star height 1 for pattern /${s}/ with minimum upper limit 10`);
	});
	return;
});
