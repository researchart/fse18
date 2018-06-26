const createRegExp = require('../lib/create-regexp.js'),
      regexpTree = require('regexp-tree'),
      RTAGI = require('../lib/regexp-tree-any-groups-intersect.js');

it('should say groups intersect', function () {
	const group_samples = [[/a/, /a/], // Normal chars

	                       // Metachar and itself
	                       [/./, /./],
	                       [/\d/, /\d/],
	                       [/\s/, /\s/],
	                       [/\t/, /\t/],

	                       // Metachar and one element thereof
	                       [/\s/, /\t/],
	                       [/\s/, / /],
	                       [/\d/, /0/],
	                       [/./, /a/],

	                       // Metachar and sub-class
	                       [/./, /\s/],
	                       [/./, /\S/],
	                       [/\w/, /\S/],
	                       [/\s/, /[\n\t]/],
	                       [/\d/, /[0-3]/],

	                       [/\xFF/, /\xFF/], // idk but should match
	                       [/\uFF00/, /\uFF00/], // idk but should match
	                       [/\uFFFF/, /\uFFFF/], // idk but should match
	                       [/[a-c]/, /[c-f]/],
	                      ];

	group_samples.forEach((s) => {
		const rtGroup = makeRTGroup(s);
		console.error(`\n\nTesting ${JSON.stringify(rtGroup)}`);
		expect(RTAGI.anyGroupsIntersect(rtGroup)).toBe(true, `Error, expected intersecting groups with rtGroup <${JSON.stringify(rtGroup)}>`);
	});
});

it('should say groups do not intersect', function () {
	const group_samples = [[/a/, /b/], // Normal chars

	                       // Metachars
	                       [/./, /\n/],
	                       [/\d/, /\D/],
	                       [/\s/, /\S/],
	                       [/\t/, / /],

	                       [/\xFF/, /\x0F/], 
	                       [/\uFF00/, /\uFF01/],
	                       [/\uFFFF/, /\uFFF0/],
	                       [/[a-c]/, /[d-f]/],
	                      ];

	group_samples.forEach((s) => {
		const rtGroup = makeRTGroup(s);
		console.error(`\n\nTesting ${JSON.stringify(rtGroup)}`);
		expect(RTAGI.anyGroupsIntersect(rtGroup)).toBe(false, `Error, expected non-intersecting groups with rtGroup <${JSON.stringify(rtGroup)}>`);
	});
});

// This is not a rigorous test.
// This API is for utility and is heavily used by any calls to RTAGI.anyGroupsInteresect.
// Just a smoke test.
it('should have a codePoints API that seems to work', function () {
	// 'a' and '[a]' should have the same codePoints.
	const astNodes = [regexpTree.parse(/a/).body,
	                  regexpTree.parse(/[a]/).body,
									 ];

	const codePoints = astNodes.map((node) => RTAGI.getCodePoints(node));

	// All codePoints sets should be the same size.
	const sizes = astNodes.map((s) => s.size);
	const allSizes = new Set();
	sizes.forEach((s) => {
		allSizes.add(s);
	});
	expect(allSizes.size).toBe(1, `Error, more than one size of codePoints from equivalent ASTs`);

	// Every codePoint in set 0 should appear in every other set.
	codePoints[0].forEach((p) => {
		for (let i = 1; i < codePoints.length; i++) {
			expect(codePoints[i].has(p)).toBe(true, `Error, could not point codePoint ${p} in codePoints set ${i}`);
		}
	});
});

// This is very much not a rigorous test.
// This API is for utility. Make sure it exists.
it('should have a codePointsToArray API', function () {
	expect(typeof(RTAGI.codePointsToArray)).toBe('function', `Error, no codePointsToArray function`);
	// TODO Should be a bit more testing here.
});


//////////////////////////////
// Utility.
//////////////////////////////

// input: (regexes) list of regexes
// output: (group) list of the 'body' of the regexp-tree AST produced from each regex
function makeRTGroup(regexes) {
	var group = [];
	regexes.forEach((regex) => {
		// Get an ast.
		const re = createRegExp(regex);
		const ast = regexpTree.parse(re);
		group.push(ast.body);
	});
	return group;
}
