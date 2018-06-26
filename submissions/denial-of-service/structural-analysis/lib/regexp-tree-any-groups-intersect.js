/* Author: Jamie Davis <davisjam@vt.edu>
 * Description: Utility. Check if a set of regexp-tree nodes has overlap.
 *              Useful for two heuristics: QOD and QOA.
 */

const regexpTree = require('regexp-tree'),
      createRegExp = require('./create-regexp.js');

/**
 * groups: an array of regexp-tree nodes
 *
 * Returns true if any of the groups intersect.
 *   Examples: (a|a)         true
 *            ([a-b]|[b-c])  true
 *            ([a-b]|[c-d])  false
 *            (\s|\S)        false
 */
function regexpTreeAnyGroupsIntersect(groups) {
	if (!groups) {
		return false;
	}

	// Make a copy.
	const groupsCopy = JSON.parse(JSON.stringify(groups));

	// Peel external groups.
	// TODO This is only a partial solution to complex grouping.
	const peeledGroups = groupsCopy.map((g) => {
		while (g.type === 'Group') {
			g = g.expression;
		}
		return g;
	});

	// Remove meta chars.
	const groupsNoMetaChars = peeledGroups.map((g) => {
		return createEquivalentGroupWithoutMetaChars(g);
	});

	// Filter out any groups we can't handle
	const groupsWeCanHandle = groupsNoMetaChars.filter((g) => {
		// Pretty conservative here. Just e.g. (a|b) or ([a-z]|d).
		const supportedClasses = ['Char', 'CharacterClass'];
		return supportedClasses.includes(g.type);
	});

	/* Now we have an array of 0 or more Char/CharacterClass entries. */

	// A group cannot intersect with itself.
	// Otherwise (a+)+ (star height) would be mis-classified as QOD.
	if (groupsWeCanHandle.length < 2) {
		return false;
	}

	// For each entry, compute the corresponding valid characters. This is explosive.
	// Probably not a good idea really.
	const eachGroupsValidCodePoints = groupsWeCanHandle.map((g) => {
		return getCodePoints(g);
	});

	// Compare pairwise codePoints for overlap.
	let overlap = false;
	for (let i = 0; i < eachGroupsValidCodePoints.length; i++) {
		for (let j = i+1; j < eachGroupsValidCodePoints.length; j++) {
			const intersect = setIntersect(eachGroupsValidCodePoints[i], eachGroupsValidCodePoints[j]);
			if (0 < intersect.size) {
				overlap = true;	
				break;
			}
		}

		if (overlap) {
			break;
		}
	}

	console.error(`regexpTreeAnyGroupsIntersect: overlap ${overlap} for groups ${JSON.stringify(groups)}`);
	return overlap;
}

// input: one of the groups that are inputs to this module (a node in a regexp-tree AST)
// output: equivalent node without any 'Char' objects of type 'meta'
// throws on error
function createEquivalentGroupWithoutMetaChars(g) {
	const miniAST = {'type': 'RegExp', 'body': g, 'flags': ''};
	try {
		console.error(`Transforming group: ${JSON.stringify(miniAST)}`);
		const transformed = regexpTree.transform(miniAST, {
			Char(path) {
				const {node} = path;
				if (node.kind === 'meta') {
					// Replace meta char nodes with equivalent char class nodes.
					const charClass = convertMetacharToCharClass(node.value);
					const charClassAsRegExp = createRegExp(charClass);
					let astNode = regexpTree.parse(charClassAsRegExp).body;
					path.replace(astNode);
				}
			}
		});

		return transformed.getAST().body;
	}
	catch (e) {
		console.error(`Unsupported meta characters: ${e}`);
		throw e;
	}

	return undefined;
}

// Input: meta character: '\\w' or 'w', etc.
// Output: equivalent char class (possibly negated)
// Examples: '\\w' -> '[a-zA-Z0-9_]'
//           '\\W' -> '[a-zA-Z0-9_]'
function convertMetacharToCharClass(meta) {

	// Members of the char class corresponding to this metachar.
	// To avoid recursion, use hex encodings for any meta chars, e.g. \s -> [hex of \n \t etc.].
	const metacharToClass = {
		'.': '^\\x0a', // Anything but newline
		'w': 'a-zA-Z0-9_',
		'd': '0-9_',
		's': '\\x20\\x09\\x0a\\x0b\\x0c\\x0d',
		't': '\\x09',
		'n': '\\x0a',
		'v': '\\x0b',
		'f': '\\x0c',
		'r': '\\x0d',
	};

	let charClass = undefined;
	switch(meta) {
		case 'w':
		case '\\w':
			charClass = `[${metacharToClass['w']}]`;
			break;
		case 'W':
		case '\\W':
			charClass = `[^${metacharToClass['w']}]`;
			break;
		case 'd':
		case '\\d':
			charClass = `[${metacharToClass['d']}]`;
			break;
		case 'D':
		case '\\D':
			charClass = `[^${metacharToClass['d']}]`;
			break;
		case 's':
		case '\\s':
			charClass = `[${metacharToClass['s']}]`;
			break;
		case 'S':
		case '\\S':
			charClass = `[^${metacharToClass['s']}]`;
			break;
		case '.':
			charClass = `[${metacharToClass['.']}]`;
			break;
		case 't':
		case '\\t':
			charClass = `[${metacharToClass['t']}]`;
			break;
		case 'n':
		case '\\n':
			charClass = `[${metacharToClass['n']}]`;
			break;
		case 'v':
		case '\\v':
			charClass = `[${metacharToClass['v']}]`;
			break;
		case 'f':
		case '\\f':
			charClass = `[${metacharToClass['f']}]`;
			break;
		case 'r':
		case '\\r':
			charClass = `[${metacharToClass['r']}]`;
			break;
		default:
			throw `Unsupported meta ${meta}`;
	}

	console.error(`meta ${meta} -> charClass ${charClass}`);
	return charClass;
}

/* I can't believe this isn't part of the JS set implementation. */
function setIntersect(s1, s2) {
	var intersection = new Set();
	for (var elem of s2) {
		if (s1.has(elem)) {
			intersection.add(elem);
		}
	}
	return intersection;
}

function setToArray(s) {
	var arr = [];
	s.forEach((v) => {
		arr.push(v);
	});
	return arr;
}

// input: (node) An AST node. Must be either 'Char' or 'CharacterClass'. Meta will be resolved.
// output: Set of codePoints corresponding to this group
function getCodePoints(node) {
	node = createEquivalentGroupWithoutMetaChars(node);

	let codePoints;
	if (node.type === 'Char') {
		const supportedKinds = ['simple', 'hex', 'unicode'];
		if (supportedKinds.includes(node.kind)) {
			console.error(`Adding ${node.codePoint}`);
			codePoints = new Set([node.codePoint]);
		}
		else {
			throw `Unexpected Char ${JSON.stringify(node)}`;
		}
	}
	else if (node.type === 'CharacterClass') {
		// Collect the contents of this class

		// For each expression, get its codePoints
		let classContents = new Set();
		node.expressions.forEach((e) => {
			if (e.type === 'Char') {
				classContents.add(e.codePoint);
			}
			else if (e.type === 'ClassRange') {
				for (let i = e.from.codePoint; i <= e.to.codePoint; i++) { // inclusive
					classContents.add(i);
				}
			}
			else if (e.type === 'CharacterClass') {
				// Recurse, recurse!
				const points = getCodePoints(e);
				points.forEach((p) => {
					classContents.add(p);
				});
			}
			else {
				throw `Unsupported CharacterClass type ${e.type} expression ${JSON.stringify(e)}`;
			}
		});

		// Is it a negated class? Then invert everything.
		if (node.negative) {
			let allCodePointsMinusNegatives = fullSetOfCodePoints();
			classContents.forEach((codePoint) => {
				allCodePointsMinusNegatives.delete(codePoint);
			});	

			codePoints = allCodePointsMinusNegatives;
		}
		else {
			codePoints = classContents;
		}
	}
	else  {
		throw `Unsupported node ${JSON.stringify(node)}`;
	}	

	console.error(`Node ${JSON.stringify(node)} -> [${setToArray(codePoints)}]`);
	return codePoints;
}

function fullSetOfCodePoints() {
	let fullSet = new Set();

	const min = 0; // \u0000
	const max = 65535; // \uFFFF
	for (let i = 0; i < 65535; i++) {
		fullSet.add(i);
	}

	return fullSet;
}

module.exports = { 'anyGroupsIntersect': regexpTreeAnyGroupsIntersect,
                   'getCodePoints': getCodePoints,
                   'codePointsToArray': setToArray,
                 };
