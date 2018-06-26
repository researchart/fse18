/* Author: Jamie Davis <davisjam@vt.edu>
 * Description: Check whether this regex has quantified overlapping disjunctions like '(a|a)+'
 *
 * QOD is a heuristic for regex vulnerability.
 * If the QOD is pumpable, the result is an exponential blow-up.
 */

const createRegExp = require('./create-regexp'),
      regexpTree = require('regexp-tree');
      RTAGI = require('./regexp-tree-any-groups-intersect.js');

/**
 * regex: String representation 'abc' or RegExp object /abc/
 *        Accepts most of the Python syntax as well.
 * options: optional -- clarify what counts as 'quantified'
 *          Keys:
 *            countQuestionMarks -- (a|a)? is not really a problem
 *              default false (i.e. ignore ?'s)
 *            minimumRepetitionUpperLimit -- (a|a){0,3} is probably OK, (a|a){0,50} is not.
 *              default 0 (i.e. all of {x}, {x,}, {,x}, {x,y} are problematic)
 *
 * Returns true or false: whether or not the regex has QOD.
 */
function hasQuantifiedOverlappingDisjunction(regex, options) {
	console.error(`regex ${regex}`);
	/* Get an ast. */
	const re = createRegExp(regex);
	console.error(`re: /${re.source}/`);
	//const ast = regexpTree.parser.setOptions({captureLocations: true}).parse(re);
	const ast = regexpTree.parse(re);

	/* Options. */
	const default_countQuestionMarks = false;
	const default_minimumRepetitionUpperLimit = 0;
	if (!options) {
		options = {};
	}
	// Apply defaults.
	if (!options.countQuestionMarks) {
		options.countQuestionMarks = default_countQuestionMarks;
	}
	if (!options.minimumRepetitionUpperLimit) {
		options.minimumRepetitionUpperLimit = default_minimumRepetitionUpperLimit;
	}
	console.error(`countQuestionMarks ${options.countQuestionMarks} minimumRepetitionUpperLimit ${options.minimumRepetitionUpperLimit}`);

	/* Here we go! */

	//console.error(`pattern /${re.source}/ yields AST:\n${regexpTree.transform(re).getAST()}`);
	console.error(`pattern /${re.source}/ yields AST:\n${JSON.stringify(ast, null, 2)}`);

	let starHeight = 0; // If > 0, we are in a repetition.
	let disjunctionDepth = 0; // If > 0, we are in a disjunction.
	let disjunctionGroups = []; // Set of options in the disjunction.
	let hasQOD = false; // Set if we ever find a QOD.
	regexpTree.traverse(ast, {
		'Repetition': {
			pre({node}) {
				// Optional things to ignore
				if (node.quantifier && node.quantifier.kind === '?' && !options.countQuestionMarks) {
					return;
				}
				else if (node.quantifier.kind === 'Range' &&
					       ('to' in node.quantifier && node.quantifier.to < options.minimumRepetitionUpperLimit)) {
					console.error(`Ignoring quantifier: q ${JSON.stringify(node.quantifier)} options ${JSON.stringify(options)}`);
					return;
				}

				starHeight++;
			},
			post({node}) {
				if (node.quantifier && node.quantifier.kind === '?' && !options.countQuestionMarks) {
					return;
				}
				else if (node.quantifier.kind === 'Range' &&
					       ('to' in node.quantifier && node.quantifier.to < options.minimumRepetitionUpperLimit)) {
					return;
				}

				starHeight--;
			}
		},

		'Disjunction': {
			pre({node}) {
				disjunctionDepth++;
				disjunctionGroups.push(node.left); // left is a "thing"
				// right might be more disjunction in which case we'll encounter it via more traversing
				if (node.right.type !== 'Disjunction') {
					disjunctionGroups.push(node.right);
				}
			},
			post({node}) {
				disjunctionDepth--;
				if (disjunctionDepth === 0) {
					// End of the disjunction. Do we have a QOD?
					console.error(`Finished a disjunction with starHeight ${starHeight}. Here are disjunctionGroups: ${JSON.stringify(disjunctionGroups)}`);
					if (0 < starHeight && RTAGI.anyGroupsIntersect(disjunctionGroups)) {
						hasQOD = true;
					}
				}
			}
		},
	});

	console.error(`hasQuantifiedOverlappingDisjunction: regex ${regex} options ${JSON.stringify(options)} hasQOD ${hasQOD}`);
	return hasQOD;
}

module.exports = hasQuantifiedOverlappingDisjunction;
