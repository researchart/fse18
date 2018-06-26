/* Author: Jamie Davis <davisjam@vt.edu>
 * Description: Check whether this regex has quantified overlapping adjacent groups like '\d+\d+'.
 *
 * QOA is a heuristic for regex vulnerability.
 * QOA is a strong (perfect?) indicator for polynomial blow-up.
 * Should be at least O(n^2).
 */

const createRegExp = require('./create-regexp'),
      regexpTree = require('regexp-tree'),
      RTAGI = require('./regexp-tree-any-groups-intersect.js');

/**
 * regex: String representation 'abc' or RegExp object /abc/
 *        Accepts most of the Python syntax as well.
 * options: optional -- clarify what counts as 'quantified'
 *          Keys:
 *            minimumRepetitionUpperLimit -- (a|a){0,3} is probably OK, (a|a){0,50} is not.
 *              default 0 (i.e. all of {x}, {x,}, {,x}, {x,y} are problematic)
 *
 * Returns true or false: whether or not the regex has QOA.
 */
function hasQuantifiedOverlappingAdjacent(regex, options) {
	console.error(`regex ${regex}`);
	/* Get an ast. */
	const re = createRegExp(regex);
	console.error(`re: /${re.source}/`);
	const ast = regexpTree.parse(re);

	/* Options. */
	const default_minimumRepetitionUpperLimit = 0;
	if (!options) {
		options = {};
	}
	// Apply defaults.
	if (!options.minimumRepetitionUpperLimit) {
		options.minimumRepetitionUpperLimit = default_minimumRepetitionUpperLimit;
	}
	console.error(`minimumRepetitionUpperLimit ${options.minimumRepetitionUpperLimit}`);

	/* Here we go! */
	console.error(`pattern /${re.source}/ yields AST:\n${JSON.stringify(ast, null, 2)}`);

	let hasQOA = false;
	regexpTree.traverse(ast, {
		'Alternative': function({node}) { // i.e. concatenation
			// If we already found one, no point in continuing.
			if (hasQOA) {
				return;
			}

			let fromEveryQuantifiedToTheEnd = []; // list of lists of nodes
			node.expressions.forEach((_exp, i) => {
				console.error(`Considering exp ${i}:\n  ${JSON.stringify(_exp)}`);

				// Add to every existing run.
				fromEveryQuantifiedToTheEnd.forEach((run) => {
					run.push(_exp);
				});


				// For analysis purposes, we can do some peeling.
				let exp = _exp;
				if (exp.type === 'Group') {
					exp = exp.expression;
				}

				if (nodeIsQuantified(exp, options)) {
					// Start a new run beginning with this quantified entry.
					fromEveryQuantifiedToTheEnd.push([exp]);
				}
			});

			// Now we need to check each run to see if it works.
			console.error(`Finished a pass over an Alternative in regex ${regex}\n  Alternative: ${JSON.stringify(node)}\n  Considering runs: ${JSON.stringify(fromEveryQuantifiedToTheEnd)}`);

			const runsWithQOA = fromEveryQuantifiedToTheEnd.filter((run, i) => {
				console.error(`Trying run ${i}: ${JSON.stringify(run)}`);
				return runHasQOA(run, options);
			});

			if (0 < runsWithQOA.length) {
				hasQOA = true;
			}
		},
	});

	console.error(`hasQuantifiedOverlappingAdjacent: regex ${regex} options ${JSON.stringify(options)} hasQOA ${hasQOA}`);
	return hasQOA;
}

/* @param run: sequence of AST nodes from an Alternative
 * Returns true if there is a QOA in this run.
 */
function runHasQOA(run, options) {
	// For QOA, a run must contain at least two quantifications.
	const quantifiedNodes = run.filter((node) => nodeIsQuantified(node, options));
	if (quantifiedNodes.length < 2) {
		console.error(`Run has only ${quantifiedNodes.length} quantified nodes, QOA is impossible. Run: ${JSON.stringify(run)}`);
		return false;
	}

	// Starting from each quantified node, walk until we reach an impassable barrier or find an overlapping quantified node.
	let hasQOA = false;
	run.forEach((outerRunNode, i) => {
		// Already got a match.
		if (hasQOA) {
			return;
		}

		// Not quantified, nothing to do here.
		if (!nodeIsQuantified(outerRunNode, options)) {
			return;
		}
		
		// We have a quantified node.

		// Walk through the remainder of the run.
		let chanceOfQOA = true;
		const runRemainder = run.slice(i+1);
		runRemainder.forEach((_subsequentNode, j) => {
			// Did we give up?
			if (!chanceOfQOA) {
				return;
			}

			// Peel groups.
			let subsequentNode = _subsequentNode;
			if (subsequentNode.type === 'Group') {
				subsequentNode = subsequentNode.expression;
			}

			// If not repetition, check for overlap. Else a brick wall.
			if (subsequentNode.type !== 'Repetition') {
				// If we overlap, continue processing.
				if (RTAGI.anyGroupsIntersect([outerRunNode.expression, subsequentNode])) {
					console.error(`Overlap between outerRunNode ${JSON.stringify(outerRunNode)} and subsequent node ${JSON.stringify(subsequentNode)}. Proceeding to next subsequentNode.`);
					return;
				}
				// No overlap: brick wall.
				else {
					console.error(`No overlap between outerRunNode ${JSON.stringify(outerRunNode)} and subsequent node ${JSON.stringify(subsequentNode)}. Hit a wall.`);
					chanceOfQOA = false;
					return;
				}
			}

			// This node is repetition of some kind.

			if (nodeIsQuantified(subsequentNode, options)) {
				// If node is "quantified enough", we have a possible QOA.
				console.error(`Possible QOA...`);
				if (RTAGI.anyGroupsIntersect([outerRunNode.expression, subsequentNode.expression])) {
					// QOA.
					console.error(`Bingo! Intersection between outerRunNode ${JSON.stringify(outerRunNode)} and subsequent node ${JSON.stringify(subsequentNode)}. QOA!`);
					hasQOA = true;
					chanceOfQOA = false; // short-circuit subsequent iterations;
					return;
				}
				else if (!nodeIsOptional(subsequentNode, options)) {
					// Uh oh. Quantified but not optional, and no overlap. e.g. {1,3}. Brick wall.
					console.error(`No overlap between outerRunNode ${JSON.stringify(outerRunNode)} and non-optional subsequent node ${JSON.stringify(subsequentNode)}. Hit a wall.`);
					chanceOfQOA = false;
					return;
				}
			}

			// Repetition but not enough to qualify as "quantified". e.g. '?'. Perhaps we can skip it.
			if (nodeIsOptional(subsequentNode, options)) {
				console.error(`No overlap, and it is repetition, but not "quantified". Luckily it is optional. Skipping it`);
				return;
			}
			else {
				console.error(`No overlap, and it is repetition, not "quantified". Sadly it is not optional. Hit a wall.`);
				chanceOfQOA = false; // short-circuit subsequent iterations;
				return;
			}
		});
	});

	return hasQOA;
}

// @param exp: entry from an Alternative's 'expressions' array. An AST node.
// @param options: module input
//
// Returns true if exp is quantified according to options
//   - false for question marks, and maybe small upper bounds on repetition
function nodeIsQuantified(node, options) {
	// Peel groups.
	while (node.type === 'Group') {
		node = node.expression;
	}

	if (node.type === 'Repetition') {
		// Question marks are irrelevant.
		if (node.quantifier.kind === '?') {
			return false;
		}
		// Small upper bounds are irrelevant.
		if (node.quantifier.kind === 'Range' &&
				('to' in node.quantifier && node.quantifier.to < options.minimumRepetitionUpperLimit)) {
			return false;
		}

		return true;
	}

	return false;
}

// @param exp: entry from an Alternative's 'expressions' array. An AST node.
// @param options: module input
//
// Returns true if exp is optional
function nodeIsOptional(node, options) {
	if (node.type === 'Repetition') {
		if (node.quantifier.kind === '?') {
			return true;
		}
		if (node.quantifier.kind === '*') {
			return true;
		}
		if (node.quantifier.kind === 'Range' && node.quantifier.from === 0) {
			return true;
		}
	}

	return false;
}

module.exports = hasQuantifiedOverlappingAdjacent;
