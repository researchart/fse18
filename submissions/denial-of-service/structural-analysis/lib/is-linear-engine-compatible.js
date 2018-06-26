/* Author: Jamie Davis <davisjam@vt.edu>
 * Description: Check if this regex uses any features incompatible with a linear-time regex engine.
 *              For more details, see Cox's work.
 *              See https://swtch.com/~rsc/regexp/regexp3.html under 'Caveats'.
 */

const createRegExp = require('./create-regexp');
const regexpTree = require('regexp-tree');

/**
 * regex: string representation 'abc' or RegExp object /abc/
 * Accepts most of the Python syntax as well.
 *
 * Returns true or false: whether or not the regex is compatible with linear-time engines.
 */
function isLinearEngineCompatible(regex) {
	const pattern = regex.source || regex;
  // regexpTree doesn't support lookbehind assertions, but they aren't linear-engine compatible.
	if (pattern.match(/\(\?<=/) || pattern.match(/\(\?<!/)) {
		return false;
	}

	const re = createRegExp(regex);
	console.error(`re: /${re.source}/`);
	const ast = regexpTree.parse(re);

	/* We're going with Cox's definition of non-linear-time features:
	 *   - backreferences
	 *   - lookahead assertions
	 *   - lookbehind assertions
	 * (see link at the top).
	 */

	let anyNonLinearFeatures = false;
	regexpTree.traverse(ast, {
		'Backreference': function ({node}) {
			anyNonLinearFeatures = true;
		},

		'Assertion': function({node}) {
			if (node.kind === 'Lookahead' ||
				  node.kind === 'Lookbehind')
			{
				anyNonLinearFeatures = true;
			}
		}
	});

	return !anyNonLinearFeatures;
}

module.exports = isLinearEngineCompatible;
