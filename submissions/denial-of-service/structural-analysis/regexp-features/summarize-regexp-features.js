#!/usr/bin/env node

/* Author: Jamie Davis <davisjam@vt.edu>
 * Description: Print summary of regexp features: overall and on a per-project basis
 * Input: A file containing one-JSON-object-per-line describing a regexp, the modules that use it, and the regexp's use of regexp features.
 * Output: Prints to stdout a summary of the regexp features used.
 */

const readline = require('readline'),
      path = require('path'),
      fs = require('fs');

/* Invocation: node $0 filename*/
var prog_short_name = path.basename(process.argv[1]);
var usage = 'Usage: ' + prog_short_name + ' file_with_json_regexp_features';
if (process.argv.length != 3) {
  console.log(usage);
  process.exit(1);
}

/* Prepare globals. */

var globalNRegexps = 0;
var globalRegexpFeatureUse = newRegexpFeatureVector();

var module2features = {}; // module name-> {nRegexps, regexpFeatureUse}
var nModules = 0;

/* Process line-by-line. */
const rl = readline.createInterface({
  input: fs.createReadStream(process.argv[2]),
  crlfDelay: Infinity
});

rl.on('line', (line) => {
  var obj = JSON.parse(line);

  // Got a new regexp?
  if (!obj.features || !obj.modules)
    return;

  globalNRegexps++;

  if (globalNRegexps % 1000 === 0)
    console.error(`Processing regexp ${globalNRegexps}`);

  // Update globalRegexpFeatureUse.
  for (var feature in globalRegexpFeatureUse) {
    if (!globalRegexpFeatureUse.hasOwnProperty(feature))
      continue;

    if (obj.features.featureVector[feature])
      globalRegexpFeatureUse[feature]++;
  }

  // Update module2features.
  obj.modules.forEach((mod) => {
    if (!module2features[mod]) {
      nModules++;
      module2features[mod] = newRegexpFeatureVector();
    }
    
    for (var feature in module2features[mod]) {
      if (!module2features[mod].hasOwnProperty(feature))
        continue;

      if (obj.features.featureVector[feature])
        module2features[mod][feature]++;
    }
  });
});

rl.on('close', () => {

  if (globalNRegexps === 0) {
    console.log('Error, no regexps in file');
    process.exit(1);
  }

	console.error('Result:');

  // Calculate feature use across projects: O(1) * O(N) passes over module2features.
  var numberModulesUsingFeature = newRegexpFeatureVector();
	for (var feature in numberModulesUsingFeature) {
    // TODO Something wrong here.
    console.error(`Summarizing number of modules using ${feature}`);
		if (!numberModulesUsingFeature.hasOwnProperty(feature))
			continue;
    
    // Count # modules using this feature
    for (var module in module2features) {
			if (!module2features.hasOwnProperty(module))
				continue;

      if (module2features[module][feature])
        numberModulesUsingFeature[feature]++;     
    }
  }

  // Report on feature use across regexps.
  for (var feature in globalRegexpFeatureUse) {
    if (!globalRegexpFeatureUse.hasOwnProperty(feature))
      continue;

		console.error(`${globalRegexpFeatureUse[feature]} regexps used ${feature} (${globalRegexpFeatureUse[feature]}/${globalNRegexps} == ${round(100*globalRegexpFeatureUse[feature]/globalNRegexps, 2)}%)`);
//	console.log(JSON.stringify(globalRegexpFeatureUse));
  }

  // Report on feature use across projects.
  for (var feature in numberModulesUsingFeature) {
    if (!numberModulesUsingFeature.hasOwnProperty(feature))
      continue;

		console.error(`${numberModulesUsingFeature[feature]} modules used ${feature} (${numberModulesUsingFeature[feature]}/${nModules} == ${round(100*numberModulesUsingFeature[feature]/nModules, 2)}%)`);
//	console.log(JSON.stringify(globalRegexpFeatureUse));
  }

  // Terse summary for copy-paste into .tex table.
  console.log('feature: nRegexps & pRegexps & nModules & pModules');
  for (var feature in numberModulesUsingFeature) {
    if (!numberModulesUsingFeature.hasOwnProperty(feature))
      continue;

    var nRegexpsWithFeature = globalRegexpFeatureUse[feature];
    var pRegexpsWithFeature = round(100*nRegexpsWithFeature/globalNRegexps, 2);
    var nModulesWithFeature = numberModulesUsingFeature[feature];
    var pModulesWithFeature = round(100*nModulesWithFeature/nModules, 2);
    console.log(`${feature}: ${nRegexpsWithFeature} & ${pRegexpsWithFeature} & ${nModulesWithFeature} & ${pModulesWithFeature}`);
  }

});

// https://stackoverflow.com/a/7343013
function round(value, precision) {
	var multiplier = Math.pow(10, precision || 0);
	return Math.round(value * multiplier) / multiplier;
}

function newRegexpFeatureVector () {
  var regexpFeatureVector = { // If a regexp uses ADD > 0, then ADD++.
		// Chapman & Stolee, ISSTA 2016
		'ADD':  0,  // one-or-more repetition: z+
		'CG' :  0,  // capture group: (caught)
		'KLE':  0,  // Kleene star: a*
		'CCC':  0,  // custom character class: [aeiou]
		'ANY':  0,  // any non-newline char: .
		'RNG':  0,  // chars within a range: [a-z]
	// NB STR, END: Slight difference from C&S definition.
		'STR':  0,  // start-of-line / string: ^
		'END':  0,  // end-of-line / string: $
		'NCCC': 0,  // negated CCC: [^abc]
		'WSP':  0,  // whitespace: \s == \t \n \r \v \f or space
		'OR' :  0,  // logical or: a|b
		'DEC':  0,  // any decimal number: \d == 012345789
		'WRD':  0,  // any 'C99 word': [a-zA-Z0-9_]
		'QST':  0,  // zero-or-one repetition: z?
		'LZY':  0,  // lazy minimal match: z+?
		'NCG':  0,  // non-capturing group: a(?:b)c
	//  'PNG':  0,  // named capture group: (?P<name>x) // NOT IN JAVASCRIPT
		'SNG':  0,  // exactly n repetition: z{8}
		'NWSP': 0,  // non-whitespace: \S
		'DBB':  0,  // doubly-bound; n <= x <= m repetition: z{3,8}
		'NLKA': 0,  // non-lookahead; sequence doesn't follow: a(?!yz)
		'WNW':  0,  // word/non-word boundary: \b
		'NWRD': 0,  // non-word character: \W
		'LWB':  0,  // at least n repetition: z{15,}
		'LKA':  0,  // lookahead: a(?=bc)
	//  'OPT':  0,  // options wrapper: (?i)CasE // NOT IN JAVASCRIPT
	//  'NLKB': 0,  // non-lookbehind; sequence doesn't precede: (?<!x)yz  // NOT IN JAVASCRIPT
	//  'LKB':  0,  // look-behind; matching sequence precedes: (?<=a)bc // NOT IN JAVASCRIPT
	//  'ENDZ': 0,  // absolute end of string: \Z // NOT IN JAVASCRIPT
		'BKR':  0,  // match the i'th CG: \1
		'NDEC': 0,  // any non-decimal: \D
	//  'BKRN': 0,  // back-reference to named capture group // NOT IN JAVASCRIPT
		'VWSP': 0,  // matches U+000B (vertical whitespace): \v
		'NWNW': 0,  // negated WNW: \B

		// Additional features from JavaScript (or at least, not in the C&S table):
		'BSP':  0,  // matches U+0008 (backspace): [\b]
		'CTL':  0,  // control character: \cX , A <= X <= Z
		'TAB':  0,  // matches U+0009 (tab): \t
		'NUL':  0,  // matches U+0000 (null): \0

		'HEX':  0,  // matches hex: \xhh
		'UNI':  0,  // matches unicode: \uUUUU
	};

  return regexpFeatureVector;
}
