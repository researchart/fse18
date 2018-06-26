#!/usr/bin/env node

/* Author: Jamie Davis <davisjam@vt.edu>
 * Description: Print interesting information about the supplied pattern.
 * Input: A file containing a single-line JSON object with at least the member 'pattern'.
 * Output: Prints to stdout the original object with a new member: features
 *   features is an object with members: length ASTNodes featureVector
 *   featureVector is an object with members 'regexp features' and values the count of these features in the regexp pattern.
 */

const path = require('path'),
      fs = require('fs'),
      regexpTree = require('regexp-tree');

var prog_short_name = path.basename(process.argv[1]);

var usage = 'Usage: ' + prog_short_name + ' pattern.json';

/* Invocation: node $0 pattern */
if (process.argv.length != 3) {
  console.log(usage);
  process.exit(1);
}

var patternFile = process.argv[2];
var patternObj = JSON.parse(fs.readFileSync(patternFile));

console.error(`pattern: ${patternObj.pattern}`);

var length = patternObj.pattern.length;
var nASTNodes = 0;
var featureVector = {
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

function featureVector_markFound (type) {
  console.error(`Found ${type}`);
  featureVector[type]++;
}

var ast;
try {
  regexpTree.parser.setOptions({captureLocations: true});

  console.error('Parsing');
  var re = new RegExp(patternObj.pattern);
  ast = regexpTree.parse(re);
} catch (e) {
  console.error('Could not parse:');
  console.error(e);
  process.exit(1);
}

console.error('Traversing the regexp-tree instance');
nASTNodes = 0;
regexpTree.traverse(ast, {
  Char({node}) {
    nASTNodes++;
    console.error('traverse: Char node');
    console.error(node);

    if (node.kind && node.kind === 'meta' && node.value === '.')
      featureVector_markFound('ANY');

    if (node.kind && node.kind === 'meta' && node.value === '\\s')
      featureVector_markFound('WSP');

    if (node.kind && node.kind === 'meta' && node.value === '\\S')
      featureVector_markFound('NWSP');

    if (node.kind && node.kind === 'meta' && node.value === '\\d')
      featureVector_markFound('DEC');

    if (node.kind && node.kind === 'meta' && node.value === '\\D')
      featureVector_markFound('NDEC');

    if (node.kind && node.kind === 'meta' && node.value === '\\w')
      featureVector_markFound('WRD');

    if (node.kind && node.kind === 'meta' && node.value === '\\W')
      featureVector_markFound('NWRD');

    if (node.kind && node.kind === 'meta' && node.value === '\\v')
      featureVector_markFound('VWSP');

    if (node.kind && node.kind === 'meta' && node.value === '\\b')
      featureVector_markFound('BSP');

    if (node.kind && node.kind === 'meta' && node.value === '\\t')
      featureVector_markFound('TAB');

    if (node.kind && node.kind === 'decimal' && node.value === '\\0')
      featureVector_markFound('NUL');

    if (node.kind && node.kind === 'control')
      featureVector_markFound('CTL');

    if (node.kind && node.kind === 'hex')
      featureVector_markFound('HEX');

    if (node.kind && node.kind === 'unicode')
      featureVector_markFound('UNI');
  },

  CharacterClass({node}) {
    nASTNodes++;

    if (node.negative)
      featureVector_markFound('NCCC');
    else
      featureVector_markFound('CCC');

  },

  ClassRange({node}) {
    nASTNodes++;

    featureVector_markFound('RNG');
  },

  Alternative({node}) {
    nASTNodes++;
  },

  Disjunction({node}) {
    nASTNodes++;

    featureVector_markFound('OR');
  },

  Group({node}) {
    nASTNodes++;
    if (node.capturing)
      featureVector_markFound('CG');
    else
      featureVector_markFound('NCG');
  },

  Backreference({node}) {
    nASTNodes++;

    if (node.kind === 'number')
      featureVector_markFound('BKR');
  },

  Quantifier({node}) {
    nASTNodes++;

    if (node.kind === '+')
      featureVector_markFound('ADD');

    if (node.kind === '*')
      featureVector_markFound('KLE');

    if (node.kind === '?')
      featureVector_markFound('QST');

    if (node.kind === 'Range') {
      if (node.from && node.to) {
        if (node.from < node.to)
          featureVector_markFound('DBB');
        else {
          // e.g. {3,3}
          if (node.loc.source.match(/,/))
            featureVector_markFound('DBB'); // test/DBB-same.json
          // e.g. {3}
          else
            featureVector_markFound('SNG');
        }
      }
      else if (node.from)
        featureVector_markFound('LWB');
    }

    if (!node.greedy)
      featureVector_markFound('LZY');
  },

  Repetition({node}) {
    nASTNodes++;
  },

  Assertion({node}) {
    nASTNodes++;

    if (node.kind && node.kind === '^')
      featureVector_markFound('STR');

    if (node.kind && node.kind === '$')
      featureVector_markFound('END');

    if (node.kind && node.kind === 'Lookahead') {
      if (node.negative)
        featureVector_markFound('NLKA');
      else
        featureVector_markFound('LKA');
    }

    if (node.kind && node.kind === '\\b')
      featureVector_markFound('WNW');

    if (node.kind && node.kind === '\\B')
      featureVector_markFound('NWNW');
  },

});

console.error(`Got ${nASTNodes} AST nodes from ast:`);
console.error(JSON.stringify(ast, null, 2));

var patternObj_enhanced = patternObj;
patternObj_enhanced.features = {
  length: length,
  nASTNodes: nASTNodes,
  featureVector: featureVector
};

console.log(JSON.stringify(patternObj_enhanced));
