const esprima = require("esprima");
const estraverse = require("estraverse");
const lineColumn = require("line-column");

/**
 * Remove all occurrences of elementToRemove from array.
 * @param array where element is removed from (array is mutated)
 * @param elementToRemove
 * @return {boolean} true iff elementToRemove was removed at least once
 */
export function remove<T>(array: T[], elementToRemove: T): boolean {
    let index = 0;
    let elementFound = false;
    while ((index = array.indexOf(elementToRemove, index)) > -1) {
        elementFound = true;
        array.splice(index, 1);
    }
    return elementFound;
}

export function lines(str: string): string[] {
    return str.split(/\r\n|\r|\n/);
}

/**
 * gives the byte index range (i.e. begin and end, both inclusive) for a line of a string
 * @param {string} str
 * @param {number} line
 * @return {[number , number]}
 */
export function lineToRange(str: string, line: number): [number, number] {
    // append two newlines to make sure line+1 always exists
    const lineColumnFinder = lineColumn(str + "\n\n", { origin: 0 });
    return [lineColumnFinder.toIndex(line, 0), lineColumnFinder.toIndex(line + 1, 0) - 1];
}

/**
 * Assert execution should never reach this point.
 * See https://www.typescriptlang.org/docs/handbook/advanced-types.html for the never type.
 * Useful for exhaustiveness checking by the compiler etc.
 */
export function assertNever(message: string = "should be dead code"): never {
    console.error(message);
    throw new Error(message);
}

export function assert(condition: boolean, message: string = "assertion failed") {
    if (!condition) {
        assertNever(message);
    }
}

export function compareNumbers(a: number, b: number): number {
    return a - b;
}

export function compareStrings(a: string, b: string): number {
    return a.localeCompare(b);
}

export function equalsArray<T>(as: T[], bs: T[], equality: (a: T, b: T) => boolean = (a, b) => a === b): boolean {
    if (as.length !== bs.length) {
        return false;
    }
    for (let i = 0; i < as.length; i++) {
        if (!equality(as[i], bs[i])) {
            return false;
        }
    }
    return true;
}

export function id<T>(x: T): T {
    return x;
}

/**
 * @param source
 * @param {number} line 0-based
 */
export function deepestAstNodeTypeContainingLine(source: string, line: number) {
    if (line < 0) return "Program";

    let [start, end] = lineToRange(source, line);
    // trim whitespace from start end end index to find deeper nested nodes
    while ((source[start] == ' ' || source[start] == '\t' || source[start] == '\n' || source[start] == '\r') && start < end) start++;
    while ((source[end] == ' ' || source[end] == '\t' || source[end] == '\n' || source[end] == '\r') && start < end) end--;

    const ast = esprima.parse(source, { range: true });
    // Program node starts only at first statement, not at comments/empty lines before,
    // so initialize with "Program" AST node type in case nothing matches (not even Program node)
    let result = "Program";
    estraverse.traverse(ast, {
        enter: function (node) {
            // AST node contains line fully
            if (node.range[0] <= start && node.range[1] >= end) {
                result = node.type;
            }
        }
    });
    return result;
}
