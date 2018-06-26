import { Difference, DifferenceClass, GlobalResults, Parameters, Result, Run } from "./results";
import { Action, Output } from "../debuggers/interface";
import { assert, compareNumbers, compareStrings, lines } from "../utils/utils";
import { execSync } from "child_process";
import { testcasesPath } from "./testcases";
import { TextFile } from "../utils/trace-file";
import { delay } from "../utils/promise";
import { randomElement, shuffle } from "../utils/random";
import prompt = require('prompt-promise');
import seedrandom = require("seedrandom");

class EquivalenceClass {
    constructor(oneRun: Run) {
        // this.result = (oneRun.result.toCsv() === new Difference("termination").toCsv()) ? new Difference("pauseLocation") : oneRun.result;
        this.result = oneRun.result;
        this.lastAction = oneRun.lastAction;
        this.lastAstNode = oneRun.lastAstNode;
        this.runs = [oneRun];
    }

    readonly result: Result;
    readonly lastAction: Action;
    readonly lastAstNode: string;
    public runs: Run[];

    size(): number {
        return this.runs.length;
    }

    id(): string {
        return `${this.result.toCsv()}|${this.lastAction}|${this.lastAstNode}`;
    }

    static readonly CsvHeader = "differenceType,lastAction,lastAstNode,numberOfRuns,failed,percentOfFailed";
    toCsvLine(totalDifferenceRuns: number): string {
        return `${this.result.toCsv()},${this.lastAction},${this.lastAstNode},${this.size()},${this.result instanceof Difference},${(this.size() / totalDifferenceRuns * 100).toFixed(2)}`;
    }

    static compareById(a: EquivalenceClass, b: EquivalenceClass): number {
        return compareStrings(a.id(), b.id());
    }

    static compareBySize(a: EquivalenceClass, b: EquivalenceClass) {
        return compareNumbers(a.size(), b.size());
    }
}

function divideIntoEqquivalenceClasses(runs: Run[]): EquivalenceClass[] {
    let classMap: Map</* id */string, EquivalenceClass> = new Map();
    for (const run of runs) {
        const newClass = new EquivalenceClass(run);
        const foundClass = classMap.get(newClass.id());
        if (foundClass !== undefined) {
            foundClass.runs.push(run);
        } else {
            classMap.set(newClass.id(), newClass);
        }
    }
    return [...classMap.values()];
}

async function inspectRun(run: Run): Promise<[string, string]> {
    const traceLength = lines(TextFile.read(run.tracePath, "")).length;
    const traceLineToJumpTo = Math.max(1, traceLength - 25);
    execSync(`subl -b --command "move_to_group { \\"group\\": 0 }" "${run.tracePath}":${traceLineToJumpTo}`);
    await delay(200);
    execSync(`subl -b --command "move_to_group { \\"group\\": 1 }" "${testcasesPath}/${run.testcase}.js":${(run.lastLine + 1)}`);

    // copy testcase to script.js for quick live inspection in both browsers
    TextFile.overwrite("/home/daniel/Documents/masterthesis/test/js-in-browser-template/script.js", TextFile.read(`${testcasesPath}/${run.testcase}.js`, ""));

    const rootCauseId: string = await prompt(`\t${run.testcase}/seed${run.seed} rootCauseId: `);
    // const comment: string = await prompt(`\t${run.testcase}/seed${run.seed} comment: `);

    execSync("subl -b --command close_all");
    await delay(200);

    return [rootCauseId, ""];
}

(async function () {
    const results = new GlobalResults(new Parameters(0.1, 0.2, 20));
    const runs = results.runs();

    /*
     * Overall stats on all runs
     */
    console.log(`${runs.length} runs:`);
    let resultCounts: Map<string, number> = new Map();
    runs.forEach(run => resultCounts.set(run.result.toCsv(), (resultCounts.get(run.result.toCsv()) || 0) + 1));
    const successfulResultCount = (resultCounts.get("maxActions") || 0) + (resultCounts.get("executionFinished") || 0);
    const failedResultCount = runs.length - successfulResultCount;
    console.log(`\tsuccessful:      ${successfulResultCount} (${(successfulResultCount / runs.length * 100).toFixed(2)}%)`);
    console.log(`\twith difference: ${failedResultCount} (${(failedResultCount / runs.length * 100).toFixed(2)}%)`);
    console.log("\t--- detailed ---");
    for (const [result, count] of resultCounts.entries()) {
        console.log(`\t${result}: ${count} (${(count / runs.length * 100).toFixed(2)}%)`)
    }

    /*
     * Compute and save eq classes
     */
    const eqClasses: EquivalenceClass[] = divideIntoEqquivalenceClasses(runs);
    const eqClassesFile = new TextFile(`${results.params.resultsDir}/eq-classes.csv`, "overwrite");
    eqClassesFile.appendLine(EquivalenceClass.CsvHeader);
    eqClasses.forEach(c => eqClassesFile.appendLine(c.toCsvLine(failedResultCount)));
    console.log(`${eqClasses.length} equivalence classes (see CSV file):`);
    console.log(`\twith difference: ${eqClasses.filter(c => c.result instanceof Difference).length}`);

    /*
     * Number of possible eq classes (# of difference types * # of last actions * # of AST nodes)
     */
    const possibleValues = {
        outputWithDifference: new Set<Output>(),
        action: new Set<Action>(),
        astNode: new Set<string>(),
    };
    runs.forEach(run => {
        if (run.result instanceof Difference) {
            possibleValues.outputWithDifference.add(run.result.outputWithDifference);
            possibleValues.action.add(run.lastAction);
            possibleValues.astNode.add(run.lastAstNode);
        }
    });
    console.log(`\tpossible equivalence class count: ${possibleValues.outputWithDifference.size * possibleValues.action.size * possibleValues.astNode.size}`);
    for (const possibleProperty in possibleValues) {
        console.log(`\t\tpossible ${possibleProperty}s (${possibleValues[possibleProperty].size}): ` + [...possibleValues[possibleProperty]].join(", "));
    }

    /*
     * Manual inspection: Sampling from equivalence classes vs. sampling all runs directly
     */

    let startIndex = 0;
    const n = 50; // how many to manually inspect

    const manualInspectionFile = new TextFile(`${results.params.resultsDir}/manual-inspection.csv`, "append");
    manualInspectionFile.appendLine("samplingStrategy,inspectionIndex,run,rootCauseId,comment");
    async function inspectRuns(runs: Run[], samplingStrategy: string) {
        manualInspectionFile.appendLine(`${samplingStrategy},${-1},,,`);
        let inspectionIndex = startIndex;
        for (let run of runs.slice(startIndex, n)) {
            const [rootCauseId, comment] = await inspectRun(run);
            manualInspectionFile.appendLine(`${samplingStrategy},${inspectionIndex++},${run.tracePath},${rootCauseId},${comment}`);
        }
    }

    // random sampling
    const randomRuns = shuffle(seedrandom(0), runs.filter(r => r.result instanceof Difference));
    console.log("sample runs uniformly:");
    await inspectRuns(randomRuns, "raw");

    // sampling from eq classes
    const rng = seedrandom(0);
    const runsFromEqClasses = eqClasses
        .filter(c => c.result instanceof Difference)
        .sort(EquivalenceClass.compareBySize).reverse()
        .map(c => randomElement(rng, c.runs));
    console.log("sample runs from largest equivalence classes:");
    await inspectRuns(runsFromEqClasses, "eqClasses");
})();