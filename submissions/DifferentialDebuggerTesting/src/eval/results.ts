import { Action, Output } from "../debuggers/interface";
import { TextFile, TraceFile } from "../utils/trace-file";
import { assertNever, deepestAstNodeTypeContainingLine } from "../utils/utils";

export class Parameters {
    constructor(readonly breakpointsPerLine: number,
                readonly breakpointRemoveProb: number,
                readonly maxActions: number) {
    }

    readonly resultsDir = `results/breakpointsPerLine${this.breakpointsPerLine}-breakpointRemoveProb${this.breakpointRemoveProb}-maxActions${this.maxActions}`;
    readonly outputsDataPath = `${this.resultsDir}/outputs.json`;
    readonly runDataPath = `${this.resultsDir}/runs.csv`;
}

export class Result {
    toTraceString(): string {
        if (this instanceof Difference) {
            return this.outputWithDifference.replace(/([A-Z])/g, ' $1').toLowerCase() + " difference";
        } else if (this instanceof NoDifference) {
            switch (this.reason) {
                case "executionFinished":
                    return "finished execution without differences";
                case "maxActions":
                    return "executed maximum number of actions without differences";
            }
        }
        return assertNever("should either be Difference or NoDifference");
    }

    toCsv(): string {
        if (this instanceof Difference) {
            return this.outputWithDifference + "Difference";
        } else if (this instanceof NoDifference) {
            return this.reason;
        }
        return assertNever("should either be Difference or NoDifference");
    }

    static fromCsv(str: string): Result {
        switch (str) {
            case "breakpointDifference":
                return new Difference("breakpoint");
            case "pauseLocationDifference":
                return new Difference("pauseLocation");
            case "terminationDifference":
                return new Difference("termination");
            case "callStackDifference":
                return new Difference("callStack");
            case "variablesDifference":
                return new Difference("variables");
            case "executionFinished":
                return new NoDifference(str);
            case "maxActions":
                return new NoDifference(str);
        }
        return assertNever("should either be Difference or NoDifference");
    }
}
export class Difference extends Result {
    constructor(readonly outputWithDifference: Output) {
        super();
    }
}
export class NoDifference extends Result {
    constructor(readonly reason: "executionFinished" | "maxActions") {
        super();
    }
}

export type DifferenceClass = "truePositive" | "falsePositive" | "noteworthy";
export const DifferenceClass = {
    fromCsv(str: string): DifferenceClass {
        if (str === "truePositive" || str === "falsePositive" || str === "noteworthy") {
            return str;
        }
        return assertNever(`invalid value ${str} for DifferenceClass`);
    }
};

export class Run {
    static readonly CsvHeader = "breakpointsPerLine,breakpointRemoveProb,maxActions,testcase,seed,breakpointCount,resumptionActionCount,result,lastAction,lastLine,lastAstNode,runtimeMs,differenceClass,differenceId,comment";

    static tracePath(params: Parameters, testcase: string, seed: number): string {
        return `${params.resultsDir}/${testcase}/seed${seed}.log`;
    }

    readonly tracePath = Run.tracePath(this.params, this.testcase, this.seed);
    constructor(readonly params: Parameters,
                readonly testcase: string,
                readonly seed: number,
                readonly breakpointCount: number,
                readonly resumptionActionCount: number,
                readonly result: Result,
                readonly lastAction: Action,
                readonly lastLine: number,
                readonly lastAstNode: string,
                readonly runtimeMs: number,
                public differenceClass?: DifferenceClass,
                public differenceId: string = "",
                public comment: string = "",
                ) {
    }

    toCsvLine(): string {
        return `${this.params.breakpointsPerLine},${this.params.breakpointRemoveProb},${this.params.maxActions},${this.testcase},${this.seed},${this.breakpointCount},${this.resumptionActionCount},${this.result.toCsv()},${this.lastAction},${this.lastLine},${this.lastAstNode},${this.runtimeMs},${(this.differenceClass === undefined) ? "" : this.differenceClass},${this.differenceId},${this.comment}`
    }

    static fromCsvLine(line: string): Run {
        const cells = line.split(",");
        return new Run(
            new Parameters(parseFloat(cells[0]), parseFloat(cells[1]), parseFloat(cells[2])),
            cells[3],
            parseFloat(cells[4]),
            parseFloat(cells[5]),
            parseFloat(cells[6]),
            Result.fromCsv(cells[7]),
            cells[8] as Action,
            parseFloat(cells[9]),
            cells[10],
            parseFloat(cells[11]),
            (cells[12] === "") ? undefined : DifferenceClass.fromCsv(cells[11]),
            cells[13],
            cells[14]
        );
    }

    update(differenceClass: DifferenceClass, differenceId: string, comment: string) {
        const oldRuns = TextFile.read(this.params.runDataPath, "");
        const oldRunLine = this.toCsvLine();
        this.differenceClass = differenceClass;
        this.differenceId = differenceId;
        this.comment = comment;
        const updatedRuns = oldRuns.replace(oldRunLine, this.toCsvLine());
        TextFile.overwrite(this.params.runDataPath, updatedRuns.trim());
    }
}

export class GlobalResults {
    constructor(readonly params: Parameters, appendHeader?: "appendHeader") {
        if (appendHeader === "appendHeader" && TextFile.read(params.runDataPath, "").length === 0) {
            TextFile.append(params.runDataPath, Run.CsvHeader);
        }
    }

    private outputsData: { [outputXactionXastNodeXdifference: string]: number } = JSON.parse(TextFile.read(this.params.outputsDataPath, "{}"));
    private runData: Run[] = TextFile.read(this.params.runDataPath, "")
        .split("\n")
        .filter(line => line !== Run.CsvHeader && line !== "")
        .map(Run.fromCsvLine);

    addOutputData(output: Output, action: Action, astNode: string, difference: "difference" | "noDifference") {
        const index = `${output}|${action}|${astNode}|${difference}`;
        this.outputsData[index] = (this.outputsData[index] === undefined) ? 1 : this.outputsData[index] + 1;
        TextFile.overwrite(this.params.outputsDataPath, JSON.stringify(this.outputsData, null, 2));
    }

    addRunData(run: Run) {
        this.runData.push(run);
        TextFile.append(this.params.runDataPath, run.toCsvLine());
    }

    runs(): Run[] {
        return this.runData;
    }

    differenceCount(output: Output, action: Action, astNode: string) {
        return this.outputsData[`${output}|${action}|${astNode}|difference`] || 0;
    }

    noDifferenceCount(output: Output, action: Action, astNode: string) {
        return this.outputsData[`${output}|${action}|${astNode}|noDifference`] || 0;
    }

}

export class RunResults {
    readonly trace = new TraceFile(Run.tracePath(this.globalResults.params, this.testcase, this.seed));
    private finished = false;
    private breakpointCount = 0;
    private resumptionActionCount = 0;
    private startTime = Date.now();

    constructor(readonly testcase: string, readonly seed: number, readonly scriptSource: string,
                private readonly globalResults: GlobalResults) {
    }

    isFinished(): boolean {
        return this.finished;
    }

    actionCount(): number {
        return this.resumptionActionCount;
    }

    action(action: Action) {
        if (action === "setBreakpoint" || action === "removeBreakpoint") {
            this.breakpointCount++;
        }
        if (action === "resume"
            || action === "stepIn"
            || action === "stepOut"
            || action === "stepOver") {
            this.resumptionActionCount++;
            this.trace.appendLine(action);
        }
        if (action === "startExecution") {
            this.trace.appendLine(action);
        }
    }

    noDifference(output: Output, lastAction: Action, lastLine: number) {
        this.globalResults.addOutputData(output, lastAction, deepestAstNodeTypeContainingLine(this.scriptSource, lastLine), "noDifference");
    }

    finish(result: Result, lastAction: Action, lastLine: number) {
        this.trace.appendLine(result.toTraceString());
        this.finished = true;

        let lastAstNode = deepestAstNodeTypeContainingLine(this.scriptSource, lastLine);
        if (result instanceof NoDifference) {
            this.globalResults.addOutputData("pauseLocation", lastAction, lastAstNode, "noDifference");
        } else if (result instanceof Difference) {
            this.globalResults.addOutputData(result.outputWithDifference, lastAction, lastAstNode, "difference");
        } else assertNever("should be Difference or NoDifference");
        this.globalResults.addRunData(new Run(
            this.globalResults.params,
            this.testcase,
            this.seed,
            this.breakpointCount,
            this.resumptionActionCount,
            result,
            lastAction,
            lastLine,
            lastAstNode,
            Date.now() - this.startTime));
    }
}
