import { assertNever, remove } from "../utils/utils";

export interface Script {
    readonly id: any;
    readonly url: string;
    readonly source?: string;
}

export interface Location {
    readonly script: Script;
    readonly lineNumber: number;
    readonly columnNumber?: number;
}

export interface Breakpoint {
    readonly id: any;
    readonly actualLocation: Location;
}

export interface PauseState {
    readonly location: Location,
    readonly hitBreakpoints: Breakpoint[];
}

export class Variable {
    constructor(public name: string,
                public type: string,
                // NOTE only present for primitives
                // if type === "undefined" this value is "not present" because it is undefined in the script
                // if type !== "undefined" (e.g. "object") it is not present because not printable
                public value?: any) {
    }

    toString(): string {
        const nameAndType = `${this.name}: ${this.type}`;
        if (this.value === undefined || this.value === null) {
            return nameAndType;
        } else {
            return `${nameAndType} = ${this.value}`;
        }
    }

    static compare(a: Variable, b: Variable): number {
        return a.toString().localeCompare(b.toString(), "en-US");
    }

    // take first (== first binding instance) if multiple occurrences
    static takeFirst(variables: Variable[]): Variable[] {
        let result = [] as Variable[];
        const alreadyTaken = new Set<string>();
        for (let b of variables) {
            if (!alreadyTaken.has(b.name)) {
                alreadyTaken.add(b.name);
                result.push(b);
            }
        }

        // sort by name
        return result.sort(Variable.compare);
    }
}

export interface Browser {
    start(): Promise<void>,
    close(): Promise<void>,

    toString(): string,

    debugScript(scriptSource: string): Promise<DebugSession>
}

export type ResumptionAction = "stepIn" | "stepOver" | "stepOut" | "resume";
export type BreakpointAction = "setBreakpoint" | "removeBreakpoint"
export type Action = ResumptionAction | BreakpointAction | "startExecution";

export type Output = "breakpoint" | "pauseLocation" | "callStack" | "variables" | "termination";

// NOTE finished == event onload for Chrome, tabNavigated for Firefox
export type NotRunningState = PauseState | "finished";

export abstract class DebugSession {
    constructor(readonly browser: Browser, readonly scripts: Script[]) {
    }

    protected breakpoints_: Breakpoint[] = [];
    breakpoints(): Breakpoint[] {
        return this.breakpoints_;
    }
    abstract setBreakpoint(location: Location): Promise<Breakpoint[]>;
    abstract removeBreakpoint(breakpoint: Breakpoint): Promise<boolean>;
    async removeBreakpoints(breakpointsToRemove: Breakpoint[]): Promise<Breakpoint[]> {
        let removedBreakpoints: Breakpoint[] = [];
        for (const bp of breakpointsToRemove) {
            if(await this.removeBreakpoint(bp)) {
                removedBreakpoints.push(bp);
            }
        }
        return removedBreakpoints;
    }

    abstract startExecution(): Promise<void>;

    async action(action: ResumptionAction): Promise<void> {
        switch (action) {
            case "resume":
                await this.resume();
                break;
            case "stepIn":
                await this.stepIn();
                break;
            case "stepOut":
                await this.stepOut();
                break;
            case "stepOver":
                await this.stepOver();
                break;
            default:
                assertNever(`unknown action ${action}`);
        }
    }
    protected abstract resume(): Promise<void>;
    protected abstract stepIn(): Promise<void>;
    protected abstract stepOut(): Promise<void>;
    protected abstract stepOver(): Promise<void>;

    abstract onNotRunning(callback: (notRunningState: NotRunningState) => void): void;

    abstract callStackFunctionNames(pauseState: PauseState): Promise<string[]>;
    abstract nonGlobalVariables(pauseState: PauseState): Promise<Variable[]>;

    abstract close(): Promise<void>;
}
