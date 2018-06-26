import fs = require("fs-extra");

const { dirname } = require("path");
const { sprintf } = require("sprintf-js");

import { Breakpoint, DebugSession, NotRunningState, Variable } from "../debuggers/interface";

export class TextFile {
    // convenience functions
    static read(path: string, defaultValue: string) {
        if (!fs.existsSync(path))
            return defaultValue;
        return fs.readFileSync(path, "utf-8");
    }

    static overwrite(path: string, line: string) {
        new TextFile(path, "overwrite").appendLine(line);
    }

    static append(path: string, line: string) {
        new TextFile(path, "append").appendLine(line);
    }

    constructor(readonly path: string, overwriteOrAppend: "overwrite" | "append") {
        fs.ensureDirSync(dirname(path));
        // make sure file exists and (optionally) truncate if non-empty
        if (overwriteOrAppend === "overwrite") {
            fs.closeSync(fs.openSync(path, "w"));
        }
    }

    appendLine(line: string): void {
        fs.appendFileSync(this.path, `${line}\n`);
    }
}

export class TraceFile extends TextFile {
    constructor(readonly path: string) {
        super(path, "overwrite");
    }

    setBreakpoint(line: number) {
        this.appendLine(`set breakpoint @ line ${line + 1}`);
    }

    setBreakpointResult(breakpoints: Breakpoint[], dbg: DebugSession) {
        this.withDebuggerPrefix("  ", breakpoints.map(bp => `set @ line ${bp.actualLocation.lineNumber + 1}, id: ${bp.id}`), dbg);
    }

    removeBreakpointsResult(breakpoints: Breakpoint[], dbg: DebugSession) {
        this.withDebuggerPrefix("  ", breakpoints.map(bp => `removed @ line ${bp.actualLocation.lineNumber + 1}, id: ${bp.id}`), dbg);
    }

    actionResult(state: NotRunningState, dbg: DebugSession) {
        this.withDebuggerPrefix(
            "  ",
            [(state === "finished") ? state : `paused @ line ${state.location.lineNumber + 1}`],
            dbg
        );
    }

    callStackResult(callStack: string[], dbg: DebugSession) {
        this.withDebuggerPrefix("    ", callStack, dbg);
    }

    variablesResult(variables: Variable[], dbg: DebugSession) {
        this.withDebuggerPrefix("    ", variables.map(v => v.toString()), dbg);
    }

    private withDebuggerPrefix(indent: string, strs: string[], dbg: DebugSession) {
        const browserString = dbg.browser.toString().charAt(0).toUpperCase() + dbg.browser.toString().slice(1);

        let [firstLine, ...rest] = strs;
        this.appendLine(sprintf("%s%-8s %s", indent, `${browserString}:`, firstLine || ""));
        for (const line of rest) {
            this.appendLine(`${indent}         ${line}`);
        }
    }
}
